//! 单线程独占 CAN 会话：查询与命令共用序号、UID 绑定及停机门禁。

use crate::calibration::{self, Job as CalibrationJob, Kind as CalibrationKind};
use crate::can::{Bitrates, ControlCanFd, Transport};
use crate::control::ControlMode;
pub use crate::control::MAX_SPEED_RAD_S;
use crate::diagnostics::{Logger, TraceTransport};
use crate::motor_configuration::{self, Info as ConfigurationInfo};
use crate::package::UpgradePackage;
use crate::target::Target;
use crate::upgrade::UpgradeClient;
use crate::wire::{self, u16le, u32le};
use anyhow::{Context, Result, bail, ensure};
use serde::{Deserialize, Serialize};
use sha2::Digest;
use std::collections::HashMap;
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::mpsc::{Receiver, Sender, TryRecvError};
use std::time::{Duration, Instant};

#[path = "device_jobs.rs"]
mod jobs;

const IDLE_SPEED_RAD_S: f64 = std::f64::consts::PI / 6.0;
const FEEDBACK_VALIDITY: Duration = Duration::from_millis(500);
const READ_BUDGET: Duration = Duration::from_millis(500);

#[derive(Debug)]
enum SessionIssue {
    Timeout(u16),
    Busy,
    Rejected(String),
}

impl std::fmt::Display for SessionIssue {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Timeout(kind) => write!(f, "Type{kind} 回复超时；设备状态未知"),
            Self::Busy => f.write_str("设备状态快照暂忙（BUSY），未更新测量"),
            Self::Rejected(message) => write!(f, "操作未执行：{message}"),
        }
    }
}
impl std::error::Error for SessionIssue {}
pub const OBSERVE_HZ: u32 = 60;
pub const OBSERVE_INTERVAL: Duration = Duration::from_nanos(1_000_000_000 / OBSERVE_HZ as u64);

pub(crate) fn advance_deadline(previous: Instant, interval: Duration, now: Instant) -> Instant {
    let scheduled = previous + interval;
    if scheduled > now {
        scheduled
    } else {
        now + interval
    }
}

#[derive(Clone, Default)]
pub struct Snapshot {
    pub uid: String,
    pub sn: String,
    pub motor_type: u32,
    pub app_version: u32,
    pub loader_version: u32,
    pub role: u8,
    pub product: u32,
    pub hardware: u16,
    pub schema: u32,
    pub abi: u32,
    pub features: u32,
    pub modes: u16,
    pub motor_state: u8,
    pub mode: u8,
    pub faults: u32,
    pub position_rad: Option<f64>,
    pub position_reference_rad: Option<f64>,
    pub speed_rad_s: Option<f64>,
    pub speed_reference_rad_s: Option<f64>,
    pub iq_a: Option<f64>,
    pub iq_reference_a: Option<f64>,
    pub vq_v: Option<f64>,
    pub temperature_c: Option<f64>,
    pub bus_v: Option<f64>,
    pub bus_a: Option<f64>,
    pub identity_at: Option<Instant>,
    pub feedback_at: Option<Instant>,
    pub stop_confirmed: bool,
    pub motion_may_run: bool,
    pub host_position_session: bool,
    pub motion_lease_error: Option<String>,
    pub active_configuration: Option<ConfigurationInfo>,
    pub saved_configuration: Option<ConfigurationInfo>,
    pub calibration: Option<CalibrationJob>,
}

impl Snapshot {
    pub fn identity_fresh(&self) -> bool {
        self.identity_at
            .is_some_and(|at| at.elapsed() <= Duration::from_secs(5))
    }
    pub fn feedback_fresh(&self) -> bool {
        self.feedback_at
            .is_some_and(|at| at.elapsed() <= FEEDBACK_VALIDITY)
    }
    /// 仅表示通信回读状态；None 表示尚未确认，不代替物理关相核查。
    pub fn enable_state(&self) -> Option<bool> {
        if !self.feedback_fresh() {
            None
        } else if self.motor_state == 6 {
            Some(true)
        } else if matches!(self.motor_state, 5 | 8)
            && self.mode == 0
            && self.stop_confirmed
            && !self.motion_may_run
        {
            Some(false)
        } else {
            None
        }
    }
    pub fn compatible(&self, target: &Target, bound_uid: &str) -> bool {
        self.identity_fresh()
            && self.uid == bound_uid
            && self.role == 1
            && self.app_version != 0
            && self.product == target.product
            && u32::from(self.hardware) == target.hardware
            && self.schema == target.schema
            && self.abi == target.abi
    }
    pub fn mode_supported(&self, mode: ControlMode) -> bool {
        let mask = (1 << 8) | (1 << 10) | (1 << 11) | (1 << 12);
        self.identity_fresh()
            && self.features & mask == mask
            && self.modes & (1 << mode.wire()) != 0
    }
    fn bus_limits_ok(&self) -> bool {
        self.feedback_fresh()
            && self.faults == 0
            && self.bus_v.is_some_and(|v| v > 0.0 && v <= 33.0)
            && self.bus_a.is_some_and(|a| a.abs() <= 3.0)
    }
    fn running_limits_ok(&self) -> bool {
        self.bus_limits_ok()
            && if self
                .calibration
                .as_ref()
                .is_some_and(CalibrationJob::active)
            {
                self.iq_a.is_none_or(|a| a.is_finite() && a.abs() <= 6.0)
            } else {
                self.iq_a.is_some_and(|a| a.abs() <= 6.0)
            }
            && (self.mode != 3 || self.position_rad.is_some_and(f64::is_finite))
            && (self.mode != 6 || self.vq_v.is_some_and(f64::is_finite))
    }
    pub fn safe_idle(&self, target: &Target, uid: &str) -> bool {
        self.compatible(target, uid)
            && !self.active_configuration.as_ref().is_some_and(ConfigurationInfo::blocked)
            && self.bus_limits_ok()
            // APP 在 IDLE 不发布 Iq；此时只接受本会话 STOP 0/0 后的 IDLE 回读。
            && self.iq_a.map_or(self.stop_confirmed, |a| a.abs() <= 6.0)
            && self.motor_state == 5
            && self.mode == 0
            && self
                .speed_rad_s
                .is_some_and(|speed| speed.abs() < IDLE_SPEED_RAD_S)
            && !self.motion_may_run
    }
    pub fn can_start_mode(&self, target: &Target, uid: &str, mode: ControlMode) -> bool {
        self.safe_idle(target, uid)
            && self.motion_lease_error.is_none()
            && self
                .active_configuration
                .as_ref()
                .is_none_or(|info| info.model != 0 && info.flags & 1 != 0 && info.calibrated())
            && self.stop_confirmed
            && self.mode_supported(mode)
            && (mode != ControlMode::Position
                || self.features & crate::control::HOST_POSITION_SESSION_FEATURE != 0)
            && (mode != ControlMode::Position || self.position_rad.is_some_and(f64::is_finite))
    }
    pub fn measured(&self, mode: ControlMode) -> Option<f64> {
        match mode {
            ControlMode::Voltage => self.vq_v,
            ControlMode::Torque => self.iq_a,
            ControlMode::Speed => self.speed_rad_s,
            ControlMode::Position => self.position_rad,
        }
    }
    pub fn can_set_target(&self, target: &Target, uid: &str, mode: ControlMode) -> bool {
        self.compatible(target, uid)
            && (mode != ControlMode::Position
                || (self.host_position_session
                    && self.features & crate::control::HOST_POSITION_SESSION_FEATURE != 0))
            && self.mode_supported(mode)
            && self.running_limits_ok()
            && self.motor_state == 6
            && self.mode == mode.wire()
            && self.measured(mode).is_some_and(f64::is_finite)
            && self
                .speed_rad_s
                .is_some_and(|speed| speed.abs() <= MAX_SPEED_RAD_S)
            && self.motion_may_run
    }
}

// 完整校验后再更新快照；独立于 SDK，离线测试覆盖单位和有效位。
fn apply_feedback(snapshot: &mut Snapshot, data: &[u8], node: u8) -> Result<()> {
    ensure!(
        data.len() == 45 && u16le(data, 2)? == 0 && data[42] == node,
        "状态帧格式错误"
    );
    let result = u16le(data, 0)?;
    if result == 6 {
        return Err(SessionIssue::Busy.into());
    }
    ensure!(result == 0, "Type108 查询被拒绝：result={result}");
    let bits = u16le(data, 24)?;
    ensure!(
        bits & 0xf000 == 0 && (2..=9).contains(&data[43]),
        "状态位非法"
    );
    let signed32 = |offset: usize, index: u16| -> Result<Option<f64>> {
        let value = i32::from_le_bytes(data[offset..offset + 4].try_into()?);
        ensure!(
            (bits & (1 << index) != 0) == (value != i32::MIN),
            "测量有效位不匹配"
        );
        Ok((value != i32::MIN).then_some(value as f64))
    };
    let signed16 = |offset: usize, index: u16| -> Result<Option<f64>> {
        let value = i16::from_le_bytes(data[offset..offset + 2].try_into()?);
        ensure!(
            (bits & (1 << index) != 0) == (value != i16::MIN),
            "测量有效位不匹配"
        );
        Ok((value != i16::MIN).then_some(value as f64))
    };
    for (offset, index) in [(8, 0), (12, 1), (16, 2), (20, 3)] {
        signed32(offset, index)?;
    }
    for (offset, index) in [
        (26, 4),
        (28, 5),
        (30, 6),
        (32, 7),
        (34, 8),
        (36, 9),
        (38, 10),
        (40, 11),
    ] {
        signed16(offset, index)?;
    }
    snapshot.faults = u32le(data, 4)?;
    snapshot.position_rad = signed32(12, 1)?.map(|value| value / 1000.0);
    snapshot.speed_rad_s = signed32(20, 3)?.map(|value| value / 1000.0);
    // 当前 Type108 在位置模式发布轨迹位置/速度参考；其他模式不解释位置参考。
    snapshot.position_reference_rad = if data[44] == 3 {
        signed32(8, 0)?.map(|value| value / 1000.0)
    } else {
        None
    };
    snapshot.speed_reference_rad_s = if matches!(data[44], 2 | 3) {
        signed32(16, 2)?.map(|value| value / 1000.0)
    } else {
        None
    };
    snapshot.iq_a = signed16(32, 7)?.map(|value| value / 1000.0);
    snapshot.iq_reference_a = signed16(30, 6)?.map(|value| value / 1000.0);
    snapshot.vq_v = signed16(38, 10)?.map(|value| value / 1000.0);
    snapshot.temperature_c = signed16(34, 8)?.map(|value| value / 100.0);
    snapshot.bus_v = signed16(26, 4)?.map(|value| value / 100.0);
    snapshot.bus_a = signed16(28, 5)?.map(|value| value / 1000.0);
    snapshot.motor_state = data[43];
    snapshot.mode = data[44];
    snapshot.feedback_at = Some(Instant::now());
    if data[43] == 6 {
        snapshot.motion_may_run = true;
    }
    if data[43] != 5 || data[44] != 0 || snapshot.faults != 0 {
        snapshot.stop_confirmed = false;
    }
    if data[44] != 3 || snapshot.faults != 0 {
        snapshot.host_position_session = false;
    }
    Ok(())
}

pub enum Command {
    ClearFault,
    StartCalibration(CalibrationKind),
    CancelCalibration,
    ReadConfiguration,
    SaveConfiguration {
        payload: [u8; 40],
        acknowledged: bool,
    },
    ActivateConfiguration,
    Observe(bool),
    Start {
        mode: ControlMode,
    },
    SetTarget {
        mode: ControlMode,
        value: f64,
    },
    Stop,
    Upgrade {
        directory: PathBuf,
        acknowledged: bool,
    },
    Disconnect,
}

pub enum Event {
    OperationDone,
    Rejected(String),
    ConfigurationSaved,
    ConfigurationActivated(u8),
    Snapshot(Box<Snapshot>),
    Status(String),
    Progress(f32),
    UpgradeDone,
    HardStop(String),
    Disconnected,
}

#[derive(Default, Serialize, Deserialize)]
struct SequenceStore(HashMap<String, u16>);

fn sequence_path() -> Result<PathBuf> {
    let appdata = std::env::var_os("APPDATA").context("缺少 APPDATA，无法保存控制序号")?;
    Ok(PathBuf::from(appdata)
        .join("YG")
        .join("YG_MC_HOST_RUST")
        .join("sequences.json"))
}

fn saved_control_sequence(path: &Path, uid: &str) -> Result<Option<u16>> {
    if !path.exists() {
        return Ok(None);
    }
    let store: SequenceStore = serde_json::from_slice(&fs::read(path)?)?;
    Ok(store.0.get(uid).copied())
}

fn reserve_control_sequence(
    path: &Path,
    uid: &str,
    manual_start: u16,
    next: u16,
    stop: bool,
) -> Result<()> {
    ensure!(!uid.is_empty(), "设备 UID 未绑定，禁止运动命令");
    let mut store: SequenceStore = if path.exists() {
        serde_json::from_slice(&fs::read(path)?)?
    } else {
        SequenceStore::default()
    };
    let previous = store.0.get(uid).copied();
    ensure!(
        stop || manual_start != 0 || previous.is_some(),
        "首次使用或其他主机曾发送命令时，须填写非零控制序号起点"
    );
    if let Some(previous) = previous {
        ensure!(
            stop || manual_start == 0 || manual_start.wrapping_sub(previous) < 0x8000,
            "手动起点早于本机保存的控制序号"
        );
    }
    store.0.insert(uid.to_owned(), next);
    persist_sequences(path, &store)
}

fn persist_sequences(path: &Path, store: &impl Serialize) -> Result<()> {
    fs::create_dir_all(path.parent().context("序号目录错误")?)?;
    let temporary = path.with_extension("tmp");
    let file = fs::File::create(&temporary)?;
    serde_json::to_writer_pretty(&file, store)?;
    file.sync_all()?;
    fs::rename(&temporary, path).context("控制序号持久化失败")?;
    Ok(())
}

fn renewal_path(path: &Path, uid: &str) -> Result<PathBuf> {
    ensure!(
        !uid.is_empty() && uid.bytes().all(|b| b.is_ascii_hexdigit()),
        "续租 UID 无效"
    );
    Ok(path.with_extension(format!("{uid}.lease.json")))
}

fn saved_renewal_counter(path: &Path) -> Result<u32> {
    let data = fs::read(path).with_context(|| {
        format!(
            "无法读取已有续租高水位 {}；禁止从1重新开始，请恢复原主机的计数记录",
            path.display()
        )
    })?;
    let counter = serde_json::from_slice(&data).context("续租高水位记录损坏，禁止使能")?;
    ensure!(counter != 0, "续租高水位为0，无法确认历史，禁止使能");
    Ok(counter)
}

// 每次新START均先预留；不沿用上一运行的未发区间，不为缺失记录创建零值。
fn reserve_renewals(
    path: &Path,
    uid: &str,
    minimum: Option<u32>,
) -> Result<std::ops::RangeInclusive<u32>> {
    let path = renewal_path(path, uid)?;
    let last = saved_renewal_counter(&path)?;
    if let Some(minimum) = minimum {
        ensure!(
            last.wrapping_sub(minimum) <= i32::MAX as u32,
            "本机续租高水位回退，禁止使能或继续续租"
        );
    }
    let first = last.wrapping_add(1).max(1);
    let end = first.saturating_add(255);
    persist_sequences(&path, &end)?;
    Ok(first..=end)
}

struct Session<T: Transport> {
    port: TraceTransport<T>,
    sequence_file: PathBuf,
    node: u8,
    target: Target,
    sequence: u16,
    manual_start: u16,
    bound_uid: String,
    expected_uid: String,
    high_rate: bool,
    snapshot: Snapshot,
    send: Sender<Event>,
    started: Instant,
    feedback_degraded: bool,
    lease_supported: bool,
    last_renewal: Option<Instant>,
    renewal_block: Option<std::ops::RangeInclusive<u32>>,
    pending_start: Option<(ControlMode, Instant)>,
}

fn check_expected_uid(expected: &str, actual: &str) -> Result<()> {
    ensure!(
        expected.is_empty() || actual == expected,
        "设备 UID 与扫描结果不一致；请重新扫描并选择设备"
    );
    Ok(())
}

impl<T: Transport> Session<T> {
    fn reserve_motion_lease(&mut self) -> Result<()> {
        let minimum = self.renewal_block.as_ref().map(|block| *block.end());
        let block = reserve_renewals(&self.sequence_file, &self.bound_uid, minimum)?;
        self.port.report(
            "info",
            "motion_lease_reserved",
            serde_json::json!({
                "sequence_file":self.sequence_file,
                "renewal_file":renewal_path(&self.sequence_file, &self.bound_uid)?,
                "first_counter":block.start(), "reserved_through":block.end(),
                "previous_reservation":minimum,
            }),
        );
        self.renewal_block = Some(block);
        Ok(())
    }

    fn dispatch(&mut self, command: Command) -> Result<bool> {
        match self.handle(command) {
            Err(error) if matches!(error.downcast_ref(), Some(SessionIssue::Rejected(_))) => {
                self.port.report(
                    "warning",
                    "command_rejected",
                    serde_json::json!({"reason":format!("{error:#}"),
                    "state":self.snapshot.motor_state,"mode":self.snapshot.mode}),
                );
                self.send.send(Event::Rejected(format!("{error:#}"))).ok();
                Ok(true)
            }
            result => result,
        }
    }

    fn renew_owned_motion(&mut self) -> Result<()> {
        let Some(last) = self.last_renewal else {
            return Ok(());
        };
        let starting = self.pending_start.is_some_and(|(mode, at)| {
            at.elapsed() < FEEDBACK_VALIDITY
                && self.snapshot.motor_state == 5
                && self.snapshot.mode == mode.wire()
        });
        if !self.snapshot.motion_may_run || (self.snapshot.motor_state != 6 && !starting) {
            self.last_renewal = None;
            return Ok(());
        }
        if last.elapsed() < Duration::from_millis(100) {
            return Ok(());
        }
        if self
            .renewal_block
            .as_ref()
            .is_none_or(|block| block.is_empty())
        {
            self.reserve_motion_lease()?;
        }
        let counter = self
            .renewal_block
            .as_mut()
            .and_then(Iterator::next)
            .context("续租计数预留失败")?;
        let seq = self.next_sequence();
        let reply = self.exchange(118, seq, &counter.to_le_bytes())?;
        ensure!(reply.len() == 4, "Type118续租回复长度错误");
        let result = u16le(&reply, 0)?;
        let detail = u16le(&reply, 2)?;
        if result != 0 || detail != 0 {
            self.port.report(
                "error",
                "motion_lease_rejected",
                serde_json::json!({
                    "counter":counter,"sequence":seq,"result":result,"detail":detail,
                    "sequence_file":self.sequence_file,
                    "renewal_file":renewal_path(&self.sequence_file, &self.bound_uid)?,
                }),
            );
            bail!("设备拒绝续租：计数{counter}，回复{result}/{detail}；停止当前运动会话");
        }
        self.last_renewal = Some(Instant::now());
        Ok(())
    }

    fn poll_feedback(&mut self) -> Result<()> {
        match self.read_feedback(true) {
            Ok(()) => {
                self.renew_owned_motion()?;
                if self.feedback_degraded {
                    self.feedback_degraded = false;
                    self.port.report(
                        "info",
                        "feedback_recovered",
                        serde_json::json!({"motion_replayed":false}),
                    );
                    self.send
                        .send(Event::Status("状态回读已恢复；未重发运动命令".into()))
                        .ok();
                }
                Ok(())
            }
            Err(error) => {
                let temporary = matches!(
                    error.downcast_ref(),
                    Some(SessionIssue::Busy | SessionIssue::Timeout(108))
                );
                let last = self.snapshot.feedback_at.unwrap_or(self.started);
                let grace = if self.snapshot.motion_may_run {
                    FEEDBACK_VALIDITY
                } else {
                    Duration::from_secs(2)
                };
                if temporary && last.elapsed() < grace {
                    if !self.feedback_degraded {
                        self.feedback_degraded = true;
                        self.send
                            .send(Event::Status(format!(
                                "{error:#}；等待有限重试，旧读数不更新"
                            )))
                            .ok();
                    }
                    Ok(())
                } else {
                    Err(error)
                }
            }
        }
    }

    fn query_uid_at(&mut self, node: u8) -> Result<Option<String>> {
        let sequence = self.next_sequence();
        let mut payload = [0u8; 12];
        payload[4..8].copy_from_slice(&u32::from(sequence).to_le_bytes());
        let frames = wire::encode(node, 1, sequence, &payload)?;
        self.port.send(frames[0].0, &frames[0].1)?;
        let deadline = Instant::now() + Duration::from_millis(250);
        while let Some(remaining) = deadline.checked_duration_since(Instant::now()) {
            let Some((id, data)) = self.port.receive(remaining)? else {
                break;
            };
            if let Ok(reply) = wire::decode(node, 1, sequence, id, &data) {
                ensure!(
                    reply.len() == 32 && reply[..12].iter().any(|b| *b != 0),
                    "目标节点回复异常，禁止切换地址"
                );
                return Ok(Some(hex::encode_upper(&reply[..12])));
            }
        }
        Ok(None)
    }
    fn fail_after_stop(&mut self, error: anyhow::Error) -> anyhow::Error {
        self.port.report(
            "error",
            "session_failure",
            serde_json::json!({"error":format!("{error:#}"),
            "state":self.snapshot.motor_state,"mode":self.snapshot.mode,
            "motion_may_run":self.snapshot.motion_may_run}),
        );
        if !self.snapshot.motion_may_run {
            return error;
        }
        let result = self.stop();
        self.port.report(if result.is_ok() { "warning" } else { "error" }, "failure_stop_result",
            serde_json::json!({"confirmed":result.is_ok(),"error":result.as_ref().err().map(|e|format!("{e:#}"))}));
        match result {
            Ok(()) => anyhow::anyhow!("{error:#}；已请求并回读 STOP，仍需现场核对输出"),
            Err(stop_error) => {
                anyhow::anyhow!("{error:#}；STOP 未获最终确认: {stop_error:#}；须物理干预")
            }
        }
    }

    fn next_sequence(&mut self) -> u16 {
        self.sequence = self.sequence.wrapping_add(1).max(1);
        self.sequence
    }

    fn query(&mut self, kind: u16, page: u16) -> Result<Vec<u8>> {
        let deadline = if self.snapshot.motion_may_run {
            self.snapshot.feedback_at.unwrap_or(self.started) + FEEDBACK_VALIDITY
        } else {
            Instant::now() + READ_BUDGET
        };
        self.query_until(kind, page, deadline)
    }

    fn query_until(&mut self, kind: u16, page: u16, deadline: Instant) -> Result<Vec<u8>> {
        let seq = self.next_sequence();
        let payload = if kind == 1 || kind == 2 {
            let mut data = [0u8; 12];
            data[4..8].copy_from_slice(&(seq as u32).to_le_bytes());
            data[8..10].copy_from_slice(&page.to_le_bytes());
            data.to_vec()
        } else if kind == motor_configuration::READ {
            vec![page as u8, 0, 0, 0]
        } else {
            vec![]
        };
        self.exchange_timed(
            kind,
            seq,
            &payload,
            deadline.saturating_duration_since(Instant::now()),
        )
    }

    fn exchange(&mut self, kind: u16, sequence: u16, payload: &[u8]) -> Result<Vec<u8>> {
        self.exchange_timed(kind, sequence, payload, Duration::from_millis(250))
    }

    fn exchange_timed(
        &mut self,
        kind: u16,
        sequence: u16,
        payload: &[u8],
        timeout: Duration,
    ) -> Result<Vec<u8>> {
        let frames = wire::encode(self.node, kind, sequence, payload)?;
        ensure!(frames.len() == 1, "常规命令意外分片");
        let deadline = Instant::now() + timeout;
        // 仅无副作用的读事务可重发；写、启用、目标和 STOP 每次仅发送一次。
        let read_only = matches!(kind, 1 | 2 | 108 | motor_configuration::READ | 161);
        let attempts = if read_only { 2 } else { 1 };
        for attempt in 0..attempts {
            if Instant::now() >= deadline {
                break;
            }
            self.port.send(frames[0].0, &frames[0].1)?;
            let attempt_deadline = if read_only {
                deadline.min(Instant::now() + Duration::from_millis(250))
            } else {
                deadline
            };
            let mut last_mismatch = None;
            let mut ignored = 0u32;
            while let Some(remaining) = attempt_deadline.checked_duration_since(Instant::now()) {
                let Some((id, data)) = self.port.receive(remaining)? else {
                    break;
                };
                match wire::decode(self.node, kind, sequence, id, &data) {
                    Ok(reply) => {
                        if ignored != 0 || attempt != 0 {
                            self.port.report("info", "request_recovered", serde_json::json!({"kind":kind,"sequence":sequence,
                                "attempt":attempt+1,"ignored":ignored,"last_mismatch":last_mismatch}));
                        }
                        return Ok(reply);
                    }
                    Err(error) => {
                        ignored = ignored.saturating_add(1);
                        last_mismatch = Some(format!("{error:#}"));
                    }
                }
            }
            self.port.report("warning", "request_timeout", serde_json::json!({"kind":kind,"sequence":sequence,
                "attempt":attempt+1,"ignored":ignored,"last_mismatch":last_mismatch,"retry_allowed":read_only}));
        }
        Err(SessionIssue::Timeout(kind).into())
    }

    fn read_identity(&mut self) -> Result<()> {
        let mut snapshot = self.snapshot.clone();
        let page0 = self.query(1, 0)?;
        let page1 = self.query(1, 1)?;
        let page2 = self.query(1, 2)?;
        let caps = self.query(2, 0)?;
        ensure!(
            page0.len() == 32 && page1.len() == 32 && page2.len() == 32 && caps.len() == 16,
            "维护页或能力页长度不匹配"
        );
        let uid = hex::encode_upper(&page0[..12]);
        ensure!(page0[..12].iter().any(|&value| value != 0), "全零 UID 非法");
        check_expected_uid(&self.expected_uid, &uid)?;
        if !self.bound_uid.is_empty() && uid != self.bound_uid {
            let _ = self.stop();
            bail!("会话 UID 变化；禁止继续操作，需现场核对 STOP 与输出");
        }
        if self.bound_uid.is_empty()
            && self.manual_start == 0
            && let Some(previous) = saved_control_sequence(&self.sequence_file, &uid)?
        {
            self.sequence = previous;
        }
        self.bound_uid = uid.clone();
        self.port.context["uid"] = serde_json::json!(uid);
        ensure!(
            page1[21] == self.node && [1, 2].contains(&page1[22]),
            "节点或角色异常"
        );
        ensure!(u16le(&caps, 6)? == 64, "设备帧能力异常");
        snapshot.uid = uid;
        snapshot.sn = if page1[20] == 1 {
            let end = page1[..16]
                .iter()
                .position(|&value| value == 0)
                .unwrap_or(16);
            let value = &page1[..end];
            ensure!(
                !value.is_empty()
                    && value
                        .iter()
                        .all(|&c| c.is_ascii_alphanumeric() || c == b'-')
                    && page1[end..16].iter().all(|&c| c == 0),
                "设备 SN 格式非法"
            );
            String::from_utf8(value.to_vec())?
        } else {
            String::new()
        };
        snapshot.motor_type = u32le(&page1, 16)?;
        snapshot.product = u32le(&page0, 12)?;
        snapshot.hardware = u16le(&page0, 16)?;
        snapshot.app_version = u32le(&page0, 20)?;
        snapshot.loader_version = u32le(&page0, 24)?;
        snapshot.role = page1[22];
        snapshot.abi = u32le(&page2, 8)?;
        snapshot.schema = u32le(&page2, 12)?;
        snapshot.features = u32le(&caps, 0)?;
        self.lease_supported = snapshot.features & (1 << 13) != 0
            && u16le(&caps, 12)? == 500
            && u16le(&caps, 14)? == 500;
        snapshot.motion_lease_error = if self.lease_supported {
            let path = renewal_path(&self.sequence_file, &self.bound_uid)?;
            saved_renewal_counter(&path)
                .err()
                .map(|error| format!("{error:#}"))
        } else {
            None
        };
        if self.snapshot.uid.is_empty() {
            self.port.report(
                "info",
                "motion_state_store",
                serde_json::json!({
                    "sequence_file":self.sequence_file,
                    "renewal_file":renewal_path(&self.sequence_file, &self.bound_uid)?,
                    "lease_supported":self.lease_supported,
                    "lease_error":snapshot.motion_lease_error,
                }),
            );
        }
        snapshot.modes = u16le(&caps, 4)?;
        snapshot.active_configuration = None;
        snapshot.saved_configuration = None;
        if snapshot.role == 1 && snapshot.features & motor_configuration::FEATURE != 0 {
            snapshot.active_configuration = Some(ConfigurationInfo::parse(&self.query(motor_configuration::READ, 0)?)?);
            if snapshot
                .active_configuration
                .as_ref()
                .is_some_and(|info| info.flags & 8 != 0)
            {
                snapshot.saved_configuration =
                    Some(ConfigurationInfo::parse(&self.query(motor_configuration::READ, 1)?)?);
            }
        }
        snapshot.identity_at = Some(Instant::now());
        self.snapshot = snapshot;
        self.send
            .send(Event::Snapshot(Box::new(self.snapshot.clone())))
            .ok();
        Ok(())
    }

    fn read_feedback(&mut self, allow_auto_stop: bool) -> Result<()> {
        let previous_faults = self.snapshot.faults;
        let deadline = if allow_auto_stop && self.snapshot.motion_may_run {
            self.snapshot.feedback_at.unwrap_or(self.started) + FEEDBACK_VALIDITY
        } else {
            Instant::now() + READ_BUDGET
        };
        for attempt in 0..3 {
            let data = self.query_until(108, 0, deadline)?;
            match apply_feedback(&mut self.snapshot, &data, self.node) {
                Ok(()) => {
                    if attempt != 0 {
                        self.port.report(
                            "info",
                            "feedback_busy_recovered",
                            serde_json::json!({"attempt":attempt+1}),
                        );
                    }
                    break;
                }
                Err(error) if matches!(error.downcast_ref(), Some(SessionIssue::Busy)) => {
                    if attempt == 0 {
                        self.port.report(
                            "warning",
                            "feedback_busy",
                            serde_json::json!({"payload":hex::encode(&data)}),
                        );
                    }
                    if attempt == 2 || Instant::now() >= deadline {
                        return Err(error);
                    }
                    std::thread::sleep(Duration::from_millis(2));
                }
                Err(error) => {
                    self.port.report("error", "feedback_invalid", serde_json::json!({"error":format!("{error:#}"),"payload":hex::encode(&data)}));
                    return Err(error);
                }
            }
        }
        if previous_faults != self.snapshot.faults {
            self.port.report(
                "warning",
                "device_fault_changed",
                serde_json::json!({"previous":previous_faults,"current":self.snapshot.faults}),
            );
        }
        let starting = if let Some((mode, at)) = self.pending_start {
            if self.snapshot.motor_state == 6 {
                ensure!(
                    self.snapshot.mode == mode.wire(),
                    "启动回读的运行模式与请求不符"
                );
                self.pending_start = None;
                false
            } else {
                ensure!(
                    at.elapsed() < FEEDBACK_VALIDITY,
                    "启动未在500ms内进入RUNNING，停止当前会话"
                );
                self.snapshot.motor_state == 5
                    && self.snapshot.mode == mode.wire()
                    && self.snapshot.bus_limits_ok()
                    && self
                        .snapshot
                        .iq_a
                        .is_none_or(|iq| iq.is_finite() && iq.abs() <= 6.0)
            }
        } else {
            false
        };
        if allow_auto_stop
            && self.snapshot.motion_may_run
            && ((!starting && !self.snapshot.running_limits_ok())
                || self
                    .snapshot
                    .speed_rad_s
                    .is_none_or(|speed| speed.abs() > MAX_SPEED_RAD_S))
        {
            self.send
                .send(Event::Status(
                    "测量失效或越过台架限值，正在请求 STOP".into(),
                ))
                .ok();
            self.stop()?;
        }
        self.send
            .send(Event::Snapshot(Box::new(self.snapshot.clone())))
            .ok();
        Ok(())
    }

    fn command(&mut self, kind: u16, payload: &[u8]) -> Result<(u16, u16)> {
        let sequence = self.next_sequence();
        let reserved = reserve_control_sequence(
            &self.sequence_file,
            &self.bound_uid,
            self.manual_start,
            sequence,
            kind == 110,
        );
        if kind == 110 {
            if let Err(error) = reserved {
                self.send
                    .send(Event::Status(format!(
                        "控制序号保存失败，仍尝试 STOP: {error:#}"
                    )))
                    .ok();
            }
        } else {
            reserved?;
        }
        self.manual_start = 0;
        self.port.report(
            "info",
            "command_requested",
            serde_json::json!({"kind":kind,"sequence":sequence,"payload":hex::encode(payload)}),
        );
        let response = self.exchange(kind, sequence, payload)?;
        ensure!(response.len() == 4, "控制回复长度非法");
        let result = u16le(&response, 0)?;
        let detail = u16le(&response, 2)?;
        ensure!(
            matches!(result, 0..=8 | 11 | 17),
            "控制回复结果码非法：{result}"
        );
        self.port.log.event(
            "info",
            "command_reply",
            serde_json::json!({"device":self.port.context,"kind":kind,
            "sequence":sequence,"result":result,"detail":detail}),
        );
        Ok((result, detail))
    }

    fn stop(&mut self) -> Result<()> {
        self.snapshot.host_position_session = false;
        self.pending_start = None;
        self.last_renewal = None;
        let deadline = Instant::now() + Duration::from_millis(1500);
        loop {
            let (result, detail) = self.command(110, &[])?;
            ensure!(
                detail <= 1 && result <= 1,
                "STOP 被拒绝，须现场确认输出关闭"
            );
            if result == 0 && detail == 0 {
                self.read_feedback(false)?;
                ensure!(
                    self.snapshot.mode == 0
                        && (self.snapshot.motor_state == 5
                            || (self.snapshot.motor_state == 8 && self.snapshot.faults != 0)),
                    "STOP 后未确认停机状态"
                );
                self.snapshot.motion_may_run = false;
                self.snapshot.stop_confirmed = true;
                self.send
                    .send(Event::Snapshot(Box::new(self.snapshot.clone())))
                    .ok();
                self.send
                    .send(Event::Status(if self.snapshot.faults == 0 {
                        "STOP 0/0、IDLE 已回读；仍需现场核对物理输出".into()
                    } else {
                        "STOP 0/0及故障停机已回读；故障仍保留，可检查后清除".into()
                    }))
                    .ok();
                return Ok(());
            }
            ensure!(Instant::now() < deadline, "STOP 最终 0/0 超时；须物理干预");
            std::thread::sleep(Duration::from_millis(50));
        }
    }

    fn handle(&mut self, command: Command) -> Result<bool> {
        match command {
            Command::ClearFault => self.clear_fault(),
            Command::StartCalibration(kind) => self.start_calibration(kind),
            Command::CancelCalibration => self.cancel_calibration(),
            Command::ReadConfiguration => {
                self.read_identity()?;
                Ok(true)
            }
            Command::SaveConfiguration {
                payload,
                acknowledged,
            } => {
                self.high_rate = false;
                self.read_identity()?;
                self.read_feedback(false)?;
                ensure!(
                    acknowledged
                        && self.snapshot.stop_confirmed
                        && self.snapshot.safe_idle(&self.target, &self.bound_uid)
                        && self.snapshot.features & motor_configuration::FEATURE != 0,
                    SessionIssue::Rejected("型号保存门禁未满足".into())
                );
                let info = self
                    .snapshot
                    .active_configuration
                    .as_ref()
                    .context("缺少当前型号配置")?;
                ensure!(
                    u32le(&payload, 4)? == info.revision,
                    SessionIssue::Rejected("型号配置已改变，请重新读取并核对".into())
                );
                if payload[2] != self.node {
                    ensure!(
                        self.query_uid_at(payload[2])?.is_none(),
                        SessionIssue::Rejected(
                            "目标节点已有设备，禁止覆盖地址；请先处理节点冲突".into()
                        )
                    );
                }
                let sequence = self.next_sequence();
                reserve_control_sequence(
                    &self.sequence_file,
                    &self.bound_uid,
                    self.manual_start,
                    sequence,
                    false,
                )?;
                self.manual_start = 0;
                let prepared = self.exchange(motor_configuration::PREPARE, sequence, &payload)?;
                ensure!(
                    prepared.len() == 12
                        && u16le(&prepared, 0)? == 0
                        && u16le(&prepared, 2)? == 0
                        && prepared[8] == 1
                        && prepared[9] == payload[1]
                        && prepared[10] == payload[2]
                        && prepared[11] & 40 != 0,
                    "型号候选准备未确认，未提交 Flash"
                );
                let mut commit = hex::decode(&self.bound_uid)?;
                ensure!(commit.len() == 12, "绑定 UID 长度非法");
                commit.extend_from_slice(&prepared[4..8]);
                let sequence = self.next_sequence();
                reserve_control_sequence(
                    &self.sequence_file,
                    &self.bound_uid,
                    self.manual_start,
                    sequence,
                    false,
                )?;
                self.manual_start = 0;
                let response =
                    self.exchange_timed(motor_configuration::COMMIT, sequence, &commit, Duration::from_secs(5))?;
                ensure!(
                    response.len() == 12
                        && u16le(&response, 0)? == 0
                        && u16le(&response, 2)? == 0
                        && response[8] == 1
                        && response[9] == payload[1]
                        && response[10] == payload[2]
                        && response[11] & 8 != 0,
                    "型号保存未确认，禁止按已生效继续操作"
                );
                let revision = u32le(&response, 4)?;
                self.read_identity()?;
                ensure!(
                    self.snapshot
                        .saved_configuration
                        .as_ref()
                        .is_some_and(
                            |info| info.revision == revision && info.matches_request(&payload)
                        ),
                    "保存配置回读不一致"
                );
                self.send.send(Event::ConfigurationSaved).ok();
                self.send
                    .send(Event::Status(
                        "新型号与配套参数已保存，尚未生效；旧标定已清除，需应用并重启".into(),
                    ))
                    .ok();
                Ok(true)
            }
            Command::ActivateConfiguration => {
                self.high_rate = false;
                self.read_identity()?;
                self.read_feedback(false)?;
                ensure!(
                    self.snapshot.compatible(&self.target, &self.bound_uid)
                        && self.snapshot.bus_limits_ok()
                        && self.snapshot.motor_state == 5
                        && self.snapshot.mode == 0
                        && !self.snapshot.motion_may_run,
                    SessionIssue::Rejected("型号激活门禁未满足".into())
                );
                let saved = self
                    .snapshot
                    .saved_configuration
                    .clone()
                    .ok_or_else(|| SessionIssue::Rejected("没有已保存的待重启配置".into()))?;
                let mut activate = hex::decode(&self.bound_uid)?;
                ensure!(activate.len() == 12, "绑定 UID 长度非法");
                activate.extend_from_slice(&saved.revision.to_le_bytes());
                let (result, detail) = self.command(motor_configuration::ACTIVATE, &activate)?;
                ensure!(result == 1 && detail == 0, "型号激活请求被拒绝");
                self.snapshot.stop_confirmed = false;
                self.node = saved.node;
                self.port.context["node"] = serde_json::json!(saved.node);
                std::thread::sleep(Duration::from_millis(200));
                let deadline = Instant::now() + Duration::from_secs(5);
                loop {
                    if let Some(uid) = self.query_uid_at(saved.node)? {
                        ensure!(
                            uid == self.bound_uid,
                            "新节点回复的 UID 不符；禁止控制，请检查地址冲突"
                        );
                        if self.read_identity().is_err()
                            || self.snapshot.role != 1
                            || self
                                .snapshot
                                .active_configuration
                                .as_ref()
                                .is_none_or(ConfigurationInfo::blocked)
                        {
                            ensure!(
                                Instant::now() < deadline,
                                "重启后尚未确认 APP 配置；请重新扫描核对"
                            );
                            std::thread::sleep(Duration::from_millis(100));
                            continue;
                        }
                        let applied = self.snapshot.active_configuration.as_ref().unwrap();
                        ensure!(
                            applied.revision == saved.revision
                                && applied.model == saved.model
                                && applied.node == saved.node
                                && !applied.blocked()
                                && applied.flags & 6 == 0,
                            "重启后型号、修订或标定状态不符合预期"
                        );
                        self.read_feedback(false)?;
                        self.send
                            .send(Event::ConfigurationActivated(saved.node))
                            .ok();
                        self.send
                            .send(Event::Status(
                                "新型号和配套参数已生效；需要重新标定，未自动使能".into(),
                            ))
                            .ok();
                        break;
                    }
                    ensure!(
                        Instant::now() < deadline,
                        "重启后未识别原 UID 与新节点；状态未知，请重新扫描核对"
                    );
                    std::thread::sleep(Duration::from_millis(100));
                }
                Ok(true)
            }
            Command::Observe(enabled) => {
                self.high_rate = enabled;
                Ok(true)
            }
            Command::Disconnect => {
                if self.snapshot.motion_may_run {
                    self.stop()?;
                }
                Ok(false)
            }
            Command::Stop => {
                self.stop()?;
                Ok(true)
            }
            Command::Start { mode } => {
                ensure!(
                    self.snapshot
                        .can_start_mode(&self.target, &self.bound_uid, mode),
                    SessionIssue::Rejected("启动门禁未满足".into())
                );
                if self.lease_supported
                    && let Err(error) = self.reserve_motion_lease()
                {
                    let message = format!("使能前无法准备续租：{error:#}");
                    self.snapshot.motion_lease_error = Some(message.clone());
                    self.send
                        .send(Event::Snapshot(Box::new(self.snapshot.clone())))
                        .ok();
                    return Err(SessionIssue::Rejected(message).into());
                }
                self.snapshot.motion_may_run = true;
                let (result, detail) = self.command(117, &mode.start_payload())?;
                if result > 1 {
                    self.snapshot.motion_may_run = false;
                    return Err(SessionIssue::Rejected(format!(
                        "{}启动被拒：{result}/{detail}",
                        mode.name()
                    ))
                    .into());
                }
                ensure!(detail == 0, "启动回复 detail 非法：{detail}");
                self.last_renewal = self.lease_supported.then(Instant::now);
                self.pending_start = Some((mode, Instant::now()));
                self.snapshot.host_position_session = mode == ControlMode::Position;
                self.snapshot.stop_confirmed = false;
                self.send
                    .send(Event::Status(format!(
                        "{}启动已受理；以随后状态回读为准",
                        mode.name()
                    )))
                    .ok();
                Ok(true)
            }
            Command::SetTarget { mode, value } => {
                ensure!(
                    self.snapshot
                        .can_set_target(&self.target, &self.bound_uid, mode),
                    SessionIssue::Rejected(format!("{}目标门禁未满足", mode.name()))
                );
                let payload = mode
                    .target_payload(value, self.snapshot.position_rad)
                    .map_err(|error| SessionIssue::Rejected(error.to_string()))?;
                let (result, detail) = self.command(116, &payload)?;
                ensure!(
                    result <= 1,
                    SessionIssue::Rejected(format!("{}目标被拒：{result}/{detail}", mode.name()))
                );
                ensure!(detail == 0, "目标回复 detail 非法：{detail}");
                self.send
                    .send(Event::Status(format!(
                        "{} {value:.3} {} 目标已受理；以随后状态回读为准",
                        mode.name(),
                        mode.unit()
                    )))
                    .ok();
                Ok(true)
            }
            Command::Upgrade {
                directory,
                acknowledged,
            } => {
                self.high_rate = false;
                ensure!(
                    acknowledged
                        && self.snapshot.safe_idle(&self.target, &self.bound_uid)
                        && self.snapshot.loader_version == self.target.loader_version,
                    SessionIssue::Rejected("升级门禁未满足：身份、Loader 或停机状态异常".into())
                );
                let package = UpgradePackage::load(&directory, &self.target)
                    .map_err(|error| SessionIssue::Rejected(format!("升级包无效：{error:#}")))?;
                self.stop()?;
                ensure!(
                    self.snapshot.stop_confirmed
                        && self.snapshot.safe_idle(&self.target, &self.bound_uid),
                    "STOP 最终状态未确认，禁止 BEGIN"
                );
                let uid: [u8; 12] = hex::decode(&self.bound_uid)?
                    .try_into()
                    .map_err(|_| anyhow::anyhow!("UID长度非法"))?;
                let root = std::env::current_dir()?.join("outputs");
                fs::create_dir_all(&root)?;
                let stamp = std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)?
                    .as_millis();
                let evidence = root.join(format!("host-rust-upgrade-{stamp}"));
                fs::create_dir(&evidence)?;
                let write_result = |success: bool, failure: Option<&str>| -> Result<()> {
                    let report = serde_json::json!({
                        "success": success,
                        "protected_regions_verified": false,
                        "failure": failure,
                        "uid": hex::encode(uid),
                        "version": package.version,
                        "crc32": format!("{:08x}", package.crc32),
                        "image_sha256": hex::encode(sha2::Sha256::digest(&package.image)),
                        "layout": self.target.layout,
                    });
                    fs::write(
                        evidence.join("summary.json"),
                        serde_json::to_vec_pretty(&report)?,
                    )?;
                    Ok(())
                };
                write_result(false, Some("upgrade in progress"))?;
                let send = self.send.clone();
                let mut client = UpgradeClient::new(&mut self.port, self.node, uid);
                let result = client.run(&package, move |written, total| {
                    send.send(Event::Progress(written as f32 / total as f32))
                        .ok();
                });
                write_result(
                    result.is_ok(),
                    result
                        .as_ref()
                        .err()
                        .map(|error| error.to_string())
                        .as_deref(),
                )?;
                result?;
                self.snapshot = Snapshot::default();
                self.read_identity()?;
                self.read_feedback(false)?;
                self.send.send(Event::UpgradeDone).ok();
                self.send
                    .send(Event::Status(
                        "APP 升级、激活及持久确认完成；请核对物理输出".into(),
                    ))
                    .ok();
                Ok(true)
            }
        }
    }
}

pub struct ConnectionIdentity {
    pub target: Target,
    pub expected_uid: String,
    pub bitrates: Bitrates,
    pub log: Logger,
}

/// 显式连接后启动独占设备线程；任何硬故障立即关闭端口，不自动恢复试验。
pub fn run(
    dll: &Path,
    device_index: u32,
    node: u8,
    manual_start: u16,
    identity: ConnectionIdentity,
    commands: Receiver<Command>,
    events: Sender<Event>,
) {
    let result = (|| -> Result<()> {
        ensure!(node > 0 && node < 255, "节点必须为 1..254");
        let port = ControlCanFd::open(dll, device_index, identity.bitrates)?;
        events
            .send(Event::Status("CAN FD 已连接，正在读取真实设备身份".into()))
            .ok();
        let mut session = Session {
            port: TraceTransport::new(
                port,
                identity.log.clone(),
                serde_json::json!({"adapter_index":device_index,
                "node":node,"bitrates":identity.bitrates.label(),"uid":identity.expected_uid}),
            ),
            sequence_file: sequence_path()?,
            node,
            target: identity.target,
            sequence: manual_start,
            manual_start,
            bound_uid: String::new(),
            expected_uid: identity.expected_uid,
            high_rate: false,
            snapshot: Snapshot::default(),
            send: events.clone(),
            started: Instant::now(),
            feedback_degraded: false,
            lease_supported: false,
            last_renewal: None,
            renewal_block: None,
            pending_start: None,
        };
        let mut discovery = Instant::now() - Duration::from_secs(4);
        let mut next_poll = Instant::now();
        let mut previous_interval = Duration::from_millis(200);
        loop {
            let cycle_started = Instant::now();
            loop {
                match commands.try_recv() {
                    Ok(command) => match session.dispatch(command) {
                        Ok(true) => {}
                        Ok(false) => return Ok(()),
                        Err(error) => return Err(session.fail_after_stop(error)),
                    },
                    Err(TryRecvError::Empty) => break,
                    Err(TryRecvError::Disconnected) => {
                        if session.snapshot.motion_may_run {
                            session.stop()?;
                        }
                        return Ok(());
                    }
                }
            }
            if session.bound_uid.is_empty() {
                if let Err(error) = session.read_identity() {
                    return Err(session.fail_after_stop(error));
                }
                discovery = Instant::now();
            }
            if let Err(error) = session.poll_feedback() {
                return Err(session.fail_after_stop(error));
            }
            if let Err(error) = session.poll_calibration() {
                return Err(session.fail_after_stop(error));
            }
            if discovery.elapsed() >= Duration::from_secs(3) {
                if let Err(error) = session.read_identity() {
                    if !matches!(error.downcast_ref(), Some(SessionIssue::Timeout(_)))
                        || !session.snapshot.identity_fresh()
                        || (session.snapshot.motion_may_run && !session.snapshot.feedback_fresh())
                    {
                        return Err(session.fail_after_stop(error));
                    }
                    session.port.report(
                        "warning",
                        "identity_refresh_deferred",
                        serde_json::json!({"error":format!("{error:#}")}),
                    );
                } else {
                    discovery = Instant::now();
                }
            }
            let interval = if session.high_rate {
                OBSERVE_INTERVAL
            } else {
                Duration::from_millis(200)
            };
            if interval != previous_interval {
                next_poll = cycle_started;
                previous_interval = interval;
            }
            next_poll = advance_deadline(next_poll, interval, Instant::now());
            if let Some(remaining) = next_poll.checked_duration_since(Instant::now()) {
                std::thread::sleep(remaining);
            }
        }
    })();
    if let Err(error) = result {
        identity.log.event("error", "connection_closed_after_failure", serde_json::json!({"error":format!("{error:#}"),"node":node,"adapter_index":device_index}));
        events
            .send(Event::HardStop(format!("{error:#}；设备状态需现场确认")))
            .ok();
    }
    events.send(Event::Disconnected).ok();
    identity.log.event(
        "info",
        "connection_closed",
        serde_json::json!({"node":node,"adapter_index":device_index}),
    );
}

#[cfg(test)]
#[path = "device_recovery_tests.rs"]
mod recovery_tests;

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn saved_failed_or_uncalibrated_configuration_never_enables_motion() {
        let target = Target::bundled().unwrap();
        let mut view = Snapshot {
            uid: "A".into(),
            role: 1,
            app_version: 1,
            product: target.product,
            hardware: target.hardware as u16,
            schema: target.schema,
            abi: target.abi,
            features: 0x21d00,
            modes: 0x4e,
            motor_state: 5,
            mode: 0,
            bus_v: Some(24.0),
            bus_a: Some(0.0),
            iq_a: Some(0.0),
            speed_rad_s: Some(0.0),
            stop_confirmed: true,
            identity_at: Some(Instant::now()),
            feedback_at: Some(Instant::now()),
            ..Default::default()
        };
        for flags in [7, 1, 0, 15, 23] {
            view.active_configuration = Some(ConfigurationInfo {
                model: 1,
                node: 5,
                flags,
                revision: 1,
                speed_rad_s: 1.0,
                thermal: [0.0; 7],
            });
            assert_eq!(
                view.can_start_mode(&target, "A", ControlMode::Speed),
                flags == 7
            );
            assert_eq!(view.safe_idle(&target, "A"), flags & 24 == 0);
        }
    }
    #[test]
    fn enable_indicator_requires_live_running_or_confirmed_stop_feedback() {
        let mut view = Snapshot {
            motor_state: 5,
            mode: 0,
            feedback_at: Some(Instant::now()),
            ..Default::default()
        };
        assert_eq!(view.enable_state(), None);
        view.stop_confirmed = true;
        assert_eq!(view.enable_state(), Some(false));
        view.motion_may_run = true;
        assert_eq!(view.enable_state(), None);
        view.motor_state = 6;
        view.mode = 6;
        assert_eq!(view.enable_state(), Some(true));
        view.feedback_at = Some(Instant::now() - Duration::from_secs(1));
        assert_eq!(view.enable_state(), None);
    }
    #[test]
    fn periodic_deadlines_preserve_phase_and_skip_overruns() {
        let base = Instant::now();
        let interval = Duration::from_millis(10);
        assert_eq!(
            advance_deadline(base, interval, base + Duration::from_millis(1)),
            base + interval
        );
        assert_eq!(
            advance_deadline(base, interval, base + Duration::from_millis(25)),
            base + Duration::from_millis(35)
        );
    }

    #[test]
    fn scanned_uid_must_match_before_binding() {
        assert!(check_expected_uid("ABCD", "ABCD").is_ok());
        assert!(check_expected_uid("ABCD", "OTHER").is_err());
        assert!(check_expected_uid("", "MANUAL").is_ok());
    }

    pub(super) fn valid_feedback() -> Vec<u8> {
        let mut data = vec![0u8; 45];
        for offset in [8, 12, 16, 20] {
            data[offset..offset + 4].copy_from_slice(&i32::MIN.to_le_bytes());
        }
        for offset in [26, 28, 30, 32, 34, 36, 38, 40] {
            data[offset..offset + 2].copy_from_slice(&i16::MIN.to_le_bytes());
        }
        let bits: u16 =
            (1 << 1) | (1 << 2) | (1 << 3) | (1 << 4) | (1 << 5) | (1 << 6) | (1 << 7) | (1 << 8);
        data[24..26].copy_from_slice(&bits.to_le_bytes());
        for (offset, value) in [(12, 1250i32), (16, 4000), (20, -3000)] {
            data[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
        }
        for (offset, value) in [
            (26, 2400i16),
            (28, 200),
            (30, 1200),
            (32, -1000),
            (34, 3150),
        ] {
            data[offset..offset + 2].copy_from_slice(&value.to_le_bytes());
        }
        data[42] = 1;
        data[43] = 6;
        data[44] = 2;
        data
    }

    #[test]
    fn feedback_decodes_si_and_rejects_corrupt_validity_without_partial_update() {
        let mut view = Snapshot::default();
        let mut data = valid_feedback();
        apply_feedback(&mut view, &data, 1).unwrap();
        assert_eq!(view.position_rad, Some(1.25));
        assert_eq!(view.speed_rad_s, Some(-3.0));
        assert_eq!(view.speed_reference_rad_s, Some(4.0));
        assert_eq!(view.iq_reference_a, Some(1.2));
        assert_eq!(view.temperature_c, Some(31.5));
        data[24] ^= 1 << 1;
        assert!(apply_feedback(&mut view, &data, 1).is_err());
        assert_eq!(view.position_rad, Some(1.25));
        data[24] ^= 1 << 1;
        data[44] = 3;
        apply_feedback(&mut view, &data, 1).unwrap();
        assert_eq!(view.speed_reference_rad_s, Some(4.0));
        assert!(apply_feedback(&mut view, &data[..44], 1).is_err());
        assert!(apply_feedback(&mut view, &data, 2).is_err());
    }

    #[test]
    fn speed_si_payload_keeps_wire_format_and_physical_limit() {
        let payload = ControlMode::Speed.target_payload(12.5, None).unwrap();
        assert_eq!(payload[0], 2);
        assert_eq!(
            i32::from_le_bytes(payload[8..12].try_into().unwrap()),
            12500
        );
        assert!(payload[1..8].iter().chain(&payload[12..]).all(|b| *b == 0));
        for value in [f64::NAN, f64::INFINITY, -0.1, MAX_SPEED_RAD_S + 0.001] {
            assert!(ControlMode::Speed.target_payload(value, None).is_err());
        }
        let maximum = ControlMode::Speed
            .target_payload(MAX_SPEED_RAD_S, None)
            .unwrap();
        let encoded = i32::from_le_bytes(maximum[8..12].try_into().unwrap());
        assert!(encoded as f64 / 1000.0 <= MAX_SPEED_RAD_S);
    }

    #[test]
    fn live_compatible_idle_can_start_but_stale_or_running_cannot() {
        let target = Target::bundled().unwrap();
        let mut view = Snapshot {
            uid: "A".into(),
            role: 1,
            app_version: 1,
            product: target.product,
            hardware: target.hardware as u16,
            schema: target.schema,
            abi: target.abi,
            features: 0x1d00,
            modes: 4,
            motor_state: 5,
            mode: 0,
            bus_v: Some(24.0),
            bus_a: Some(0.0),
            iq_a: Some(0.0),
            speed_rad_s: Some(0.0),
            identity_at: Some(Instant::now()),
            feedback_at: Some(Instant::now()),
            ..Default::default()
        };
        assert!(view.safe_idle(&target, "A") && view.mode_supported(ControlMode::Speed));
        assert!(!view.safe_idle(&target, "B"));
        // 实机 PITCH IDLE Type108：有效位 0x003a，母线电流有效、Iq 无效。
        let idle = hex::decode("0000000000000000000000804d03000000000080010000003a00cd0bf1ff008000800080008000800080060500").unwrap();
        apply_feedback(&mut view, &idle, 6).unwrap();
        assert_eq!(view.iq_a, None);
        assert!(!view.safe_idle(&target, "A"));
        view.stop_confirmed = true;
        assert!(view.safe_idle(&target, "A"));
        view.bus_a = None;
        assert!(!view.safe_idle(&target, "A"));
        apply_feedback(&mut view, &idle, 6).unwrap();
        view.iq_a = Some(6.1);
        assert!(!view.safe_idle(&target, "A"));
        view.iq_a = None;
        view.motor_state = 6;
        view.mode = 2;
        view.motion_may_run = true;
        assert!(!view.safe_idle(&target, "A"));
        assert!(
            !view.can_set_target(&target, "A", ControlMode::Speed) && !view.running_limits_ok()
        );
        view.iq_a = Some(0.0);
        assert!(view.can_set_target(&target, "A", ControlMode::Speed));
        view.feedback_at = Some(Instant::now() - Duration::from_millis(600));
        assert!(!view.can_set_target(&target, "A", ControlMode::Speed));
        view.feedback_at = Some(Instant::now());
        view.bus_v = Some(34.0);
        assert!(!view.can_set_target(&target, "A", ControlMode::Speed));
    }
    #[test]
    fn stale_and_incompatible_never_enable_motion() {
        let target = Target::bundled().unwrap();
        let view = Snapshot::default();
        assert!(!view.safe_idle(&target, "123"));
        assert!(!view.can_set_target(&target, "123", ControlMode::Speed));
    }

    #[test]
    fn four_modes_require_declared_capability_stop_and_matching_active_mode() {
        let target = Target::bundled().unwrap();
        let mut view = Snapshot {
            uid: "A".into(),
            role: 1,
            app_version: 1,
            product: target.product,
            hardware: target.hardware as u16,
            schema: target.schema,
            abi: target.abi,
            features: 0x41d00,
            modes: 0x4e,
            motor_state: 5,
            mode: 0,
            bus_v: Some(24.0),
            bus_a: Some(0.0),
            iq_a: Some(0.0),
            vq_v: Some(0.0),
            speed_rad_s: Some(0.0),
            position_rad: Some(1.0),
            identity_at: Some(Instant::now()),
            feedback_at: Some(Instant::now()),
            ..Default::default()
        };
        for mode in ControlMode::ALL {
            assert!(!view.can_start_mode(&target, "A", mode));
            view.stop_confirmed = true;
            assert!(view.can_start_mode(&target, "A", mode));
            assert!(!view.can_start_mode(&target, "B", mode));
            view.modes ^= 1 << mode.wire();
            assert!(!view.can_start_mode(&target, "A", mode));
            view.modes ^= 1 << mode.wire();
            view.motor_state = 6;
            view.mode = mode.wire();
            view.host_position_session = mode == ControlMode::Position;
            view.motion_may_run = true;
            assert!(!view.can_start_mode(&target, "A", mode));
            for requested in ControlMode::ALL {
                assert_eq!(
                    view.can_set_target(&target, "A", requested),
                    requested == mode
                );
            }
            view.feedback_at = Some(Instant::now() - Duration::from_secs(1));
            assert!(!view.can_set_target(&target, "A", mode));
            view.feedback_at = Some(Instant::now());
            view.faults = 1;
            assert!(!view.can_set_target(&target, "A", mode));
            view.faults = 0;
            view.motor_state = 5;
            view.mode = 0;
            view.motion_may_run = false;
            view.stop_confirmed = false;
        }
        view.stop_confirmed = true;
        view.position_rad = None;
        assert!(!view.can_start_mode(&target, "A", ControlMode::Position));
        assert!(view.can_start_mode(&target, "A", ControlMode::Torque));
        view.motor_state = 6;
        view.mode = 6;
        view.motion_may_run = true;
        view.vq_v = None;
        assert!(!view.running_limits_ok());
        assert!(!view.can_set_target(&target, "A", ControlMode::Voltage));
    }

    #[test]
    fn position_reference_and_vq_use_validated_feedback_units_and_mode() {
        let mut view = Snapshot::default();
        let mut data = valid_feedback();
        let bits = u16le(&data, 24).unwrap() | 1 | (1 << 10);
        data[24..26].copy_from_slice(&bits.to_le_bytes());
        data[8..12].copy_from_slice(&1200i32.to_le_bytes());
        data[38..40].copy_from_slice(&(-1250i16).to_le_bytes());
        data[44] = 3;
        apply_feedback(&mut view, &data, 1).unwrap();
        assert_eq!(view.position_reference_rad, Some(1.2));
        assert_eq!(view.speed_reference_rad_s, Some(4.0));
        assert_eq!(view.vq_v, Some(-1.25));
        data[44] = 6;
        apply_feedback(&mut view, &data, 1).unwrap();
        assert_eq!(view.position_reference_rad, None);
        assert_eq!(view.speed_reference_rad_s, None);
        data[25] ^= 1 << 2;
        assert!(apply_feedback(&mut view, &data, 1).is_err());
        assert_eq!(view.vq_v, Some(-1.25));
    }
}
