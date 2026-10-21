//! 单线程独占 CAN 会话：查询与命令共用序号、UID 绑定及停机门禁。

use crate::can::{ControlCanFd, Transport};
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

/// 原台架 300rpm 上限的 SI 表示，不改变物理包络。
pub const MAX_SPEED_RAD_S: f64 = 10.0 * std::f64::consts::PI;
const IDLE_SPEED_RAD_S: f64 = std::f64::consts::PI / 6.0;

fn speed_target_payload(speed_rad_s: f64) -> Result<[u8; 20]> {
    ensure!(
        speed_rad_s.is_finite() && (0.0..=MAX_SPEED_RAD_S).contains(&speed_rad_s),
        "速度目标必须为 0..31.415927 rad/s"
    );
    let mut payload = [0u8; 20];
    payload[0] = 2;
    let encoded = (speed_rad_s * 1000.0)
        .round()
        .min((MAX_SPEED_RAD_S * 1000.0).floor()) as i32;
    payload[8..12].copy_from_slice(&encoded.to_le_bytes());
    Ok(payload)
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
    pub speed_rad_s: Option<f64>,
    pub speed_reference_rad_s: Option<f64>,
    pub iq_a: Option<f64>,
    pub iq_reference_a: Option<f64>,
    pub temperature_c: Option<f64>,
    pub bus_v: Option<f64>,
    pub bus_a: Option<f64>,
    pub identity_at: Option<Instant>,
    pub feedback_at: Option<Instant>,
    pub stop_confirmed: bool,
    pub motion_may_run: bool,
}

impl Snapshot {
    pub fn identity_fresh(&self) -> bool {
        self.identity_at
            .is_some_and(|at| at.elapsed() <= Duration::from_secs(5))
    }
    pub fn feedback_fresh(&self) -> bool {
        self.feedback_at
            .is_some_and(|at| at.elapsed() <= Duration::from_millis(500))
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
    pub fn speed_supported(&self) -> bool {
        let mask = (1 << 8) | (1 << 10) | (1 << 11) | (1 << 12);
        self.identity_fresh() && self.features & mask == mask && self.modes & (1 << 2) != 0
    }
    fn limits_ok(&self) -> bool {
        self.feedback_fresh()
            && self.faults == 0
            && self.bus_v.is_some_and(|v| v > 0.0 && v <= 33.0)
            && self.bus_a.is_some_and(|a| a.abs() <= 3.0)
            && self.iq_a.is_some_and(|a| a.abs() <= 6.0)
    }
    pub fn safe_idle(&self, target: &Target, uid: &str) -> bool {
        self.compatible(target, uid)
            && self.limits_ok()
            && self.motor_state == 5
            && self.mode == 0
            && self
                .speed_rad_s
                .is_some_and(|speed| speed.abs() < IDLE_SPEED_RAD_S)
            && !self.motion_may_run
    }
    pub fn can_set_speed(&self, target: &Target, uid: &str) -> bool {
        self.compatible(target, uid)
            && self.speed_supported()
            && self.limits_ok()
            && self.motor_state == 6
            && self.mode == 2
            && self
                .speed_rad_s
                .is_some_and(|speed| speed.abs() <= MAX_SPEED_RAD_S)
            && self.motion_may_run
    }
}

// 完整校验后再更新快照；独立于 SDK，离线测试覆盖单位和有效位。
fn apply_feedback(snapshot: &mut Snapshot, data: &[u8], node: u8) -> Result<()> {
    ensure!(
        data.len() == 45 && u16le(data, 0)? == 0 && u16le(data, 2)? == 0 && data[42] == node,
        "状态帧格式错误"
    );
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
    // 位置目标在部分模式是最终目标，暂不冒充轨迹瞬时参考；速度参考只在 SPEED 模式解释。
    snapshot.speed_reference_rad_s = if data[44] == 2 {
        signed32(16, 2)?.map(|value| value / 1000.0)
    } else {
        None
    };
    snapshot.iq_a = signed16(32, 7)?.map(|value| value / 1000.0);
    snapshot.iq_reference_a = signed16(30, 6)?.map(|value| value / 1000.0);
    snapshot.temperature_c = signed16(34, 8)?.map(|value| value / 100.0);
    snapshot.bus_v = signed16(26, 4)?.map(|value| value / 100.0);
    snapshot.bus_a = signed16(28, 5)?.map(|value| value / 1000.0);
    snapshot.motor_state = data[43];
    snapshot.mode = data[44];
    snapshot.feedback_at = Some(Instant::now());
    if data[43] == 6 {
        snapshot.motion_may_run = true;
        snapshot.stop_confirmed = false;
    }
    Ok(())
}

pub enum Command {
    Start {
        acknowledged: bool,
    },
    SetSpeed {
        speed_rad_s: f64,
        acknowledged: bool,
    },
    Stop,
    Upgrade {
        directory: PathBuf,
        acknowledged: bool,
    },
    Disconnect,
}

pub enum Event {
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

fn saved_control_sequence(uid: &str) -> Result<Option<u16>> {
    let path = sequence_path()?;
    if !path.exists() {
        return Ok(None);
    }
    let store: SequenceStore = serde_json::from_slice(&fs::read(path)?)?;
    Ok(store.0.get(uid).copied())
}

fn reserve_control_sequence(uid: &str, manual_start: u16, next: u16, stop: bool) -> Result<()> {
    ensure!(!uid.is_empty(), "设备 UID 未绑定，禁止运动命令");
    let path = sequence_path()?;
    let mut store: SequenceStore = if path.exists() {
        serde_json::from_slice(&fs::read(&path)?)?
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
    fs::create_dir_all(path.parent().context("序号目录错误")?)?;
    let temporary = path.with_extension("tmp");
    let file = fs::File::create(&temporary)?;
    serde_json::to_writer_pretty(&file, &store)?;
    file.sync_all()?;
    fs::rename(&temporary, &path).context("控制序号持久化失败")?;
    Ok(())
}

struct Session {
    port: ControlCanFd,
    node: u8,
    target: Target,
    sequence: u16,
    manual_start: u16,
    bound_uid: String,
    snapshot: Snapshot,
    send: Sender<Event>,
}

impl Session {
    fn fail_after_stop(&mut self, error: anyhow::Error) -> anyhow::Error {
        if !self.snapshot.motion_may_run {
            return error;
        }
        match self.stop() {
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
        let seq = self.next_sequence();
        let payload = if kind == 1 || kind == 2 {
            let mut data = [0u8; 12];
            data[4..8].copy_from_slice(&(seq as u32).to_le_bytes());
            data[8..10].copy_from_slice(&page.to_le_bytes());
            data.to_vec()
        } else {
            vec![]
        };
        self.exchange(kind, seq, &payload)
    }

    fn exchange(&mut self, kind: u16, sequence: u16, payload: &[u8]) -> Result<Vec<u8>> {
        let frames = wire::encode(self.node, kind, sequence, payload)?;
        ensure!(frames.len() == 1, "常规命令意外分片");
        self.port.send(frames[0].0, &frames[0].1)?;
        let deadline = Instant::now() + Duration::from_millis(250);
        while let Some(remaining) = deadline.checked_duration_since(Instant::now()) {
            let Some((id, data)) = self.port.receive(remaining)? else {
                break;
            };
            if let Ok(reply) = wire::decode(self.node, kind, sequence, id, &data) {
                return Ok(reply);
            }
        }
        bail!("Type{kind} 回复超时；设备状态未知")
    }

    fn read_identity(&mut self) -> Result<()> {
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
        if !self.bound_uid.is_empty() && uid != self.bound_uid {
            let _ = self.stop();
            bail!("会话 UID 变化；禁止继续操作，需现场核对 STOP 与输出");
        }
        if self.bound_uid.is_empty()
            && self.manual_start == 0
            && let Some(previous) = saved_control_sequence(&uid)?
        {
            self.sequence = previous;
        }
        self.bound_uid = uid.clone();
        ensure!(
            page1[21] == self.node && [1, 2].contains(&page1[22]),
            "节点或角色异常"
        );
        ensure!(u16le(&caps, 6)? == 64, "设备帧能力异常");
        self.snapshot.uid = uid;
        self.snapshot.sn = if page1[20] == 1 {
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
        self.snapshot.motor_type = u32le(&page1, 16)?;
        self.snapshot.product = u32le(&page0, 12)?;
        self.snapshot.hardware = u16le(&page0, 16)?;
        self.snapshot.app_version = u32le(&page0, 20)?;
        self.snapshot.loader_version = u32le(&page0, 24)?;
        self.snapshot.role = page1[22];
        self.snapshot.abi = u32le(&page2, 8)?;
        self.snapshot.schema = u32le(&page2, 12)?;
        self.snapshot.features = u32le(&caps, 0)?;
        self.snapshot.modes = u16le(&caps, 4)?;
        self.snapshot.identity_at = Some(Instant::now());
        self.send
            .send(Event::Snapshot(Box::new(self.snapshot.clone())))
            .ok();
        Ok(())
    }

    fn read_feedback(&mut self, allow_auto_stop: bool) -> Result<()> {
        let data = self.query(108, 0)?;
        apply_feedback(&mut self.snapshot, &data, self.node)?;
        if allow_auto_stop
            && self.snapshot.motion_may_run
            && (!self.snapshot.limits_ok()
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
        let reserved =
            reserve_control_sequence(&self.bound_uid, self.manual_start, sequence, kind == 110);
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
        let response = self.exchange(kind, sequence, payload)?;
        ensure!(response.len() == 4, "控制回复长度非法");
        Ok((u16le(&response, 0)?, u16le(&response, 2)?))
    }

    fn stop(&mut self) -> Result<()> {
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
                    self.snapshot.motor_state == 5 && self.snapshot.mode == 0,
                    "STOP 后未确认 IDLE"
                );
                self.snapshot.motion_may_run = false;
                self.snapshot.stop_confirmed = true;
                self.send
                    .send(Event::Snapshot(Box::new(self.snapshot.clone())))
                    .ok();
                self.send
                    .send(Event::Status(
                        "STOP 0/0、IDLE 已回读；仍需现场核对物理输出".into(),
                    ))
                    .ok();
                return Ok(());
            }
            ensure!(Instant::now() < deadline, "STOP 最终 0/0 超时；须物理干预");
            std::thread::sleep(Duration::from_millis(50));
        }
    }

    fn handle(&mut self, command: Command) -> Result<bool> {
        match command {
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
            Command::Start { acknowledged } => {
                ensure!(
                    acknowledged
                        && self.snapshot.safe_idle(&self.target, &self.bound_uid)
                        && self.snapshot.speed_supported(),
                    "启动门禁未满足"
                );
                self.snapshot.motion_may_run = true;
                let (result, detail) = self.command(117, &[2, 0, 0, 0])?;
                ensure!(result <= 1 && detail == 0, "零速启动被拒");
                self.snapshot.stop_confirmed = false;
                self.send
                    .send(Event::Status("零速启动已受理；以随后状态回读为准".into()))
                    .ok();
                Ok(true)
            }
            Command::SetSpeed {
                speed_rad_s,
                acknowledged,
            } => {
                ensure!(
                    acknowledged && self.snapshot.can_set_speed(&self.target, &self.bound_uid),
                    "速度命令门禁未满足"
                );
                let payload = speed_target_payload(speed_rad_s)?;
                let (result, detail) = self.command(116, &payload)?;
                ensure!(result <= 1 && detail == 0, "速度设置被拒");
                self.send
                    .send(Event::Status(format!(
                        "{speed_rad_s:.3} rad/s 目标已受理；以随后状态回读为准"
                    )))
                    .ok();
                Ok(true)
            }
            Command::Upgrade {
                directory,
                acknowledged,
            } => {
                ensure!(
                    acknowledged
                        && self.snapshot.safe_idle(&self.target, &self.bound_uid)
                        && self.snapshot.loader_version == self.target.loader_version,
                    "升级门禁未满足：身份、Loader 或停机状态异常"
                );
                let package = UpgradePackage::load(&directory, &self.target)?;
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

/// 显式连接后启动独占设备线程；任何硬故障立即关闭端口，不自动恢复试验。
pub fn run(
    dll: &Path,
    device_index: u32,
    node: u8,
    manual_start: u16,
    target: Target,
    commands: Receiver<Command>,
    events: Sender<Event>,
) {
    let result = (|| -> Result<()> {
        ensure!(node > 0 && node < 255, "节点必须为 1..254");
        let port = ControlCanFd::open(dll, device_index)?;
        events
            .send(Event::Status("CAN FD 已连接，正在读取真实设备身份".into()))
            .ok();
        let mut session = Session {
            port,
            node,
            target,
            sequence: manual_start,
            manual_start,
            bound_uid: String::new(),
            snapshot: Snapshot::default(),
            send: events.clone(),
        };
        let mut discovery = Instant::now() - Duration::from_secs(4);
        loop {
            loop {
                match commands.try_recv() {
                    Ok(command) => match session.handle(command) {
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
            if discovery.elapsed() >= Duration::from_secs(3) {
                if let Err(error) = session.read_identity() {
                    return Err(session.fail_after_stop(error));
                }
                discovery = Instant::now();
            }
            if let Err(error) = session.read_feedback(true) {
                return Err(session.fail_after_stop(error));
            }
            std::thread::sleep(Duration::from_millis(200));
        }
    })();
    if let Err(error) = result {
        events
            .send(Event::HardStop(format!("{error:#}；设备状态需现场确认")))
            .ok();
    }
    events.send(Event::Disconnected).ok();
}

#[cfg(test)]
mod tests {
    use super::*;

    fn valid_feedback() -> Vec<u8> {
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
        assert_eq!(view.speed_reference_rad_s, None);
        assert!(apply_feedback(&mut view, &data[..44], 1).is_err());
        assert!(apply_feedback(&mut view, &data, 2).is_err());
    }

    #[test]
    fn speed_si_payload_keeps_wire_format_and_physical_limit() {
        let payload = speed_target_payload(12.5).unwrap();
        assert_eq!(payload[0], 2);
        assert_eq!(
            i32::from_le_bytes(payload[8..12].try_into().unwrap()),
            12500
        );
        assert!(payload[1..8].iter().chain(&payload[12..]).all(|b| *b == 0));
        for value in [f64::NAN, f64::INFINITY, -0.1, MAX_SPEED_RAD_S + 0.001] {
            assert!(speed_target_payload(value).is_err());
        }
        let maximum = speed_target_payload(MAX_SPEED_RAD_S).unwrap();
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
        assert!(view.safe_idle(&target, "A") && view.speed_supported());
        assert!(!view.safe_idle(&target, "B"));
        view.motor_state = 6;
        view.mode = 2;
        view.motion_may_run = true;
        assert!(!view.safe_idle(&target, "A"));
        assert!(view.can_set_speed(&target, "A"));
        view.feedback_at = Some(Instant::now() - Duration::from_millis(600));
        assert!(!view.can_set_speed(&target, "A"));
        view.feedback_at = Some(Instant::now());
        view.bus_v = Some(34.0);
        assert!(!view.can_set_speed(&target, "A"));
    }
    #[test]
    fn stale_and_incompatible_never_enable_motion() {
        let target = Target::bundled().unwrap();
        let view = Snapshot::default();
        assert!(!view.safe_idle(&target, "123"));
        assert!(!view.can_set_speed(&target, "123"));
    }
}
