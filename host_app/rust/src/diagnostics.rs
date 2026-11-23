//! 主机故障证据：事件追加到 JSONL，最近原始帧只在故障时落盘，避免影响曲线采集。

use crate::can::Transport;
use anyhow::{Context, Result};
use serde_json::{Value, json};
use std::collections::VecDeque;
use std::fs::{self, File, OpenOptions};
use std::io::Write;
use std::path::PathBuf;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::mpsc::{self, SyncSender};
use std::sync::{Arc, Mutex};
use std::time::{Instant, SystemTime, UNIX_EPOCH};

const PART_BYTES: u64 = 8 * 1024 * 1024;
const RECENT_FRAMES: usize = 64;

fn unix_ms() -> u128 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis()
}

struct State {
    file: Option<File>,
    paths: Vec<PathBuf>,
    error: Option<String>,
    bytes: u64,
}

enum Job {
    Record(Value),
    Barrier(mpsc::Sender<()>),
}

#[derive(Clone)]
pub struct Logger {
    state: Arc<Mutex<State>>,
    queue: SyncSender<Job>,
    dropped: Arc<AtomicU64>,
    started: Instant,
}

impl Default for Logger {
    fn default() -> Self {
        Self::new()
    }
}

impl Logger {
    pub fn new() -> Self {
        let directory = std::env::var_os("LOCALAPPDATA")
            .map(PathBuf::from)
            .map(|root| root.join("YG/YG_MC_HOST/logs"));
        Self::at(directory)
    }

    pub(crate) fn at(directory: Option<PathBuf>) -> Self {
        let result = (|| -> Result<(File, PathBuf)> {
            let directory = directory.context("缺少 LOCALAPPDATA，故障日志不可保存")?;
            fs::create_dir_all(&directory)?;
            let path = directory.join(format!("host-{}-{}.jsonl", unix_ms(), std::process::id()));
            let file = OpenOptions::new()
                .write(true)
                .create_new(true)
                .open(&path)?;
            Ok((file, path))
        })();
        let (file, paths, error) = match result {
            Ok((file, path)) => (Some(file), vec![path], None),
            Err(error) => (None, vec![], Some(format!("{error:#}"))),
        };
        let state = Arc::new(Mutex::new(State {
            file,
            paths,
            error,
            bytes: 0,
        }));
        let (queue, receiver) = mpsc::sync_channel(256);
        let writer = state.clone();
        let spawned = std::thread::Builder::new()
            .name("fault-log".into())
            .spawn(move || {
                while let Ok(job) = receiver.recv() {
                    match job {
                        Job::Record(row) => Self::append(&writer, row),
                        Job::Barrier(done) => {
                            done.send(()).ok();
                        }
                    }
                }
            });
        if let Err(error) = spawned {
            let mut status = state.lock().unwrap();
            status.error = Some(format!("无法创建故障日志线程：{error}"));
            status.file = None;
        }
        Self {
            state,
            queue,
            dropped: Arc::new(AtomicU64::new(0)),
            started: Instant::now(),
        }
    }

    /// 不可用的日志不会阻止 STOP；错误始终可在界面查询。
    pub fn event(&self, level: &str, event: &str, details: Value) {
        let row = json!({"utc_unix_ms":unix_ms(), "elapsed_ms":self.started.elapsed().as_millis(),
            "version":env!("CARGO_PKG_VERSION"), "pid":std::process::id(),
            "level":level, "event":event, "details":details,
            "dropped_events":self.dropped.load(Ordering::Relaxed)});
        // 设备线程不等待日志文件锁、磁盘或导出；队列满时公开丢失计数。
        if self.queue.try_send(Job::Record(row)).is_err() {
            self.dropped.fetch_add(1, Ordering::Relaxed);
        }
    }

    fn append(shared: &Mutex<State>, row: Value) {
        let mut state = shared.lock().unwrap();
        let result = (|| -> Result<()> {
            if state.file.is_none() {
                return Ok(());
            }
            if state.bytes >= PART_BYTES {
                let first = &state.paths[0];
                let path = first.with_file_name(format!(
                    "{}-part{}.jsonl",
                    first.file_stem().unwrap().to_string_lossy(),
                    state.paths.len()
                ));
                let file = OpenOptions::new()
                    .write(true)
                    .create_new(true)
                    .open(&path)?;
                state.file = Some(file);
                state.paths.push(path);
                state.bytes = 0;
            }
            let mut bytes = serde_json::to_vec(&row)?;
            bytes.push(b'\n');
            let file = state.file.as_mut().unwrap();
            file.write_all(&bytes)?;
            file.flush()?;
            state.bytes += bytes.len() as u64;
            Ok(())
        })();
        if let Err(error) = result {
            state.error = Some(format!("故障日志写入失败：{error:#}"));
            state.file = None;
        }
    }

    pub fn location(&self) -> String {
        let Ok(state) = self.state.try_lock() else {
            return "日志写入或导出中".into();
        };
        if let Some(error) = &state.error {
            format!("日志不可用：{error}")
        } else {
            let location = state
                .paths
                .last()
                .map(|p| p.display().to_string())
                .unwrap_or_default();
            let dropped = self.dropped.load(Ordering::Relaxed);
            if dropped == 0 {
                location
            } else {
                format!("{location}（日志拥塞，丢失 {dropped} 条事件）")
            }
        }
    }

    pub fn available(&self) -> bool {
        self.state
            .try_lock()
            .is_ok_and(|state| !state.paths.is_empty())
    }

    /// 仅导出及离线验证等待写盘；设备控制路径禁止调用。
    pub(crate) fn flush(&self) -> Result<()> {
        let (done, receiver) = mpsc::channel();
        self.queue
            .send(Job::Barrier(done))
            .context("日志线程不可用")?;
        receiver.recv().context("日志写入结果未知")?;
        Ok(())
    }

    pub fn export(&self, destination: PathBuf) -> Result<()> {
        self.flush()?;
        let mut state = self.state.lock().unwrap();
        anyhow::ensure!(!state.paths.is_empty(), "没有可导出的日志");
        // 防止用户将导出目标选为当前源文件，先检查再创建。
        let canonical = destination.canonicalize().ok();
        anyhow::ensure!(
            !state.paths.iter().any(|p| p == &destination
                || canonical
                    .as_ref()
                    .is_some_and(|c| p.canonicalize().ok().as_ref() == Some(c))),
            "不能覆盖正在记录的日志"
        );
        if let Some(file) = &mut state.file {
            file.flush()?;
        }
        let mut output = File::create(destination)?;
        for path in &state.paths {
            std::io::copy(&mut File::open(path)?, &mut output)?;
        }
        output.flush()?;
        Ok(())
    }
}

/// 包含 SDK 错误、升级事务在内的唯一 Transport 装饰层；仅设备线程持有原始帧。
pub struct TraceTransport<T> {
    inner: T,
    pub log: Logger,
    pub context: Value,
    frames: VecDeque<Value>,
    started: Instant,
}

impl<T: Transport> TraceTransport<T> {
    pub fn new(inner: T, log: Logger, context: Value) -> Self {
        Self {
            inner,
            log,
            context,
            frames: VecDeque::new(),
            started: Instant::now(),
        }
    }

    fn frame(&mut self, direction: &str, id: u32, data: &[u8]) {
        if self.frames.len() == RECENT_FRAMES {
            self.frames.pop_front();
        }
        self.frames
            .push_back(json!({"elapsed_us":self.started.elapsed().as_micros(),
            "direction":direction,"can_id":format!("{id:08X}"),"hex":hex::encode(data)}));
    }

    pub fn report(&self, level: &str, event: &str, details: Value) {
        self.log.event(
            level,
            event,
            json!({"device":self.context,"transaction":details,"recent_frames":self.frames}),
        );
    }
}

impl<T: Transport> Transport for TraceTransport<T> {
    fn send(&mut self, id: u32, data: &[u8]) -> Result<()> {
        self.frame("tx", id, data);
        let result = self.inner.send(id, data);
        if let Err(error) = &result {
            self.report(
                "error",
                "transport_send_failed",
                json!({"error":format!("{error:#}")}),
            );
        }
        result
    }

    fn receive(&mut self, timeout: std::time::Duration) -> Result<Option<(u32, Vec<u8>)>> {
        let result = self.inner.receive(timeout);
        match &result {
            Ok(Some((id, data))) => self.frame("rx", *id, data),
            Err(error) => self.report(
                "error",
                "transport_receive_failed",
                json!({"error":format!("{error:#}")}),
            ),
            _ => {}
        }
        result
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn directory(label: &str) -> PathBuf {
        std::env::temp_dir().join(format!(
            "yg-log-{label}-{}-{}",
            std::process::id(),
            SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ))
    }

    #[test]
    fn unavailable_or_failed_log_is_visible_without_panicking() {
        let unavailable = Logger::at(None);
        unavailable.event("error", "stop_required", json!({}));
        unavailable.flush().unwrap();
        assert!(!unavailable.available());
        assert!(unavailable.location().contains("日志不可用"));
        let path = directory("not-a-directory");
        fs::write(&path, b"occupied").unwrap();
        let failed = Logger::at(Some(path.clone()));
        failed.event("error", "stop_failed", json!({}));
        failed.flush().unwrap();
        assert!(failed.location().contains("日志不可用"));
        fs::remove_file(path).unwrap();
    }

    #[test]
    fn rotation_keeps_all_parts_and_export_preserves_order() {
        let directory = directory("rotation");
        let log = Logger::at(Some(directory.clone()));
        log.event("info", "first", json!({}));
        log.flush().unwrap();
        log.state.lock().unwrap().bytes = PART_BYTES;
        log.event("error", "second", json!({}));
        log.flush().unwrap();
        assert_eq!(log.state.lock().unwrap().paths.len(), 2);
        let exported = directory.join("export.jsonl");
        log.export(exported.clone()).unwrap();
        let data = fs::read_to_string(exported).unwrap();
        let events: Vec<Value> = data
            .lines()
            .map(|line| serde_json::from_str(line).unwrap())
            .collect();
        assert_eq!(events[0]["event"], "first");
        assert_eq!(events[1]["event"], "second");
        log.state.lock().unwrap().file = None;
        drop(log);
        fs::remove_dir_all(directory).unwrap();
    }

    #[test]
    fn trace_ring_is_bounded_and_not_persisted_on_normal_traffic() {
        struct Port;
        impl Transport for Port {
            fn send(&mut self, _id: u32, _data: &[u8]) -> Result<()> {
                Ok(())
            }
            fn receive(&mut self, _timeout: std::time::Duration) -> Result<Option<(u32, Vec<u8>)>> {
                Ok(None)
            }
        }
        let directory = directory("trace");
        let log = Logger::at(Some(directory.clone()));
        let mut port = TraceTransport::new(Port, log.clone(), json!({"uid":"test"}));
        for i in 0..1000 {
            port.send(i, &[0; 20]).unwrap();
        }
        assert_eq!(fs::metadata(log.location()).unwrap().len(), 0);
        assert_eq!(port.frames.len(), RECENT_FRAMES);
        port.report("error", "test_fault", json!({}));
        log.flush().unwrap();
        let row: Value =
            serde_json::from_str(fs::read_to_string(log.location()).unwrap().trim()).unwrap();
        assert_eq!(
            row["details"]["recent_frames"].as_array().unwrap().len(),
            RECENT_FRAMES
        );
        log.state.lock().unwrap().file = None;
        drop(port);
        drop(log);
        fs::remove_dir_all(directory).unwrap();
    }

    #[test]
    fn busy_disk_or_export_never_blocks_device_side_log_submission() {
        let directory = directory("congestion");
        let log = Logger::at(Some(directory.clone()));
        let held = log.state.lock().unwrap();
        let start = Instant::now();
        for _ in 0..300 {
            log.event("error", "fault", json!({"requires_stop":true}));
        }
        assert!(start.elapsed() < std::time::Duration::from_millis(250));
        assert!(log.dropped.load(Ordering::Relaxed) > 0);
        drop(held);
        log.flush().unwrap();
        assert!(log.location().contains("日志拥塞"));
        log.state.lock().unwrap().file = None;
        drop(log);
        fs::remove_dir_all(directory).unwrap();
    }
}
