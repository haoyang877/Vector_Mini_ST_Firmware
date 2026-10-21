//! 有界主机接收时间趋势；不把轮询当高速示波器，不混合 UID，不补造缺失参考量。

use crate::device::Snapshot;
use anyhow::{Result, ensure};
use std::collections::VecDeque;
use std::io::Write;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

const MAX_SAMPLES: usize = 6000;

#[derive(Clone)]
pub struct Sample {
    pub time_s: f64,
    pub segment: u32,
    /// 位置/速度/Iq 实际量，随后是同顺序参考量；位置瞬时参考目前未确认。
    pub values: [Option<f64>; 6],
}

pub struct Capture {
    pub uid: String,
    pub observing: bool,
    pub preview: bool,
    pub samples: VecDeque<Sample>,
    pub gaps: usize,
    pub evicted: usize,
    started_at: Instant,
    started_unix_ms: u128,
    last_at: Option<Instant>,
    segment: u32,
}

impl Default for Capture {
    fn default() -> Self {
        Self {
            uid: String::new(),
            observing: false,
            preview: false,
            samples: VecDeque::new(),
            gaps: 0,
            evicted: 0,
            started_at: Instant::now(),
            started_unix_ms: 0,
            last_at: None,
            segment: 0,
        }
    }
}

impl Capture {
    /// 显式开始或继续同一 UID；不同 UID 必须由操作者先导出并清空旧记录。
    pub fn start(&mut self, uid: &str) -> Result<()> {
        ensure!(!uid.is_empty(), "等待真实设备 UID");
        ensure!(!self.preview, "预览记录不能接入设备");
        ensure!(
            self.uid.is_empty() || self.uid == uid,
            "记录属于另一 UID；请先导出，再新建记录"
        );
        if self.uid.is_empty() {
            self.uid = uid.to_owned();
            self.started_at = Instant::now();
            self.started_unix_ms = SystemTime::now().duration_since(UNIX_EPOCH)?.as_millis();
        }
        if !self.observing {
            self.segment += 1;
            self.last_at = None;
        }
        self.observing = true;
        Ok(())
    }

    /// 暂停仅停止接纳观察样本，保留历史且不发送设备命令。
    pub fn pause(&mut self) {
        self.observing = false;
        self.last_at = None;
    }

    /// 仅接纳同 UID、新鲜且未重复的回读；返回本次是否追加样本。
    pub fn accept(&mut self, view: &Snapshot) -> bool {
        if !self.observing || view.uid != self.uid || !view.feedback_fresh() {
            return false;
        }
        let Some(at) = view.feedback_at else {
            return false;
        };
        if self.last_at.is_some_and(|last| at <= last) {
            return false;
        }
        let Some(elapsed) = at.checked_duration_since(self.started_at) else {
            return false;
        };
        if self
            .last_at
            .is_some_and(|last| at.duration_since(last) > Duration::from_millis(750))
        {
            self.segment += 1;
            self.gaps += 1;
        }
        self.last_at = Some(at);
        self.push(Sample {
            time_s: elapsed.as_secs_f64(),
            segment: self.segment,
            values: [
                view.position_rad,
                view.speed_rad_s,
                view.iq_a,
                None,
                view.speed_reference_rad_s,
                view.iq_reference_a,
            ],
        });
        true
    }

    fn push(&mut self, sample: Sample) {
        if self.samples.len() == MAX_SAMPLES {
            self.samples.pop_front();
            self.evicted += 1;
        }
        self.samples.push_back(sample);
    }

    pub fn observed_hz(&self) -> Option<f64> {
        let last = self.samples.back()?;
        let mut points = self
            .samples
            .iter()
            .rev()
            .take_while(|p| p.segment == last.segment);
        let latest = points.next()?;
        let remaining: Vec<_> = points.collect();
        let earliest = remaining.last()?;
        let duration = latest.time_s - earliest.time_s;
        (duration > 0.0).then_some(remaining.len() as f64 / duration)
    }

    /// 导出已接纳的主机回读，缺失值为空，主机时间不能冒充设备同步采样时间。
    pub fn write_csv(&self, mut output: impl Write) -> Result<()> {
        writeln!(
            output,
            "# YG_MC_HOST capture v1; source={}; uid={}; host_start_unix_ms={}",
            if self.preview {
                "preview"
            } else {
                "CAN108_host_receive"
            },
            self.uid,
            self.started_unix_ms
        )?;
        writeln!(
            output,
            "# timing=host_monotonic_receive; missing=empty; evicted={}; gaps={}",
            self.evicted, self.gaps
        )?;
        writeln!(
            output,
            "time_s,segment,position_rad,speed_rad_s,iq_a,position_reference_rad,speed_reference_rad_s,iq_reference_a"
        )?;
        for point in &self.samples {
            write!(output, "{:.6},{}", point.time_s, point.segment)?;
            for value in point.values {
                match value {
                    Some(value) => write!(output, ",{value:.6}")?,
                    None => write!(output, ",")?,
                }
            }
            writeln!(output)?;
        }
        Ok(())
    }

    /// 只供显式离线截图入口使用；数据来源随记录与导出保留。
    pub fn preview_record() -> Self {
        let mut capture = Self {
            uid: "PREVIEW-NO-DEVICE".into(),
            preview: true,
            ..Self::default()
        };
        for index in 0..101 {
            let t = index as f64 * 0.2;
            let x = (t - 3.0).max(0.0);
            let speed = if x > 0.0 {
                12.0 * (1.0 - (-x * 1.8).exp() * (x * 3.8).cos())
            } else {
                0.0
            };
            capture.push(Sample {
                time_s: t,
                segment: 1,
                values: [
                    Some(12.0 * x),
                    Some(speed),
                    Some(0.3 + (-x * 1.4).exp() * 0.9),
                    None,
                    Some(if x > 0.0 { 12.0 } else { 0.0 }),
                    None,
                ],
            });
        }
        capture
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn explicit_observe_uid_dedup_and_pause() {
        let mut capture = Capture::default();
        let mut view = Snapshot {
            uid: "A".into(),
            feedback_at: Some(Instant::now()),
            ..Default::default()
        };
        assert!(!capture.accept(&view));
        capture.start("A").unwrap();
        view.feedback_at = Some(Instant::now());
        view.speed_rad_s = Some(3.0);
        assert!(capture.accept(&view));
        assert!(!capture.accept(&view));
        view.uid = "B".into();
        assert!(!capture.accept(&view));
        assert!(capture.start("B").is_err());
        capture.pause();
        assert_eq!(capture.samples.len(), 1);
        assert!(!capture.accept(&view));
        capture.start("A").unwrap();
        view.uid = "A".into();
        view.feedback_at = Some(Instant::now() - Duration::from_secs(1));
        assert!(!capture.accept(&view));
    }

    #[test]
    fn bounded_samples_and_csv_preserve_missing_values_and_units() {
        let mut capture = Capture::default();
        for index in 0..MAX_SAMPLES + 2 {
            capture.push(Sample {
                time_s: index as f64,
                segment: 1,
                values: [None, Some(3.25), None, None, None, None],
            });
        }
        assert_eq!(capture.samples.len(), MAX_SAMPLES);
        assert_eq!(capture.evicted, 2);
        let mut output = vec![];
        capture.write_csv(&mut output).unwrap();
        let text = String::from_utf8(output).unwrap();
        assert!(text.contains("position_rad,speed_rad_s,iq_a"));
        assert!(text.contains("2.000000,1,,3.250000,,,,"));
    }

    #[test]
    fn gaps_and_unknown_references_never_become_continuous_zero_lines() {
        let mut capture = Capture::default();
        capture.start("A").unwrap();
        capture.started_at = Instant::now() - Duration::from_secs(2);
        let mut view = Snapshot {
            uid: "A".into(),
            feedback_at: Some(Instant::now()),
            ..Default::default()
        };
        capture.accept(&view);
        capture.last_at = Some(Instant::now() - Duration::from_secs(1));
        view.feedback_at = Some(Instant::now());
        capture.accept(&view);
        assert_eq!(capture.gaps, 1);
        assert_ne!(capture.samples[0].segment, capture.samples[1].segment);
        assert!(capture.samples.iter().all(|p| p.values[3].is_none()));
        assert!(Capture::preview_record().start("A").is_err());
    }
}
