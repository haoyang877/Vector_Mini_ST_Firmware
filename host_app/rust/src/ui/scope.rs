//! 四种信号共用时间窗和游标；缺测或观察中断断线，不插值跨越未知数据。

use super::Palette;
use crate::capture::{MAX_SAMPLES, Sample};
use crate::device::Snapshot;
use std::collections::VecDeque;
use std::sync::{Arc, Mutex, mpsc};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

pub(super) const REFRESH_PERIOD: Duration = crate::device::OBSERVE_INTERVAL;

#[derive(Clone)]
pub(super) struct Curve {
    pub range: (f64, f64),
    pub actual: Vec<Vec<[f64; 2]>>,
    pub reference: Vec<Vec<[f64; 2]>>,
}

pub(super) struct Scene {
    pub start: f64,
    pub end: f64,
    pub references: bool,
    pub has_samples: bool,
    pub latest_sample_time: Option<f64>,
    pub curves: [Curve; 4],
}

enum Message {
    #[cfg(test)]
    Sample(Sample),
    Snapshot(Box<Snapshot>),
    Observe(String, Instant, u32),
    Pause,
    Reset(Vec<Sample>),
    View(f64, bool),
    Stop,
}

pub(super) struct SnapshotSink(mpsc::Sender<Message>);
impl SnapshotSink {
    pub fn publish(&self, snapshot: Box<Snapshot>) {
        let _ = self.0.send(Message::Snapshot(snapshot));
    }
}

struct Recording {
    uid: String,
    start: Instant,
    segment: u32,
    last_at: Option<Instant>,
}
impl Recording {
    fn sample(&mut self, snapshot: &Snapshot) -> Option<Sample> {
        if snapshot.uid != self.uid || !snapshot.feedback_fresh() {
            return None;
        }
        let at = snapshot.feedback_at?;
        if self.last_at.is_some_and(|last| at <= last) {
            return None;
        }
        let time_s = at.checked_duration_since(self.start)?.as_secs_f64();
        if self
            .last_at
            .is_some_and(|last| at.duration_since(last) > Duration::from_millis(100))
        {
            self.segment += 1;
        }
        self.last_at = Some(at);
        Some(Sample::from_snapshot(time_s, self.segment, snapshot))
    }
}

pub(super) struct Worker {
    sender: mpsc::Sender<Message>,
    latest: Arc<Mutex<Arc<Scene>>>,
    thread: Option<JoinHandle<()>>,
    view: (f64, bool),
}

impl Worker {
    pub fn new(ctx: egui::Context) -> Self {
        let (sender, receiver) = mpsc::channel();
        let latest = Arc::new(Mutex::new(Arc::new(prepare(&VecDeque::new(), 30.0, true))));
        let output = latest.clone();
        let thread = std::thread::Builder::new()
            .name("yg-curve-refresh".into())
            .spawn(move || {
                let mut samples = VecDeque::new();
                let mut window = 30.0;
                let mut references = true;
                let mut dirty = false;
                let mut recording: Option<Recording> = None;
                let mut next = Instant::now() + REFRESH_PERIOD;
                loop {
                    let message =
                        receiver.recv_timeout(next.saturating_duration_since(Instant::now()));
                    match message {
                        Ok(Message::Stop) | Err(mpsc::RecvTimeoutError::Disconnected) => break,
                        #[cfg(test)]
                        Ok(Message::Sample(sample)) => {
                            if samples.len() == MAX_SAMPLES {
                                samples.pop_front();
                            }
                            samples.push_back(sample);
                            dirty = true;
                        }
                        Ok(Message::Snapshot(snapshot)) => {
                            if let Some(sample) = recording
                                .as_mut()
                                .and_then(|recording| recording.sample(&snapshot))
                            {
                                if samples.len() == MAX_SAMPLES {
                                    samples.pop_front();
                                }
                                samples.push_back(sample);
                                dirty = true;
                            }
                        }
                        Ok(Message::Observe(uid, start, segment)) => {
                            recording = Some(Recording {
                                uid,
                                start,
                                segment,
                                last_at: None,
                            });
                        }
                        Ok(Message::Pause) => recording = None,
                        Ok(Message::Reset(values)) => {
                            samples = values.into();
                            recording = None;
                            dirty = true;
                        }
                        Ok(Message::View(seconds, show)) => {
                            window = seconds;
                            references = show;
                            dirty = true;
                        }
                        Err(mpsc::RecvTimeoutError::Timeout) => {}
                    }
                    if Instant::now() >= next {
                        if dirty {
                            let scene = Arc::new(prepare(&samples, window, references));
                            if let Ok(mut latest) = output.lock() {
                                *latest = scene;
                            }
                            // egui 的即时请求会触发两次重绘；非零延迟只唤醒一次。
                            ctx.request_repaint_after(Duration::from_millis(1));
                            dirty = false;
                        }
                        next =
                            crate::device::advance_deadline(next, REFRESH_PERIOD, Instant::now());
                    }
                }
            })
            .expect("曲线工作线程启动失败");
        Self {
            sender,
            latest,
            thread: Some(thread),
            view: (30.0, true),
        }
    }
    #[cfg(test)]
    pub fn push(&self, sample: Sample) {
        let _ = self.sender.send(Message::Sample(sample));
    }
    pub fn sink(&self) -> SnapshotSink {
        SnapshotSink(self.sender.clone())
    }
    pub fn observe(&self, (uid, start, segment): (String, Instant, u32)) {
        let _ = self.sender.send(Message::Observe(uid, start, segment));
    }
    pub fn pause(&self) {
        let _ = self.sender.send(Message::Pause);
    }
    pub fn reset(&self, samples: Vec<Sample>) {
        let _ = self.sender.send(Message::Reset(samples));
    }
    pub fn scene(&mut self, window: f64, references: bool) -> Arc<Scene> {
        if self.view != (window, references) {
            self.view = (window, references);
            let _ = self.sender.send(Message::View(window, references));
        }
        self.latest.lock().expect("曲线快照锁异常").clone()
    }
}
impl Drop for Worker {
    fn drop(&mut self) {
        let _ = self.sender.send(Message::Stop);
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
    }
}

// 每个时间桶保留首尾和极值，减少绘制点数且保留尖峰；缺测和分段保持断线。
fn simplify(path: &[[f64; 2]], start: f64, end: f64) -> Vec<[f64; 2]> {
    let bucket = |p: &[f64; 2]| (((p[0] - start) / (end - start)) * 800.0).floor() as i32;
    let mut result = vec![];
    let mut offset = 0;
    while offset < path.len() {
        let mut stop = offset + 1;
        while stop < path.len() && bucket(&path[stop]) == bucket(&path[offset]) {
            stop += 1;
        }
        let min = (offset..stop)
            .min_by(|&a, &b| path[a][1].total_cmp(&path[b][1]))
            .unwrap();
        let max = (offset..stop)
            .max_by(|&a, &b| path[a][1].total_cmp(&path[b][1]))
            .unwrap();
        let mut keep = vec![offset, min, max, stop - 1];
        keep.sort_unstable();
        keep.dedup();
        result.extend(keep.into_iter().map(|index| path[index]));
        offset = stop;
    }
    result
}

fn prepare(samples: &VecDeque<Sample>, window: f64, references: bool) -> Scene {
    let end = samples
        .back()
        .map(|p| p.time_s)
        .unwrap_or(window)
        .max(window.min(5.0));
    let start = (end - window).max(0.0);
    let points: Vec<_> = samples
        .iter()
        .filter(|p| p.time_s >= start && p.time_s <= end)
        .collect();
    let series = |index: usize| {
        let mut paths = vec![];
        let mut path = vec![];
        let mut segment = None;
        let flush = |path: &mut Vec<[f64; 2]>, paths: &mut Vec<Vec<[f64; 2]>>| {
            if !path.is_empty() {
                paths.push(simplify(path, start, end));
                path.clear();
            }
        };
        for p in &points {
            if segment.is_some_and(|value| value != p.segment) {
                flush(&mut path, &mut paths);
            }
            segment = Some(p.segment);
            if let Some(value) = p.values[index].filter(|v| v.is_finite()) {
                path.push([p.time_s, value]);
            } else {
                flush(&mut path, &mut paths);
            }
        }
        flush(&mut path, &mut paths);
        paths
    };
    let curves = std::array::from_fn(|channel| {
        let actual_index = [0, 1, 2, 6][channel];
        let reference_index = [3, 4, 5, 7][channel];
        Curve {
            range: bounds(
                points
                    .iter()
                    .flat_map(|p| {
                        [
                            p.values[actual_index],
                            if references {
                                p.values[reference_index]
                            } else {
                                None
                            },
                        ]
                    })
                    .flatten(),
            ),
            actual: series(actual_index),
            reference: if references {
                series(reference_index)
            } else {
                vec![]
            },
        }
    });
    Scene {
        start,
        end,
        references,
        has_samples: !points.is_empty(),
        latest_sample_time: samples.back().map(|sample| sample.time_s),
        curves,
    }
}
use eframe::egui::{self, Align2, FontId, Pos2, Rect, Sense, Shape, Stroke, StrokeKind, Vec2};

fn bounds(values: impl Iterator<Item = f64>) -> (f64, f64) {
    let mut low = f64::INFINITY;
    let mut high = f64::NEG_INFINITY;
    for value in values.filter(|v| v.is_finite()) {
        low = low.min(value);
        high = high.max(value);
    }
    if !low.is_finite() {
        return (-1.0, 1.0);
    }
    let padding = ((high - low) * 0.12)
        .max(high.abs().max(low.abs()) * 0.04)
        .max(0.05);
    (low - padding, high + padding)
}

pub(super) fn plot(
    ui: &mut egui::Ui,
    scene: &Scene,
    channel: usize,
    height: f32,
    cursor: Option<f64>,
) -> (Option<f64>, Rect) {
    let c = Palette::from_ctx(ui.ctx());
    let (rect, response) =
        ui.allocate_exact_size(Vec2::new(ui.available_width(), height), Sense::hover());
    let painter = ui.painter_at(rect);
    let signal = if ui.ctx().theme() == egui::Theme::Dark {
        [
            egui::Color32::from_rgb(75, 163, 224),
            egui::Color32::from_rgb(83, 192, 163),
            egui::Color32::from_rgb(218, 183, 113),
            egui::Color32::from_rgb(186, 142, 235),
        ][channel]
    } else {
        [
            c.accent,
            egui::Color32::from_rgb(32, 133, 104),
            egui::Color32::from_rgb(162, 109, 29),
            egui::Color32::from_rgb(127, 71, 174),
        ][channel]
    };
    painter.rect_filled(rect, 2.0, c.panel);
    painter.rect_stroke(rect, 2.0, Stroke::new(1.0, c.line), StrokeKind::Inside);
    painter.line_segment(
        [
            rect.left_top() + Vec2::new(1.0, 10.0),
            rect.left_top() + Vec2::new(1.0, 24.0),
        ],
        Stroke::new(2.0, signal),
    );
    let chart = Rect::from_min_max(
        rect.min + Vec2::new(66.0, 35.0),
        rect.max - Vec2::new(18.0, 24.0),
    );
    if !chart.is_positive() {
        return (None, rect);
    }
    let (start, end) = (scene.start, scene.end);
    let curve = &scene.curves[channel];
    let has_reference = !curve.reference.is_empty();
    let (low, high) = curve.range;
    let sx = |time: f64| chart.left() + ((time - start) / (end - start)) as f32 * chart.width();
    let sy = |value: f64| chart.bottom() - ((value - low) / (high - low)) as f32 * chart.height();
    let font = FontId::proportional(12.0);
    let title = ["位置 · rad", "速度 · rad/s", "电流 Iq · A", "电压 Vq · V"][channel];
    painter.text(
        rect.min + Vec2::new(12.0, 9.0),
        Align2::LEFT_TOP,
        title,
        font.clone(),
        signal,
    );
    painter.text(
        rect.right_top() + Vec2::new(-12.0, 9.0),
        Align2::RIGHT_TOP,
        if !scene.references {
            "参考已隐藏"
        } else if has_reference {
            "实际 / 参考（虚线）"
        } else {
            "实际 / 参考未提供"
        },
        font.clone(),
        c.muted,
    );
    for tick in 0..3 {
        let value = low + (high - low) * tick as f64 / 2.0;
        let y = sy(value);
        painter.line_segment(
            [Pos2::new(chart.left(), y), Pos2::new(chart.right(), y)],
            Stroke::new(0.6, c.line),
        );
        painter.text(
            Pos2::new(chart.left() - 8.0, y),
            Align2::RIGHT_CENTER,
            format!("{value:.2}"),
            font.clone(),
            c.muted,
        );
    }
    for tick in 0..4 {
        let t = start + (end - start) * tick as f64 / 3.0;
        let x = sx(t);
        painter.line_segment(
            [Pos2::new(x, chart.top()), Pos2::new(x, chart.bottom())],
            Stroke::new(0.6, c.line),
        );
        painter.text(
            Pos2::new(x, chart.bottom() + 5.0),
            if tick == 0 {
                Align2::LEFT_TOP
            } else if tick == 3 {
                Align2::RIGHT_TOP
            } else {
                Align2::CENTER_TOP
            },
            format!("{t:.1}{}", if tick == 3 { " s" } else { "" }),
            font.clone(),
            c.muted,
        );
    }
    let draw_series = |paths: &[Vec<[f64; 2]>], dashed: bool| {
        for points in paths {
            let path: Vec<_> = points
                .iter()
                .map(|p| Pos2::new(sx(p[0]), sy(p[1])))
                .collect();
            if path.len() > 1 {
                if dashed {
                    painter.extend(Shape::dashed_line(
                        &path,
                        Stroke::new(1.2, c.muted),
                        5.0,
                        4.0,
                    ));
                } else {
                    painter.add(Shape::line(path, Stroke::new(1.7, signal)));
                }
            } else if let Some(point) = path.first() {
                painter.circle_filled(*point, 2.0, signal);
            }
        }
    };
    draw_series(&curve.reference, true);
    draw_series(&curve.actual, false);
    let hover = response
        .hover_pos()
        .filter(|p| chart.contains(*p))
        .map(|p| start + ((p.x - chart.left()) / chart.width()) as f64 * (end - start));
    if let Some(time) = hover.or(cursor).filter(|t| *t >= start && *t <= end) {
        painter.line_segment(
            [
                Pos2::new(sx(time), chart.top()),
                Pos2::new(sx(time), chart.bottom()),
            ],
            Stroke::new(1.0, c.warning),
        );
        if let Some(point) = curve
            .actual
            .iter()
            .flatten()
            .min_by(|a, b| (a[0] - time).abs().total_cmp(&(b[0] - time).abs()))
            && (point[0] - time).abs() <= 0.5
        {
            let text = format!("t {time:.2} s / {:.3}", point[1]);
            painter.text(
                Pos2::new(chart.center().x, rect.top() + 9.0),
                Align2::CENTER_TOP,
                text,
                font,
                c.ink,
            );
        }
    }
    if !scene.has_samples {
        painter.text(
            chart.center(),
            Align2::CENTER_CENTER,
            "暂无记录 · 连接设备后开始观察",
            FontId::proportional(13.0),
            c.muted,
        );
    }
    (hover, rect)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn four_signal_scene_preserves_real_references_without_fabricating_vq_reference() {
        let snapshot = Snapshot {
            position_rad: Some(1.0),
            position_reference_rad: Some(1.1),
            speed_rad_s: Some(2.0),
            speed_reference_rad_s: Some(2.1),
            iq_a: Some(3.0),
            iq_reference_a: Some(3.1),
            vq_v: Some(-4.0),
            ..Default::default()
        };
        let samples = VecDeque::from([Sample::from_snapshot(1.0, 1, &snapshot)]);
        let scene = prepare(&samples, 30.0, true);
        for (channel, actual, reference) in [(0, 1.0, 1.1), (1, 2.0, 2.1), (2, 3.0, 3.1)] {
            assert_eq!(scene.curves[channel].actual[0][0][1], actual);
            assert_eq!(scene.curves[channel].reference[0][0][1], reference);
        }
        assert_eq!(scene.curves[3].actual[0][0][1], -4.0);
        assert!(scene.curves[3].reference.is_empty());
        let scene = prepare(&samples, 30.0, false);
        assert!(scene.curves.iter().all(|curve| curve.reference.is_empty()));
    }
    #[test]
    fn direct_snapshot_stream_does_not_require_ui_event_processing() {
        let mut worker = Worker::new(egui::Context::default());
        let now = Instant::now();
        worker.observe(("A".into(), now - Duration::from_secs(1), 1));
        worker.sink().publish(Box::new(Snapshot {
            uid: "A".into(),
            feedback_at: Some(now),
            position_rad: Some(2.0),
            ..Default::default()
        }));
        let deadline = Instant::now() + Duration::from_secs(1);
        while !worker.scene(30.0, true).has_samples && Instant::now() < deadline {
            std::thread::sleep(Duration::from_millis(10));
        }
        let scene = worker.scene(30.0, true);
        assert_eq!(scene.latest_sample_time, Some(1.0));
        assert_eq!(scene.curves[0].actual[0][0][1], 2.0);
    }

    #[test]
    fn direct_stream_rejects_other_uid_and_duplicate_feedback() {
        let now = Instant::now();
        let mut recording = Recording {
            uid: "A".into(),
            start: now - Duration::from_secs(1),
            segment: 1,
            last_at: None,
        };
        let mut snapshot = Snapshot {
            uid: "B".into(),
            feedback_at: Some(now),
            ..Default::default()
        };
        assert!(recording.sample(&snapshot).is_none());
        snapshot.uid = "A".into();
        assert!(recording.sample(&snapshot).is_some());
        assert!(recording.sample(&snapshot).is_none());
    }
    #[test]
    fn preparation_preserves_extrema_and_missing_data_boundaries() {
        let mut samples = VecDeque::new();
        for index in 0..10000 {
            let value = if index == 3050 { 100.0 } else { 1.0 };
            samples.push_back(Sample {
                time_s: index as f64 / 1000.0,
                segment: if index < 6000 { 1 } else { 2 },
                values: [
                    if index == 4000 { None } else { Some(value) },
                    None,
                    None,
                    None,
                    None,
                    None,
                    None,
                    None,
                ],
            });
        }
        let scene = prepare(&samples, 30.0, true);
        assert_eq!(scene.curves[0].actual.len(), 3);
        let points: Vec<_> = scene.curves[0].actual.iter().flatten().collect();
        assert!(points.iter().any(|p| p[1] == 100.0));
        assert!(points.len() < samples.len());
        assert!(scene.curves[1].actual.is_empty());
        assert!(scene.curves[0].reference.is_empty());
    }

    #[test]
    fn worker_prepares_an_independent_snapshot_and_resets_history() {
        let mut worker = Worker::new(egui::Context::default());
        worker.push(Sample {
            time_s: 1.0,
            segment: 1,
            values: [Some(2.0), None, None, None, None, None, None, None],
        });
        let deadline = Instant::now() + Duration::from_secs(1);
        while !worker.scene(30.0, true).has_samples && Instant::now() < deadline {
            std::thread::sleep(Duration::from_millis(10));
        }
        let previous = worker.scene(30.0, true);
        assert!(previous.has_samples);
        worker.reset(vec![]);
        let deadline = Instant::now() + Duration::from_secs(1);
        while worker.scene(30.0, true).has_samples && Instant::now() < deadline {
            std::thread::sleep(Duration::from_millis(10));
        }
        assert!(!worker.scene(30.0, true).has_samples);
        assert!(previous.has_samples);
    }
    #[test]
    fn ranges_reject_nonfinite_and_pad_constant_signals() {
        assert_eq!(bounds([f64::NAN, f64::INFINITY].into_iter()), (-1.0, 1.0));
        let (low, high) = bounds([3.0, 3.0].into_iter());
        assert!(low < 3.0 && high > 3.0);
        let (low, high) = bounds([-100.0, 42.0].into_iter());
        assert!(low < -100.0 && high > 42.0);
    }
}
