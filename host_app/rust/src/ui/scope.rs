//! 三组 SI 趋势共用时间窗和游标；缺测或观察中断断线，不插值跨越未知数据。

use super::Palette;
use crate::capture::Capture;
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
    capture: &Capture,
    channel: usize,
    window_s: f64,
    height: f32,
    cursor: Option<f64>,
    references: bool,
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
        ][channel]
    } else {
        [
            c.accent,
            egui::Color32::from_rgb(32, 133, 104),
            egui::Color32::from_rgb(162, 109, 29),
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
    let end = capture
        .samples
        .back()
        .map(|p| p.time_s)
        .unwrap_or(window_s)
        .max(window_s.min(5.0));
    let start = (end - window_s).max(0.0);
    let points: Vec<_> = capture
        .samples
        .iter()
        .filter(|p| p.time_s >= start && p.time_s <= end)
        .collect();
    let reference = channel + 3;
    let available_reference = points.iter().any(|p| p.values[reference].is_some());
    let has_reference = references && available_reference;
    let (low, high) = bounds(
        points
            .iter()
            .flat_map(|p| {
                [
                    p.values[channel],
                    if has_reference {
                        p.values[reference]
                    } else {
                        None
                    },
                ]
            })
            .flatten(),
    );
    let sx = |time: f64| chart.left() + ((time - start) / (end - start)) as f32 * chart.width();
    let sy = |value: f64| chart.bottom() - ((value - low) / (high - low)) as f32 * chart.height();
    let font = FontId::proportional(12.0);
    let title = ["01   位置 · rad", "02   速度 · rad/s", "03   电流 Iq · A"][channel];
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
        if !references {
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
    let draw_series = |index: usize, dashed: bool| {
        let mut path: Vec<Pos2> = vec![];
        let mut segment = None;
        let flush = |path: &mut Vec<Pos2>| {
            if path.len() > 1 {
                if dashed {
                    painter.extend(Shape::dashed_line(
                        path,
                        Stroke::new(1.2, c.muted),
                        5.0,
                        4.0,
                    ));
                } else {
                    painter.add(Shape::line(path.clone(), Stroke::new(1.7, signal)));
                }
            } else if let Some(point) = path.first() {
                painter.circle_filled(*point, 2.0, signal);
            }
            path.clear();
        };
        for point in &points {
            if segment.is_some_and(|id| id != point.segment) {
                flush(&mut path);
            }
            segment = Some(point.segment);
            match point.values[index].filter(|value| value.is_finite()) {
                Some(value) => path.push(Pos2::new(sx(point.time_s), sy(value))),
                None => flush(&mut path),
            }
        }
        flush(&mut path);
    };
    if has_reference {
        draw_series(reference, true);
    }
    draw_series(channel, false);
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
        if let Some(point) = points
            .iter()
            .min_by(|a, b| (a.time_s - time).abs().total_cmp(&(b.time_s - time).abs()))
            && (point.time_s - time).abs() <= 0.5
            && let Some(value) = point.values[channel]
        {
            let text = format!("t {time:.2} s / {value:.3}");
            painter.text(
                Pos2::new(chart.center().x, rect.top() + 9.0),
                Align2::CENTER_TOP,
                text,
                font,
                c.ink,
            );
        }
        if hover.is_some() {
            ui.ctx().request_repaint();
        }
    }
    if points.is_empty() {
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
    fn ranges_reject_nonfinite_and_pad_constant_signals() {
        assert_eq!(bounds([f64::NAN, f64::INFINITY].into_iter()), (-1.0, 1.0));
        let (low, high) = bounds([3.0, 3.0].into_iter());
        assert!(low < 3.0 && high > 3.0);
        let (low, high) = bounds([-100.0, 42.0].into_iter());
        assert!(low < -100.0 && high > 42.0);
    }
}
