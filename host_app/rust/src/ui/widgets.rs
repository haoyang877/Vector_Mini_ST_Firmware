//! 全工作区共用的信息层级、卡片和状态样式。
use super::Palette;
use eframe::egui::{self, Frame, RichText, Stroke, Vec2};

pub(crate) fn page_header(ui: &mut egui::Ui, title: &str, subtitle: &str, badge: &str) {
    let c = Palette::from_ctx(ui.ctx());
    ui.horizontal(|ui| {
        ui.label(RichText::new(title).size(22.0).strong());
        if !badge.is_empty() {
            ui.add_space(6.0);
            Frame::new()
                .fill(c.panel)
                .corner_radius(3.0)
                .inner_margin(egui::Margin::symmetric(8, 3))
                .show(ui, |ui| {
                    ui.label(RichText::new(badge).size(11.0).color(c.muted));
                });
        }
    });
    ui.label(RichText::new(subtitle).size(12.0).color(c.muted));
    ui.add_space(16.0);
}

pub(crate) fn section<R>(
    ui: &mut egui::Ui,
    title: &str,
    content: impl FnOnce(&mut egui::Ui) -> R,
) -> R {
    let c = Palette::from_ctx(ui.ctx());
    Frame::new()
        .fill(c.panel)
        .stroke(Stroke::new(1.0, c.line))
        .corner_radius(4.0)
        .inner_margin(16.0)
        .show(ui, |ui| {
            ui.set_min_width(ui.available_width());
            ui.spacing_mut().item_spacing = Vec2::new(10.0, 8.0);
            ui.label(RichText::new(title).size(13.0).strong());
            ui.add_space(6.0);
            content(ui)
        })
        .inner
}

pub(crate) fn hint(ui: &mut egui::Ui, text: &str) {
    ui.label(
        RichText::new(text)
            .size(12.0)
            .color(Palette::from_ctx(ui.ctx()).muted),
    );
}

pub(crate) fn workflow(ui: &mut egui::Ui, steps: &[&str], current: usize) {
    let c = Palette::from_ctx(ui.ctx());
    ui.columns(steps.len(), |columns| {
        for (index, (column, step)) in columns.iter_mut().zip(steps).enumerate() {
            column.horizontal(|ui| {
                ui.label(
                    RichText::new(format!("{:02}", index + 1))
                        .small()
                        .color(if index == current { c.accent } else { c.muted }),
                );
                ui.label(RichText::new(*step).color(if index == current {
                    c.ink
                } else {
                    c.muted
                }));
            });
        }
    });
    ui.add_space(14.0);
}
