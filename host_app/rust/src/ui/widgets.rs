//! 全工作区共用的信息层级、卡片和状态样式。
use super::Palette;
use eframe::egui::{self, Frame, RichText, Stroke, Vec2};

pub(crate) fn page_header(ui: &mut egui::Ui, title: &str, subtitle: &str, badge: &str) {
    let c = Palette::from_ctx(ui.ctx());
    ui.horizontal(|ui| {
        ui.label(RichText::new(title).size(18.0).strong());
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
    ui.add_space(10.0);
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
        .inner_margin(12.0)
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

pub(super) fn primary(label: &str, c: Palette) -> egui::Button<'_> {
    egui::Button::new(RichText::new(label).strong().color(egui::Color32::WHITE))
        .fill(if c.panel == egui::Color32::WHITE {
            c.accent
        } else {
            egui::Color32::from_rgb(29, 98, 158)
        })
        .min_size(Vec2::new(120.0, 34.0))
}

pub(super) fn control_group(ui: &mut egui::Ui, name: &str, body: impl FnOnce(&mut egui::Ui)) {
    Frame::new()
        .fill(egui::Color32::WHITE)
        .corner_radius(4.0)
        .inner_margin(3.0)
        .show(ui, |ui| {
            ui.set_min_width(ui.available_width());
            Frame::new()
                .fill(egui::Color32::from_rgb(232, 241, 254))
                .corner_radius(3.0)
                .inner_margin(4.0)
                .show(ui, |ui| {
                    ui.set_min_width(ui.available_width());
                    ui.label(
                        RichText::new(name)
                            .strong()
                            .color(egui::Color32::from_rgb(30, 83, 156)),
                    );
                });
            Frame::new().inner_margin(4.0).show(ui, |ui| {
                ui.set_min_width(ui.available_width());
                body(ui);
            });
        });
}
