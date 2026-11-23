//! 中控、日志与当前设备共用的窗口外壳，设备操作仍交由同一个会话。
use super::*;
use crate::motor_configuration as config;

const BLUE: Color32 = Color32::from_rgb(29, 110, 242);
const SIDEBAR: Color32 = Color32::from_rgb(245, 246, 249);
const BAR: Color32 = Color32::from_rgb(60, 60, 60);

impl HostApp {
    pub(super) fn workspace_shell(&mut self, ui: &mut egui::Ui) {
        let c = Palette::from_ctx(ui.ctx());
        let wide = ui.available_width() >= 800.0;
        let rail = self.device_list_open && wide;
        if !matches!(self.page, Page::Connection | Page::Records) {
            self.last_device_page = self.page;
        }
        egui::Panel::top("studio_global")
            .frame(
                Frame::new()
                    .fill(BAR)
                    .inner_margin(egui::Margin::symmetric(4, 4)),
            )
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.spacing_mut().item_spacing.x = 4.0;
                    let (label_w, tab_w) = if rail {
                        (
                            136.0,
                            ((ui.available_width() - 360.0) / 3.0).clamp(70.0, 160.0),
                        )
                    } else {
                        (
                            32.0,
                            ((ui.available_width() - 160.0) / 3.0).clamp(50.0, 116.0),
                        )
                    };
                    if ui
                        .add_sized(
                            [label_w, 32.0],
                            egui::Button::new(
                                RichText::new(if rail { "设备列表" } else { "☰" })
                                    .color(Color32::WHITE)
                                    .strong(),
                            )
                            .frame(false),
                        )
                        .on_hover_text("显示或收起设备列表")
                        .clicked()
                    {
                        if wide {
                            self.device_list_open = !self.device_list_open;
                        } else {
                            self.page = Page::Connection;
                        }
                    }
                    let tab = if self.page == Page::Connection {
                        0
                    } else if self.page == Page::Records {
                        1
                    } else {
                        2
                    };
                    for (i, title) in ["中控", "日志", "当前设备"].into_iter().enumerate() {
                        let selected = i == tab;
                        let ink = if selected {
                            Color32::WHITE
                        } else {
                            Color32::from_gray(23)
                        };
                        let button = egui::Button::new(RichText::new(title).strong().color(ink))
                            .fill(if selected {
                                BLUE
                            } else {
                                Color32::from_rgb(231, 232, 235)
                            })
                            .stroke(Stroke::NONE)
                            .corner_radius(4.0);
                        let response = ui.add_sized([tab_w, 32.0], button);
                        if tab_w >= 108.0 {
                            draw_nav_icon(
                                ui.painter(),
                                response.rect.left_center() + egui::vec2(20.0, 0.0),
                                i,
                                ink,
                            );
                        }
                        if response.clicked() {
                            self.page = match i {
                                0 => Page::Connection,
                                1 => Page::Records,
                                _ => self.last_device_page,
                            };
                        }
                    }
                    ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                        self.view_menu(ui);
                        if ui.available_width() > 65.0 && self.fixture_mode {
                            ui.label(
                                RichText::new("离线样机")
                                    .small()
                                    .color(Color32::from_rgb(243, 187, 102)),
                            );
                        }
                    });
                });
            });
        if rail {
            egui::Panel::left("studio_devices")
                .exact_size(144.0)
                .resizable(false)
                .frame(Frame::new().fill(SIDEBAR))
                .show(ui, |ui| {
                    if !self.bound_uid.is_empty() {
                        let selected = !matches!(self.page, Page::Connection | Page::Records);
                        let name = self
                            .snapshot
                            .active_configuration
                            .as_ref()
                            .map(|i| config::model_name(i.model))
                            .unwrap_or("设备");
                        if ui
                            .add_sized(
                                [ui.available_width(), 42.0],
                                egui::Button::new(
                                    RichText::new(format!("{}  ·  N{}", name, self.node))
                                        .strong()
                                        .color(if selected {
                                            Color32::WHITE
                                        } else {
                                            Color32::BLACK
                                        }),
                                )
                                .stroke(Stroke::NONE)
                                .corner_radius(0.0)
                                .fill(if selected {
                                    Color32::from_rgb(227, 99, 11)
                                } else {
                                    SIDEBAR
                                }),
                            )
                            .on_hover_text(&self.bound_uid)
                            .clicked()
                        {
                            self.page = self.last_device_page;
                        }
                        ui.with_layout(Layout::bottom_up(Align::Min), |ui| {
                            Frame::new().inner_margin(10.0).show(ui, |ui| {
                                ui.vertical(|ui| {
                                    let ink = Color32::from_rgb(59, 70, 84);
                                    ui.label(
                                        RichText::new(
                                            if !self.connected || !self.snapshot.identity_fresh() {
                                                "上次读取"
                                            } else if self.studio.is_some() {
                                                "示例身份"
                                            } else {
                                                "设备身份"
                                            },
                                        )
                                        .strong()
                                        .color(ink),
                                    );
                                    for line in [
                                        format!(
                                            "{}  /  N{}",
                                            config::axis_name(self.node),
                                            self.node
                                        ),
                                        format!("APP    {}", version(self.snapshot.app_version)),
                                        format!(
                                            "Loader  {}",
                                            version(self.snapshot.loader_version)
                                        ),
                                        format!("硬件   Rev {}", self.snapshot.hardware),
                                    ] {
                                        ui.label(RichText::new(line).small().color(ink));
                                    }
                                    if ui.link("设备详情").clicked() {
                                        self.page = Page::Home;
                                    }
                                });
                            });
                        });
                    }
                });
        }
        if self.page != Page::Connection || self.connected || self.hard_stop {
            egui::Panel::bottom("motor_tools")
                .frame(
                    Frame::new()
                        .fill(c.background)
                        .inner_margin(egui::Margin::symmetric(8, 4)),
                )
                .show(ui, |ui| {
                    if self.hard_stop {
                        ui.add(
                            egui::Label::new(RichText::new(&self.status).color(c.danger))
                                .truncate(),
                        )
                        .on_hover_text(&self.status);
                    }
                    ui.horizontal_wrapped(|ui| {
                        ui.spacing_mut().item_spacing = Vec2::new(8.0, 2.0);
                        self.fault_status_bar(ui);
                        let (link, color) = if self.connected && self.snapshot.identity_fresh() {
                            (
                                if self.fixture_mode {
                                    "示例连接"
                                } else {
                                    "已连接"
                                },
                                Color32::from_rgb(67, 177, 112),
                            )
                        } else {
                            ("未确认连接", c.muted)
                        };
                        ui.label(RichText::new(format!("● {link}")).color(color));
                        ui.label(
                            match self.snapshot.enable_state().filter(|_| self.connected) {
                                Some(true) => "● 已使能",
                                Some(false) => "○ 已失能",
                                None => "○ 输出未知",
                            },
                        );
                        let stop = ui.add_enabled(
                            self.connected && !self.fixture_mode && !self.upgrading,
                            egui::Button::new(
                                RichText::new("■  STOP").strong().color(Color32::WHITE),
                            )
                            .fill(Color32::from_rgb(171, 45, 58)),
                        );
                        self.control("stop", &stop);
                        if stop.clicked() {
                            self.send(Command::Stop);
                        }
                        ui.menu_button("设备状态", |ui| {
                            self.instrument_telemetry(ui);
                            ui.label(&self.status);
                            if ui.button("身份详情").clicked() {
                                self.page = Page::Home;
                                ui.close();
                            }
                        });
                    });
                });
        }
        if !matches!(self.page, Page::Connection | Page::Records) {
            egui::Panel::top("motor_tabs")
                .frame(
                    Frame::new()
                        .fill(c.background)
                        .inner_margin(egui::Margin::symmetric(9, 4)),
                )
                .show(ui, |ui| {
                    ui.horizontal_wrapped(|ui| {
                        ui.spacing_mut().item_spacing.x = 1.0;
                        let width = ((ui.available_width() - 8.0) / 4.0).clamp(60.0, 157.0);
                        for (i, (page, name)) in Page::WORKSPACES.into_iter().enumerate() {
                            let selected = page == self.page
                                || (page == Page::Calibration && self.page == Page::Identification);
                            let btn = ui.add_sized(
                                [width, 32.0],
                                egui::Button::new(
                                    RichText::new(name).strong().color(if selected {
                                        c.ink
                                    } else {
                                        c.muted
                                    }),
                                )
                                .frame(false),
                            );
                            if width >= 110.0 {
                                draw_nav_icon(
                                    ui.painter(),
                                    btn.rect.left_center() + egui::vec2(28.0, 0.0),
                                    i + 3,
                                    if selected { c.accent } else { c.muted },
                                );
                            }
                            if selected {
                                let p = btn.rect.center_bottom();
                                ui.painter().line_segment(
                                    [
                                        p - egui::vec2(width * 0.34, 0.0),
                                        p + egui::vec2(width * 0.34, 0.0),
                                    ],
                                    Stroke::new(2.0, c.accent),
                                );
                            }
                            if btn.clicked() {
                                self.page = page;
                            }
                        }
                        if matches!(self.page, Page::Calibration | Page::Identification)
                            && ui.small_button("辨识支持范围").clicked()
                        {
                            self.page = Page::Identification;
                        }
                    });
                });
        }
    }

    fn view_menu(&mut self, ui: &mut egui::Ui) {
        let view = egui::ComboBox::from_id_salt("studio_view")
            .height(490.0)
            .width(62.0)
            .selected_text("视图")
            .show_ui(ui, |ui| {
                ui.label("界面缩放");
                ui.horizontal(|ui| {
                    for pct in [100, 125, 150, 175, 200] {
                        let factor = pct as f32 / 100.0;
                        if ui
                            .selectable_label(
                                (ui.ctx().zoom_factor() - factor).abs() < 0.01,
                                format!("{pct}%"),
                            )
                            .clicked()
                        {
                            ui.ctx().set_zoom_factor(factor);
                        }
                    }
                });
                ui.separator();
                ui.label("配色");
                ui.horizontal(|ui| {
                    ui.selectable_value(&mut self.theme, egui::ThemePreference::System, "系统");
                    ui.selectable_value(&mut self.theme, egui::ThemePreference::Light, "浅色");
                    ui.selectable_value(&mut self.theme, egui::ThemePreference::Dark, "深色");
                });
                if let Some(scene) = self.studio {
                    ui.separator();
                    ui.label("离线状态预览");
                    for (option, label) in studio::Scenario::OPTIONS {
                        if ui.selectable_label(scene == option, label).clicked() {
                            self.set_studio_scenario(option);
                        }
                    }
                }
            });
        if self.fixture_mode && self.preview.as_ref().is_some_and(|p| p.theme_menu()) {
            egui::Popup::open_id(ui.ctx(), view.response.id.with("popup"));
        }
        ui.ctx().set_theme(self.theme);
    }

    pub(super) fn device_center(&mut self, ui: &mut egui::Ui) {
        let has_motor = !self.bound_uid.is_empty();
        ui.vertical_centered(|ui| {
            ui.add_space(7.0);
            Frame::new().fill(BLUE).corner_radius(4.0).show(ui, |ui| {
                ui.add_sized(
                    [160.0, 35.0],
                    egui::Label::new(RichText::new("设备管理").color(Color32::WHITE).strong()),
                );
            });
        });
        ui.add_space(16.0);
        let tile = 206.0_f32
            .min((ui.available_width() - 16.0) / 2.0)
            .max(115.0);
        let cols = if ui.available_width() >= 3.0 * tile + 32.0 {
            3.0
        } else {
            2.0
        };
        let offset = ((ui.available_width() - cols * tile - (cols - 1.0) * 16.0) * 0.5).max(0.0);
        ui.horizontal(|ui| {
            ui.spacing_mut().item_spacing.x = 16.0;
            if offset > 16.0 {
                ui.add_space(offset);
            }
            if has_motor {
                let model = self
                    .snapshot
                    .active_configuration
                    .as_ref()
                    .map(|m| config::model_name(m.model))
                    .unwrap_or("设备");
                let card = ui.add_sized(
                    [tile, tile * 1.04],
                    egui::Button::new("")
                        .fill(Color32::from_rgb(41, 41, 45))
                        .corner_radius(7.0)
                        .stroke(Stroke::NONE),
                );
                let center = card.rect.center() - Vec2::new(0.0, 18.0);
                let fresh = self.connected && self.snapshot.identity_fresh();
                let status_color = if fresh {
                    Color32::from_rgb(52, 214, 104)
                } else {
                    Color32::GRAY
                };
                ui.painter()
                    .circle_stroke(center, 42.0, Stroke::new(3.0, status_color));
                ui.painter()
                    .circle_stroke(center, 35.0, Stroke::new(1.0, Color32::GRAY));
                ui.painter().text(
                    center - egui::vec2(0.0, 7.0),
                    egui::Align2::CENTER_CENTER,
                    "M",
                    egui::FontId::proportional(25.0),
                    Color32::WHITE,
                );
                ui.painter().text(
                    center + egui::vec2(0.0, 15.0),
                    egui::Align2::CENTER_CENTER,
                    format!("N{}", self.node),
                    egui::FontId::monospace(13.0),
                    Color32::WHITE,
                );
                ui.painter().text(
                    card.rect.left_bottom() + egui::vec2(12.0, -31.0),
                    egui::Align2::LEFT_BOTTOM,
                    model,
                    egui::FontId::proportional(16.0),
                    Color32::WHITE,
                );
                ui.painter().text(
                    card.rect.left_bottom() + egui::vec2(12.0, -8.0),
                    egui::Align2::LEFT_BOTTOM,
                    if fresh {
                        "● 已连接"
                    } else {
                        "○ 历史设备"
                    },
                    egui::FontId::proportional(13.0),
                    status_color,
                );
                card.widget_info(|| {
                    egui::WidgetInfo::labeled(egui::WidgetType::Button, true, model)
                });
                if card.clicked() {
                    self.page = self.last_device_page;
                }
            }
            let add = ui.add_sized(
                [tile, tile * 1.04],
                egui::Button::new(
                    RichText::new("⊕    添加设备")
                        .size(18.0)
                        .strong()
                        .color(Color32::WHITE),
                )
                .fill(Color32::from_rgb(41, 41, 45))
                .stroke(Stroke::NONE)
                .corner_radius(7.0),
            );
            if add.clicked() {
                self.add_device_open = true;
                self.start_scan();
            }
        });
        ui.vertical_centered(|ui| {
            if ui.link("烧录整机固件（J-Link / SWD）").clicked() {
                self.page = Page::Maintenance;
                self.upgrade_mode = UpgradeMode::Combined;
            }
        });
    }
    pub(super) fn instrument_telemetry(&self, ui: &mut egui::Ui) {
        let fresh = self.connected && self.snapshot.feedback_fresh();
        let c = Palette::from_ctx(ui.ctx());
        ui.horizontal_wrapped(|ui| {
            for (name, value, unit) in [
                ("位置", self.snapshot.position_rad, "rad"),
                ("速度", self.snapshot.speed_rad_s, "rad/s"),
                ("Iq", self.snapshot.iq_a, "A"),
                ("母线", self.snapshot.bus_v, "V"),
                ("温度", self.snapshot.temperature_c, "°C"),
            ] {
                ui.label(RichText::new(name).small().color(c.muted));
                let text = value.map_or_else(
                    || "—".into(),
                    |v| format!("{v:.3} {unit}{}", if fresh { "" } else { " · 过期" }),
                );
                ui.label(RichText::new(text).small().monospace().color(if fresh {
                    c.ink
                } else {
                    c.muted
                }));
                ui.add_space(8.0);
            }
        });
    }
}

// 每个导航图符都独立绘制，沿用本地字体及 egui 的 DPI 变换。
fn draw_nav_icon(p: &egui::Painter, center: egui::Pos2, item: usize, ink: Color32) {
    let point = |x, y| center + egui::vec2(x, y);
    let stroke = Stroke::new(1.3, ink);
    let line = |a, b| {
        p.line_segment([a, b], stroke);
    };
    let rect = |x: f32, y: f32, w, h| {
        p.rect_stroke(
            egui::Rect::from_min_size(point(x, y), Vec2::new(w, h)),
            2.0,
            stroke,
            egui::StrokeKind::Inside,
        );
    };
    match item {
        0 => {
            for (x, y) in [(0., -6.), (-6., 5.), (6., 5.)] {
                p.circle_stroke(point(x, y), 2.6, stroke);
            }
            line(point(-1., -3.), point(-5., 2.));
            line(point(1., -3.), point(5., 2.));
            line(point(-3., 5.), point(3., 5.));
        }
        1 => {
            rect(-8., -6., 16., 12.);
            line(point(-5., -2.), point(-2., 0.));
            line(point(-2., 0.), point(-5., 2.));
            line(point(0., 3.), point(5., 3.));
        }
        2 => {
            rect(-7., -7., 14., 14.);
            for y in [-3., 0., 3.] {
                line(point(-4., y), point(4., y));
            }
        }
        3 => {
            for (y, x) in [(-5., -2.), (0., 3.), (5., -3.)] {
                line(point(-7., y), point(7., y));
                p.circle_filled(point(x, y), 2., ink);
            }
        }
        4 => {
            p.circle_stroke(center, 7., stroke);
            line(point(-3., 0.), point(-1., 3.));
            line(point(-1., 3.), point(4., -3.));
        }
        5 => {
            p.circle_stroke(center, 7., stroke);
            line(point(-5., 1.), point(-2., 1.));
            line(point(-2., 1.), point(0., -3.));
            line(point(0., -3.), point(2., 3.));
            line(point(2., 3.), point(5., 0.));
        }
        _ => {
            line(point(0., 5.), point(0., -6.));
            line(point(-4., -2.), point(0., -6.));
            line(point(0., -6.), point(4., -2.));
            line(point(-6., 7.), point(6., 7.));
        }
    }
}
