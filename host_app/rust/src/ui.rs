//! 会话壳与命令边界；页面共享会话，显式预览永远不能打开 SDK 或发送命令。

mod pages;
mod preview;
mod scope;

use crate::capture::Capture;
use crate::device::{self, Command, Event, Snapshot};
use crate::package::UpgradePackage;
use crate::target::Target;
use eframe::egui::{self, Align, Color32, Frame, Layout, RichText, Stroke, Vec2};
use std::path::PathBuf;
use std::sync::mpsc::{self, Receiver, Sender};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Page {
    Connection,
    Home,
    Workbench,
    Parameters,
    Calibration,
    Identification,
    Records,
    Maintenance,
}

impl Page {
    const WORKSPACES: [(Self, &'static str); 7] = [
        (Self::Home, "首页"),
        (Self::Workbench, "调试工作台"),
        (Self::Parameters, "参数配置"),
        (Self::Calibration, "标定"),
        (Self::Identification, "辨识"),
        (Self::Records, "记录与诊断"),
        (Self::Maintenance, "设备维护"),
    ];
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum UpgradeMode {
    App,
    Loader,
    Combined,
}

#[derive(Clone, Copy)]
struct Palette {
    background: Color32,
    panel: Color32,
    ink: Color32,
    muted: Color32,
    line: Color32,
    accent: Color32,
    danger: Color32,
    warning: Color32,
}

impl Palette {
    fn from_ctx(ctx: &egui::Context) -> Self {
        Self::for_theme(ctx.theme())
    }

    fn for_theme(theme: egui::Theme) -> Self {
        let c = Color32::from_rgb;
        if theme == egui::Theme::Dark {
            Self {
                background: c(24, 24, 24),
                panel: c(31, 31, 31),
                ink: c(220, 220, 220),
                muted: c(150, 156, 165),
                line: c(48, 48, 48),
                accent: c(75, 163, 224),
                danger: c(244, 135, 135),
                warning: c(218, 183, 113),
            }
        } else {
            Self {
                background: c(243, 243, 243),
                panel: Color32::WHITE,
                ink: c(36, 50, 70),
                muted: c(94, 109, 128),
                line: c(214, 222, 232),
                accent: c(32, 94, 207),
                danger: c(177, 46, 59),
                warning: c(136, 96, 22),
            }
        }
    }
}

pub enum PreviewRequest {
    Single(PathBuf),
    Gallery(PathBuf),
}

pub struct HostApp {
    target: Target,
    page: Page,
    dll: String,
    device_index: u32,
    node: u8,
    manual_sequence: u16,
    package_dir: String,
    package: Option<UpgradePackage>,
    upgrade_mode: UpgradeMode,
    snapshot: Snapshot,
    bound_uid: String,
    connected: bool,
    acknowledged: bool,
    intervention_ack: bool,
    hard_stop: bool,
    speed_rad_s: f64,
    progress: f32,
    upgrading: bool,
    status: String,
    log: Vec<String>,
    command_tx: Option<Sender<Command>>,
    event_rx: Option<Receiver<Event>>,
    pending_close: bool,
    capture: Capture,
    window_s: f64,
    cursor_s: Option<f64>,
    show_reference: bool,
    task_index: usize,
    theme: egui::ThemePreference,
    fixture_mode: bool,
    preview: Option<preview::PreviewRun>,
    control_evidence: Vec<serde_json::Value>,
}

impl HostApp {
    pub fn new(
        cc: &eframe::CreationContext<'_>,
        target: Target,
        request: Option<PreviewRequest>,
    ) -> Self {
        for theme in [egui::Theme::Light, egui::Theme::Dark] {
            let colors = Palette::for_theme(theme);
            let mut style = (*cc.egui_ctx.style_of(theme)).clone();
            if request.is_some() {
                style.animation_time = 0.0;
            }
            style.visuals = if theme == egui::Theme::Dark {
                egui::Visuals::dark()
            } else {
                egui::Visuals::light()
            };
            style.spacing.item_spacing = Vec2::new(10.0, 8.0);
            style.spacing.button_padding = Vec2::new(12.0, 6.0);
            style.spacing.interact_size.y = 28.0;
            style.visuals.override_text_color = Some(colors.ink);
            style.visuals.panel_fill = colors.background;
            style.visuals.window_fill = colors.panel;
            style.visuals.extreme_bg_color = colors.background;
            style.visuals.widgets.noninteractive.bg_stroke = Stroke::new(1.0, colors.line);
            style.visuals.widgets.noninteractive.corner_radius = 2.into();
            style.visuals.widgets.inactive.corner_radius = 2.into();
            style.visuals.widgets.hovered.corner_radius = 2.into();
            style.visuals.widgets.active.corner_radius = 2.into();
            style.visuals.widgets.inactive.bg_fill = colors.line;
            style.visuals.widgets.inactive.weak_bg_fill = colors.line;
            style.visuals.widgets.inactive.bg_stroke = Stroke::new(1.0, colors.line);
            style.visuals.widgets.inactive.fg_stroke = Stroke::new(1.0, colors.ink);
            style.visuals.widgets.hovered.bg_fill = colors.line;
            style.visuals.widgets.hovered.weak_bg_fill = colors.line;
            style.visuals.widgets.hovered.bg_stroke = Stroke::new(1.0, colors.accent);
            style.visuals.selection.bg_fill = colors.accent.gamma_multiply(0.16);
            style.visuals.selection.stroke = Stroke::new(1.0, colors.accent);
            for (kind, size) in [
                (egui::TextStyle::Body, 14.0),
                (egui::TextStyle::Button, 14.0),
                (egui::TextStyle::Small, 12.0),
                (egui::TextStyle::Heading, 17.0),
            ] {
                style
                    .text_styles
                    .insert(kind, egui::FontId::proportional(size));
            }
            cc.egui_ctx.set_style_of(theme, style);
        }
        if let Some(windows) = std::env::var_os("WINDIR") {
            for font in ["msyh.ttc", "simhei.ttf"] {
                if let Ok(bytes) = std::fs::read(PathBuf::from(&windows).join("Fonts").join(font)) {
                    let mut fonts = egui::FontDefinitions::default();
                    fonts
                        .font_data
                        .insert("chinese".into(), egui::FontData::from_owned(bytes).into());
                    for family in [egui::FontFamily::Proportional, egui::FontFamily::Monospace] {
                        fonts
                            .families
                            .entry(family)
                            .or_default()
                            .push("chinese".into());
                    }
                    cc.egui_ctx.set_fonts(fonts);
                    break;
                }
            }
        }
        Self::disconnected(target, request)
    }

    fn disconnected(target: Target, request: Option<PreviewRequest>) -> Self {
        Self {
            target,
            page: Page::Connection,
            dll: String::new(),
            device_index: 0,
            node: 1,
            manual_sequence: 0,
            package_dir: String::new(),
            package: None,
            upgrade_mode: UpgradeMode::App,
            snapshot: Snapshot::default(),
            bound_uid: String::new(),
            connected: false,
            acknowledged: false,
            intervention_ack: false,
            hard_stop: false,
            speed_rad_s: 0.0,
            progress: 0.0,
            upgrading: false,
            status: "未连接 · 设备状态未知".into(),
            log: vec!["工作台就绪，等待显式连接。".into()],
            command_tx: None,
            event_rx: None,
            pending_close: false,
            capture: Capture::default(),
            window_s: 30.0,
            cursor_s: None,
            show_reference: true,
            task_index: 0,
            theme: egui::ThemePreference::Dark,
            fixture_mode: request.is_some(),
            preview: request.map(preview::PreviewRun::new),
            control_evidence: vec![],
        }
    }

    fn push_log(&mut self, message: impl Into<String>) {
        self.log.push(message.into());
        if self.log.len() > 200 {
            self.log.remove(0);
        }
    }
    fn note(&mut self, message: impl Into<String>) {
        self.status = message.into();
        self.push_log(self.status.clone());
    }
    fn poll_events(&mut self) {
        let events: Vec<_> = self
            .event_rx
            .as_ref()
            .map(|rx| rx.try_iter().collect())
            .unwrap_or_default();
        for event in events {
            match event {
                Event::Snapshot(snapshot) => {
                    let snapshot = *snapshot;
                    if !self.bound_uid.is_empty() && snapshot.uid != self.bound_uid {
                        self.hard_stop = true;
                        self.acknowledged = false;
                        self.capture.pause();
                        self.note("UID 已改变；禁止继续控制，须现场检查输出");
                    } else if self.bound_uid.is_empty() && !snapshot.uid.is_empty() {
                        self.bound_uid = snapshot.uid.clone();
                        self.note(format!("绑定 UID {}", self.bound_uid));
                        if self.page == Page::Connection {
                            self.page = if snapshot.role == 2 {
                                Page::Maintenance
                            } else {
                                Page::Home
                            };
                        }
                    }
                    self.capture.accept(&snapshot);
                    self.snapshot = snapshot;
                }
                Event::Status(message) => self.note(message),
                Event::Progress(value) => self.progress = value,
                Event::UpgradeDone => self.upgrading = false,
                Event::HardStop(message) => {
                    self.upgrading = false;
                    self.pending_close = false;
                    self.hard_stop = true;
                    self.acknowledged = false;
                    self.capture.pause();
                    self.note(message);
                }
                Event::Disconnected => {
                    self.upgrading = false;
                    self.connected = false;
                    self.acknowledged = false;
                    self.capture.pause();
                    self.command_tx = None;
                    self.event_rx = None;
                    if !self.hard_stop {
                        self.status = "已断开 · 最后读数过期，设备当前状态未知".into();
                    }
                    self.push_log("CAN 端口已关闭；历史曲线保留，未自动重连或重放命令。");
                }
            }
        }
    }
    fn connect(&mut self) {
        if self.fixture_mode
            || self.connected
            || self.dll.trim().is_empty()
            || (self.hard_stop && !self.intervention_ack)
        {
            return;
        }
        let (command_tx, command_rx) = mpsc::channel();
        let (event_tx, event_rx) = mpsc::channel();
        let dll = PathBuf::from(&self.dll);
        let device_index = self.device_index;
        let node = self.node;
        let sequence = self.manual_sequence;
        let target = self.target.clone();
        std::thread::spawn(move || {
            device::run(
                &dll,
                device_index,
                node,
                sequence,
                target,
                command_rx,
                event_tx,
            )
        });
        self.command_tx = Some(command_tx);
        self.event_rx = Some(event_rx);
        self.connected = true;
        self.hard_stop = false;
        self.intervention_ack = false;
        self.bound_uid.clear();
        self.snapshot = Snapshot::default();
        self.capture.pause();
        self.acknowledged = false;
        self.speed_rad_s = 0.0;
        self.note(format!(
            "连接 CAN 设备 {device_index} / 节点 {node}，正在识别设备；控制序号起点 {sequence}"
        ));
    }
    fn send(&mut self, command: Command) {
        if self.fixture_mode {
            return;
        }
        if self
            .command_tx
            .as_ref()
            .is_none_or(|tx| tx.send(command).is_err())
        {
            self.hard_stop = true;
            self.acknowledged = false;
            self.note("设备线程不可用，须现场核对 STOP 和输出");
        }
    }
    fn inspect_package(&mut self) {
        match UpgradePackage::load(&PathBuf::from(&self.package_dir), &self.target) {
            Ok(package) => {
                self.note(format!(
                    "APP {} · {} B · CRC32 {:08X} · layout {} 校验通过",
                    version(package.version),
                    package.image.len(),
                    package.crc32,
                    self.target.layout
                ));
                self.package = Some(package);
            }
            Err(error) => {
                self.package = None;
                self.note(format!("APP 包拒绝：{error:#}"));
            }
        }
    }
    fn export_capture(&mut self) {
        let result = (|| -> anyhow::Result<PathBuf> {
            let dir = std::env::current_dir()?
                .join("outputs")
                .join("host-captures");
            std::fs::create_dir_all(&dir)?;
            let stamp = SystemTime::now().duration_since(UNIX_EPOCH)?.as_millis();
            let path = dir.join(format!("capture-{stamp}.csv"));
            self.capture.write_csv(std::fs::File::create(&path)?)?;
            Ok(path)
        })();
        match result {
            Ok(path) => self.note(format!("已导出记录：{}", path.display())),
            Err(error) => self.note(format!("记录导出失败：{error:#}")),
        }
    }
    fn panel<R>(ui: &mut egui::Ui, title: &str, content: impl FnOnce(&mut egui::Ui) -> R) -> R {
        let c = Palette::from_ctx(ui.ctx());
        Frame::new()
            .fill(c.panel)
            .stroke(Stroke::new(1.0, c.line))
            .corner_radius(2.0)
            .inner_margin(12.0)
            .show(ui, |ui| {
                ui.spacing_mut().item_spacing.y = 6.0;
                ui.label(RichText::new(title).size(13.0).strong());
                ui.separator();
                content(ui)
            })
            .inner
    }
    fn control(&mut self, name: &str, response: &egui::Response) {
        if self.fixture_mode {
            self.control_evidence.push(serde_json::json!({ "name":name,
            "enabled":response.enabled(), "rect":[response.rect.min.x,response.rect.min.y,response.rect.max.x,response.rect.max.y] }));
        }
    }
    fn header(&mut self, ui: &mut egui::Ui) {
        let c = Palette::from_ctx(ui.ctx());
        ui.horizontal(|ui| {
            Frame::new()
                .fill(c.accent)
                .corner_radius(3.0)
                .inner_margin(egui::Margin::symmetric(6, 3))
                .show(ui, |ui| {
                    ui.label(
                        RichText::new("YG")
                            .color(Color32::WHITE)
                            .strong()
                            .size(13.0),
                    );
                });
            ui.label(RichText::new("YG_MC_HOST").strong().size(14.0));
            ui.add_space(8.0);
            ui.separator();
            let device = if self.bound_uid.is_empty() {
                "未识别设备".into()
            } else {
                format!(
                    "节点 {} · {}",
                    self.node,
                    &self.bound_uid[..self.bound_uid.len().min(12)]
                )
            };
            ui.label(RichText::new(device).color(c.muted).size(12.0))
                .on_hover_text(&self.bound_uid);
            let state = if self.fixture_mode {
                "离线预览"
            } else if self.hard_stop {
                "硬停 · 需现场检查"
            } else if self.upgrading {
                "升级事务进行中"
            } else if !self.connected {
                "未连接"
            } else if !self.snapshot.identity_fresh() {
                "识别中 / 身份过期"
            } else if self.snapshot.role == 2 {
                "Loader"
            } else if !self.snapshot.compatible(&self.target, &self.bound_uid) {
                "不兼容 · 禁止控制"
            } else if !self.snapshot.feedback_fresh() {
                "反馈过期"
            } else {
                "APP · 已识别"
            };
            ui.label(
                RichText::new(format!("●  {state}"))
                    .small()
                    .color(if self.hard_stop { c.danger } else { c.muted }),
            );
            ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                let stop = ui.add_enabled(
                    self.connected && !self.fixture_mode && !self.upgrading,
                    egui::Button::new(RichText::new("■  STOP").color(c.danger).strong())
                        .stroke(Stroke::new(1.0, c.danger.gamma_multiply(0.6)))
                        .min_size(Vec2::new(92.0, 28.0)),
                );
                self.control("stop", &stop);
                if stop.clicked() {
                    self.send(Command::Stop);
                    self.push_log("已请求 STOP，等待最终回复与状态。");
                }
                if ui.add(egui::Button::new("连接管理").frame(false)).clicked() {
                    self.page = Page::Connection;
                }
                let theme_menu = egui::ComboBox::from_id_salt("theme")
                    .width(72.0)
                    .selected_text(match self.theme {
                        egui::ThemePreference::System => "跟随系统",
                        egui::ThemePreference::Light => "浅色",
                        egui::ThemePreference::Dark => "深色",
                    })
                    .show_ui(ui, |ui| {
                        ui.selectable_value(
                            &mut self.theme,
                            egui::ThemePreference::System,
                            "跟随系统",
                        );
                        ui.selectable_value(&mut self.theme, egui::ThemePreference::Light, "浅色");
                        ui.selectable_value(&mut self.theme, egui::ThemePreference::Dark, "深色");
                    });
                if self.fixture_mode && self.preview.as_ref().is_some_and(|p| p.theme_menu()) {
                    egui::Popup::open_id(ui.ctx(), theme_menu.response.id.with("popup"));
                }
                ui.ctx().set_theme(self.theme);
            });
        });
        if self.hard_stop {
            ui.label(RichText::new(&self.status).color(c.danger));
        } else if self.upgrading {
            ui.label(
                RichText::new("升级独占通道，设备保持停机；等待结果后可断开或关闭。")
                    .small()
                    .color(c.warning),
            );
        } else if !self.connected && !self.bound_uid.is_empty() {
            ui.label(
                RichText::new("链路已断开；保留最后观测与记录，设备当前状态未知。")
                    .small()
                    .color(c.warning),
            );
        }
    }
}

impl eframe::App for HostApp {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        self.poll_events();
        if let Some(mut preview) = self.preview.take() {
            preview.tick(self, ui.ctx());
            self.preview = Some(preview);
        }
        self.control_evidence.clear();
        if self.pending_close && !self.connected && !self.hard_stop {
            ui.ctx().send_viewport_cmd(egui::ViewportCommand::Close);
        }
        if self.connected && ui.ctx().input(|i| i.viewport().close_requested()) {
            ui.ctx()
                .send_viewport_cmd(egui::ViewportCommand::CancelClose);
            if self.upgrading {
                self.status = "升级事务进行中，等待结果后关闭".into();
            } else if !self.pending_close {
                self.pending_close = true;
                self.send(Command::Disconnect);
                self.status = "等待 STOP 核对并断开".into();
            }
        }
        ui.ctx().request_repaint_after(Duration::from_millis(100));
        let c = Palette::from_ctx(ui.ctx());
        egui::Panel::top("device")
            .frame(
                Frame::new()
                    .fill(c.panel)
                    .inner_margin(egui::Margin::symmetric(14, 7)),
            )
            .show(ui, |ui| self.header(ui));
        if self.page != Page::Connection {
            egui::Panel::top("navigation")
                .frame(Frame::new().fill(c.background).inner_margin(0.0))
                .show(ui, |ui| {
                    ui.horizontal(|ui| {
                        ui.spacing_mut().item_spacing.x = 0.0;
                        for (page, title) in Page::WORKSPACES {
                            let selected = self.page == page;
                            let width = title.chars().count() as f32 * 14.0 + 40.0;
                            let (rect, response) = ui
                                .allocate_exact_size(Vec2::new(width, 38.0), egui::Sense::click());
                            response.widget_info(|| {
                                egui::WidgetInfo::selected(
                                    egui::WidgetType::SelectableLabel,
                                    true,
                                    selected,
                                    title,
                                )
                            });
                            if selected || response.hovered() || response.has_focus() {
                                ui.painter().rect_filled(rect, 0.0, c.panel);
                            }
                            ui.painter().line_segment(
                                [rect.right_top(), rect.right_bottom()],
                                Stroke::new(1.0, c.line),
                            );
                            ui.painter().text(
                                rect.center(),
                                egui::Align2::CENTER_CENTER,
                                title,
                                egui::FontId::proportional(13.0),
                                if selected { c.ink } else { c.muted },
                            );
                            if selected {
                                ui.painter().line_segment(
                                    [rect.left_top(), rect.right_top()],
                                    Stroke::new(2.0, c.accent),
                                );
                            }
                            if response.clicked() {
                                self.page = page;
                            }
                            if response.has_focus() {
                                ui.painter().rect_stroke(
                                    rect.shrink(3.0),
                                    0.0,
                                    Stroke::new(1.0, c.accent),
                                    egui::StrokeKind::Inside,
                                );
                            }
                        }
                    });
                });
        }
        egui::Panel::bottom("status")
            .frame(
                Frame::new()
                    .fill(c.panel)
                    .inner_margin(egui::Margin::symmetric(12, 4)),
            )
            .show(ui, |ui| {
                let text = if self.fixture_mode {
                    "离线预览 / 未连接硬件 / 示例曲线有来源标记"
                } else {
                    &self.status
                };
                ui.horizontal(|ui| {
                    ui.label(RichText::new("●").color(c.accent).small());
                    ui.add(egui::Label::new(RichText::new(text).small().color(c.muted)).truncate())
                        .on_hover_text(&self.status);
                    ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                        ui.label(
                            RichText::new(concat!(
                                "G431   ·   CAN FD   ·   v",
                                env!("CARGO_PKG_VERSION")
                            ))
                            .small()
                            .color(c.muted),
                        );
                    });
                });
            });
        egui::CentralPanel::default()
            .frame(Frame::new().fill(c.background).inner_margin(14.0))
            .show(ui, |ui| match self.page {
                Page::Workbench => self.workbench(ui),
                Page::Connection => self.connection(ui),
                Page::Home => self.home(ui),
                Page::Parameters => self.parameters(ui),
                Page::Calibration => self.calibration(ui),
                Page::Identification => self.identification(ui),
                Page::Records => self.records(ui),
                Page::Maintenance => self.maintenance(ui),
            });
    }
}

fn version(v: u32) -> String {
    if v == 0 {
        return "未提供".into();
    }
    format!("v{}.{}.{}", v >> 16, (v >> 8) & 255, v & 255)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn complete_navigation_has_seven_workspaces_and_separate_connection() {
        assert_eq!(Page::WORKSPACES.len(), 7);
        assert!(!Page::WORKSPACES.iter().any(|(p, _)| *p == Page::Connection));
        for (index, (page, _)) in Page::WORKSPACES.iter().enumerate() {
            assert!(
                !Page::WORKSPACES[index + 1..]
                    .iter()
                    .any(|(other, _)| other == page)
            );
        }
    }

    #[test]
    fn disconnect_retains_history_and_last_values_but_revokes_control() {
        let mut app = HostApp::disconnected(Target::bundled().unwrap(), None);
        let (tx, rx) = mpsc::channel();
        app.event_rx = Some(rx);
        app.connected = true;
        app.acknowledged = true;
        app.capture.start("A").unwrap();
        tx.send(Event::Snapshot(Box::new(Snapshot {
            uid: "A".into(),
            role: 1,
            speed_rad_s: Some(3.0),
            feedback_at: Some(std::time::Instant::now()),
            ..Default::default()
        })))
        .unwrap();
        tx.send(Event::Disconnected).unwrap();
        app.poll_events();
        assert_eq!(app.snapshot.speed_rad_s, Some(3.0));
        assert_eq!(app.capture.samples.len(), 1);
        assert!(!app.connected && !app.acknowledged && !app.capture.observing);
        assert_eq!(app.page, Page::Home);
    }

    #[test]
    fn preview_cannot_connect_or_enqueue_physical_commands() {
        let mut app = HostApp::disconnected(
            Target::bundled().unwrap(),
            Some(PreviewRequest::Single("preview.png".into())),
        );
        let (tx, rx) = mpsc::channel();
        app.dll = "should-never-be-loaded.dll".into();
        app.command_tx = Some(tx);
        app.connect();
        app.send(Command::Start { acknowledged: true });
        assert!(!app.connected);
        assert!(rx.try_recv().is_err());
    }
}
