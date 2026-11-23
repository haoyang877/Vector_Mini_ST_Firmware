//! 会话壳与命令边界；页面共享会话，显式预览永远不能打开 SDK 或发送命令。

mod bench;
mod calibration;
mod control_panel;
mod faults;
mod instrument;
mod motor_configuration;
mod pages;
mod parameters;
mod preview;
mod scope;
mod studio;
pub(crate) mod widgets;

use crate::can::Bitrates;
use crate::capture::Capture;
use crate::control::ControlMode;
use crate::device::{self, Command, Event, Snapshot};
use crate::discovery::{self, ScanEvent};
use crate::package::UpgradePackage;
use crate::target::Target;
use eframe::egui::{self, Align, Color32, Frame, Layout, RichText, Stroke, Vec2};
use std::path::PathBuf;
use std::sync::mpsc::{self, Receiver, Sender};
use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
};
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
    const WORKSPACES: [(Self, &'static str); 4] = [
        (Self::Parameters, "参数设置"),
        (Self::Calibration, "辨识校准"),
        (Self::Workbench, "运动控制"),
        (Self::Maintenance, "固件升级"),
    ];
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum UpgradeMode {
    App,
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
                background: c(18, 19, 26),
                panel: c(40, 40, 49),
                ink: c(226, 232, 240),
                muted: c(154, 167, 181),
                line: c(53, 65, 80),
                accent: c(87, 173, 238),
                danger: c(244, 135, 135),
                warning: c(218, 183, 113),
            }
        } else {
            Self {
                background: c(242, 245, 249),
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
    Interactive,
    Single(PathBuf),
    Gallery(PathBuf),
}

pub struct BenchRequest {
    pub uid: String,
    pub output: PathBuf,
}

pub struct HostApp {
    maintenance_panel: crate::maintenance::Panel,
    target: Target,
    page: Page,
    dll: String,
    device_index: u32,
    bitrates: Bitrates,
    scan_rx: Option<Receiver<ScanEvent>>,
    scan_cancel: Arc<AtomicBool>,
    scan_adapters: Vec<(u32, String)>,
    scan_devices: Vec<discovery::Device>,
    scan_selected: Option<usize>,
    scan_status: String,
    manual_connect: bool,
    add_device_open: bool,
    device_list_open: bool,
    last_device_page: Page,
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
    control_mode: ControlMode,
    control_targets: [f64; 4],
    motor_draft: crate::motor_configuration::Draft,
    motor_thermal: String,
    motor_acknowledged: bool,
    configuration_busy: bool,
    operation_pending: bool,
    parameter_state: parameters::State,
    progress: f32,
    upgrading: bool,
    status: String,
    log: Vec<String>,
    diagnostics: Option<crate::diagnostics::Logger>,
    command_tx: Option<Sender<Command>>,
    event_rx: Option<Receiver<Event>>,
    pending_close: bool,
    capture: Capture,
    curve_worker: Option<scope::Worker>,
    bench: Option<bench::Run>,
    window_s: f64,
    cursor_s: Option<f64>,
    show_reference: bool,
    task_index: usize,
    theme: egui::ThemePreference,
    fixture_mode: bool,
    studio: Option<studio::Scenario>,
    preview: Option<preview::PreviewRun>,
    control_evidence: Vec<serde_json::Value>,
}

impl Drop for HostApp {
    fn drop(&mut self) {
        if let Some(log) = &self.diagnostics {
            log.flush().ok();
        }
    }
}

impl HostApp {
    pub fn new(
        cc: &eframe::CreationContext<'_>,
        target: Target,
        request: Option<PreviewRequest>,
        bench: Option<BenchRequest>,
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
        let mut app = Self::disconnected(target, request);
        if !app.fixture_mode {
            let log = crate::diagnostics::Logger::new();
            log.event(
                "info",
                "application_started",
                serde_json::json!({"target":app.target.name}),
            );
            app.diagnostics = Some(log);
        }
        app.curve_worker = Some(scope::Worker::new(cc.egui_ctx.clone()));
        if app.studio.is_some() {
            app.init_studio();
        }
        app.bench = bench.map(bench::Run::new);
        app
    }

    fn disconnected(target: Target, request: Option<PreviewRequest>) -> Self {
        Self {
            maintenance_panel: crate::maintenance::Panel::default(),
            target,
            page: Page::Connection,
            dll: Self::find_sdk().unwrap_or_default(),
            device_index: 0,
            bitrates: Bitrates::default(),
            scan_rx: None,
            scan_cancel: Arc::new(AtomicBool::new(false)),
            scan_adapters: vec![],
            scan_devices: vec![],
            scan_selected: None,
            scan_status: "点击扫描，发现 CAN 适配器和设备。".into(),
            manual_connect: false,
            add_device_open: false,
            device_list_open: true,
            last_device_page: Page::Workbench,
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
            control_mode: ControlMode::Speed,
            control_targets: [0.0; 4],
            motor_draft: crate::motor_configuration::Draft::default(),
            motor_thermal: String::new(),
            motor_acknowledged: false,
            configuration_busy: false,
            operation_pending: false,
            parameter_state: parameters::State::default(),
            progress: 0.0,
            upgrading: false,
            status: "未连接 · 设备状态未知".into(),
            log: vec!["工作台就绪，等待显式连接。".into()],
            diagnostics: None,
            command_tx: None,
            event_rx: None,
            pending_close: false,
            capture: Capture::default(),
            curve_worker: None,
            bench: None,
            window_s: 30.0,
            cursor_s: None,
            show_reference: true,
            task_index: 0,
            theme: egui::ThemePreference::Dark,
            fixture_mode: request.is_some(),
            studio: matches!(request, Some(PreviewRequest::Interactive))
                .then_some(studio::Scenario::Disconnected),
            preview: request.and_then(preview::PreviewRun::new),
            control_evidence: vec![],
        }
    }

    fn push_log(&mut self, message: impl Into<String>) {
        let message = message.into();
        if let Some(log) = &self.diagnostics {
            log.event(
                "info",
                "ui_status",
                serde_json::json!({"message":message,"uid":self.bound_uid,"node":self.node}),
            );
        }
        self.log.push(message);
        if self.log.len() > 200 {
            self.log.remove(0);
        }
    }
    fn find_sdk() -> Option<String> {
        let dir = std::env::current_exe().ok()?.parent()?.to_path_buf();
        [
            dir.join("ControlCANFD.dll"),
            dir.join("drivers").join("ControlCANFD.dll"),
        ]
        .into_iter()
        .find(|path| path.is_file())
        .map(|path| path.to_string_lossy().into_owned())
    }
    fn start_scan(&mut self) {
        if self.fixture_mode
            || self.connected
            || self.scan_rx.is_some()
            || self.dll.trim().is_empty()
            || self.maintenance_panel.busy()
            || (self.hard_stop && !self.intervention_ack)
        {
            return;
        }
        self.scan_devices.clear();
        self.scan_adapters.clear();
        self.scan_selected = None;
        self.manual_connect = false;
        self.scan_status = "正在查找适配器…".into();
        self.scan_cancel.store(false, Ordering::Relaxed);
        let cancel = self.scan_cancel.clone();
        let path = PathBuf::from(&self.dll);
        let bitrates = self.bitrates;
        let (tx, rx) = mpsc::channel();
        self.scan_rx = Some(rx);
        std::thread::spawn(move || discovery::scan(&path, bitrates, cancel, tx));
    }
    fn poll_scan(&mut self) {
        let mut events = Vec::new();
        let mut disconnected = false;
        if let Some(rx) = &self.scan_rx {
            loop {
                match rx.try_recv() {
                    Ok(event) => events.push(event),
                    Err(mpsc::TryRecvError::Empty) => break,
                    Err(mpsc::TryRecvError::Disconnected) => {
                        disconnected = true;
                        break;
                    }
                }
            }
        }
        for event in events {
            match event {
                ScanEvent::Progress(message) => self.scan_status = message,
                ScanEvent::Adapter(index, name) => self.scan_adapters.push((index, name)),
                ScanEvent::Device(device) => self.scan_devices.push(device),
                ScanEvent::Finished(message) => {
                    self.scan_status = message;
                    self.scan_rx = None;
                }
            }
        }
        if disconnected && self.scan_rx.is_some() {
            self.scan_status = "扫描线程异常退出；请检查 SDK 和设备。".into();
            self.scan_rx = None;
        }
    }
    fn pause_capture(&mut self) {
        self.capture.pause();
        if let Some(worker) = &self.curve_worker {
            worker.pause();
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
                Event::OperationDone => self.operation_pending = false,
                Event::Snapshot(snapshot) => {
                    let snapshot = *snapshot;
                    if self.snapshot.active_configuration.is_none()
                        && (self.bound_uid.is_empty() || self.bound_uid == snapshot.uid)
                        && let Some(info) = &snapshot.active_configuration
                        && let Some(model) = crate::motor_configuration::Model::ALL
                            .into_iter()
                            .find(|model| model.id() == info.model)
                    {
                        self.motor_draft.model = model;
                        self.motor_draft.node = info.node;
                        self.motor_draft.speed_rad_s = info.speed_rad_s.max(0.001);
                        self.motor_thermal = info
                            .thermal
                            .iter()
                            .map(|value| format!("{value:.6}"))
                            .collect::<Vec<_>>()
                            .join(" ");
                        self.motor_acknowledged = false;
                    }
                    if self
                        .snapshot
                        .active_configuration
                        .as_ref()
                        .map(|info| info.revision)
                        != snapshot
                            .active_configuration
                            .as_ref()
                            .map(|info| info.revision)
                    {
                        self.motor_acknowledged = false;
                    }
                    if !self.bound_uid.is_empty() && snapshot.uid != self.bound_uid {
                        self.hard_stop = true;
                        self.acknowledged = false;
                        self.pause_capture();
                        self.note("UID 已改变；禁止继续控制，须现场检查输出");
                    } else if self.bound_uid.is_empty() && !snapshot.uid.is_empty() {
                        self.bound_uid = snapshot.uid.clone();
                        self.note(format!("绑定 UID {}", self.bound_uid));
                        if self.page == Page::Connection {
                            self.page = if snapshot.role == 2 {
                                Page::Maintenance
                            } else {
                                Page::Workbench
                            };
                        }
                    }
                    self.capture.accept(&snapshot);
                    self.snapshot = snapshot;
                }
                Event::ConfigurationActivated(node) => {
                    self.configuration_busy = false;
                    self.node = node;
                    self.scan_selected = None;
                    self.scan_devices.clear();
                    self.manual_connect = true;
                    self.acknowledged = false;
                    self.motor_acknowledged = false;
                    self.pause_capture();
                }
                Event::Parameters(report) => self.receive_parameters(report),
                Event::Status(message) => self.note(message),
                Event::Rejected(message) => {
                    self.parameter_state.cancel_read();
                    self.operation_pending = false;
                    self.configuration_busy = false;
                    self.upgrading = false;
                    self.note(message);
                }
                Event::ConfigurationSaved => {
                    self.configuration_busy = false;
                }
                Event::Progress(value) => self.progress = value,
                Event::UpgradeDone => self.upgrading = false,
                Event::HardStop(message) => {
                    self.parameter_state.cancel_read();
                    self.operation_pending = false;
                    self.configuration_busy = false;
                    self.upgrading = false;
                    self.pending_close = false;
                    self.hard_stop = true;
                    self.acknowledged = false;
                    self.motor_acknowledged = false;
                    self.pause_capture();
                    self.note(message);
                }
                Event::Disconnected => {
                    self.parameter_state.cancel_read();
                    self.operation_pending = false;
                    self.configuration_busy = false;
                    self.upgrading = false;
                    self.connected = false;
                    self.acknowledged = false;
                    self.motor_acknowledged = false;
                    self.pause_capture();
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
            || self.scan_rx.is_some()
            || self.maintenance_panel.busy()
            || self.dll.trim().is_empty()
            || (self.hard_stop && !self.intervention_ack)
        {
            return;
        }
        let (command_tx, command_rx) = mpsc::channel();
        let (event_tx, event_rx) = mpsc::channel();
        let (device_event_tx, device_event_rx) = mpsc::channel::<Event>();
        let curve_sink = self.curve_worker.as_ref().map(scope::Worker::sink);
        // 快照直接分发给曲线线程；不等待 UI 下一帧处理事件。
        std::thread::spawn(move || {
            while let Ok(event) = device_event_rx.recv() {
                if let Event::Snapshot(snapshot) = &event
                    && let Some(sink) = &curve_sink
                {
                    sink.publish(snapshot.clone());
                }
                if event_tx.send(event).is_err() {
                    break;
                }
            }
        });
        let dll = PathBuf::from(&self.dll);
        let device_index = self.device_index;
        let node = self.node;
        let sequence = self.manual_sequence;
        let bitrates = self.bitrates;
        let target = self.target.clone();
        let log = self.diagnostics.clone().unwrap_or_default();
        let expected_uid = self
            .scan_selected
            .and_then(|index| self.scan_devices.get(index))
            .map(|device| device.uid.clone())
            .unwrap_or_default();
        std::thread::spawn(move || {
            device::run(
                &dll,
                device_index,
                node,
                sequence,
                device::ConnectionIdentity {
                    target,
                    expected_uid,
                    bitrates,
                    log,
                },
                command_rx,
                device_event_tx,
            )
        });
        self.command_tx = Some(command_tx);
        self.event_rx = Some(event_rx);
        self.connected = true;
        self.hard_stop = false;
        self.intervention_ack = false;
        self.bound_uid.clear();
        self.snapshot = Snapshot::default();
        self.parameter_state = parameters::State::default();
        self.motor_acknowledged = false;
        self.pause_capture();
        self.acknowledged = false;
        self.control_targets = [0.0; 4];
        self.note(format!(
            "连接 CAN 设备 {device_index} / 节点 {node}，{}；正在识别设备；控制序号起点 {sequence}",
            self.bitrates.label()
        ));
    }
    fn send(&mut self, command: Command) {
        if matches!(&command, Command::Stop | Command::Disconnect) {
            self.parameter_state.cancel_read();
        }
        if self.fixture_mode {
            return;
        }
        if self.bench.is_some()
            && !matches!(
                command,
                Command::Observe(_) | Command::Stop | Command::Disconnect
            )
        {
            self.operation_pending = false;
            self.note("静止观察正在运行；该命令未发送。");
            return;
        }
        if self
            .command_tx
            .as_ref()
            .is_none_or(|tx| tx.send(command).is_err())
        {
            self.hard_stop = true;
            self.operation_pending = false;
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
            let dir = PathBuf::from(
                std::env::var_os("LOCALAPPDATA")
                    .ok_or_else(|| anyhow::anyhow!("缺少 LOCALAPPDATA，无法定位用户数据目录"))?,
            )
            .join("YG")
            .join("YG_MC_HOST")
            .join("captures");
            std::fs::create_dir_all(&dir)?;
            let stamp = SystemTime::now().duration_since(UNIX_EPOCH)?.as_millis();
            let path = dir.join(format!("capture-{stamp}.csv"));
            self.capture.write_csv(std::fs::File::create(&path)?)?;
            let curves = dir.join(format!("capture-{stamp}.curve.csv"));
            self.capture
                .write_curve_csv(std::fs::File::create(curves)?)?;
            Ok(path)
        })();
        match result {
            Ok(path) => self.note(format!("已导出记录：{}", path.display())),
            Err(error) => self.note(format!("记录导出失败：{error:#}")),
        }
    }
    fn panel<R>(ui: &mut egui::Ui, title: &str, content: impl FnOnce(&mut egui::Ui) -> R) -> R {
        widgets::section(ui, title, content)
    }
    fn control(&mut self, name: &str, response: &egui::Response) {
        if self.fixture_mode {
            self.control_evidence.push(serde_json::json!({ "name":name,
            "enabled":response.enabled(), "rect":[response.rect.min.x,response.rect.min.y,response.rect.max.x,response.rect.max.y] }));
        }
    }
    fn motion_ready(&self) -> bool {
        self.connected
            && !self.operation_pending
            && !self.configuration_busy
            && !self.fixture_mode
            && !self.hard_stop
            && !self.upgrading
            && (self.capture.uid.is_empty() || self.capture.uid == self.bound_uid)
    }
}

impl eframe::App for HostApp {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        self.refresh_studio();
        self.poll_events();
        self.poll_scan();
        self.maintenance_panel.poll(ui.ctx());
        if self.page != Page::Maintenance || self.upgrade_mode != UpgradeMode::Combined {
            self.maintenance_panel.leave_page();
        }
        if let Some(mut bench) = self.bench.take() {
            bench.tick(self, ui.ctx());
            self.bench = Some(bench);
        }
        if let Some(mut preview) = self.preview.take() {
            preview.tick(self, ui.ctx());
            self.preview = Some(preview);
        }
        self.control_evidence.clear();
        if self.scan_rx.is_some() && ui.ctx().input(|i| i.viewport().close_requested()) {
            ui.ctx()
                .send_viewport_cmd(egui::ViewportCommand::CancelClose);
            self.scan_cancel.store(true, Ordering::Relaxed);
            self.status = "正在取消扫描，等待 CAN 端口释放后关闭".into();
            self.pending_close = true;
        }
        if self.maintenance_panel.busy() && ui.ctx().input(|i| i.viewport().close_requested()) {
            ui.ctx()
                .send_viewport_cmd(egui::ViewportCommand::CancelClose);
            self.status = "SWD事务进行中，等待结果后关闭".into();
        }
        if self.pending_close
            && !self.connected
            && !self.hard_stop
            && !self.maintenance_panel.busy()
            && self.scan_rx.is_none()
        {
            ui.ctx().send_viewport_cmd(egui::ViewportCommand::Close);
        }
        if self.connected
            && !self.fixture_mode
            && ui.ctx().input(|i| i.viewport().close_requested())
        {
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
        // 曲线工作线程驱动有新数据的刷新；这里仅保留状态事件的低频兜底。
        ui.ctx().request_repaint_after(Duration::from_millis(100));
        let c = Palette::from_ctx(ui.ctx());
        self.workspace_shell(ui);
        egui::CentralPanel::default()
            .frame(
                Frame::new()
                    .fill(
                        if self.page == Page::Connection && ui.ctx().theme() == egui::Theme::Dark {
                            Color32::from_gray(19)
                        } else {
                            c.background
                        },
                    )
                    .inner_margin(10.0),
            )
            .show(ui, |ui| match self.page {
                Page::Workbench => self.workbench(ui),
                Page::Connection => self.device_center(ui),
                Page::Home => self.home(ui),
                Page::Parameters => self.parameter_workspace(ui),
                Page::Calibration => self.calibration(ui),
                Page::Identification => self.identification(ui),
                Page::Records => self.records(ui),
                Page::Maintenance => self.maintenance(ui),
            });
        if self.page != Page::Connection {
            self.add_device_open = false;
        }
        if self.add_device_open {
            let mut open = true;
            egui::Window::new("添加设备")
                .open(&mut open)
                .default_width(720.0)
                .max_height((ui.ctx().content_rect().height() - 85.0).max(180.0))
                .anchor(egui::Align2::CENTER_CENTER, Vec2::ZERO)
                .collapsible(false)
                .show(ui.ctx(), |ui| self.connection(ui));
            self.add_device_open = open && self.page == Page::Connection;
        }
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
    fn add_device_starts_one_scan_and_leaves_connection_to_the_user() {
        let mut app = HostApp::disconnected(Target::bundled().unwrap(), None);
        app.dll = "missing-sdk-for-discovery-test.dll".into();
        let ctx = egui::Context::default();
        ctx.enable_accesskit();
        let mut output = ctx.run_ui(egui::RawInput::default(), |ui| app.device_center(ui));
        output.textures_delta.clear();
        let tree = output.platform_output.accesskit_update.unwrap();
        let (id, _) = tree
            .nodes
            .iter()
            .find(|(_, node)| node.label().is_some_and(|s| s.contains("添加设备")))
            .unwrap();
        let mut input = egui::RawInput::default();
        input.events.push(egui::Event::AccessKitActionRequest(
            egui::accesskit::ActionRequest {
                action: egui::accesskit::Action::Click,
                target_tree: egui::accesskit::TreeId::ROOT,
                target_node: *id,
                data: None,
            },
        ));
        ctx.run_ui(input, |ui| app.device_center(ui))
            .textures_delta
            .clear();
        assert!(app.add_device_open && app.scan_rx.is_some());
        assert!(!app.connected && app.command_tx.is_none() && app.scan_selected.is_none());
    }
    #[test]
    fn rejected_operation_retains_connection_and_live_snapshot() {
        let mut app = HostApp::disconnected(Target::bundled().unwrap(), None);
        app.connected = true;
        app.snapshot.feedback_at = Some(std::time::Instant::now());
        app.snapshot.position_rad = Some(1.5);
        app.upgrading = true;
        let (tx, rx) = mpsc::channel();
        app.event_rx = Some(rx);
        tx.send(Event::Rejected("目标门禁未满足".into())).unwrap();
        app.poll_events();
        assert!(app.connected);
        assert!(!app.hard_stop);
        assert!(!app.upgrading);
        assert_eq!(app.snapshot.position_rad, Some(1.5));
        assert!(app.snapshot.feedback_fresh());
    }

    #[test]
    fn device_tabs_are_unique_and_separate_from_global_navigation() {
        assert_eq!(Page::WORKSPACES.len(), 4);
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
        app.page = Page::Workbench;
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
        assert_eq!(app.page, Page::Workbench);
    }

    #[test]
    fn bench_observation_rejects_motion_but_allows_stop() {
        let mut app = HostApp::disconnected(Target::bundled().unwrap(), None);
        app.bench = Some(bench::Run::new(BenchRequest {
            uid: "test".into(),
            output: "unused".into(),
        }));
        let (tx, rx) = mpsc::channel();
        app.command_tx = Some(tx);
        app.send(Command::Start {
            mode: ControlMode::Speed,
        });
        app.send(Command::Stop);
        assert!(matches!(rx.try_recv(), Ok(Command::Stop)));
        assert!(rx.try_recv().is_err());
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
        app.send(Command::Start {
            mode: ControlMode::Speed,
        });
        app.send(Command::Stop);
        app.send(Command::ReadConfiguration);
        app.send(Command::SaveConfiguration {
            payload: [0; 40],
            acknowledged: true,
        });
        app.send(Command::ActivateConfiguration);
        app.send(Command::Parameters(crate::parameters::Action::Read {
            ids: vec![11],
            saved: false,
        }));
        app.send(Command::Parameters(crate::parameters::Action::Write {
            id: 11,
            raw: 1,
            revision: 1,
        }));
        app.send(Command::Parameters(crate::parameters::Action::Save {
            revision: 1,
        }));
        app.send(Command::SetTarget {
            mode: ControlMode::Torque,
            value: 0.5,
        });
        assert!(!app.connected);
        assert!(rx.try_recv().is_err());
    }

    #[test]
    fn motion_no_longer_requires_upgrade_checkbox_but_live_device_gates_remain() {
        let mut app = HostApp::disconnected(Target::bundled().unwrap(), None);
        app.connected = true;
        app.bound_uid = "A".into();
        app.acknowledged = false;
        app.snapshot = Snapshot {
            uid: "A".into(),
            role: 1,
            app_version: 1,
            product: app.target.product,
            hardware: app.target.hardware as u16,
            schema: app.target.schema,
            abi: app.target.abi,
            features: 0x41d00,
            modes: 0x4e,
            motor_state: 5,
            mode: 0,
            stop_confirmed: true,
            position_rad: Some(1.0),
            speed_rad_s: Some(0.0),
            bus_v: Some(24.0),
            bus_a: Some(0.0),
            identity_at: Some(std::time::Instant::now()),
            feedback_at: Some(std::time::Instant::now()),
            ..Default::default()
        };
        for mode in ControlMode::ALL {
            assert!(app.motion_ready() && app.snapshot.can_start_mode(&app.target, "A", mode));
        }
        app.snapshot.feedback_at = Some(std::time::Instant::now() - Duration::from_secs(1));
        assert!(
            !app.snapshot
                .can_start_mode(&app.target, "A", ControlMode::Speed)
        );
        app.snapshot.feedback_at = Some(std::time::Instant::now());
        app.snapshot.faults = 1;
        assert!(
            !app.snapshot
                .can_start_mode(&app.target, "A", ControlMode::Speed)
        );
        app.hard_stop = true;
        assert!(!app.motion_ready());
        app.hard_stop = false;
        app.configuration_busy = true;
        assert!(!app.motion_ready());
        app.configuration_busy = false;
        app.capture.uid = "B".into();
        assert!(!app.motion_ready());
        app.capture.uid.clear();
        app.fixture_mode = true;
        assert!(!app.motion_ready());
    }
}
