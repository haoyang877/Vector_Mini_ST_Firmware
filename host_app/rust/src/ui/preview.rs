//! 实际 Rust 窗口的离线截图序列；所有物理操作由 fixture_mode 无条件禁止。

use super::*;

struct Step {
    name: &'static str,
    page: Page,
    sample: bool,
    dark: bool,
    scale: f32,
    size: Vec2,
    mode: UpgradeMode,
    stale: bool,
}

pub(super) struct PreviewRun {
    directory: PathBuf,
    single: Option<PathBuf>,
    steps: Vec<Step>,
    index: usize,
    frames: u32,
    report: Vec<serde_json::Value>,
}

impl PreviewRun {
    pub(super) fn new(request: PreviewRequest) -> Self {
        let (directory, single) = match request {
            PreviewRequest::Single(path) => (
                path.parent()
                    .unwrap_or(std::path::Path::new("."))
                    .to_path_buf(),
                Some(path),
            ),
            PreviewRequest::Gallery(path) => (path, None),
        };
        let mut steps = vec![];
        for (name, page, sample) in [
            ("connection", Page::Connection, false),
            ("home", Page::Home, false),
            ("workbench-empty", Page::Workbench, false),
            ("workbench", Page::Workbench, true),
            ("parameters", Page::Parameters, false),
            ("calibration", Page::Calibration, false),
            ("identification", Page::Identification, false),
            ("records", Page::Records, true),
            ("maintenance-app", Page::Maintenance, false),
        ] {
            steps.push(Step {
                name,
                page,
                sample,
                dark: true,
                scale: 1.0,
                size: Vec2::new(1366.0, 768.0),
                mode: UpgradeMode::App,
                stale: false,
            });
        }
        if single.is_some() {
            steps.truncate(1);
        } else {
            for (name, mode) in [
                ("maintenance-loader", UpgradeMode::Loader),
                ("maintenance-combined", UpgradeMode::Combined),
            ] {
                steps.push(Step {
                    name,
                    page: Page::Maintenance,
                    sample: false,
                    dark: true,
                    scale: 1.0,
                    size: Vec2::new(1366.0, 768.0),
                    mode,
                    stale: false,
                });
            }
            for (name, dark, scale, size, stale) in [
                (
                    "workbench-offline",
                    true,
                    1.0,
                    Vec2::new(1366.0, 768.0),
                    true,
                ),
                (
                    "workbench-light",
                    false,
                    1.0,
                    Vec2::new(1366.0, 768.0),
                    false,
                ),
                ("workbench-150", true, 1.5, Vec2::new(1366.0, 768.0), false),
                ("workbench-200", true, 2.0, Vec2::new(1366.0, 768.0), false),
                ("workbench-1024", true, 1.0, Vec2::new(1024.0, 700.0), false),
                ("theme-menu", true, 1.0, Vec2::new(1366.0, 768.0), false),
            ] {
                steps.push(Step {
                    name,
                    page: Page::Workbench,
                    sample: true,
                    dark,
                    scale,
                    size,
                    mode: UpgradeMode::App,
                    stale,
                });
            }
        }
        Self {
            directory,
            single,
            steps,
            index: 0,
            frames: 0,
            report: vec![],
        }
    }

    pub(super) fn theme_menu(&self) -> bool {
        self.frames == 2
            && self
                .steps
                .get(self.index)
                .is_some_and(|step| step.name == "theme-menu")
    }

    pub(super) fn tick(&mut self, app: &mut HostApp, ctx: &egui::Context) {
        if self.index >= self.steps.len() {
            return;
        }
        let step = &self.steps[self.index];
        if self.frames == 0 {
            app.page = step.page;
            app.upgrade_mode = step.mode;
            app.capture = if step.sample {
                Capture::preview_record()
            } else {
                Capture::default()
            };
            app.theme = if step.dark {
                egui::ThemePreference::Dark
            } else {
                egui::ThemePreference::Light
            };
            app.snapshot = Snapshot::default();
            app.bound_uid.clear();
            app.window_s = 30.0;
            if step.stale {
                app.snapshot.uid = "PREVIEW-STALE".into();
                app.bound_uid = app.snapshot.uid.clone();
                app.snapshot.position_rad = Some(8.2);
                app.snapshot.speed_rad_s = Some(0.0);
                app.snapshot.iq_a = Some(0.0);
                app.snapshot.feedback_at = Some(std::time::Instant::now() - Duration::from_secs(8));
            }
            ctx.set_pixels_per_point(step.scale);
            ctx.send_viewport_cmd(egui::ViewportCommand::InnerSize(step.size));
            if let Err(error) = std::fs::create_dir_all(&self.directory) {
                eprintln!("预览目录失败：{error}");
            }
        }
        let captures = ctx.input(|i| {
            i.events
                .iter()
                .filter_map(|event| match event {
                    egui::Event::Screenshot { image, .. } => Some(image.clone()),
                    _ => None,
                })
                .collect::<Vec<_>>()
        });
        if let Some(capture) = captures.first() {
            let pixels: Vec<u8> = capture.pixels.iter().flat_map(|p| p.to_array()).collect();
            let path = self
                .single
                .clone()
                .unwrap_or_else(|| self.directory.join(format!("{}.png", step.name)));
            let saved = image::save_buffer(
                &path,
                &pixels,
                capture.width() as u32,
                capture.height() as u32,
                image::ColorType::Rgba8,
            );
            self.report.push(serde_json::json!({"page":step.name,"pixels":[capture.width(),capture.height()],
                "logical_size":[ctx.content_rect().width(),ctx.content_rect().height()],"requested_size":[step.size.x,step.size.y],
                "scale":ctx.pixels_per_point(),
                "sample_count":app.capture.samples.len(),"preview":true,"connected":app.connected,
                "controls":app.control_evidence,"saved":saved.is_ok()}));
            if let Err(error) = saved {
                eprintln!("预览保存失败：{error}");
            }
            self.index += 1;
            self.frames = 0;
            if self.index == self.steps.len() {
                let report = serde_json::json!({"hardware_contacted":false,"screens":self.report});
                if let Err(error) = std::fs::write(
                    self.directory.join("preview-summary.json"),
                    serde_json::to_vec_pretty(&report).unwrap_or_default(),
                ) {
                    eprintln!("预览报告失败：{error}");
                }
                ctx.send_viewport_cmd(egui::ViewportCommand::Close);
            }
            ctx.request_repaint();
            return;
        }
        self.frames += 1;
        // 缩放在下一帧才进入窗口积分；连续提交尺寸，避免用上一阶段的缩放换算。
        if self.frames <= 4 {
            ctx.send_viewport_cmd(egui::ViewportCommand::InnerSize(step.size));
        }
        if self.frames == 6 {
            ctx.send_viewport_cmd(egui::ViewportCommand::Screenshot(egui::UserData::default()));
        }
        if self.frames > 100 {
            eprintln!("截图超时");
            ctx.send_viewport_cmd(egui::ViewportCommand::Close);
        }
    }
}
