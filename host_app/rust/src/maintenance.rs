//! SWD维护工具的异步入口；所有硬件参数显式传递，不抢占现有CAN会话。

use crate::target::Target;
use eframe::egui;
use std::path::PathBuf;
use std::process::Command;
use std::sync::mpsc::{self, Receiver};

pub struct Panel {
    python: String,
    script: String,
    probe: String,
    uid: String,
    bundle: String,
    input: String,
    after: String,
    migration: String,
    confirmed: bool,
    result: String,
    receiver: Option<Receiver<Result<String, String>>>,
}

impl Default for Panel {
    fn default() -> Self {
        Self {
            python: ".venv/Scripts/python.exe".into(),
            script: "loader/tools/device_maintenance.py".into(),
            probe: String::new(),
            uid: String::new(),
            bundle: String::new(),
            input: String::new(),
            after: String::new(),
            migration: String::new(),
            confirmed: false,
            result: String::new(),
            receiver: None,
        }
    }
}

fn field(ui: &mut egui::Ui, name: &str, value: &mut String) {
    ui.horizontal(|ui| {
        ui.label(name);
        ui.text_edit_singleline(value);
    });
}

impl Panel {
    pub fn busy(&self) -> bool {
        self.receiver.is_some()
    }
    fn start(&mut self, action: &str, target: &Target) {
        let mut args = vec![
            self.script.clone(),
            action.into(),
            "--target".into(),
            target.name.clone(),
        ];
        if matches!(action, "backup" | "install") {
            args.extend([
                "--probe".into(),
                self.probe.clone(),
                "--uid".into(),
                self.uid.clone(),
                "--exclusive-bench".into(),
            ]);
            let stamp = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_millis();
            args.extend(["--out".into(), format!("outputs/swd-{action}-{stamp}")]);
        }
        match action {
            "install" => {
                args.extend(["--bundle".into(), self.bundle.clone()]);
                if !self.migration.is_empty() {
                    args.extend(["--migration".into(), self.migration.clone()]);
                }
            }
            "parameters" => args.extend(["--input".into(), self.input.clone()]),
            "verify" => args.extend([
                "--input".into(),
                self.input.clone(),
                "--after".into(),
                self.after.clone(),
            ]),
            "prepare-migration" => {
                use sha2::Digest;
                match std::fs::read(PathBuf::from(&self.input).join("flash.bin")) {
                    Ok(raw) => args.extend([
                        "--input".into(),
                        self.input.clone(),
                        "--source-sha256".into(),
                        hex::encode(sha2::Sha256::digest(raw)),
                        "--out".into(),
                        self.migration.clone(),
                    ]),
                    Err(error) => {
                        self.result = error.to_string();
                        return;
                    }
                }
            }
            _ => {}
        }
        let python = self.python.clone();
        let (send, receive) = mpsc::channel();
        self.receiver = Some(receive);
        self.result = format!("正在执行 {action}；完成前不启动其他下载/调试会话");
        std::thread::spawn(move || {
            let mut command = Command::new(python);
            command.args(args);
            #[cfg(windows)]
            {
                use std::os::windows::process::CommandExt;
                command.creation_flags(0x08000000);
            }
            let result = command
                .output()
                .map_err(|error| error.to_string())
                .and_then(|output| {
                    let text = format!(
                        "{}{}",
                        String::from_utf8_lossy(&output.stdout),
                        String::from_utf8_lossy(&output.stderr)
                    );
                    if output.status.success() {
                        Ok(text)
                    } else {
                        Err(text)
                    }
                });
            let _ = send.send(result);
        });
    }

    pub fn show(&mut self, ui: &mut egui::Ui, target: &Target, disconnected: bool) {
        if let Some(receiver) = &self.receiver {
            if let Ok(result) = receiver.try_recv() {
                self.result = result.unwrap_or_else(|error| format!("失败：{error}"));
                self.receiver = None;
            } else {
                ui.ctx()
                    .request_repaint_after(std::time::Duration::from_millis(100));
            }
        }
        ui.label(format!(
            "目标：{} · 16/96/16 KiB · SWD初装、参数备份及离线校验",
            target.name
        ));
        ui.label(
            "先断开CAN连接并关闭其他调试器。初装完成保持CPU暂停；正常APP更新使用Keil的APP工程。",
        );
        ui.add_enabled_ui(self.receiver.is_none(), |ui| {
            field(ui, "Python", &mut self.python);
            field(ui, "维护脚本", &mut self.script);
            field(ui, "探针序列号", &mut self.probe);
            field(ui, "MCU UID（24位十六进制）", &mut self.uid);
            field(ui, "配套构建包目录", &mut self.bundle);
            field(ui, "输入快照文件／迁移源目录", &mut self.input);
            field(ui, "下载后完整Flash文件", &mut self.after);
            field(ui, "AT32迁移包目录（可选）", &mut self.migration);
            ui.checkbox(&mut self.confirmed, "已核对探针和UID，设备已停机、独占SWD");
            ui.horizontal(|ui| {
                let hardware = disconnected && self.confirmed;
                if ui
                    .add_enabled(hardware, egui::Button::new("读取完整备份"))
                    .clicked()
                {
                    self.start("backup", target);
                }
                if ui
                    .add_enabled(
                        hardware && !self.bundle.is_empty(),
                        egui::Button::new("初装配套固件"),
                    )
                    .clicked()
                {
                    self.start("install", target);
                }
                if ui.button("查看保存参数与LUT").clicked() {
                    self.start("parameters", target);
                }
            });
            ui.horizontal(|ui| {
                if ui.button("比较Keil下载前后保护区").clicked() {
                    self.start("verify", target);
                }
                if ui
                    .add_enabled(
                        target.name == "yg_esc_v1_at32f456",
                        egui::Button::new("离线生成原样迁移包"),
                    )
                    .clicked()
                {
                    self.start("prepare-migration", target);
                }
            });
        });
        egui::ScrollArea::vertical()
            .max_height(400.0)
            .show(ui, |ui| {
                ui.monospace(&self.result);
            });
    }
}
