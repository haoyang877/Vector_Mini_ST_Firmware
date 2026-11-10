//! SWD维护工具的异步入口；所有硬件参数显式传递，不抢占现有CAN会话。

use crate::target::Target;
use eframe::egui::{self, RichText};
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
    ui.vertical(|ui| {
        ui.label(
            RichText::new(name)
                .small()
                .color(ui.visuals().weak_text_color()),
        );
        ui.add_sized(
            [ui.available_width(), 28.0],
            egui::TextEdit::singleline(value),
        );
    });
}

fn swd_ready(disconnected: bool, confirmed: bool, probe: &str, uid: &str) -> bool {
    let probe_valid = probe.trim().parse::<u32>().is_ok_and(|serial| serial > 0);
    let uid = uid.trim();
    disconnected
        && confirmed
        && probe_valid
        && uid.len() == 24
        && uid.bytes().all(|byte| byte.is_ascii_hexdigit())
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
            match receiver.try_recv() {
                Ok(result) => {
                    self.result = result.unwrap_or_else(|error| format!("失败：{error}"));
                    self.receiver = None;
                }
                Err(mpsc::TryRecvError::Disconnected) => {
                    self.result = "失败：维护任务意外结束；结果未知，请检查设备和日志。".into();
                    self.receiver = None;
                }
                Err(mpsc::TryRecvError::Empty) => {
                    ui.ctx()
                        .request_repaint_after(std::time::Duration::from_millis(100));
                }
            }
        }
        ui.label(format!(
            "目标：{} · Loader + APP · 通过 J-Link / SWD",
            target.name
        ));
        ui.label(RichText::new("整机包用于初装或恢复整机固件；运行中的设备请使用「升级 APP（CAN）」。操作前断开 CAN 并关闭其他调试器。")
            .small().color(ui.visuals().weak_text_color()));
        ui.add_space(8.0);
        ui.add_enabled_ui(self.receiver.is_none(), |ui| {
            crate::ui::widgets::section(ui, "01  构建包", |ui| {
                ui.label(
                    egui::RichText::new(
                        "必须选择与目标板匹配的构建包文件夹。目录不匹配或缺少文件时不会开始安装。",
                    )
                    .small()
                    .color(ui.visuals().weak_text_color()),
                );
                ui.horizontal(|ui| {
                    ui.add_sized(
                        [(ui.available_width() - 84.0).max(100.0), 26.0],
                        egui::TextEdit::singleline(&mut self.bundle)
                            .hint_text("选择配套构建输出文件夹"),
                    );
                    if ui.button("浏览目录").clicked()
                        && let Some(path) = rfd::FileDialog::new().pick_folder()
                    {
                        self.bundle = path.display().to_string();
                    }
                });
            });
            ui.add_space(14.0);
            crate::ui::widgets::section(ui, "02  目标设备", |ui| {
                ui.columns(2, |columns| {
                    field(&mut columns[0], "J-Link 探针序列号", &mut self.probe);
                    field(&mut columns[1], "MCU UID（24 位十六进制）", &mut self.uid);
                });
            });
            ui.add_space(12.0);
            ui.checkbox(
                &mut self.confirmed,
                "已核对探针与 MCU UID，设备已停机，当前由本工具独占 SWD",
            );
            let probe_valid = self
                .probe
                .trim()
                .parse::<u32>()
                .is_ok_and(|serial| serial > 0);
            let uid = self.uid.trim();
            let uid_valid = uid.len() == 24 && uid.bytes().all(|byte| byte.is_ascii_hexdigit());
            ui.horizontal(|ui| {
                let hardware = swd_ready(disconnected, self.confirmed, &self.probe, &self.uid);
                if ui
                    .add_enabled(hardware, egui::Button::new("读取完整备份"))
                    .clicked()
                {
                    self.start("backup", target);
                }
                if ui
                    .add_enabled(
                        hardware && !self.bundle.trim().is_empty(),
                        egui::Button::new("安装整机包（J-Link / SWD）"),
                    )
                    .clicked()
                {
                    self.start("install", target);
                }
            });
            let reason = if !disconnected {
                "请先断开 CAN；离线预览不能操作设备。"
            } else if !probe_valid {
                "请填写有效的 J-Link 探针序列号。"
            } else if !uid_valid {
                "请填写 24 位十六进制 MCU UID。"
            } else if !self.confirmed {
                "请确认设备已停机并独占 SWD。"
            } else if self.bundle.trim().is_empty() {
                "安装整机包前请选择配套构建包目录。"
            } else {
                "备份或安装前请再次核对目标与构建包。"
            };
            ui.label(egui::RichText::new(reason).small());
            egui::CollapsingHeader::new("高级：维护脚本与离线工具").show(ui, |ui| {
                field(ui, "Python", &mut self.python);
                field(ui, "维护脚本", &mut self.script);
                field(ui, "输入快照／迁移源目录", &mut self.input);
                field(ui, "下载后完整 Flash 文件", &mut self.after);
                field(ui, "AT32 迁移包目录（可选）", &mut self.migration);
                ui.horizontal(|ui| {
                    if ui.button("查看保存参数与 LUT").clicked() {
                        self.start("parameters", target);
                    }
                    if ui.button("比较下载前后的保护区").clicked() {
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
        });
        if self.busy() {
            ui.horizontal(|ui| {
                ui.spinner();
                ui.label(egui::RichText::new(&self.result).strong());
            });
        } else if !self.result.is_empty() {
            let headline = self.result.lines().next().unwrap_or("维护任务已完成");
            let is_error = self.result.starts_with("失败：");
            ui.label(
                egui::RichText::new(headline)
                    .color(if is_error {
                        egui::Color32::from_rgb(244, 135, 135)
                    } else {
                        egui::Color32::from_rgb(120, 190, 145)
                    })
                    .strong(),
            );
            egui::CollapsingHeader::new("结果详情与完整日志")
                .default_open(is_error)
                .show(ui, |ui| {
                    egui::ScrollArea::vertical()
                        .max_height(260.0)
                        .show(ui, |ui| {
                            ui.monospace(&self.result);
                        });
                });
        }
    }
}

#[cfg(test)]
mod tests {
    use super::swd_ready;

    #[test]
    fn swd_requires_exclusive_confirmation_and_valid_identity() {
        let uid = "0123456789abcdef01234567";
        assert!(swd_ready(true, true, "12345", uid));
        assert!(!swd_ready(false, true, "12345", uid));
        assert!(!swd_ready(true, false, "12345", uid));
        assert!(!swd_ready(true, true, "0", uid));
        assert!(!swd_ready(true, true, "abc", uid));
        assert!(!swd_ready(true, true, "12345", "1234"));
        assert!(!swd_ready(true, true, "12345", "0123456789abcdef0123456g"));
    }
}
