//! 型号配置 UI：明确区分当前活动配置、保存候选和重启后的生效状态。
use super::*;
use crate::motor_configuration::{self as config, Model};

impl HostApp {
    pub(super) fn motor_configuration_panel(&mut self, ui: &mut egui::Ui) {
        let c = Palette::from_ctx(ui.ctx());
        let identity = self.connected && self.snapshot.compatible(&self.target, &self.bound_uid);
        let supported = identity && self.snapshot.features & config::FEATURE != 0;
        let active = self.snapshot.active_configuration.clone();
        let saved = self.snapshot.saved_configuration.clone();
        let available = supported
            && !self.fixture_mode
            && !self.upgrading
            && !self.hard_stop
            && !self.configuration_busy;
        Self::panel(ui, "电机型号配置 · CAN", |ui| {
            if self.configuration_busy {
                ui.horizontal(|ui| {
                    ui.spinner();
                    widgets::hint(ui, "正在处理型号配置，请等待设备回读与重新识别…");
                });
            }
            ui.horizontal_wrapped(|ui| {
                ui.strong("当前生效");
                if let Some(info) = active.as_ref().filter(|_| supported) {
                    ui.label(format!(
                        "{} · {} · 节点 {}",
                        config::model_name(info.model),
                        config::axis_name(info.node),
                        info.node
                    ));
                    widgets::hint(
                        ui,
                        if info.calibrated() {
                            "已标定"
                        } else {
                            "需要标定"
                        },
                    );
                    widgets::hint(
                        ui,
                        &format!(
                            "修订 {:08X} · 速度上限 {:.3} rad/s",
                            info.revision, info.speed_rad_s
                        ),
                    );
                } else {
                    widgets::hint(ui, "未读取 / 固件未提供型号配置能力");
                }
                let read = ui.add_enabled(available, egui::Button::new("读取当前配置"));
                self.control("motor_config_read", &read);
                if read.clicked() {
                    self.send(Command::ReadConfiguration);
                }
            });
            if let Some(info) = saved.as_ref().filter(|_| supported) {
                ui.label(
                    RichText::new(format!(
                        "已保存待重启：{} · {} · 节点 {} · 修订 {:08X}",
                        config::model_name(info.model),
                        config::axis_name(info.node),
                        info.node,
                        info.revision
                    ))
                    .color(c.warning),
                );
            }
            ui.add_space(8.0);
            ui.add_enabled_ui(saved.is_none() && !self.configuration_busy,|ui| {
                ui.horizontal_wrapped(|ui| {
                    ui.label("指定型号");
                    let previous=self.motor_draft.model;
                    egui::ComboBox::from_id_salt("motor_model")
                        .selected_text(self.motor_draft.model.name()).width(140.0).show_ui(ui,|ui| {
                            for model in Model::ALL { ui.selectable_value(&mut self.motor_draft.model,model,model.name()); }
                        });
                    if previous!=self.motor_draft.model {
                        self.motor_draft.node=self.motor_draft.model.nodes()[0];
                        self.motor_thermal.clear(); self.motor_acknowledged=false;
                    }
                    ui.label("轴位置 / 节点");
                    egui::ComboBox::from_id_salt("motor_axis")
                        .selected_text(format!("{} · {}",config::axis_name(self.motor_draft.node),self.motor_draft.node))
                        .width(150.0).show_ui(ui,|ui| {
                            for &node in self.motor_draft.model.nodes() {
                                if ui.selectable_value(&mut self.motor_draft.node,node,format!("{} · {node}",config::axis_name(node))).changed() { self.motor_acknowledged=false; }
                            }
                        });
                });
                ui.horizontal_wrapped(|ui| {
                    ui.label("电机轴速度上限");
                    if ui.add(egui::DragValue::new(&mut self.motor_draft.speed_rad_s).range(0.001..=1000.0).speed(0.01).fixed_decimals(3).suffix(" rad/s")).changed() { self.motor_acknowledged=false; }
                    widgets::hint(ui,"固件按所选轴的产品上限校验");
                });
                ui.add_space(4.0);
                ui.label("绕组热参数：持续电流 A、峰值电流 A、时间常数 s、进入比、退出比、跳闸比、清除比");
                if ui.add_sized([ui.available_width(),28.0],egui::TextEdit::singleline(&mut self.motor_thermal)
                    .hint_text(if self.motor_draft.model==Model::Ht8115 {"留空使用 HT8115-4 已登记热参数"}else{"YQ 型号必须填写七项已核实值，空格分隔"})).changed() { self.motor_acknowledged=false; }
                if let Some(info)=active.as_ref().filter(|info|supported && info.model==self.motor_draft.model.id())
                    && ui.small_button("使用当前型号已保存的热参数").clicked() {
                    self.motor_thermal=info.thermal.iter().map(|v|format!("{v:.6}")).collect::<Vec<_>>().join(" ");
                    self.motor_acknowledged=false;
                }
            });
            ui.add_space(8.0);
            ui.label(RichText::new("此操作重建配套参数并清除旧编码器、Hall 和输出轴标定；节点随所选轴位置改变。保存后须重启并重新标定。").color(c.warning));
            ui.checkbox(
                &mut self.motor_acknowledged,
                "已核对实物型号、目标节点和热参数，允许清除旧标定",
            );
            let revision = active.as_ref().map_or(0, |info| info.revision);
            let payload = self.motor_draft.payload(revision, &self.motor_thermal);
            ui.horizontal_wrapped(|ui| {
                let save = ui.add_enabled(
                    available
                        && active.is_some()
                        && saved.is_none()
                        && self.motor_acknowledged
                        && self.snapshot.stop_confirmed
                        && self.snapshot.safe_idle(&self.target, &self.bound_uid)
                        && payload.is_ok(),
                    egui::Button::new("保存型号与配套参数").min_size(Vec2::new(180.0, 32.0)),
                );
                self.control("motor_config_save", &save);
                if save.clicked() {
                    self.configuration_busy = true;
                    self.pause_capture();
                    self.send(Command::Observe(false));
                    self.send(Command::SaveConfiguration {
                        payload: payload.as_ref().copied().unwrap(),
                        acknowledged: self.motor_acknowledged,
                    });
                    self.motor_acknowledged = false;
                    self.acknowledged = false;
                }
                let activate = ui.add_enabled(
                    available
                        && saved.is_some()
                        && self.snapshot.feedback_fresh()
                        && self.snapshot.motor_state == 5
                        && self.snapshot.mode == 0
                        && self.snapshot.faults == 0,
                    egui::Button::new("应用已保存配置并重启").min_size(Vec2::new(190.0, 32.0)),
                );
                self.control("motor_config_activate", &activate);
                if activate.clicked() {
                    self.configuration_busy = true;
                    self.pause_capture();
                    self.send(Command::Observe(false));
                    self.acknowledged = false;
                    self.send(Command::ActivateConfiguration);
                }
            });
            let reason = if self.fixture_mode {
                "离线预览不读写设备"
            } else if !identity {
                "先连接并识别兼容设备"
            } else if !supported {
                "设备未声明型号配置服务；需先升级支持命令 190–193 的配套 APP并重新识别"
            } else if active.as_ref().is_some_and(|info| info.flags & 16 != 0) {
                "保存失败且启动已锁定；请检查 Flash 故障，复位后重新识别配置"
            } else if saved.is_some() {
                "保存不代表生效；重启后按原 UID 和新节点核对，标定完成前保持失能"
            } else if !self.snapshot.stop_confirmed {
                "先失能并等待 STOP/IDLE 确认，再保存型号"
            } else {
                "写入前还会由设备复核当前修订、型号匹配、停机状态和参数范围"
            };
            widgets::hint(ui, reason);
            if saved.is_none()
                && let Err(error) = payload
            {
                ui.label(RichText::new(error.to_string()).small().color(c.warning));
            }
        });
    }
}
