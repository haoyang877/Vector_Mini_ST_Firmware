//! 右侧电机控制区：模式、输出操作和目标就近排列，准入仍由设备快照复核。
use super::*;

impl HostApp {
    pub(super) fn workbench_control_panel(&mut self, ui: &mut egui::Ui) {
        let c = Palette::from_ctx(ui.ctx());
        let fresh = self.connected && self.snapshot.feedback_fresh();
        Self::panel(ui, "电机控制", |ui| {
            let (state, color) = if self.hard_stop {
                ("● 硬停 · 状态待确认", c.danger)
            } else if !self.connected {
                ("○ 未连接", c.muted)
            } else {
                match self.snapshot.enable_state() {
                    Some(true) => ("● 已使能", c.accent),
                    Some(false) => ("○ 已失能", c.muted),
                    None => ("● 状态待确认", c.warning),
                }
            };
            ui.label(RichText::new(state).strong().size(16.0).color(color))
                .on_hover_text("状态来自新鲜设备回读；点击按钮不代表操作已经完成。");
            ui.add_space(8.0);
            ui.label(RichText::new("控制模式").small().color(c.muted));
            let can_switch = !self.upgrading
                && !self.configuration_busy
                && ((!self.connected && !self.hard_stop)
                    || (self.snapshot.stop_confirmed
                        && self.snapshot.safe_idle(&self.target, &self.bound_uid)));
            let previous = self.control_mode;
            let menu = ui
                .add_enabled_ui(can_switch, |ui| {
                    egui::ComboBox::from_id_salt("workbench_mode")
                        .selected_text(self.control_mode.name())
                        .width(ui.available_width())
                        .show_ui(ui, |ui| {
                            for mode in ControlMode::ALL {
                                let label = if self.connected && !self.snapshot.mode_supported(mode)
                                {
                                    format!("{} · 未声明", mode.name())
                                } else {
                                    mode.name().into()
                                };
                                ui.selectable_value(&mut self.control_mode, mode, label);
                            }
                        })
                })
                .inner;
            self.control("mode_selector", &menu.response);
            if self.fixture_mode && self.preview.as_ref().is_some_and(|p| p.mode_menu()) {
                egui::Popup::open_id(ui.ctx(), menu.response.id.with("popup"));
            }
            if self.control_mode != previous {
                self.control_targets[self.control_mode.index()] =
                    if self.control_mode == ControlMode::Position {
                        self.snapshot.position_rad.filter(|_| fresh).unwrap_or(0.0)
                    } else {
                        0.0
                    };
            }
            if !can_switch {
                widgets::hint(ui, "先失能并确认空闲，再切换模式。");
            }
            let mode = self.control_mode;
            let live = self.motion_ready();
            let can_start = self
                .snapshot
                .can_start_mode(&self.target, &self.bound_uid, mode);
            let can_set = self
                .snapshot
                .can_set_target(&self.target, &self.bound_uid, mode);
            let stop_available = self.connected && !self.fixture_mode && !self.upgrading;
            ui.add_space(8.0);
            ui.columns(2, |columns| {
                let enable = columns[0].add_enabled(
                    live && can_start,
                    egui::Button::new(
                        RichText::new("使能")
                            .strong()
                            .size(17.0)
                            .color(Color32::WHITE),
                    )
                    .fill(Color32::from_rgb(27, 124, 82))
                    .min_size(Vec2::new(columns[0].available_width(), 40.0)),
                );
                self.control("mode_enable", &enable);
                if enable.clicked() {
                    self.send(Command::Start { mode });
                }
                enable.on_hover_text("启动所选模式：Vq/Iq/速度从零输出开始，位置保持当前位置。");
                let disable = columns[1].add_enabled(
                    stop_available,
                    egui::Button::new(
                        RichText::new("失能")
                            .strong()
                            .size(17.0)
                            .color(Color32::WHITE),
                    )
                    .fill(Color32::from_rgb(157, 98, 24))
                    .min_size(Vec2::new(columns[1].available_width(), 40.0)),
                );
                self.control("mode_disable", &disable);
                if disable.clicked() {
                    self.send(Command::Stop);
                }
                disable.on_hover_text("请求 STOP，等待最终 0/0 和 IDLE 回读。");
            });
            ui.add_space(6.0);
            let stop = ui.add_enabled(
                stop_available,
                egui::Button::new(
                    RichText::new("STOP · 停止输出")
                        .strong()
                        .size(18.0)
                        .color(Color32::WHITE),
                )
                .fill(Color32::from_rgb(181, 43, 57))
                .stroke(Stroke::new(1.5, c.danger))
                .min_size(Vec2::new(ui.available_width(), 44.0)),
            );
            self.control("workbench_stop", &stop);
            if stop.clicked() {
                self.send(Command::Stop);
            }
            stop.on_hover_text(
                "立即请求协议 STOP；等待停机回读，物理输出仍需现场核查。与失能使用同一 Type110。",
            );
            ui.add_space(10.0);
            ui.separator();
            ui.add_space(6.0);
            ui.columns(2, |columns| {
                columns[0].label(
                    RichText::new(format!("目标 · {}", mode.unit()))
                        .small()
                        .color(c.muted),
                );
                columns[0].add_sized(
                    [columns[0].available_width(), 36.0],
                    egui::DragValue::new(&mut self.control_targets[mode.index()])
                        .range(mode.range())
                        .speed(if mode == ControlMode::Position {
                            0.001
                        } else {
                            0.01
                        })
                        .fixed_decimals(3),
                );
                columns[1].label(
                    RichText::new(format!("实测 · {}", mode.unit()))
                        .small()
                        .color(c.muted),
                );
                columns[1].add_sized(
                    [columns[1].available_width(), 36.0],
                    egui::Label::new(
                        RichText::new(
                            self.snapshot
                                .measured(mode)
                                .map(|v| format!("{v:.3}"))
                                .unwrap_or("—".into()),
                        )
                        .monospace()
                        .size(19.0)
                        .color(if fresh { c.ink } else { c.muted }),
                    ),
                );
            });
            if mode == ControlMode::Position {
                ui.horizontal(|ui| {
                    if ui
                        .add_enabled(
                            fresh && self.snapshot.position_rad.is_some(),
                            egui::Button::new("取当前位置").small(),
                        )
                        .clicked()
                    {
                        self.control_targets[mode.index()] = self.snapshot.position_rad.unwrap();
                    }
                    if let Some(actual) = self.snapshot.position_rad.filter(|_| fresh) {
                        widgets::hint(
                            ui,
                            &format!(
                                "偏移 {:+.3} rad",
                                self.control_targets[mode.index()] - actual
                            ),
                        );
                    }
                });
            }
            let target_result = mode.target_payload(
                self.control_targets[mode.index()],
                self.snapshot.position_rad.filter(|_| fresh),
            );
            ui.add_space(6.0);
            let set = ui.add_enabled(
                live && can_set && target_result.is_ok(),
                egui::Button::new(RichText::new("下发目标").strong())
                    .min_size(Vec2::new(ui.available_width(), 34.0)),
            );
            self.control("mode_set", &set);
            if set.clicked() {
                self.send(Command::SetTarget {
                    mode,
                    value: self.control_targets[mode.index()],
                });
            }
            let record_matches = self.capture.uid.is_empty() || self.capture.uid == self.bound_uid;
            let reason = if self.fixture_mode {
                "预览不发送设备命令"
            } else if self.upgrading {
                "升级期间禁用运动"
            } else if self.configuration_busy {
                "配置作业进行中，等待完成"
            } else if self.hard_stop {
                "硬停；须现场确认输出"
            } else if !self.connected {
                "未连接；请先连接并识别设备"
            } else if !record_matches {
                "记录 UID 不同；先导出并清空旧记录"
            } else if !self.snapshot.compatible(&self.target, &self.bound_uid) {
                "身份过期或目标不兼容"
            } else if !self.snapshot.mode_supported(mode) {
                "设备未声明所选模式及控制能力"
            } else if mode == ControlMode::Position
                && self.snapshot.features & crate::control::HOST_POSITION_SESSION_FEATURE == 0
            {
                "位置控制需配套APP支持上位机不限位会话"
            } else if !fresh {
                "反馈过期；等待设备更新"
            } else if self.snapshot.faults != 0 {
                "设备存在故障；请检查故障状态"
            } else if self
                .snapshot
                .active_configuration
                .as_ref()
                .is_some_and(|info| info.blocked())
            {
                "型号配置待重启或保存失败；请查看参数管理"
            } else if self
                .snapshot
                .active_configuration
                .as_ref()
                .is_some_and(|info| info.flags & 1 == 0 || !info.calibrated())
            {
                "当前型号需要确认或重新标定，暂不能使能"
            } else if self.snapshot.motor_state == 5 && !self.snapshot.stop_confirmed {
                "先点击失能或 STOP，等待确认空闲"
            } else if can_start {
                "可使能；使能后再下发目标"
            } else if can_set && target_result.is_err() {
                "请调整目标后再下发"
            } else if can_set {
                "可下发目标；结束后失能或 STOP"
            } else if self.snapshot.motor_state == 6 && self.snapshot.mode != mode.wire() {
                "设备运行模式不同；先失能再选模式"
            } else {
                "暂不可控制；检查状态、有效测量及限值"
            };
            ui.label(RichText::new(reason).small().color(if self.hard_stop {
                c.danger
            } else {
                c.warning
            }));
            if fresh && let Err(error) = target_result {
                ui.label(RichText::new(error.to_string()).small().color(c.warning));
            }
            ui.add_space(6.0);
            egui::CollapsingHeader::new("模式说明与调试限值").show(ui, |ui| {
                ui.label(mode.description());
                ui.label(mode.limit_text());
                ui.label("母线 ≤33 V / 3 A；运行 Iq ≤6 A；实测速率 ≤31.416 rad/s。");
                ui.label("Iq 不代替相电流监视。设备自身保护和限值仍然生效。");
                if mode == ControlMode::Position {
                    ui.label("位置会话不限位；失能/STOP或失联后结束此权限。");
                }
                if !self.bound_uid.is_empty() {
                    ui.label(format!("UID：{} · 节点 {}", self.bound_uid, self.node));
                }
            });
        });
        ui.add_space(8.0);
        if ui.button("电机型号与参数 →").clicked() {
            self.page = Page::Parameters;
        }
    }
}
