//! 工作区组合；尚未接入的业务只显示支持范围与禁用原因，不制造设备值或完成结果。

use super::*;

fn field(ui: &mut egui::Ui, label: &str, value: impl Into<String>) {
    ui.label(RichText::new(label).color(Palette::from_ctx(ui.ctx()).muted));
    ui.label(value.into());
    ui.end_row();
}

fn reading(value: Option<f64>, unit: &str, fresh: bool) -> String {
    match value {
        Some(value) => format!("{value:.3} {unit}{}", if fresh { "" } else { " · 过期" }),
        None => "未知".into(),
    }
}

impl HostApp {
    pub(super) fn connection(&mut self, ui: &mut egui::Ui) {
        let c = Palette::from_ctx(ui.ctx());
        egui::ScrollArea::vertical().id_salt("device_center").show(ui, |ui| {
            ui.label(RichText::new("设备中心").size(24.0).strong());
            ui.add_space(12.0);
            if self.maintenance_panel.busy() {
                ui.label("J-Link / SWD 作业进行中，完成后可连接 CAN。");
                return;
            }
            if self.hard_stop && !self.fixture_mode {
                ui.checkbox(&mut self.intervention_ack, "已完成现场检查，允许重新连接");
            }
            let wide = ui.available_width() >= 780.0;
            let list_width = (ui.available_width() * 0.55).clamp(370.0, 760.0);
            if wide {
                ui.horizontal_top(|ui| {
                    ui.allocate_ui_with_layout(Vec2::new(list_width, 0.0), Layout::top_down(Align::Min), |ui| self.device_list(ui));
                    ui.add_space(4.0);
                    ui.vertical(|ui| self.device_summary(ui));
                });
            } else {
                self.device_list(ui);
                ui.add_space(8.0);
                self.device_summary(ui);
            }
            ui.add_space(12.0);
            if self.studio.is_some() {
                widgets::hint(ui, "离线样机使用示例节点，可在右上角「视图」调整缩放和预览状态。");
                return;
            }
            let scanning = self.scan_rx.is_some();
            egui::CollapsingHeader::new("CAN 通信设置").show(ui, |ui| {
                let before = self.bitrates;
                ui.add_enabled_ui(!self.connected && !scanning, |ui| {
                    ui.horizontal_wrapped(|ui| {
                        ui.label("仲裁段");
                        egui::ComboBox::from_id_salt("can_arbitration_rate")
                            .selected_text(crate::can::bitrate_label(self.bitrates.arbitration))
                            .width(106.0).show_ui(ui, |ui| {
                                for rate in crate::can::ARBITRATION_RATES {
                                    ui.selectable_value(&mut self.bitrates.arbitration, rate, crate::can::bitrate_label(rate));
                                }
                            });
                        ui.label("数据段");
                        egui::ComboBox::from_id_salt("can_data_rate")
                            .selected_text(crate::can::bitrate_label(self.bitrates.data))
                            .width(106.0).show_ui(ui, |ui| {
                                for rate in crate::can::DATA_RATES {
                                    if rate >= self.bitrates.arbitration {
                                        ui.selectable_value(&mut self.bitrates.data, rate, crate::can::bitrate_label(rate));
                                    }
                                }
                            });
                    });
                });
                if self.bitrates != before {
                    self.bitrates.data = self.bitrates.data.max(self.bitrates.arbitration);
                    self.scan_devices.clear(); self.scan_adapters.clear(); self.scan_selected = None;
                    self.manual_connect = false; self.acknowledged = false;
                    self.start_scan();
                }

            });
                egui::CollapsingHeader::new("高级设置 / 手动连接").show(ui, |ui| {
                    ui.add_enabled_ui(!self.connected && !scanning, |ui| {
                        let before = (self.device_index, self.node, self.target.name.clone(), self.dll.clone());
                        ui.checkbox(&mut self.manual_connect, "手动指定目标和节点");
                        if self.manual_connect {
                            self.scan_selected = None;
                            ui.horizontal(|ui| {
                                ui.label("板卡");
                                let mut selected = self.target.name.clone();
                                let display = if selected == "yg_esc_v1_at32f456" { "AT32F456" } else { "STM32G431" };
                                egui::ComboBox::from_id_salt("connection_target").selected_text(display).show_ui(ui, |ui| {
                                    ui.selectable_value(&mut selected, "vector_mini_st_g431".into(), "STM32G431");
                                    ui.selectable_value(&mut selected, "yg_esc_v1_at32f456".into(), "AT32F456");
                                });
                                if selected != self.target.name {
                                    match Target::named(&selected) {
                                        Ok(target) => { self.target = target; self.package = None; self.bound_uid.clear(); self.acknowledged = false; }
                                        Err(error) => self.note(error.to_string()),
                                    }
                                }
                                ui.label("适配器索引");
                                ui.add(egui::DragValue::new(&mut self.device_index).range(0..=15));
                                ui.label("节点");
                                ui.add(egui::DragValue::new(&mut self.node).range(1..=254));
                            });
                        }
                        ui.label(RichText::new("厂家 SDK DLL").small().color(c.muted));
                        ui.horizontal(|ui| {
                            let width = (ui.available_width() - 80.0).max(100.0);
                            ui.add_sized([width, 30.0], egui::TextEdit::singleline(&mut self.dll).hint_text("ControlCANFD.dll 路径"));
                            if ui.button("浏览").clicked() && let Some(path) = rfd::FileDialog::new().add_filter("DLL", &["dll"]).pick_file() {
                                self.dll = path.display().to_string();
                            }
                        });
                        if before != (self.device_index, self.node, self.target.name.clone(), self.dll.clone()) {
                            self.scan_selected = None;
                            if before.3 != self.dll { self.scan_devices.clear(); self.scan_adapters.clear(); }
                        }
                        egui::CollapsingHeader::new("控制序号").show(ui, |ui| {
                            ui.horizontal(|ui| { ui.label("起点"); ui.add(egui::DragValue::new(&mut self.manual_sequence).range(0..=65535)); });
                            ui.label("0 使用本机按 UID 保存的记录；其他主机操作过时，须按真实历史填写非零起点，不要猜测。");
                        });
                    });
                    ui.label(RichText::new("支持 USBCANFD-200U、通道 0；节点范围 1–254。RS-485 尚未接入。").small().color(c.muted));
                });
        });
    }

    fn device_list(&mut self, ui: &mut egui::Ui) {
        let c = Palette::from_ctx(ui.ctx());
        let scanning = self.scan_rx.is_some();
        Self::panel(ui, "适配器与节点", |ui| {
            ui.horizontal_wrapped(|ui| {
                let allowed = !self.fixture_mode
                    && !self.connected
                    && !scanning
                    && !self.dll.trim().is_empty()
                    && (!self.hard_stop || self.intervention_ack);
                if ui
                    .add_enabled(allowed, egui::Button::new("重新扫描"))
                    .clicked()
                {
                    self.start_scan();
                }
                if scanning && ui.button("取消扫描").clicked() {
                    self.scan_cancel.store(true, Ordering::Relaxed);
                }
                widgets::hint(ui, &format!("CAN FD  ·  {}", self.bitrates.label()));
            });
            ui.add_space(8.0);
            if self.scan_adapters.is_empty() {
                ui.separator();
                ui.add_space(10.0);
                ui.label(RichText::new("等待发现设备").strong());
                widgets::hint(
                    ui,
                    if self.dll.is_empty() {
                        "请在高级设置中选择 ControlCANFD.dll。"
                    } else {
                        &self.scan_status
                    },
                );
            }
            for (adapter, name) in self.scan_adapters.clone() {
                ui.separator();
                ui.label(
                    RichText::new(format!("{name}   /   通道 0"))
                        .color(c.muted)
                        .small(),
                );
                for (i, d) in self
                    .scan_devices
                    .clone()
                    .into_iter()
                    .enumerate()
                    .filter(|(_, d)| d.adapter_index == adapter)
                {
                    let axis = crate::motor_configuration::axis_name(d.node);
                    let role = if d.role == 2 { "Loader" } else { "APP" };
                    let label = if self.studio.is_some() {
                        studio::model(d.node).name()
                    } else if d.uid == self.snapshot.uid {
                        self.snapshot
                            .active_configuration
                            .as_ref()
                            .map_or(axis, |info| {
                                crate::motor_configuration::model_name(info.model)
                            })
                    } else {
                        axis
                    };
                    let right = format!("{axis}  ·  N{:02}   /   {role}", d.node);
                    let reply = ui.add_enabled(
                        (!self.connected || self.studio.is_some()) && !scanning,
                        egui::Button::new(RichText::new(label).size(16.0).strong())
                            .right_text(RichText::new(right).small())
                            .selected(self.scan_selected == Some(i))
                            .min_size(Vec2::new(ui.available_width(), 56.0)),
                    );
                    if reply
                        .on_hover_text(format!("{}  /  {}", d.adapter, d.uid))
                        .clicked()
                    {
                        self.scan_selected = Some(i);
                        if let Some(name) = &d.target_name
                            && let Ok(target) = Target::named(name)
                        {
                            if self.studio.is_none() {
                                self.target = target;
                                self.device_index = d.adapter_index;
                                self.node = d.node;
                                self.bound_uid.clear();
                                self.manual_connect = false;
                                self.package = None;
                                self.acknowledged = false;
                            }
                        } else {
                            self.manual_connect = false;
                        }
                    }
                }
            }
            if !self.scan_adapters.is_empty() && self.studio.is_none() {
                widgets::hint(ui, &self.scan_status);
            }
        });
    }

    fn device_summary(&mut self, ui: &mut egui::Ui) {
        let c = Palette::from_ctx(ui.ctx());
        let selected = self
            .scan_selected
            .and_then(|i| self.scan_devices.get(i))
            .cloned();
        Self::panel(ui, "设备详情", |ui| {
            let known = selected
                .as_ref()
                .is_some_and(|d| d.uid == self.snapshot.uid);
            let model = if self.studio.is_some() {
                selected.as_ref().map(|d| studio::model(d.node).name())
            } else {
                self.snapshot
                    .active_configuration
                    .as_ref()
                    .filter(|_| known || selected.is_none())
                    .map(|cfg| crate::motor_configuration::model_name(cfg.model))
            };
            ui.horizontal(|ui| {
                let (icon, _) = ui.allocate_exact_size(Vec2::splat(54.0), egui::Sense::hover());
                let color = if selected.is_some() {
                    c.accent
                } else {
                    c.muted
                };
                let center = icon.center();
                let brush = ui.painter();
                brush.circle_stroke(center, 20.0, Stroke::new(2.0, color));
                brush.line_segment(
                    [icon.left_center(), center - Vec2::new(20.0, 0.0)],
                    Stroke::new(2.0, color),
                );
                brush.line_segment(
                    [center + Vec2::new(20.0, 0.0), icon.right_center()],
                    Stroke::new(2.0, color),
                );
                brush.text(
                    center,
                    egui::Align2::CENTER_CENTER,
                    "M",
                    egui::FontId::proportional(22.0),
                    color,
                );
                ui.vertical(|ui| {
                    ui.label(
                        RichText::new(model.unwrap_or(if selected.is_some() {
                            "型号待回读"
                        } else {
                            "请选择节点"
                        }))
                        .size(24.0)
                        .strong(),
                    );
                    if let Some(d) = &selected {
                        ui.label(
                            RichText::new(format!(
                                "{}   /   节点 {:02}",
                                crate::motor_configuration::axis_name(d.node),
                                d.node
                            ))
                            .color(c.muted),
                        );
                    }
                });
            });
            if let Some(d) = &selected {
                ui.separator();
                egui::Grid::new("selected_identity")
                    .spacing([18.0, 6.0])
                    .show(ui, |ui| {
                        field(ui, "UID", &d.uid);
                        field(
                            ui,
                            "身份来源",
                            if self.studio.is_some() {
                                "离线示例"
                            } else if d.target_name.is_none() {
                                "不兼容节点"
                            } else {
                                "本次 CAN 扫描"
                            },
                        );
                        if known && self.connected {
                            field(
                                ui,
                                "固件",
                                format!(
                                    "APP {}   /   Loader {}",
                                    version(self.snapshot.app_version),
                                    version(self.snapshot.loader_version)
                                ),
                            );
                        }
                    });
            }
            if !self.snapshot.uid.is_empty() && !self.connected && self.studio.is_none() {
                widgets::hint(ui, &format!("上次连接   {}", self.snapshot.uid));
            }
            ui.add_space(12.0);
            if self.studio.is_some() {
                if ui
                    .add_enabled(selected.is_some(), widgets::primary("打开示例工作台 →", c))
                    .clicked()
                    && let Some(d) = selected
                {
                    self.node = d.node;
                    self.set_studio_scenario(studio::Scenario::Idle);
                    self.page = Page::Workbench;
                }
            } else {
                let enabled = !self.fixture_mode
                    && !self.connected
                    && self.scan_rx.is_none()
                    && (self.manual_connect || selected.is_some_and(|d| d.target_name.is_some()))
                    && !self.dll.trim().is_empty()
                    && (!self.hard_stop || self.intervention_ack);
                let connect = ui.add_enabled(enabled, widgets::primary("连接所选设备", c));
                self.control("connect", &connect);
                if connect.clicked() {
                    self.connect();
                }
                if self.connected {
                    if ui.button("打开工作台").clicked() {
                        self.page = Page::Workbench;
                    }
                    if ui
                        .add_enabled(!self.upgrading, egui::Button::new("断开连接"))
                        .clicked()
                    {
                        self.send(Command::Disconnect);
                    }
                } else if !self.fixture_mode
                    && ui.link("打开离线样机 ↗").clicked()
                    && let Err(e) = std::env::current_exe().and_then(|exe| {
                        std::process::Command::new(exe)
                            .arg("--studio-preview")
                            .spawn()
                    })
                {
                    self.note(format!("样机启动失败 {e}"));
                }
            }
        });
    }

    pub(super) fn home(&mut self, ui: &mut egui::Ui) {
        egui::ScrollArea::vertical().show(ui, |ui| {
            widgets::page_header(
                ui,
                "开始",
                "选择要完成的操作，或查看当前功能支持情况。",
                "工作入口",
            );
            ui.columns(3, |columns| {
                for (column, (title, description, action, page)) in columns.iter_mut().zip([
                    (
                        "1  连接设备",
                        "扫描 CAN 适配器和节点，选择设备。",
                        "打开设备连接",
                        Page::Connection,
                    ),
                    (
                        "2  运行调试",
                        "观察曲线、选择控制模式、执行 STOP。",
                        "打开运行调试",
                        Page::Workbench,
                    ),
                    (
                        "3  固件升级",
                        "检查固件包，选择下载方式。",
                        "打开固件升级",
                        Page::Maintenance,
                    ),
                ]) {
                    Self::panel(column, title, |ui| {
                        ui.set_min_height(64.0);
                        widgets::hint(ui, description);
                        ui.add_space(8.0);
                        if ui
                            .add_sized([ui.available_width(), 32.0], egui::Button::new(action))
                            .clicked()
                        {
                            self.page = page;
                        }
                    });
                }
            });
            ui.add_space(14.0);
            Self::panel(ui, "当前设备", |ui| {
                if self.connected && self.snapshot.identity_fresh() {
                    ui.horizontal_wrapped(|ui| {
                        ui.strong(format!("节点 {}", self.node));
                        ui.label(format!("UID {}", self.bound_uid));
                        widgets::hint(
                            ui,
                            &format!(
                                "APP {}  /  Loader {}",
                                version(self.snapshot.app_version),
                                version(self.snapshot.loader_version)
                            ),
                        );
                    });
                    widgets::hint(
                        ui,
                        if self.snapshot.feedback_fresh() {
                            if self.snapshot.faults == 0 {
                                "已识别 · 详细读数和状态可在下方展开"
                            } else {
                                "设备存在故障，请先查看详细状态"
                            }
                        } else {
                            "身份已识别，正在等待新鲜状态反馈"
                        },
                    );
                } else {
                    widgets::hint(ui, "尚未识别设备。请先扫描并选择设备，再进行控制或升级。");
                }
            });
            ui.add_space(14.0);
            ui.columns(2, |columns| {
                Self::panel(&mut columns[0], "已有功能入口", |ui| {
                    for (name, detail) in [
                        ("设备发现与连接", "识别适配器、节点与 UID"),
                        ("四模式调试与曲线", "Vq / Iq / 速度 / 位置 / CSV / STOP"),
                        ("故障状态与清除", "底部常驻故障码；清除后保持失能"),
                        (
                            "标定任务",
                            "LUT / 电气零位 / 机械零位，阶段、取消与保存回读",
                        ),
                        ("指定电机型号", "CAN 保存配套参数，重启后生效；需配套固件"),
                        ("APP 升级", "CAN 流程已接入，实机升级待验收"),
                        ("Loader + APP 安装", "通过 J-Link，支持备份与离线参数查看"),
                    ] {
                        ui.strong(name);
                        widgets::hint(ui, detail);
                        ui.add_space(4.0);
                    }
                });
                Self::panel(&mut columns[1], "尚未接入执行", |ui| {
                    for (name, detail) in [
                        ("辨识任务", "目前没有可执行任务"),
                        ("仅升级 Loader", "独立下载入口尚未接入"),
                    ] {
                        ui.strong(name);
                        widgets::hint(ui, detail);
                        ui.add_space(4.0);
                    }
                });
            });
            ui.add_space(12.0);
            egui::CollapsingHeader::new("设备详细信息与实时读数")
                .show(ui, |ui| self.device_details(ui));
        });
    }

    fn device_details(&mut self, ui: &mut egui::Ui) {
        egui::ScrollArea::vertical().show(ui, |ui| {
            let identity = self.connected && self.snapshot.identity_fresh();
            let feedback = self.connected && self.snapshot.feedback_fresh();
            if !identity {
                widgets::hint(ui, "当前身份未知或过期，设备值以新鲜回读为准。");
                ui.add_space(8.0);
            }
            Self::panel(ui, "实时状态", |ui| {
                let metrics = [
                    ("位置", reading(self.snapshot.position_rad, "rad", feedback)),
                    (
                        "速度",
                        reading(self.snapshot.speed_rad_s, "rad/s", feedback),
                    ),
                    ("Iq", reading(self.snapshot.iq_a, "A", feedback)),
                    (
                        "MCU 温度",
                        reading(self.snapshot.temperature_c, "°C", feedback),
                    ),
                    ("母线电压", reading(self.snapshot.bus_v, "V", feedback)),
                    ("母线电流", reading(self.snapshot.bus_a, "A", feedback)),
                    (
                        "状态 / 模式",
                        if feedback {
                            format!("{} / {}", self.snapshot.motor_state, self.snapshot.mode)
                        } else {
                            "未知".into()
                        },
                    ),
                    (
                        "故障",
                        if feedback {
                            format!("0x{:08X}", self.snapshot.faults)
                        } else {
                            "未知".into()
                        },
                    ),
                ];
                for row in metrics.chunks(4) {
                    ui.columns(4, |columns| {
                        for (column, (label, value)) in columns.iter_mut().zip(row) {
                            column.label(
                                RichText::new(*label)
                                    .small()
                                    .color(Palette::from_ctx(column.ctx()).muted),
                            );
                            column.label(RichText::new(value).size(15.0).strong());
                        }
                    });
                    ui.add_space(10.0);
                }
            });
            ui.add_space(12.0);
            ui.columns(2, |columns| {
                Self::panel(&mut columns[0], "设备身份", |ui| {
                    egui::Grid::new("identity")
                        .spacing([18.0, 10.0])
                        .show(ui, |ui| {
                            field(
                                ui,
                                "UID",
                                if self.snapshot.uid.is_empty() {
                                    "未知"
                                } else {
                                    &self.snapshot.uid
                                },
                            );
                            field(
                                ui,
                                "序列号",
                                if self.snapshot.sn.is_empty() {
                                    "未提供 / 未读取"
                                } else {
                                    &self.snapshot.sn
                                },
                            );
                            field(
                                ui,
                                "节点",
                                if self.connected {
                                    self.node.to_string()
                                } else {
                                    "未知".into()
                                },
                            );
                            field(ui, "电机 ID", "未提供（与节点号分开）");
                            field(
                                ui,
                                "生产身份型号编号",
                                if identity {
                                    format!("协议枚举 {}", self.snapshot.motor_type)
                                } else {
                                    "未知".into()
                                },
                            );
                            field(
                                ui,
                                "实际电机型号",
                                if identity {
                                    self.snapshot
                                        .active_configuration
                                        .as_ref()
                                        .map(|info| {
                                            crate::motor_configuration::model_name(info.model)
                                                .to_owned()
                                        })
                                        .unwrap_or_else(|| "固件未提供配置查询".into())
                                } else {
                                    "未知".into()
                                },
                            );
                            field(
                                ui,
                                "角色",
                                if identity {
                                    if self.snapshot.role == 2 {
                                        "Loader"
                                    } else {
                                        "APP"
                                    }
                                } else {
                                    "未知"
                                },
                            );
                        });
                });
                Self::panel(&mut columns[1], "版本与兼容", |ui| {
                    egui::Grid::new("versions")
                        .spacing([18.0, 10.0])
                        .show(ui, |ui| {
                            field(
                                ui,
                                "APP",
                                if identity {
                                    version(self.snapshot.app_version)
                                } else {
                                    "未知".into()
                                },
                            );
                            field(
                                ui,
                                "Loader",
                                if identity {
                                    version(self.snapshot.loader_version)
                                } else {
                                    "未知".into()
                                },
                            );
                            field(
                                ui,
                                "硬件版本",
                                if identity {
                                    self.snapshot.hardware.to_string()
                                } else {
                                    "未知".into()
                                },
                            );
                            field(
                                ui,
                                "参数结构",
                                if identity {
                                    format!(
                                        "schema {} / ABI {:08X}",
                                        self.snapshot.schema, self.snapshot.abi
                                    )
                                } else {
                                    "未知".into()
                                },
                            );
                            field(ui, "参数集版本", "未读取（与 schema 分开）");
                            for mode in ControlMode::ALL {
                                field(
                                    ui,
                                    &format!("{}能力", mode.name()),
                                    if self.snapshot.mode_supported(mode) && identity {
                                        "设备已声明"
                                    } else {
                                        "未声明 / 过期"
                                    },
                                );
                            }
                        });
                });
            });
            ui.add_space(12.0);
            egui::CollapsingHeader::new("标定与辨识支持范围").show(ui, |ui| {
                widgets::hint(ui, "任务执行及结果读取尚未接入；请到对应页面查看支持状态。");
            });
        });
    }

    pub(super) fn workbench(&mut self, ui: &mut egui::Ui) {
        let c = Palette::from_ctx(ui.ctx());
        let control_width = (ui.available_width() * 0.35).clamp(215.0, 385.0);
        egui::Panel::right("workbench_controls")
            .resizable(false)
            .exact_size(control_width)
            .frame(Frame::new().fill(c.background).inner_margin(egui::Margin {
                left: 6,
                right: 0,
                top: 0,
                bottom: 0,
            }))
            .show(ui, |ui| {
                egui::ScrollArea::vertical()
                    .id_salt("control_groups")
                    .show(ui, |ui| self.workbench_control_panel(ui));
            });
        egui::Panel::bottom("scope_controls")
            .frame(Frame::new().fill(c.panel).inner_margin(4.0))
            .show(ui, |ui| self.scope_tools(ui));
        let scene = self
            .curve_worker
            .as_mut()
            .expect("曲线线程未初始化")
            .scene(self.window_s, self.show_reference);
        self.capture.record_curve_update(scene.latest_sample_time);
        let channels = self.control_mode.channels();
        let total = (ui.available_height() - 2.0).max(220.0);
        let gap = ui.spacing().item_spacing.y;
        let height = ((total - 2.0 * gap) / 3.0).max(94.0);
        egui::ScrollArea::vertical()
            .id_salt("scope_stack")
            .auto_shrink([false, false])
            .show(ui, |ui| {
                for channel in channels {
                    let (hover, rect) = scope::plot(ui, &scene, channel, height, self.cursor_s);
                    if self.fixture_mode {
                        self.control_evidence
                            .push(serde_json::json!({"name":format!("plot_{channel}"),
                        "enabled":false,"rect":[rect.min.x,rect.min.y,rect.max.x,rect.max.y]}));
                    }
                    if let Some(time) = hover {
                        self.cursor_s = Some(time);
                    }
                }
            });
    }

    fn scope_tools(&mut self, ui: &mut egui::Ui) {
        let c = Palette::from_ctx(ui.ctx());
        ui.spacing_mut().item_spacing = Vec2::new(5.0, 3.0);
        ui.horizontal_wrapped(|ui| {
            let can_observe = !self.fixture_mode
                && self.connected
                && !self.upgrading
                && self.snapshot.identity_fresh()
                && self.snapshot.role == 1;
            if ui
                .add_enabled(
                    can_observe && !self.capture.observing,
                    egui::Button::new("▶  开始观察").min_size(Vec2::new(104.0, 28.0)),
                )
                .clicked()
            {
                match self.capture.start(&self.bound_uid) {
                    Ok(()) => {
                        if let Some(worker) = &self.curve_worker {
                            worker.observe(self.capture.recording_context());
                        }
                        self.send(Command::Observe(true));
                        self.note("开始观察 CAN108 趋势，目标 60 Hz（主机接收时间）");
                    }
                    Err(error) => self.note(error.to_string()),
                }
            }
            if ui
                .add_enabled(
                    self.capture.observing,
                    egui::Button::new("暂停").frame(false),
                )
                .clicked()
            {
                self.pause_capture();
                self.send(Command::Observe(false));
            }
            if ui
                .add_enabled(
                    !self.capture.observing,
                    egui::Button::new("清空").frame(false),
                )
                .clicked()
            {
                self.capture = Capture::default();
                if let Some(worker) = &self.curve_worker {
                    worker.reset(vec![]);
                }
                self.cursor_s = None;
            }
            if ui
                .add_enabled(
                    !self.capture.samples.is_empty(),
                    egui::Button::new("导出 CSV").frame(false),
                )
                .clicked()
            {
                self.export_capture();
            }
            ui.separator();
            ui.label(RichText::new("时间窗").small().color(c.muted));
            egui::ComboBox::from_id_salt("scope_window")
                .selected_text(format!("{} s", self.window_s))
                .width(60.0)
                .show_ui(ui, |ui| {
                    for seconds in [5.0, 10.0, 30.0, 60.0, 120.0] {
                        ui.selectable_value(&mut self.window_s, seconds, format!("{seconds} s"));
                    }
                });
            ui.checkbox(&mut self.show_reference, "参考量");
            if ui
                .small_button("复位视图")
                .on_hover_text("恢复 30 秒时间窗并清除游标")
                .clicked()
            {
                self.window_s = 30.0;
                self.cursor_s = None;
            }
        });
        ui.add_space(6.0);
        ui.horizontal_wrapped(|ui| {
            let rate = |value: Option<f64>| {
                value
                    .map(|hz| format!("{hz:.1} Hz"))
                    .unwrap_or_else(|| "—".into())
            };
            widgets::hint(ui, "曲线目标 60 Hz");
            ui.separator();
            ui.label(
                RichText::new(if self.capture.observing {
                    format!("实际更新 {}", rate(self.capture.refresh_hz()))
                } else {
                    "尚未开始观察".into()
                })
                .small()
                .color(c.accent),
            )
            .on_hover_text(format!(
                "只统计采用新数据快照的次数。设备反馈接收：{}",
                rate(self.capture.observed_hz())
            ));
            ui.separator();
            widgets::hint(ui, &format!("{} 个样本", self.capture.samples.len()));
            if self.capture.preview {
                widgets::hint(ui, "示例数据 · 非设备回读");
            }
        });
        if self.capture.observing
            && self.capture.samples.len() >= 60
            && self.capture.observed_hz().is_some_and(|hz| hz < 54.0)
        {
            ui.colored_label(
                c.warning,
                "数据接收低于目标频率，请检查设备响应和总线负载。",
            );
        }
    }

    pub(super) fn identification(&mut self, ui: &mut egui::Ui) {
        egui::ScrollArea::vertical().show(ui, |ui| {
            widgets::page_header(
                ui,
                "辨识任务",
                "当前没有可执行任务，以下仅显示固件支持范围。",
                "暂不可执行",
            );
            Self::panel(ui, "辨识任务", |ui| {
                widgets::hint(ui, "当前没有可执行的辨识任务，结果以设备回读为准。");
                ui.add_space(8.0);
                egui::ScrollArea::horizontal().show(ui, |ui| {
                    egui::Grid::new("identification_catalog")
                        .striped(true)
                        .min_col_width(((ui.available_width() - 48.0) / 3.0).max(140.0))
                        .spacing([24.0, 18.0])
                        .show(ui, |ui| {
                            for title in ["任务", "结果与单位", "支持状态"] {
                                ui.strong(title);
                            }
                            ui.end_row();
                            for (name, unit, state) in [
                                ("电阻辨识", "相 / 相间电阻 · Ω", "当前固件已撤回"),
                                ("电感辨识", "Ld / Lq · H", "暂无可用任务契约"),
                                ("转动惯量辨识", "参照轴惯量 · kg·m²", "暂无可用任务契约"),
                                (
                                    "摩擦力辨识",
                                    "转矩 · N·m / 黏性系数 · N·m·s/rad",
                                    "当前固件已撤回",
                                ),
                            ] {
                                ui.label(name);
                                ui.label(unit);
                                widgets::hint(ui, state);
                                ui.end_row();
                            }
                        });
                });
            });
            ui.add_space(14.0);
            widgets::hint(
                ui,
                "接入后将分别显示任务执行、结果质量、应用与掉电保存状态。",
            );
        });
    }

    pub(super) fn records(&mut self, ui: &mut egui::Ui) {
        egui::ScrollArea::vertical().show(ui, |ui| {
            widgets::page_header(
                ui,
                "数据与日志",
                "导出已采集的数据，查看本次连接与操作日志。",
                if self.capture.observing {
                    "正在记录"
                } else {
                    "历史记录"
                },
            );
            Self::panel(ui, "趋势记录", |ui| {
                ui.columns(3, |columns| {
                    widgets::hint(&mut columns[0], "样本数");
                    columns[0].strong(self.capture.samples.len().to_string());
                    widgets::hint(&mut columns[1], "接收频率");
                    columns[1].strong(
                        self.capture
                            .observed_hz()
                            .map(|hz| format!("{hz:.1} Hz"))
                            .unwrap_or_else(|| "—".into()),
                    );
                    widgets::hint(&mut columns[2], "数据中断 / 容量淘汰");
                    columns[2].strong(format!("{} / {}", self.capture.gaps, self.capture.evicted));
                });
                ui.add_space(8.0);
                widgets::hint(
                    ui,
                    &format!(
                        "UID  {}",
                        if self.capture.uid.is_empty() {
                            "未绑定"
                        } else {
                            &self.capture.uid
                        }
                    ),
                );
                ui.horizontal_wrapped(|ui| {
                    if ui
                        .add_enabled(
                            !self.capture.samples.is_empty(),
                            egui::Button::new("导出 CSV"),
                        )
                        .clicked()
                    {
                        self.export_capture();
                    }
                    if ui.button("查看曲线").clicked() {
                        self.page = Page::Workbench;
                    }
                    ui.add_enabled(false, egui::Button::new("回放 · 未接入"));
                });
                widgets::hint(
                    ui,
                    if self.capture.preview {
                        "来源：离线示例"
                    } else {
                        "来源：CAN108 · 主机接收时间"
                    },
                );
            });
            ui.add_space(14.0);
            Self::panel(ui, "运行日志", |ui| {
                if let Some(log) = &self.diagnostics {
                    ui.label(RichText::new(log.location()).small());
                    if ui
                        .add_enabled(log.available(), egui::Button::new("导出故障日志"))
                        .clicked()
                        && let Some(path) = rfd::FileDialog::new()
                            .add_filter("故障日志", &["jsonl"])
                            .set_file_name("YG_MC_HOST-faults.jsonl")
                            .save_file()
                    {
                        let result = log.export(path);
                        self.note(match result {
                            Ok(()) => "故障日志已导出".into(),
                            Err(error) => format!("日志导出失败：{error:#}"),
                        });
                    }
                } else {
                    widgets::hint(ui, "离线预览不记录设备故障日志。");
                }
                egui::ScrollArea::vertical()
                    .max_height((ui.available_height() - 20.0).max(160.0))
                    .stick_to_bottom(true)
                    .show(ui, |ui| {
                        for (index, message) in self.log.iter().enumerate() {
                            ui.horizontal_wrapped(|ui| {
                                ui.label(
                                    RichText::new(format!("{:03}", index + 1))
                                        .monospace()
                                        .color(Palette::from_ctx(ui.ctx()).muted),
                                );
                                ui.label(message);
                            });
                        }
                    });
            });
        });
    }

    pub(super) fn maintenance(&mut self, ui: &mut egui::Ui) {
        egui::ScrollArea::vertical().show(ui, |ui| {
            widgets::page_header(
                ui,
                "固件升级",
                "先选择升级对象与通信方式，再检查文件和执行条件。",
                "固件与备份",
            );
            ui.add_enabled_ui(!self.upgrading, |ui| {
                ui.columns(3, |columns| {
                    let app = columns[0].add(
                        egui::Button::new(RichText::new("升级 APP").strong())
                            .selected(self.upgrade_mode == UpgradeMode::App)
                            .min_size(Vec2::new(columns[0].available_width(), 46.0)),
                    );
                    widgets::hint(&mut columns[0], "CAN · 保留 Loader 与设备参数");
                    if app.clicked() {
                        self.upgrade_mode = UpgradeMode::App;
                    }
                    let bundle = columns[1].add(
                        egui::Button::new(RichText::new("安装 Loader + APP").strong())
                            .selected(self.upgrade_mode == UpgradeMode::Combined)
                            .min_size(Vec2::new(columns[1].available_width(), 46.0)),
                    );
                    widgets::hint(&mut columns[1], "J-Link / SWD · 初装或恢复整机固件");
                    if bundle.clicked() {
                        self.upgrade_mode = UpgradeMode::Combined;
                    }
                    columns[2]
                        .add_enabled(
                            false,
                            egui::Button::new("仅升级 Loader")
                                .min_size(Vec2::new(columns[2].available_width(), 46.0)),
                        )
                        .on_disabled_hover_text("独立 Loader 下载入口尚未接入");
                    widgets::hint(&mut columns[2], "未接入 · 当前不可执行");
                });
            });
            ui.add_space(10.0);
            if self.upgrade_mode != UpgradeMode::App {
                self.maintenance_panel.show(
                    ui,
                    &self.target,
                    !self.connected
                        && !self.upgrading
                        && !self.fixture_mode
                        && self.scan_rx.is_none(),
                );
            } else {
                Self::panel(ui, "01  固件包", |ui| {
                    ui.add_enabled_ui(!self.upgrading, |ui| {
                        ui.horizontal(|ui| {
                            let response = ui.add(
                                egui::TextEdit::singleline(&mut self.package_dir)
                                    .desired_width((ui.available_width() - 190.0).max(180.0))
                                    .hint_text(
                                        "包含 image.bin / manifest.bin / package.json 的目录",
                                    ),
                            );
                            if response.changed() {
                                self.package = None;
                            }
                            if ui.button("浏览").clicked()
                                && let Some(path) = rfd::FileDialog::new().pick_folder()
                            {
                                self.package_dir = path.display().to_string();
                                self.package = None;
                            }
                            if ui.button("离线检查").clicked() {
                                self.inspect_package();
                            }
                        });
                    });
                    if let Some(package) = &self.package {
                        ui.label(format!(
                            "APP {} / {} B / CRC32 {:08X}",
                            version(package.version),
                            package.image.len(),
                            package.crc32
                        ));
                    } else if self.package_dir.is_empty() {
                        ui.label("请选择 APP 包目录并执行离线检查。");
                    } else {
                        ui.label("请执行离线检查；仅写 APP 槽，保留参数与身份页。");
                    }
                });
                ui.add_space(8.0);
                let checks = [
                    (
                        "设备身份与目标匹配",
                        self.connected && self.snapshot.compatible(&self.target, &self.bound_uid),
                    ),
                    (
                        "Loader 版本匹配",
                        self.snapshot.identity_fresh()
                            && self.snapshot.loader_version == self.target.loader_version,
                    ),
                    (
                        "停机与近零速已确认",
                        self.snapshot.stop_confirmed
                            && self.snapshot.safe_idle(&self.target, &self.bound_uid),
                    ),
                    ("固件包检查通过", self.package.is_some()),
                ];
                ui.columns(2, |columns| {
                    Self::panel(&mut columns[0], "02  执行前检查", |ui| {
                        for (label, pass) in checks {
                            let color = if pass {
                                Palette::from_ctx(ui.ctx()).accent
                            } else {
                                Palette::from_ctx(ui.ctx()).muted
                            };
                            ui.horizontal(|ui| {
                                ui.label(RichText::new(if pass { "●" } else { "○" }).color(color));
                                ui.label(label);
                            });
                        }
                        ui.add_space(5.0);
                        ui.checkbox(&mut self.acknowledged, "已核对 UID 与架空台架");
                        ui.label(
                            RichText::new(
                                "33 V / 母线 3 A / 相 6 A。STOP 后仍需现场核对物理输出。",
                            )
                            .small()
                            .color(Palette::from_ctx(ui.ctx()).muted),
                        );
                    });
                    Self::panel(&mut columns[1], "03  下载与确认", |ui| {
                        ui.label(
                            RichText::new("升级进度")
                                .small()
                                .color(Palette::from_ctx(ui.ctx()).muted),
                        );
                        ui.add(
                            egui::ProgressBar::new(self.progress)
                                .corner_radius(3.0)
                                .desired_width(ui.available_width())
                                .show_percentage(),
                        );
                        ui.add_space(9.0);
                        let enabled = !self.fixture_mode
                            && self.connected
                            && !self.hard_stop
                            && !self.upgrading
                            && self.acknowledged
                            && checks.iter().all(|(_, pass)| *pass);
                        let response = ui.add_enabled(
                            enabled,
                            egui::Button::new("开始升级 APP（CAN）")
                                .min_size(Vec2::new(180.0, 34.0)),
                        );
                        self.control("app_upgrade", &response);
                        if response.clicked() {
                            self.progress = 0.0;
                            self.upgrading = true;
                            self.pause_capture();
                            self.send(Command::Upgrade {
                                directory: PathBuf::from(&self.package_dir),
                                acknowledged: self.acknowledged,
                            });
                            self.note(
                                "APP 升级已请求；等待实际版本、CRC、同 UID 及持久确认，不恢复运动",
                            );
                        }
                        ui.label(
                            RichText::new("升级期间独占通信通道，完成后保持停机。")
                                .small()
                                .color(Palette::from_ctx(ui.ctx()).muted),
                        );
                    });
                });
            }
            ui.add_space(10.0);
            egui::CollapsingHeader::new("参数、备份与设备资料").show(ui, |ui| {
                ui.horizontal(|ui| {
                    if ui.button("在线参数区").clicked() {
                        self.page = Page::Parameters;
                    }
                    if ui.button("完整Flash备份 / 保存参数查看").clicked() {
                        self.upgrade_mode = UpgradeMode::Combined;
                    }
                });
                ui.label("在线标量读写和 SWD 原始页备份使用独立流程。");
            });
        });
    }
}
