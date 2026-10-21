//! 工作区组合；尚未接入的业务只显示支持范围与禁用原因，不制造设备值或完成结果。

use super::*;
use crate::device::MAX_SPEED_RAD_S;

// 两处视图消费同一目录；在线参数服务接入前不建立可写的假草稿。
const PARAMETERS: [(&str, &str, &str, &str); 7] = [
    ("最大速度", "1", "rad/s", "以设备 READ 权限为准"),
    ("最大 Iq", "2", "A", "以设备 READ 权限为准"),
    ("速度环 Kp", "16", "内部系数", "当前通用写入口未开放"),
    ("速度环 Ki", "17", "内部系数", "当前通用写入口未开放"),
    ("持续向量电流", "0x60", "A", "停机可写，待主机接入"),
    ("前馈加速度上限", "0x6A", "rad/s²", "停机可写，待主机接入"),
    ("前馈电流上限", "0x6C", "A", "停机可写，待主机接入"),
];

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
        let available = ui.available_rect_before_wrap();
        let width = available.width().min(860.0);
        let area = egui::Rect::from_min_size(
            egui::pos2(available.center().x - width / 2.0, available.top() + 12.0),
            Vec2::new(width, (available.height() - 12.0).max(100.0)),
        );
        ui.scope_builder(egui::UiBuilder::new().max_rect(area),|ui| {
        egui::ScrollArea::vertical().show(ui, |ui| {
            ui.heading("连接设备");
            ui.label("配置接口 → 指定节点 → 识别身份与能力 → 进入工作区");
            ui.add_space(12.0);
            Self::panel(ui, "CAN FD / 通道 0 / 1M–5M", |ui| {
                ui.add_enabled_ui(!self.connected, |ui| {
                    ui.label("厂家 SDK DLL");
                    ui.horizontal(|ui| {
                        let width = (ui.available_width()-86.0).max(160.0);
                        ui.add_sized([width,30.0],egui::TextEdit::singleline(&mut self.dll).hint_text("选择 ControlCANFD.dll"));
                        if ui.button("浏览").clicked()
                            && let Some(path) = rfd::FileDialog::new().add_filter("DLL", &["dll"]).pick_file() {
                            self.dll = path.display().to_string();
                        }
                    });
                    ui.horizontal(|ui| {
                        ui.label("CAN 设备索引");
                        ui.add(egui::DragValue::new(&mut self.device_index).range(0..=15));
                        ui.label("目标节点");
                        ui.add(egui::DragValue::new(&mut self.node).range(1..=254));
                    });
                    egui::CollapsingHeader::new("高级：控制序号").show(ui, |ui| {
                        ui.horizontal(|ui| { ui.label("起点"); ui.add(egui::DragValue::new(&mut self.manual_sequence).range(0..=65535)); });
                        ui.label("0 恢复本机按 UID 保存的记录。首次接入或其他主机操作过时，须按真实历史填写非零起点。");
                    });
                });
                ui.add_space(10.0);
                if self.hard_stop {
                    ui.colored_label(Palette::from_ctx(ui.ctx()).danger,"硬停后需现场确认输出已关闭和接线状态。");
                    ui.checkbox(&mut self.intervention_ack,"已完成现场检查，允许重新连接");
                }
                ui.horizontal(|ui| {
                    let enabled = !self.fixture_mode && !self.connected && !self.dll.is_empty()
                        && (!self.hard_stop || self.intervention_ack);
                    let connect = ui.add_enabled(enabled,egui::Button::new("连接并识别"));
                    self.control("connect", &connect);
                    if connect.clicked() { self.connect(); }
                    if ui.add_enabled(self.connected && !self.upgrading,egui::Button::new("断开连接")).clicked() {
                        self.send(Command::Disconnect);
                    }
                    if ui.button(if self.connected {"返回工作台"} else {"离线查看工作台"}).clicked() { self.page = Page::Workbench; }
                });
                ui.label(RichText::new("连接只读取身份与状态，不自动启动电机或开始记录。").small());
            });
            ui.add_space(12.0);
            Self::panel(ui,"接口支持",|ui| {
                ui.label("CAN FD 指定节点连接：可用，待实机验收。");
                ui.label("自动发现 / 多设备选择：未接入。RS-485：暂无后端。");
                ui.label("APP 识别成功进入首页；Loader 独立驻留/救援连接尚待完善。");
                ui.label(format!("本地目标：G431 / layout {} / schema {} / Loader {}",self.target.layout,self.target.schema,version(self.target.loader_version)));
            });
        });
        });
    }

    pub(super) fn home(&mut self, ui: &mut egui::Ui) {
        egui::ScrollArea::vertical().show(ui, |ui| {
            ui.heading("设备与运行准备");
            ui.add_space(8.0);
            let identity = self.connected && self.snapshot.identity_fresh();
            let feedback = self.connected && self.snapshot.feedback_fresh();
            if !identity {
                ui.label("当前身份未知或过期；保留的历史值不能作为操作依据。");
            }
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
                            field(ui, "节点", self.node.to_string());
                            field(ui, "电机 ID", "未提供（与节点号分开）");
                            field(
                                ui,
                                "电机类型",
                                if identity {
                                    format!("协议枚举 {}", self.snapshot.motor_type)
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
                            field(
                                ui,
                                "SPEED 能力",
                                if self.snapshot.speed_supported() && identity {
                                    "设备已声明"
                                } else {
                                    "未声明 / 过期"
                                },
                            );
                        });
                });
            });
            ui.add_space(12.0);
            Self::panel(ui, "实时状态", |ui| {
                egui::Grid::new("feedback")
                    .num_columns(4)
                    .spacing([20.0, 10.0])
                    .show(ui, |ui| {
                        ui.label("位置");
                        ui.label(reading(self.snapshot.position_rad, "rad", feedback));
                        ui.label("速度");
                        ui.label(reading(self.snapshot.speed_rad_s, "rad/s", feedback));
                        ui.end_row();
                        ui.label("Iq");
                        ui.label(reading(self.snapshot.iq_a, "A", feedback));
                        ui.label("MCU 温度");
                        ui.label(reading(self.snapshot.temperature_c, "°C", feedback));
                        ui.end_row();
                        ui.label("母线电压");
                        ui.label(reading(self.snapshot.bus_v, "V", feedback));
                        ui.label("母线电流");
                        ui.label(reading(self.snapshot.bus_a, "A", feedback));
                        ui.end_row();
                        ui.label("状态 / 模式");
                        ui.label(if feedback {
                            format!("{} / {}", self.snapshot.motor_state, self.snapshot.mode)
                        } else {
                            "未知".into()
                        });
                        ui.label("故障");
                        ui.label(if feedback {
                            format!("0x{:08X}", self.snapshot.faults)
                        } else {
                            "未知".into()
                        });
                        ui.end_row();
                    });
            });
            ui.add_space(12.0);
            Self::panel(ui, "标定与辨识摘要", |ui| {
                ui.label("尚未接入设备任务与结果读取；没有可确认的最近结果、应用或持久化状态。");
            });
        });
    }

    pub(super) fn workbench(&mut self, ui: &mut egui::Ui) {
        let c = Palette::from_ctx(ui.ctx());
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
                    Ok(()) => self.note("开始观察 CAN108 趋势（主机接收时间）"),
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
                self.capture.pause();
            }
            if ui
                .add_enabled(
                    !self.capture.observing,
                    egui::Button::new("清空").frame(false),
                )
                .clicked()
            {
                self.capture = Capture::default();
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
            ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                ui.label(
                    RichText::new(if self.capture.preview {
                        "示例记录 · 非设备数据"
                    } else {
                        "CAN108 · 低速轮询趋势"
                    })
                    .small()
                    .color(if self.capture.preview {
                        c.warning
                    } else {
                        c.muted
                    }),
                );
            });
        });
        ui.add_space(6.0);
        let fresh = self.connected && self.snapshot.feedback_fresh();
        Frame::new()
            .fill(c.panel)
            .stroke(Stroke::new(1.0, c.line))
            .inner_margin(egui::Margin::symmetric(14, 6))
            .show(ui, |ui| {
                ui.spacing_mut().item_spacing.y = 0.0;
                ui.spacing_mut().interact_size.y = 16.0;
                let values = [
                    ("位置", self.snapshot.position_rad, "rad"),
                    ("速度", self.snapshot.speed_rad_s, "rad/s"),
                    ("Iq", self.snapshot.iq_a, "A"),
                    ("母线", self.snapshot.bus_v, "V"),
                    ("温度", self.snapshot.temperature_c, "°C"),
                ];
                ui.columns(5, |columns| {
                    for (column, (name, value, unit)) in columns.iter_mut().zip(values) {
                        column.horizontal(|ui| {
                            ui.label(RichText::new(name).small().color(c.muted));
                            ui.label(RichText::new(unit).small().color(c.muted));
                        });
                        column.horizontal(|ui| {
                            ui.label(
                                RichText::new(
                                    value.map(|v| format!("{v:.3}")).unwrap_or("—".into()),
                                )
                                .monospace()
                                .size(19.0)
                                .color(if fresh {
                                    c.ink
                                } else {
                                    c.muted
                                }),
                            );
                            if value.is_some() && !fresh {
                                ui.label(RichText::new("过期").small().color(c.warning));
                            } else if value.is_none() {
                                ui.label(RichText::new("未知").small().color(c.muted));
                            }
                        });
                    }
                });
            });
        ui.add_space(6.0);
        ui.label(
            RichText::new(if self.capture.uid.is_empty() {
                "尚无观察记录".into()
            } else {
                format!(
                    "记录  {}    ·    {} 个样本    ·    {}",
                    self.capture.uid,
                    self.capture.samples.len(),
                    self.capture
                        .observed_hz()
                        .map(|hz| format!("本段接收约 {hz:.2} Hz"))
                        .unwrap_or("频率待统计".into())
                )
            })
            .small()
            .color(Palette::from_ctx(ui.ctx()).muted),
        );
        let record_matches = self.capture.uid.is_empty() || self.capture.uid == self.bound_uid;
        egui::Panel::right("workbench_controls")
            .resizable(false)
            .exact_size(302.0)
            .frame(Frame::new().inner_margin(egui::Margin {
                left: 10,
                right: 0,
                top: 0,
                bottom: 0,
            }))
            .show(ui, |ui| {
                egui::ScrollArea::vertical().show(ui, |ui| {
                    Self::panel(ui, "运动控制", |ui| {
                        ui.horizontal(|ui| {
                            ui.label(RichText::new("SPEED").color(c.accent).strong());
                            ui.label(RichText::new("位置 / Iq 未开放").small().color(c.muted));
                        });
                        ui.add_space(4.0);
                        ui.horizontal(|ui| {
                            ui.label("目标速度");
                            ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                                ui.label(RichText::new("rad/s").small().color(c.muted));
                            });
                        });
                        ui.add_sized(
                            [ui.available_width(), 36.0],
                            egui::DragValue::new(&mut self.speed_rad_s)
                                .range(0.0..=MAX_SPEED_RAD_S)
                                .speed(0.1)
                                .fixed_decimals(3),
                        );
                        ui.label(
                            RichText::new("0–31.416 rad/s  ·  0–300 rpm")
                                .small()
                                .color(c.muted),
                        );
                        ui.add_space(4.0);
                        ui.checkbox(&mut self.acknowledged, "已核对 UID 与架空台架条件");
                        ui.label(
                            RichText::new("33 V / 母线 3 A / 相 6 A；Iq 不代替相电流监视。")
                                .small()
                                .color(c.muted),
                        );
                        let live = record_matches
                            && self.connected
                            && !self.fixture_mode
                            && !self.hard_stop
                            && !self.upgrading
                            && self.acknowledged;
                        ui.horizontal(|ui| {
                            let start = ui.add_enabled(
                                live && self.snapshot.safe_idle(&self.target, &self.bound_uid)
                                    && self.snapshot.speed_supported(),
                                egui::Button::new("零速启动").min_size(Vec2::new(120.0, 30.0)),
                            );
                            self.control("speed_start", &start);
                            if start.clicked() {
                                self.send(Command::Start {
                                    acknowledged: self.acknowledged,
                                });
                            }
                            let set = ui.add_enabled(
                                live && self.snapshot.can_set_speed(&self.target, &self.bound_uid),
                                egui::Button::new("设置目标").min_size(Vec2::new(120.0, 30.0)),
                            );
                            self.control("speed_set", &set);
                            if set.clicked() {
                                self.send(Command::SetSpeed {
                                    speed_rad_s: self.speed_rad_s,
                                    acknowledged: self.acknowledged,
                                });
                            }
                        });
                        let reason = if self.fixture_mode {
                            "预览不发送设备命令"
                        } else if self.upgrading {
                            "升级期间禁用运动"
                        } else if !self.connected {
                            "未连接；请先连接并识别设备"
                        } else if self.hard_stop {
                            "硬停；须现场确认输出"
                        } else if !record_matches {
                            "记录 UID 与当前设备不同；先导出并清空旧记录"
                        } else if !self.snapshot.compatible(&self.target, &self.bound_uid) {
                            "身份过期或目标不兼容"
                        } else if !self.snapshot.speed_supported() {
                            "设备未声明 SPEED 能力"
                        } else if !fresh {
                            "反馈过期"
                        } else if !self.acknowledged {
                            "等待台架确认"
                        } else {
                            "按钮随 IDLE/RUN、故障与限值更新"
                        };
                        ui.label(RichText::new(reason).small().color(c.muted));
                    });
                    ui.add_space(8.0);
                    Self::panel(ui, "常用参数", |ui| {
                        ui.spacing_mut().interact_size.y = 16.0;
                        for index in [2, 4, 6] {
                            let (name, _, unit, _) = PARAMETERS[index];
                            ui.horizontal(|ui| {
                                ui.label(name);
                                ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                                    ui.label(RichText::new(format!("— {unit}")).color(c.muted));
                                });
                            });
                        }
                        ui.label(
                            RichText::new("设备参数读写未接入，当前只显示目录。")
                                .small()
                                .color(c.warning),
                        );
                        if ui
                            .add_sized(
                                [ui.available_width(), 28.0],
                                egui::Button::new("打开参数配置  →").frame(false),
                            )
                            .clicked()
                        {
                            self.page = Page::Parameters;
                        }
                    });
                    ui.add_space(6.0);
                    ui.label(
                        RichText::new("全局 STOP 位于窗口右上角。通信确认仍需现场核对物理输出。")
                            .small()
                            .color(c.muted),
                    );
                });
            });
        let height = ((ui.available_height() - 12.0) / 3.0).max(90.0);
        for channel in 0..3 {
            let (hover, rect) = scope::plot(
                ui,
                &self.capture,
                channel,
                self.window_s,
                height,
                self.cursor_s,
                self.show_reference,
            );
            if self.fixture_mode {
                self.control_evidence
                    .push(serde_json::json!({"name":format!("plot_{channel}"),
                    "enabled":false,"rect":[rect.min.x,rect.min.y,rect.max.x,rect.max.y]}));
            }
            if let Some(time) = hover {
                self.cursor_s = Some(time);
            }
            if channel < 2 {
                ui.add_space(4.0);
            }
        }
    }

    pub(super) fn parameters(&mut self, ui: &mut egui::Ui) {
        ui.heading("参数配置");
        ui.label("未接入设备参数读写。下表为协议目录节选，完整目录待接入；不是设备参数值。");
        ui.add_space(10.0);
        ui.horizontal(|ui| {
            for (name, label) in [
                ("parameter_read", "读取设备"),
                ("parameter_write", "应用并读回"),
                ("parameter_save", "掉电保存"),
            ] {
                let response = ui
                    .add_enabled(false, egui::Button::new(label))
                    .on_disabled_hover_text("120/121/122 后端尚未接入主机");
                self.control(name, &response);
            }
        });
        ui.separator();
        egui::ScrollArea::both().show(ui, |ui| {
            egui::Grid::new("parameter_catalog")
                .striped(true)
                .min_col_width(125.0)
                .spacing([18.0, 12.0])
                .show(ui, |ui| {
                    for heading in ["参数", "ID", "单位", "设备值", "状态"] {
                        ui.strong(heading);
                    }
                    ui.end_row();
                    for (name, id, unit, state) in PARAMETERS {
                        ui.label(name);
                        ui.monospace(id);
                        ui.label(unit);
                        ui.label("未读取");
                        ui.label(state);
                        ui.end_row();
                    }
                });
            ui.add_space(14.0);
            ui.label("操作流程：读取 → 检查修改 → 版本校验与写入 → 读回 → 掉电保存。");
            ui.label("设备活动值、草稿和已保存版本分别确认；写入与保存功能尚未接入。");
        });
    }

    pub(super) fn calibration(&mut self, ui: &mut egui::Ui) {
        const TASKS: [(&str, &str, &str); 4] = [
            (
                "角度传感器 LUT",
                "kind 1",
                "固件有任务协议，主机执行/进度/结果尚未接入",
            ),
            (
                "电气零位",
                "kind 2",
                "固件有任务协议，主机尚未接入；不同于机械位置置零",
            ),
            ("机械零位", "kind 6", "固件有任务协议，主机尚未接入"),
            ("齿槽转矩标定", "kind 3", "当前固件已撤回，不可执行"),
        ];
        ui.heading("标定");
        ui.add_space(8.0);
        ui.horizontal_wrapped(|ui| {
            for (index, (name, _, _)) in TASKS.iter().enumerate() {
                ui.selectable_value(&mut self.task_index, index, *name);
            }
        });
        ui.add_space(12.0);
        let (name, kind, support) = TASKS[self.task_index];
        Self::panel(ui, name, |ui| {
            ui.label(support);
            ui.label(RichText::new(kind).small());
            ui.add_space(10.0);
            egui::Grid::new("task_facts")
                .spacing([20.0, 10.0])
                .show(ui, |ui| {
                    field(ui, "执行状态", "未发起");
                    field(ui, "当前阶段", "无设备任务");
                    field(ui, "结果质量", "未评估");
                    field(ui, "应用 / 保存", "未知，没有设备结果");
                });
            ui.add_space(12.0);
            let start = ui
                .add_enabled(false, egui::Button::new("开始任务"))
                .on_disabled_hover_text(support);
            self.control("calibration_start", &start);
            ui.add_enabled(false, egui::Button::new("请求取消"));
            ui.label("接入后显示设备真实阶段与结果状态；取消须等待设备确认。");
        });
    }

    pub(super) fn identification(&mut self, ui: &mut egui::Ui) {
        ui.heading("辨识");
        ui.label("以下功能尚不可执行，不会发送候选或历史命令。");
        ui.add_space(12.0);
        Self::panel(ui, "任务能力", |ui| {
            egui::Grid::new("identification_catalog")
                .striped(true)
                .spacing([24.0, 16.0])
                .show(ui, |ui| {
                    ui.strong("任务");
                    ui.strong("结果含义 / 单位");
                    ui.strong("状态");
                    ui.end_row();
                    for (name, unit, state) in [
                        ("电阻辨识", "相 / 相间电阻 · Ω", "当前固件已撤回"),
                        ("电感辨识", "Ld / Lq / 等效值 · H", "暂无可用任务契约"),
                        ("转动惯量辨识", "明确参照轴 · kg·m²", "暂无可用任务契约"),
                        (
                            "摩擦力辨识",
                            "摩擦转矩 N·m / 黏性系数 N·m·s/rad",
                            "当前固件已撤回",
                        ),
                    ] {
                        ui.label(name);
                        ui.label(unit);
                        ui.label(state);
                        ui.end_row();
                    }
                });
            ui.separator();
            ui.label("任务执行、质量、结果应用、掉电保存和 PC 导出分别确认。");
        });
    }

    pub(super) fn records(&mut self, ui: &mut egui::Ui) {
        ui.heading("记录与诊断");
        ui.add_space(8.0);
        Self::panel(ui, "当前趋势记录", |ui| {
            ui.horizontal_wrapped(|ui| {
                ui.label(format!(
                    "UID：{}",
                    if self.capture.uid.is_empty() {
                        "未绑定"
                    } else {
                        &self.capture.uid
                    }
                ));
                ui.label(format!(
                    "{} 个样本 / 间隔中断 {} / 容量淘汰 {}",
                    self.capture.samples.len(),
                    self.capture.gaps,
                    self.capture.evicted
                ));
                ui.label(
                    self.capture
                        .observed_hz()
                        .map(|hz| format!("本段接收约 {hz:.2} Hz"))
                        .unwrap_or("频率待统计".into()),
                );
            });
            ui.label(if self.capture.preview {
                "来源：离线示例；不代表设备数据"
            } else {
                "来源：CAN108；时间为主机单调接收时间，不是设备同步采样时间"
            });
            ui.horizontal(|ui| {
                if ui
                    .add_enabled(
                        !self.capture.samples.is_empty(),
                        egui::Button::new("导出 CSV"),
                    )
                    .clicked()
                {
                    self.export_capture();
                }
                if ui.button("返回调试工作台").clicked() {
                    self.page = Page::Workbench;
                }
                ui.add_enabled(false, egui::Button::new("记录回放 · 未接入"));
            });
        });
        ui.add_space(12.0);
        Self::panel(ui, "运行记录", |ui| {
            egui::ScrollArea::vertical()
                .max_height((ui.available_height() - 20.0).max(120.0))
                .stick_to_bottom(true)
                .show(ui, |ui| {
                    for (index, message) in self.log.iter().enumerate() {
                        ui.horizontal_wrapped(|ui| {
                            ui.monospace(format!("{:03}", index + 1));
                            ui.label(message);
                        });
                    }
                });
        });
    }

    pub(super) fn maintenance(&mut self, ui: &mut egui::Ui) {
        egui::ScrollArea::vertical().show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.heading("设备维护");
                ui.add_enabled_ui(!self.upgrading, |ui| {
                    ui.selectable_value(&mut self.upgrade_mode, UpgradeMode::App, "APP / CAN");
                    ui.selectable_value(
                        &mut self.upgrade_mode,
                        UpgradeMode::Loader,
                        "Loader / SWD",
                    );
                    ui.selectable_value(
                        &mut self.upgrade_mode,
                        UpgradeMode::Combined,
                        "Loader + APP / SWD",
                    );
                });
            });
            ui.add_space(8.0);
            if self.upgrade_mode != UpgradeMode::App {
                Self::panel(ui, "安装工具尚未接入", |ui| {
                    ui.label("当前 Loader 仅允许在线写 APP 槽，不能覆盖自身。");
                    ui.label(if self.upgrade_mode == UpgradeMode::Loader {
                        "Loader 单独安装需独占 SWD，仅写 Loader 槽。"
                    } else {
                        "联合安装需独占 SWD，分别核对 Loader 与 APP 的写域。"
                    });
                    ui.label(
                        "执行工具、参数备份、身份保护及写后读回将在后续阶段接入；禁止整片擦除。",
                    );
                    let response = ui.add_enabled(false, egui::Button::new("执行安装 · 未接入"));
                    self.control("swd_install", &response);
                });
            } else {
                Self::panel(ui, "1 选择完整 APP 包", |ui| {
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
                    } else {
                        ui.label("包未通过检查；仅写 APP 槽，保留参数与身份页。");
                    }
                });
                ui.add_space(8.0);
                let checks = [
                    (
                        "同一 UID / 产品 / schema / ABI",
                        self.connected && self.snapshot.compatible(&self.target, &self.bound_uid),
                    ),
                    (
                        "配套 Loader",
                        self.snapshot.identity_fresh()
                            && self.snapshot.loader_version == self.target.loader_version,
                    ),
                    (
                        "STOP 最终 0/0、IDLE 与近零速",
                        self.snapshot.stop_confirmed
                            && self.snapshot.safe_idle(&self.target, &self.bound_uid),
                    ),
                    ("完整包校验", self.package.is_some()),
                ];
                Self::panel(ui, "2 执行前检查", |ui| {
                    for (label, pass) in checks {
                        ui.label(format!("{}  {label}", if pass { "通过" } else { "等待" }));
                    }
                    ui.checkbox(
                        &mut self.acknowledged,
                        "已核对 UID 与架空台架，33 V / 母线 3 A / 相 6 A",
                    );
                    ui.label("可先使用右上角 STOP 获得通信停机确认；物理输出须现场核对。");
                });
                ui.add_space(8.0);
                Self::panel(ui, "3 下载、激活与持久确认", |ui| {
                    ui.add(
                        egui::ProgressBar::new(self.progress)
                            .corner_radius(2.0)
                            .desired_width(ui.available_width())
                            .show_percentage(),
                    );
                    let enabled = !self.fixture_mode
                        && self.connected
                        && !self.hard_stop
                        && !self.upgrading
                        && self.acknowledged
                        && checks.iter().all(|(_, pass)| *pass);
                    let response = ui.add_enabled(enabled, egui::Button::new("开始 APP 升级"));
                    self.control("app_upgrade", &response);
                    if response.clicked() {
                        self.progress = 0.0;
                        self.upgrading = true;
                        self.capture.pause();
                        self.send(Command::Upgrade {
                            directory: PathBuf::from(&self.package_dir),
                            acknowledged: self.acknowledged,
                        });
                        self.note(
                            "APP 升级已请求；等待实际版本、CRC、同 UID 及持久确认，不恢复运动",
                        );
                    }
                    ui.label("升级独占通信通道；切换页面不会中断，完成后保持停机。");
                });
            }
            ui.add_space(10.0);
            Self::panel(ui, "参数与设备资料", |ui| {
                ui.horizontal(|ui| {
                    if ui.button("在线参数区").clicked() {
                        self.page = Page::Parameters;
                    }
                    ui.add_enabled(false, egui::Button::new("原始参数备份 / 恢复 · 未接入"));
                });
                ui.label("在线标量读写和 SWD 原始页备份使用独立流程。");
            });
        });
    }
}
