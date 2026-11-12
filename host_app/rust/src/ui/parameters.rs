//! 统一参数表；设备缓存绑定本次连接，编辑草稿不回写设备值。
use super::*;
use crate::parameters::{self as param, Action, Report, Value};
use std::collections::{BTreeMap, VecDeque};
use std::time::Instant;

pub(super) struct State {
    category: String,
    group: String,
    search: String,
    model_tab: bool,
    uid: String,
    active: BTreeMap<u8, Value>,
    saved: BTreeMap<u8, Value>,
    unsupported: Vec<u8>,
    drafts: BTreeMap<u8, String>,
    selected: Option<u8>,
    state: Option<Value>,
    read_at: Option<Instant>,
    remaining: VecDeque<u8>,
    reading_saved: bool,
}

impl Default for State {
    fn default() -> Self {
        Self {
            category: "all".into(),
            group: String::new(),
            search: String::new(),
            model_tab: false,
            uid: String::new(),
            active: BTreeMap::new(),
            saved: BTreeMap::new(),
            unsupported: vec![],
            drafts: BTreeMap::new(),
            selected: None,
            state: None,
            read_at: None,
            remaining: VecDeque::new(),
            reading_saved: false,
        }
    }
}

impl State {
    pub(super) fn focus_group(&mut self, group: &str) {
        self.category = "control".into();
        self.group = group.into();

        self.model_tab = false;
        self.search.clear();
    }

    pub(super) fn studio_values(&mut self, uid: &str) {
        self.uid = uid.into();
        for (id, text) in [(11, "0.6"), (12, "3200"), (13, "0.6"), (14, "3200")] {
            if let Some(raw) = param::scalar(id).and_then(|s| s.encode(text).ok()) {
                let value = Value {
                    raw,
                    revision: 1,
                    saved_revision: 1,
                    flags: 1,
                };
                self.active.insert(id, value);
                self.saved.insert(id, value);
                self.state = Some(value);
            }
        }
    }

    pub(super) fn cancel_read(&mut self) {
        self.remaining.clear();
    }

    pub(super) fn preview(name: &str) -> Self {
        let (category, group) = if name.contains("speed") {
            ("control", "speed")
        } else if name.contains("position") {
            ("control", "position")
        } else if name.contains("motor") {
            ("motor", "electrical")
        } else if name.contains("threshold") {
            ("threshold", "thermal")
        } else if name.contains("all") {
            ("all", "")
        } else {
            ("control", "current")
        };
        Self {
            category: category.into(),
            group: group.into(),
            model_tab: name == "parameters-yq",
            ..Self::default()
        }
    }
}

impl HostApp {
    fn parameter_ready(&self) -> bool {
        self.motion_ready()
            && self.snapshot.stop_confirmed
            && self.snapshot.safe_idle(&self.target, &self.bound_uid)
            && self.snapshot.features & param::FEATURE != 0
    }

    fn parameter_read(&mut self, saved: bool) {
        if !self.parameter_ready() {
            return;
        }
        if self.parameter_state.uid != self.bound_uid {
            self.parameter_state = State {
                uid: self.bound_uid.clone(),
                ..State::default()
            };
        }
        self.parameter_state.remaining = param::catalog().iter().map(|entry| entry.id).collect();
        self.parameter_state.reading_saved = saved;
        if saved {
            self.parameter_state.saved.clear();
        } else {
            self.parameter_state.active.clear();
            self.parameter_state.unsupported.clear();
        }
        self.operation_pending = true;
        self.parameter_read_next();
    }

    fn parameter_read_next(&mut self) {
        let ids: Vec<_> = (0..4)
            .filter_map(|_| self.parameter_state.remaining.pop_front())
            .collect();
        if ids.is_empty() {
            self.operation_pending = false;
            self.note("参数目录读取完成；无接口项目保持未接入标记。");
        } else {
            self.send(Command::Parameters(Action::Read {
                ids,
                saved: self.parameter_state.reading_saved,
            }));
        }
    }

    pub(super) fn receive_parameters(&mut self, report: Report) {
        if report.uid != self.bound_uid || report.uid != self.parameter_state.uid {
            self.parameter_state.remaining.clear();
            self.operation_pending = false;
            return;
        }
        let old_revision = self.parameter_state.state.map(|v| v.revision);
        if old_revision.is_some_and(|revision| revision != report.state.revision)
            && !self.parameter_state.remaining.is_empty()
        {
            self.parameter_state.remaining.clear();
            self.note("读取期间参数修订改变，已停止批次；请重新读取。");
        }
        self.parameter_state.state = Some(report.state);
        self.parameter_state.read_at = Some(Instant::now());
        self.parameter_state.unsupported.extend(report.unsupported);
        if report.saved {
            self.parameter_state.saved.extend(report.values);
        } else {
            for (id, value) in report.values {
                self.parameter_state.drafts.remove(&id);
                self.parameter_state.active.insert(id, value);
            }
        }
        self.parameter_read_next();
    }

    fn parameter_extension(&self, field: &str) -> Option<String> {
        if self.snapshot.uid.is_empty() {
            return None;
        }
        let s = &self.snapshot;
        let info = s.active_configuration.as_ref();
        Some(match field {
            "preset.model_name" => crate::motor_configuration::model_name(info?.model).into(),
            "axis_profile.joint_type" => crate::motor_configuration::axis_name(self.node).into(),
            "GET_INFO.sn" => s.sn.clone(),
            "GET_INFO.uid" => s.uid.clone(),
            "GET_INFO.product" => format!("0x{:08X}", s.product),
            "GET_INFO.app_version" => version(s.app_version),
            "GET_INFO.loader_version" => version(s.loader_version),
            "GET_INFO.hardware" => format!("Rev {}", s.hardware),
            "schema_version" => s.schema.to_string(),
            "GET_INFO.abi" => format!("0x{:08X}", s.abi),
            "GET_CAPS.features" => format!("0x{:08X}", s.features),
            "GET_CAPS.supported_modes" => format!("0x{:04X}", s.modes),
            "axis_profile.config_revision" => info?.revision.to_string(),
            _ => return None,
        })
    }

    pub(super) fn control_loop_summary(&mut self, ui: &mut egui::Ui, label: &str, groups: &[&str]) {
        widgets::control_group(ui, label, |ui| {
            for &group in groups {
                for row in param::presentation()
                    .rows
                    .iter()
                    .filter(|r| r.group == group && r.id.is_some())
                    .take(if groups.len() == 1 { 4 } else { 2 })
                {
                    let value = row
                        .id
                        .and_then(|id| self.parameter_state.active.get(&id).zip(param::scalar(id)))
                        .filter(|_| {
                            self.parameter_state.uid == self.bound_uid && !self.bound_uid.is_empty()
                        })
                        .map(|(v, s)| {
                            format!(
                                "{}{}",
                                s.format(v.raw),
                                if self.connected
                                    && self
                                        .parameter_state
                                        .state
                                        .is_some_and(|state| state.revision == v.revision)
                                {
                                    ""
                                } else {
                                    " · 历史"
                                }
                            )
                        });
                    ui.horizontal(|ui| {
                        let width = (ui.available_width() * 0.62).max(66.0);
                        ui.add_sized(
                            [width, 19.0],
                            egui::Label::new(
                                RichText::new(format!("{} [{}]", row.name, row.unit)).size(12.0),
                            )
                            .truncate(),
                        )
                        .on_hover_text(&row.name);
                        ui.monospace(value.as_deref().unwrap_or("—"));
                    });
                }
            }
            ui.horizontal_wrapped(|ui| {
                for &group in groups {
                    let name = match group {
                        "current" => "电流环",
                        "speed" => "速度环",
                        _ => "位置环",
                    };
                    if ui.small_button(format!("查看{name}参数")).clicked() {
                        self.parameter_state.focus_group(group);
                        self.page = Page::Parameters;
                    }
                }
            });
        });
    }

    pub(super) fn parameter_workspace(&mut self, ui: &mut egui::Ui) {
        let c = Palette::from_ctx(ui.ctx());
        ui.horizontal(|ui| {
            let search_width = (ui.available_width() - 94.0).max(160.0);
            let input = egui::TextEdit::singleline(&mut self.parameter_state.search)
                .hint_text(RichText::new("搜索名称、参数索引或字段…").color(Color32::from_gray(98)))
                .background_color(Color32::WHITE)
                .text_color(Color32::from_gray(22))
                .margin(Vec2::new(10.0, 7.0));
            if ui
                .scope(|ui| {
                    ui.set_style(ui.ctx().style_of(egui::Theme::Light));
                    ui.add_sized([search_width, 34.0], input)
                })
                .inner
                .changed()
            {
                self.parameter_state.model_tab = false;
            }
            ui.label(
                RichText::new(format!("{} 项", param::presentation().rows.len()))
                    .color(c.muted)
                    .strong(),
            );
        });
        ui.add_space(3.0);
        ui.horizontal_wrapped(|ui| {
            ui.selectable_value(&mut self.parameter_state.model_tab, false, "全参数表");
            ui.selectable_value(&mut self.parameter_state.model_tab, true, "型号配置");
            if !self.parameter_state.model_tab {
                ui.separator();
                egui::ComboBox::from_id_salt("param_tree_compact")
                    .height(380.0)
                    .selected_text(self.category_label())
                    .show_ui(ui, |ui| self.parameter_tree(ui));
                ui.separator();
                self.parameter_read_tools(ui);
            }
        });
        if self.parameter_state.model_tab {
            egui::ScrollArea::vertical().show(ui, |ui| self.motor_configuration_panel(ui));
            return;
        }
        egui::ScrollArea::both()
            .id_salt("parameter_canvas")
            .auto_shrink([false, false])
            .show(ui, |ui| {
                ui.set_min_width(ui.available_width().max(570.0));
                self.parameter_grid(ui);
            });
    }
    fn category_label(&self) -> &str {
        if let Some(label) = param::presentation()
            .groups
            .get(&self.parameter_state.category)
            .and_then(|groups| groups.get(&self.parameter_state.group))
        {
            return label;
        }
        match self.parameter_state.category.as_str() {
            "motor" => "电机参数",
            "control" => "控制参数",
            "threshold" => "阈值参数",
            _ => "全部参数",
        }
    }

    fn parameter_tree(&mut self, ui: &mut egui::Ui) {
        for (key, title) in [
            ("all", "全部参数   134"),
            ("motor", "电机参数   85"),
            ("control", "控制参数   19"),
            ("threshold", "阈值参数   30"),
        ] {
            let active = self.parameter_state.category == key;
            if ui
                .selectable_label(active && self.parameter_state.group.is_empty(), title)
                .clicked()
            {
                self.parameter_state.category = key.into();
                self.parameter_state.group.clear();
                self.parameter_state.search.clear();
            }
            if key == "control" || (active && key != "all") {
                ui.indent(key, |ui| {
                    if let Some(groups) = param::presentation().groups.get(key) {
                        let ordered: Vec<_> = if key == "control" {
                            ["current", "speed", "position"]
                                .into_iter()
                                .filter_map(|k| groups.get_key_value(k))
                                .collect()
                        } else {
                            groups.iter().collect()
                        };
                        for (group, label) in ordered {
                            if ui
                                .selectable_label(
                                    active && self.parameter_state.group == *group,
                                    label,
                                )
                                .clicked()
                            {
                                self.parameter_state.category = key.into();
                                self.parameter_state.group = group.clone();
                                self.parameter_state.search.clear();
                            }
                        }
                    }
                });
            }
        }
    }

    fn parameter_read_tools(&mut self, ui: &mut egui::Ui) {
        let c = Palette::from_ctx(ui.ctx());
        let ready = self.parameter_ready();
        let same_session = self.parameter_state.uid == self.bound_uid && !self.bound_uid.is_empty();
        for (name, label, saved) in [
            ("parameter_read", "读取设备参数", false),
            ("parameter_read_saved", "读取 Flash", true),
        ] {
            let response = ui.add_enabled(ready, egui::Button::new(label));
            self.control(name, &response);
            if response.clicked() {
                self.parameter_read(saved);
            }
        }
        let state = self.parameter_state.state;
        if self.operation_pending && !self.parameter_state.remaining.is_empty() {
            ui.spinner();
            widgets::hint(
                ui,
                &format!("剩余 {} 项", self.parameter_state.remaining.len()),
            );
        } else if let Some(state) = state.filter(|_| same_session) {
            widgets::hint(
                ui,
                &format!(
                    "活动 r{}  /  保存 {}",
                    state.revision,
                    if state.flags & 1 == 0 {
                        "未知".into()
                    } else {
                        format!("r{}", state.saved_revision)
                    }
                ),
            );
            ui.label(
                RichText::new(if state.flags & 4 != 0 {
                    "保存失败"
                } else if state.flags & 2 != 0 {
                    "RAM 未保存"
                } else {
                    "已同步"
                })
                .small()
                .color(if state.flags & 6 != 0 {
                    c.warning
                } else {
                    c.accent
                }),
            );
        } else {
            widgets::hint(ui, "尚未读取设备参数");
        }
    }
    fn parameter_grid(&mut self, ui: &mut egui::Ui) {
        let c = Palette::from_ctx(ui.ctx());
        let ready = self.parameter_ready();
        let same_session = self.parameter_state.uid == self.bound_uid && !self.bound_uid.is_empty();

        ui.add_space(10.0);
        let rows: Vec<_> = param::presentation()
            .rows
            .iter()
            .filter(|row| {
                if self.parameter_state.search.trim().is_empty() {
                    (self.parameter_state.category == "all"
                        || row.category == self.parameter_state.category)
                        && (self.parameter_state.group.is_empty()
                            || row.group == self.parameter_state.group)
                } else {
                    format!(
                        "{} {} {}",
                        row.name,
                        row.field,
                        row.id.map_or(String::new(), |v| v.to_string())
                    )
                    .to_lowercase()
                    .contains(&self.parameter_state.search.trim().to_lowercase())
                }
            })
            .collect();
        let table_height = (ui.clip_rect().bottom() - ui.cursor().min.y - 114.0).max(110.0);
        let width = (ui.available_width() - 5.0 * ui.spacing().item_spacing.x - 23.0).max(490.0);
        let widths = [0.28, 0.10, 0.14, 0.14, 0.15, 0.19].map(|fraction| width * fraction);
        Frame::new()
            .fill(if ui.ctx().theme() == egui::Theme::Dark {
                Color32::from_rgb(20, 32, 49)
            } else {
                c.panel
            })
            .stroke(Stroke::new(1.0, c.line))
            .corner_radius(8.0)
            .inner_margin(8.0)
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    for (text, width) in [
                        "参数名称",
                        "单位",
                        "活动值",
                        "Flash 值",
                        "待写值",
                        "权限 / 来源",
                    ]
                    .into_iter()
                    .zip(widths)
                    {
                        ui.allocate_ui_with_layout(
                            Vec2::new(width, 28.0),
                            if matches!(text, "活动值" | "Flash 值") {
                                Layout::right_to_left(Align::Center)
                            } else {
                                Layout::left_to_right(Align::Center)
                            },
                            |ui| {
                                ui.set_min_width(width);
                                ui.label(RichText::new(text).small().color(c.muted).strong());
                            },
                        );
                    }
                });
                ui.separator();
                egui::ScrollArea::vertical()
                    .id_salt((
                        "parameter_rows",
                        &self.parameter_state.category,
                        &self.parameter_state.group,
                    ))
                    .max_height(table_height)
                    .auto_shrink([false, false])
                    .show(ui, |ui| {
                        if rows.is_empty() {
                            widgets::hint(ui, "没有匹配参数");
                        }
                        for (index, row) in rows.iter().enumerate() {
                            ui.push_id(&row.key, |ui| {
                                self.parameter_table_row(
                                    ui,
                                    row,
                                    widths,
                                    index,
                                    ready,
                                    same_session,
                                )
                            });
                        }
                    });
            });
        ui.add_space(4.0);
        ui.horizontal(|ui| {
            widgets::hint(
                ui,
                &format!("{} / {} 项", rows.len(), param::presentation().rows.len()),
            );
            ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                let state = self.parameter_state.state.filter(|_| same_session);
                let save = ui.add_enabled(
                    ready && state.is_some_and(|v| v.flags != 1),
                    egui::Button::new("保存至 Flash"),
                );
                self.control("parameter_save", &save);
                if save.clicked() {
                    self.operation_pending = true;
                    self.send(Command::Parameters(Action::Save {
                        revision: state.unwrap().revision,
                    }));
                }
                let edit = self.parameter_state.selected.and_then(|id| {
                    let entry = param::scalar(id)?;
                    let value = self.parameter_state.active.get(&id)?;
                    let raw = entry.encode(self.parameter_state.drafts.get(&id)?).ok()?;
                    (Some(value.revision) == state.map(|v| v.revision) && value.raw != raw)
                        .then_some((id, raw, value.revision))
                });
                let apply = ui.add_enabled(
                    ready && edit.is_some(),
                    egui::Button::new(self.parameter_state.selected.map_or_else(
                        || "应用选中项并读回".into(),
                        |id| format!("应用 ID {id} 并读回"),
                    )),
                );
                self.control("parameter_write", &apply);
                if apply.clicked() {
                    let (id, raw, revision) = edit.unwrap();
                    self.operation_pending = true;
                    self.send(Command::Parameters(Action::Write { id, raw, revision }));
                }
            });
        });
        if let Some(at) = self.parameter_state.read_at.filter(|_| same_session) {
            widgets::hint(
                ui,
                &format!(
                    "本次回读距今 {:.1} s；修改后旧修订须重新读取。",
                    at.elapsed().as_secs_f64()
                ),
            );
        }
        widgets::hint(
            ui,
            if self.studio.is_some() {
                "离线示例 · 设备读写已关闭"
            } else if !self.connected {
                "连接设备并读取参数后可编辑。"
            } else if !ready {
                "参数操作需要失能、STOP 确认空闲和兼容固件；操作期间禁止使能。"
            } else {
                "编辑一项后应用并回读；保存至 Flash 是独立操作。固件负责最终范围与状态校验。"
            },
        );
    }
    fn parameter_table_row(
        &mut self,
        ui: &mut egui::Ui,
        row: &param::Row,
        widths: [f32; 6],
        index: usize,
        ready: bool,
        same_session: bool,
    ) {
        let c = Palette::from_ctx(ui.ctx());
        let scalar = row.id.and_then(param::scalar);
        let current = row
            .id
            .and_then(|id| self.parameter_state.active.get(&id))
            .filter(|_| same_session)
            .copied();
        let extension = self.parameter_extension(&row.field);
        let old = !self.connected
            || !same_session
            || current.is_some_and(|v| {
                Some(v.revision) != self.parameter_state.state.map(|s| s.revision)
            });
        let value = current
            .zip(scalar)
            .map(|(v, s)| s.format(v.raw))
            .or(extension);
        let fill = if ui.ctx().theme() == egui::Theme::Dark {
            if index.is_multiple_of(2) {
                Color32::from_rgb(20, 32, 50)
            } else {
                Color32::from_rgb(25, 39, 59)
            }
        } else if index.is_multiple_of(2) {
            c.panel
        } else {
            c.background
        };
        Frame::new()
            .fill(fill)
            .inner_margin(egui::Margin::symmetric(0, 4))
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    for (column, width) in widths.into_iter().enumerate() {
                        ui.allocate_ui_with_layout(
                            Vec2::new(width, 42.0),
                            Layout::top_down(Align::Min),
                            |ui| {
                                ui.set_min_height(42.0);
                                ui.set_min_width(width);
                                ui.set_max_width(width);
                                match column {
                                    0 => {
                                        ui.add(
                                            egui::Label::new(RichText::new(&row.name).strong())
                                                .truncate(),
                                        )
                                        .on_hover_text(&row.name);
                                        let group = param::presentation()
                                            .groups
                                            .get(&row.category)
                                            .and_then(|groups| groups.get(&row.group))
                                            .map_or("", String::as_str);
                                        let field = row.id.map_or_else(
                                            || group.to_string(),
                                            |id| format!("{group} · ID {id}"),
                                        );
                                        ui.add(
                                            egui::Label::new(
                                                RichText::new(field).small().color(c.muted),
                                            )
                                            .truncate(),
                                        )
                                        .on_hover_text(&row.field);
                                    }
                                    1 => {
                                        ui.add(egui::Label::new(&row.unit).truncate())
                                            .on_hover_text(&row.unit);
                                    }
                                    2 | 3 => {
                                        let saved = row
                                            .id
                                            .and_then(|id| self.parameter_state.saved.get(&id))
                                            .filter(|_| same_session)
                                            .copied();
                                        let text = if column == 2 {
                                            value.clone()
                                        } else {
                                            saved.zip(scalar).map(|(v, s)| s.format(v.raw))
                                        };
                                        let stale = if column == 2 {
                                            old
                                        } else {
                                            !self.connected
                                                || saved.is_some_and(|v| {
                                                    Some(v.saved_revision)
                                                        != self
                                                            .parameter_state
                                                            .state
                                                            .map(|s| s.saved_revision)
                                                })
                                        };
                                        ui.with_layout(Layout::top_down(Align::Max), |ui| {
                                            ui.add(
                                                egui::Label::new(
                                                    RichText::new(text.as_deref().unwrap_or("—"))
                                                        .monospace()
                                                        .color(if stale { c.muted } else { c.ink }),
                                                )
                                                .truncate(),
                                            )
                                            .on_hover_text(text.as_deref().unwrap_or("未读取"));
                                            if stale && text.is_some() {
                                                widgets::hint(ui, "历史");
                                            }
                                        });
                                    }
                                    4 => {
                                        if let Some((scalar, current)) =
                                            scalar.zip(current).filter(|(s, _)| s.access == 3)
                                        {
                                            let draft = self
                                                .parameter_state
                                                .drafts
                                                .entry(scalar.id)
                                                .or_insert_with(|| scalar.format(current.raw));
                                            let edit = ui.add_enabled(
                                                ready && !old,
                                                egui::TextEdit::singleline(draft)
                                                    .desired_width(width - 4.0),
                                            );
                                            if edit.has_focus() || edit.changed() {
                                                self.parameter_state.selected = row.id;
                                            }
                                            if let Err(error) = scalar.encode(draft) {
                                                ui.label(
                                                    RichText::new(error.to_string())
                                                        .small()
                                                        .color(c.warning),
                                                );
                                            }
                                        } else {
                                            widgets::hint(
                                                ui,
                                                if scalar.is_some_and(|s| s.access == 3) {
                                                    "先读取设备值"
                                                } else {
                                                    "—"
                                                },
                                            );
                                        }
                                    }
                                    _ => {
                                        ui.label(
                                            RichText::new(if row.origin == "compile" {
                                                "编译配置 · 未回读"
                                            } else if row.id.is_some_and(|id| {
                                                self.parameter_state.unsupported.contains(&id)
                                            }) {
                                                "设备未支持"
                                            } else if scalar.is_none() && value.is_none() {
                                                "扩展 · 未接入"
                                            } else {
                                                &row.access
                                            })
                                            .small()
                                            .color(c.muted),
                                        );
                                        ui.add(
                                            egui::Label::new(
                                                RichText::new(if scalar.is_some() {
                                                    "CAN 参数服务"
                                                } else if value.is_some() {
                                                    "设备身份回读"
                                                } else if row.origin == "compile" {
                                                    "固件编译定义"
                                                } else {
                                                    "暂无读取接口"
                                                })
                                                .small()
                                                .color(c.muted),
                                            )
                                            .truncate(),
                                        )
                                        .on_hover_text(&row.source);
                                    }
                                }
                            },
                        );
                    }
                });
            })
            .response
            .on_hover_text(&row.key);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mismatched_uid_cannot_replace_cache_or_submit_remaining_reads() {
        let mut app = HostApp::disconnected(Target::bundled().unwrap(), None);
        app.bound_uid = "A".into();
        app.parameter_state.uid = "A".into();
        app.parameter_state.remaining.push_back(12);
        app.operation_pending = true;
        let (tx, rx) = mpsc::channel();
        app.command_tx = Some(tx);
        app.receive_parameters(Report {
            uid: "B".into(),
            saved: false,
            values: BTreeMap::new(),
            unsupported: vec![],
            state: Value {
                raw: 0,
                revision: 4,
                saved_revision: 4,
                flags: 1,
            },
        });
        assert!(app.parameter_state.state.is_none());
        assert!(app.parameter_state.remaining.is_empty());
        assert!(!app.operation_pending);
        assert!(rx.try_recv().is_err());
    }
}
