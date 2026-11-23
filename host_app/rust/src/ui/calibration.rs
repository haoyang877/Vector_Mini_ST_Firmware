//! 三项真实标定入口；展示设备阶段与保存事实，不生成虚假质量或进度。
use super::*;
use crate::calibration::{self, Kind};

impl HostApp {
    pub(super) fn calibration(&mut self, ui: &mut egui::Ui) {
        self.task_index = self.task_index.min(2);
        let active = self
            .snapshot
            .calibration
            .as_ref()
            .is_some_and(calibration::Job::active);
        let supported =
            self.snapshot.identity_fresh() && self.snapshot.features & calibration::FEATURE != 0;
        let can_start = self.motion_ready()
            && supported
            && self.snapshot.stop_confirmed
            && self.snapshot.safe_idle(&self.target, &self.bound_uid)
            && !active;
        egui::ScrollArea::vertical().show(ui, |ui| {
            widgets::page_header(ui,"标定任务","先失能并确认空闲，再选择任务。设备执行并自动保存；取消需等待停机回读。",
                if active {"任务执行中"} else if !self.connected {"未连接"} else if supported {"已接入"} else {"固件未声明任务状态接口"});
            let width=(ui.available_width()*0.30).clamp(220.0,320.0);
            ui.horizontal_top(|ui| {
                ui.allocate_ui_with_layout(Vec2::new(width,0.0),Layout::top_down(Align::Min),|ui| {
                    Self::panel(ui,"标定任务",|ui| {
                        ui.set_min_height(300.0);
                        ui.add_enabled_ui(!active && !self.operation_pending,|ui| {
                            for (index,kind) in Kind::ALL.into_iter().enumerate() {
                                if ui.add(egui::Button::new(kind.name()).selected(self.task_index==index)
                                    .min_size(Vec2::new(ui.available_width(),42.0))).clicked() {self.task_index=index;}
                            }
                        });
                        ui.add_space(12.0);
                        widgets::hint(ui,"齿槽标定已由固件撤回，不提供执行入口。");
                    });
                });
                let kind=Kind::ALL[self.task_index];
                let job=self.snapshot.calibration.clone().filter(|job|job.kind==kind);
                ui.allocate_ui_with_layout(Vec2::new(ui.available_width(),0.0),Layout::top_down(Align::Min),|ui| {
                    Self::panel(ui,kind.name(),|ui| {
                        ui.set_min_height(300.0);
                        widgets::hint(ui,kind.description());
                        ui.add_space(12.0);
                        egui::Grid::new("calibration_status").spacing([20.0,12.0]).show(ui,|ui| {
                            for (label,value) in [
                                ("任务状态",job.as_ref().map(|job|job.state().to_owned()).unwrap_or("尚未启动".into())),
                                ("设备票据",job.as_ref().map(|job|job.ticket.to_string()).unwrap_or("—".into())),
                                ("保存状态",job.as_ref().map(|job|job.saved().to_owned()).unwrap_or("未读取".into())),
                                ("设备阶段",job.as_ref().map(|job|format!("{} · 0x{:04X}",job.detail_text(),job.detail)).unwrap_or("—".into())),
                                ("质量评估","设备未提供量化指标".into()),
                            ] {ui.label(label);ui.label(value);ui.end_row();}
                        });
                        if let Some(job)=&job && job.active() {
                            ui.horizontal(|ui| {ui.spinner();ui.label(format!("已运行 {:.1} s · 设备未提供百分比进度",job.started.elapsed().as_secs_f64()));});
                        }
                        ui.add_space(14.0);
                        ui.horizontal_wrapped(|ui| {
                            let start=ui.add_enabled(can_start,egui::Button::new("开始标定"));
                            self.control("calibration_start",&start);
                            if start.clicked() {self.operation_pending=true;self.send(Command::StartCalibration(kind));}
                            let cancel=ui.add_enabled(self.motion_ready() && active,egui::Button::new("请求取消"));
                            self.control("calibration_cancel",&cancel);
                            if cancel.clicked() {self.operation_pending=true;self.send(Command::CancelCalibration);}
                        });
                        if !supported {widgets::hint(ui,"需要支持标定状态与取消接口的配套APP；旧固件保持禁用。");}
                        else if !can_start && !active {widgets::hint(ui,"请核对身份、故障、STOP确认、配置及空闲状态。");}
                        widgets::hint(ui,"结果与保存状态来自设备；取消不能撤回已经保存的标定。标定后不会自动使能。");
                    });
                });
            });
        });
    }
}
