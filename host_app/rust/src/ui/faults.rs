//! 全工作区共用的故障码、红灯和显式清错；离线/过期不得伪装正常。
use super::*;

fn fault_names(bits: u32) -> String {
    let mut names = vec![];
    let mut known = 0;
    for (bit, name) in [
        (0, "过流"),
        (1, "过压"),
        (2, "欠压"),
        (4, "MCU过温"),
        (5, "编码器"),
        (6, "CAN断连"),
        (9, "参数或标定"),
        (12, "传感器或其他故障"),
        (14, "校零/标定"),
        (15, "辨识故障"),
    ] {
        known |= 1 << bit;
        if bits & (1 << bit) != 0 {
            names.push(name.to_owned());
        }
    }
    if bits & !known != 0 {
        names.push(format!("未知位0x{:08X}", bits & !known));
    }
    if names.is_empty() {
        "无故障".into()
    } else {
        names.join("、")
    }
}

impl HostApp {
    fn can_clear_fault(&self) -> bool {
        self.motion_ready()
            && self.snapshot.compatible(&self.target, &self.bound_uid)
            && self.snapshot.feedback_fresh()
            && self.snapshot.faults != 0
            && !self
                .snapshot
                .calibration
                .as_ref()
                .is_some_and(crate::calibration::Job::active)
    }
    pub(super) fn fault_status_bar(&mut self, ui: &mut egui::Ui) {
        let c = Palette::from_ctx(ui.ctx());
        let fresh = self.connected && self.snapshot.feedback_fresh();
        let fault = self.snapshot.faults != 0;
        let (label, color) = match (fresh, fault) {
            (true, true) => ("设备故障", c.danger),
            (true, false) => ("无故障", Color32::from_rgb(69, 170, 113)),
            (false, true) => ("历史故障 · 当前未知", c.danger),
            _ => ("故障状态未知", c.muted),
        };
        ui.horizontal_wrapped(|ui| {
            ui.label(RichText::new("●").color(color));
            ui.label(RichText::new(label).color(color));
            ui.monospace(if fresh || fault {
                format!("0x{:08X}", self.snapshot.faults)
            } else {
                "—".into()
            })
            .on_hover_text(fault_names(self.snapshot.faults));
            let clear = ui.add_enabled(
                self.can_clear_fault(),
                egui::Button::new("清除故障").small(),
            );
            self.control("clear_fault", &clear);
            if clear.clicked() {
                self.operation_pending = true;
                self.send(Command::ClearFault);
            }
            if self.operation_pending {
                widgets::hint(ui, "等待设备确认");
            }
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn names_preserve_multiple_and_unknown_fault_bits() {
        let text = fault_names(1 | (1 << 6) | (1 << 31));
        assert!(text.contains("过流") && text.contains("CAN断连") && text.contains("80000000"));
        assert_eq!(fault_names(0), "无故障");
    }
    #[test]
    fn stale_preview_and_disconnected_cannot_clear_faults() {
        let mut app = HostApp::disconnected(Target::bundled().unwrap(), None);
        app.snapshot.faults = 1;
        assert!(!app.can_clear_fault());
        app.connected = true;
        assert!(!app.can_clear_fault());
        app.fixture_mode = true;
        assert!(!app.can_clear_fault());
    }
}
