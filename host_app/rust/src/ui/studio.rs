//! 交互式离线样机。只有显式预览入口构造示例，设备命令仍受 fixture_mode 统一禁止。
use super::*;
use crate::motor_configuration::{Info, Model};
use std::time::Instant;

#[derive(Clone, Copy, PartialEq, Eq)]
pub(super) enum Scenario {
    Disconnected,
    Idle,
    Running,
    HardStop,
    History,
}

impl Scenario {
    pub(super) const OPTIONS: [(Self, &'static str); 5] = [
        (Self::Disconnected, "未连接"),
        (Self::Idle, "空闲 / 已失能"),
        (Self::Running, "运行 / 已使能"),
        (Self::HardStop, "硬停待确认"),
        (Self::History, "历史身份"),
    ];
}

pub(super) fn model(node: u8) -> Model {
    if node == 7 {
        Model::Yq5708
    } else {
        Model::Yq5740
    }
}

impl HostApp {
    pub(super) fn init_studio(&mut self) {
        self.target = Target::named("yg_esc_v1_at32f456").expect("内置示例目标");
        self.scan_adapters = vec![(0, "USBCANFD-200U · 示例".into())];
        self.scan_devices = [6, 7]
            .into_iter()
            .map(|node| discovery::Device {
                adapter_index: 0,
                adapter: self.scan_adapters[0].1.clone(),
                node,
                uid: format!("DEMO-NODE-{node:02}"),
                role: 1,
                target_name: Some("yg_esc_v1_at32f456".into()),
            })
            .collect();
        self.scan_selected = Some(0);
        self.node = 6;
        self.load_preview_record(true);
    }

    pub(super) fn load_preview_record(&mut self, sample: bool) {
        self.capture = if sample {
            Capture::preview_record()
        } else {
            Capture::default()
        };
        for sample in &mut self.capture.samples {
            match self.control_mode {
                ControlMode::Voltage => {
                    sample.values[4] = None;
                }
                ControlMode::Torque => {
                    sample.values[4] = None;
                    sample.values[5] = Some(0.3);
                }
                ControlMode::Position => {
                    let x = (sample.time_s - 3.0).max(0.0);
                    sample.values[0] = Some(0.12 * (1.0 - (-x * 1.8).exp()));
                    sample.values[3] = Some(0.12 * (1.0 - (-x * 2.0).exp()));
                    sample.values[1] = Some(0.216 * (-x * 1.8).exp());
                    sample.values[4] = Some(0.24 * (-x * 2.0).exp());
                }
                ControlMode::Speed => {}
            }
        }
        if let Some(worker) = &self.curve_worker {
            worker.reset(self.capture.samples.iter().cloned().collect());
        }
    }

    pub(super) fn set_studio_scenario(&mut self, scene: Scenario) {
        if self.studio.is_none() {
            return;
        }
        self.studio = Some(scene);
        self.connected = matches!(scene, Scenario::Idle | Scenario::Running);
        self.hard_stop = scene == Scenario::HardStop;
        self.bound_uid.clear();
        self.snapshot = Snapshot::default();
        self.parameter_state = parameters::State::default();
        self.control_targets = [0.0; 4];
        self.cursor_s = None;
        self.load_preview_record(true);
        if scene == Scenario::Disconnected {
            return;
        }
        self.bound_uid = format!("DEMO-NODE-{:02}", self.node);
        self.parameter_state.studio_values(&self.bound_uid);
        self.snapshot = Snapshot {
            uid: self.bound_uid.clone(),
            role: 1,
            app_version: 0x010602,
            loader_version: 0x020200,
            product: self.target.product,
            hardware: self.target.hardware as u16,
            schema: self.target.schema,
            abi: self.target.abi,
            features: 0x7fd00,
            modes: 0x4e,
            active_configuration: Some(Info {
                model: model(self.node).id(),
                node: self.node,
                flags: 1,
                revision: 1,
                speed_rad_s: 10.0,
                thermal: [0.0; 7],
            }),
            motor_state: if scene == Scenario::Running { 6 } else { 5 },
            mode: if scene == Scenario::Running {
                self.control_mode.wire()
            } else {
                0
            },
            motion_may_run: scene == Scenario::Running,
            stop_confirmed: scene == Scenario::Idle,
            bus_v: Some(24.0),
            bus_a: Some(0.0),
            position_rad: Some(1.2),
            speed_rad_s: Some(0.0),
            vq_v: Some(0.0),
            iq_a: Some(0.0),
            ..Default::default()
        };
        if self.hard_stop {
            self.status = "示例：通信中断，输出状态待确认".into();
        }
        self.refresh_studio();
    }

    pub(super) fn refresh_studio(&mut self) {
        if self.studio.is_some() && self.connected {
            let now = Instant::now();
            self.snapshot.identity_at = Some(now);
            self.snapshot.feedback_at = Some(now);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn interactive_studio_scenarios_never_open_hardware_or_send_commands() {
        let mut app = HostApp::disconnected(
            Target::bundled().unwrap(),
            Some(PreviewRequest::Interactive),
        );
        app.init_studio();
        assert_eq!(app.page, Page::Connection);
        assert!(app.preview.is_none());
        assert_eq!(app.scan_devices.len(), 2);
        let (tx, rx) = mpsc::channel();
        app.command_tx = Some(tx);
        for (scenario, _) in Scenario::OPTIONS {
            app.set_studio_scenario(scenario);
            assert!(!app.motion_ready());
            app.start_scan();
            app.connect();
            app.send(Command::Stop);
            assert!(app.scan_rx.is_none());
            assert!(app.event_rx.is_none());
            assert!(rx.try_recv().is_err());
        }
    }
}
