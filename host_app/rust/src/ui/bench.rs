//! 显式 CLI 实机观察入口：在真实窗口中执行扫描、选 UID、静止观察、导出、断开。
//! 不允许启动、速度目标、STOP、升级或参数写入。
use super::*;
use std::time::Instant;

enum Stage {
    Start,
    Scanning,
    Connecting,
    Observing(Instant),
    Disconnecting,
    Done,
}

pub(super) struct Run {
    request: BenchRequest,
    stage: Stage,
    started: Instant,
    failure: Option<String>,
    observed_seconds: f64,
    final_state: serde_json::Value,
}

impl Run {
    pub fn new(request: BenchRequest) -> Self {
        Self {
            request,
            stage: Stage::Start,
            started: Instant::now(),
            failure: None,
            observed_seconds: 0.0,
            final_state: serde_json::Value::Null,
        }
    }
    fn write_capture(&mut self, app: &HostApp) -> anyhow::Result<()> {
        let output = &self.request.output;
        app.capture
            .write_csv(std::fs::File::create(output.join("samples.csv"))?)?;
        app.capture
            .write_curve_csv(std::fs::File::create(output.join("curve.csv"))?)?;
        self.final_state = serde_json::json!({"uid":app.snapshot.uid,"node":app.node,
            "state":app.snapshot.motor_state,"mode":app.snapshot.mode,"faults":app.snapshot.faults,
            "speed_rad_s":app.snapshot.speed_rad_s,"bus_v":app.snapshot.bus_v,
            "app_version":app.snapshot.app_version,"loader_version":app.snapshot.loader_version});
        Ok(())
    }
    fn finish(&self, app: &HostApp) -> anyhow::Result<()> {
        let report = serde_json::json!({"version":env!("CARGO_PKG_VERSION"),
            "hardware_contacted":true,"mode":"native_gui_static_observation",
            "success":self.failure.is_none(),"failure":self.failure,
            "expected_uid":self.request.uid,"adapters":app.scan_adapters,
            "sample_count":app.capture.samples.len(),"observed_seconds":self.observed_seconds,
            "final_state":self.final_state,"disconnected":!app.connected,
            "requested_actions":["scan_get_info","connect","observe","disconnect"],"log":app.log});
        std::fs::write(
            self.request.output.join("run.json"),
            serde_json::to_vec_pretty(&report)?,
        )?;
        Ok(())
    }
    fn abort(&mut self, app: &mut HostApp, message: String) {
        self.failure = Some(message);
        app.pause_capture();
        if app.connected {
            app.send(Command::Disconnect);
        }
        self.stage = Stage::Disconnecting;
    }
    pub fn tick(&mut self, app: &mut HostApp, ctx: &egui::Context) {
        if matches!(self.stage, Stage::Done) {
            return;
        }
        if app.hard_stop && !matches!(self.stage, Stage::Disconnecting) {
            self.abort(app, format!("测试停止：{}", app.status));
        }
        if self.started.elapsed() > Duration::from_secs(130)
            && !matches!(self.stage, Stage::Disconnecting)
        {
            self.abort(app, "只读测试总时限已到".into());
        }
        match self.stage {
            Stage::Start => {
                if app.dll.trim().is_empty() {
                    self.abort(app, "未找到厂家 SDK".into());
                } else {
                    app.start_scan();
                    self.stage = Stage::Scanning;
                }
            }
            Stage::Scanning if app.scan_rx.is_none() => {
                let found = app
                    .scan_devices
                    .iter()
                    .enumerate()
                    .find(|(_, device)| device.uid == self.request.uid)
                    .map(|(index, device)| (index, device.clone()));
                if let Some((index, device)) = found {
                    if device.target_name.as_deref() != Some(&app.target.name) {
                        self.abort(app, "扫描目标与指定目标不匹配".into());
                        return;
                    }
                    app.device_index = device.adapter_index;
                    app.node = device.node;
                    app.scan_selected = Some(index);
                    app.connect();
                    self.stage = Stage::Connecting;
                } else {
                    self.abort(app, format!("未发现指定 UID：{}", self.request.uid));
                }
            }
            Stage::Connecting if app.snapshot.identity_fresh() && app.snapshot.feedback_fresh() => {
                if app.bound_uid != self.request.uid
                    || app.snapshot.motor_state != 5
                    || app.snapshot.mode != 0
                    || app.snapshot.faults != 0
                    || !app
                        .snapshot
                        .speed_rad_s
                        .is_some_and(|speed| speed.abs() < 0.1)
                {
                    self.abort(app, "设备未处于指定 UID 的无故障静止 IDLE".into());
                    return;
                }
                match app.capture.start(&app.bound_uid) {
                    Ok(()) => {
                        if let Some(worker) = &app.curve_worker {
                            worker.observe(app.capture.recording_context());
                        }
                        app.send(Command::Observe(true));
                        app.page = Page::Workbench;
                        app.note("只读实机曲线测试：60 秒，目标 60 Hz");
                        self.stage = Stage::Observing(Instant::now());
                    }
                    Err(error) => self.abort(app, error.to_string()),
                }
            }
            Stage::Observing(start) => {
                if !app.connected
                    || app.snapshot.motor_state != 5
                    || app.snapshot.mode != 0
                    || app.snapshot.faults != 0
                {
                    self.abort(app, "静止观察期间设备状态异常".into());
                    return;
                }
                if start.elapsed() >= Duration::from_secs(60) {
                    self.observed_seconds = start.elapsed().as_secs_f64();
                    app.pause_capture();
                    app.send(Command::Observe(false));
                    if let Err(error) = self.write_capture(app) {
                        self.failure = Some(error.to_string());
                    }
                    app.send(Command::Disconnect);
                    self.stage = Stage::Disconnecting;
                }
            }
            Stage::Disconnecting if !app.connected => {
                if let Err(error) = self.finish(app) {
                    eprintln!("测试报告写入失败：{error:#}");
                }
                self.stage = Stage::Done;
                ctx.send_viewport_cmd(egui::ViewportCommand::Close);
            }
            _ => {}
        }
    }
}
