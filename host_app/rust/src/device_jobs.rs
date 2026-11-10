//! 设备线程中的故障清除和标定事务；不另开 CAN 连接或运动线程。
use super::*;

impl<T: Transport> Session<T> {
    pub(super) fn clear_fault(&mut self) -> Result<bool> {
        ensure!(
            self.snapshot.compatible(&self.target, &self.bound_uid)
                && self.snapshot.feedback_fresh()
                && self.snapshot.faults != 0
                && !self
                    .snapshot
                    .calibration
                    .as_ref()
                    .is_some_and(CalibrationJob::active),
            SessionIssue::Rejected("清除故障门禁未满足".into())
        );
        let (result, detail) = self.command(107, &[])?;
        ensure!(
            result <= 1,
            SessionIssue::Rejected(format!("设备拒绝清除故障：{result}/{detail}"))
        );
        ensure!(detail == 0, "清错回复detail非法：{detail}");
        self.read_feedback(false)?;
        ensure!(
            self.snapshot.faults == 0 && self.snapshot.motor_state == 5 && self.snapshot.mode == 0,
            SessionIssue::Rejected(format!(
                "故障未清除或未回空闲，当前故障码0x{:08X}",
                self.snapshot.faults
            ))
        );
        self.snapshot.motion_may_run = false;
        self.send
            .send(Event::Snapshot(Box::new(self.snapshot.clone())))
            .ok();
        self.send
            .send(Event::Status("故障已清除并回读空闲；未自动使能".into()))
            .ok();
        self.send.send(Event::OperationDone).ok();
        Ok(true)
    }

    fn job_exchange(&mut self, kind: u16, payload: &[u8]) -> Result<Vec<u8>> {
        let sequence = self.next_sequence();
        if kind != 161 {
            reserve_control_sequence(
                &self.sequence_file,
                &self.bound_uid,
                self.manual_start,
                sequence,
                false,
            )?;
            self.manual_start = 0;
            self.port.report(
                "info",
                "calibration_command",
                serde_json::json!({"kind":kind,"sequence":sequence,"payload":hex::encode(payload)}),
            );
        }
        let timeout = if kind == 161 {
            READ_BUDGET
        } else {
            Duration::from_millis(250)
        };
        let response = self.exchange_timed(kind, sequence, payload, timeout)?;
        ensure!(response.len() == 12, "标定回复长度错误");
        let result = u16le(&response, 0)?;
        ensure!(
            matches!(result, 0..=8 | 11 | 17),
            "标定结果码非法：{result}"
        );
        if kind != 161 && result > 1 {
            return Err(SessionIssue::Rejected(format!(
                "标定请求被拒：{result}/{}",
                u16le(&response, 2)?
            ))
            .into());
        }
        Ok(response)
    }

    pub(super) fn start_calibration(&mut self, kind: CalibrationKind) -> Result<bool> {
        self.read_identity()?;
        self.read_feedback(false)?;
        ensure!(
            self.snapshot.features & calibration::FEATURE != 0
                && self.snapshot.stop_confirmed
                && self.snapshot.safe_idle(&self.target, &self.bound_uid)
                && !self
                    .snapshot
                    .calibration
                    .as_ref()
                    .is_some_and(CalibrationJob::active),
            SessionIssue::Rejected("标定需要配套固件、STOP确认和无故障空闲状态".into())
        );
        self.snapshot.motion_may_run = true;
        let response = match self.job_exchange(160, &kind.start_payload()) {
            Err(error) if matches!(error.downcast_ref(), Some(SessionIssue::Rejected(_))) => {
                self.snapshot.motion_may_run = false;
                return Err(error);
            }
            result => result?,
        };
        let job = CalibrationJob::parse(&response)?;
        ensure!(job.kind == kind && job.phase == 1, "标定启动票据不匹配");
        self.snapshot.calibration = Some(job);
        self.snapshot.stop_confirmed = false;
        self.send
            .send(Event::Snapshot(Box::new(self.snapshot.clone())))
            .ok();
        self.send
            .send(Event::Status(format!(
                "{}已启动；等待设备结果和保存状态",
                kind.name()
            )))
            .ok();
        self.send.send(Event::OperationDone).ok();
        Ok(true)
    }

    pub(super) fn cancel_calibration(&mut self) -> Result<bool> {
        let job = self
            .snapshot
            .calibration
            .clone()
            .filter(CalibrationJob::active)
            .ok_or_else(|| SessionIssue::Rejected("没有本会话的活动标定票据".into()))?;
        let response = self.job_exchange(162, &job.ticket.to_le_bytes())?;
        self.snapshot.calibration = Some(job.followup(&response)?);
        self.send
            .send(Event::Snapshot(Box::new(self.snapshot.clone())))
            .ok();
        self.finish_calibration()?;
        self.send.send(Event::OperationDone).ok();
        Ok(true)
    }

    pub(super) fn poll_calibration(&mut self) -> Result<()> {
        let Some(job) = self
            .snapshot
            .calibration
            .clone()
            .filter(CalibrationJob::active)
        else {
            return Ok(());
        };
        if job.updated.elapsed() < Duration::from_millis(200) {
            return Ok(());
        }
        let response = self.job_exchange(161, &job.ticket.to_le_bytes())?;
        let next = job.followup(&response)?;
        if next.phase != job.phase || next.effects != job.effects {
            self.port.report(
                "info",
                "calibration_state",
                serde_json::json!({"ticket":next.ticket,"kind":next.kind as u16,
                "phase":next.phase,"effects":next.effects,"detail":next.detail}),
            );
        }
        self.snapshot.calibration = Some(next);
        self.send
            .send(Event::Snapshot(Box::new(self.snapshot.clone())))
            .ok();
        self.finish_calibration()
    }

    fn finish_calibration(&mut self) -> Result<()> {
        let Some(job) = self
            .snapshot
            .calibration
            .as_ref()
            .filter(|job| !job.active())
            .cloned()
        else {
            return Ok(());
        };
        self.stop()?;
        self.read_identity()?;
        self.send
            .send(Event::Status(format!(
                "{}：{}，{}；已回读停机",
                job.kind.name(),
                job.state(),
                job.saved()
            )))
            .ok();
        Ok(())
    }
}
