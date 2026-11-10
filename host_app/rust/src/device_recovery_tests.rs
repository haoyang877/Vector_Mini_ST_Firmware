//! 经现有 Transport 注入丢包、BUSY、损坏回复和 SDK 错误，覆盖真实会话分支。
use super::*;
use std::collections::VecDeque;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex, mpsc};

enum Reply {
    Data(Vec<u8>),
    Busy,
    Lost,
    Wait,
    DamagedThenGood(Vec<u8>),
    Error,
}

struct FakePort {
    script: VecDeque<Reply>,
    pending: VecDeque<(u32, Vec<u8>)>,
    current: Option<Reply>,
    sent: Arc<Mutex<Vec<u16>>>,
}

impl Transport for FakePort {
    fn send(&mut self, _id: u32, data: &[u8]) -> Result<()> {
        let kind = u16le(data, 6)?;
        let sequence = u16le(data, 8)?;
        self.sent.lock().unwrap().push(kind);
        self.current = self.script.pop_front();
        let payload = match self.current.as_ref() {
            Some(Reply::Data(data) | Reply::DamagedThenGood(data)) => Some(data.clone()),
            Some(Reply::Busy) => {
                let mut data = super::tests::valid_feedback();
                data[..2].copy_from_slice(&6u16.to_le_bytes());
                Some(data)
            }
            _ => None,
        };
        if let Some(payload) = payload {
            if matches!(self.current, Some(Reply::DamagedThenGood(_))) {
                self.pending.push_back(wire::test_reply(
                    1,
                    kind,
                    sequence.wrapping_sub(1),
                    &payload,
                ));
                let (id, mut corrupt) = wire::test_reply(1, kind, sequence, &payload);
                corrupt[16] ^= 1;
                self.pending.push_back((id, corrupt));
            }
            self.pending
                .push_back(wire::test_reply(1, kind, sequence, &payload));
        }
        Ok(())
    }

    fn receive(&mut self, timeout: Duration) -> Result<Option<(u32, Vec<u8>)>> {
        if matches!(self.current, Some(Reply::Error)) {
            bail!("模拟 SDK 接收失败");
        }
        if matches!(self.current, Some(Reply::Wait)) {
            std::thread::sleep(timeout);
        }
        Ok(self.pending.pop_front())
    }
}

struct Fixture {
    session: Session<FakePort>,
    events: Receiver<Event>,
    sent: Arc<Mutex<Vec<u16>>>,
    directory: PathBuf,
}

fn fixture(script: Vec<Reply>) -> Fixture {
    static NEXT: AtomicU64 = AtomicU64::new(0);
    let directory = std::env::temp_dir().join(format!(
        "yg-link-test-{}-{}",
        std::process::id(),
        NEXT.fetch_add(1, Ordering::Relaxed)
    ));
    fs::create_dir_all(&directory).unwrap();
    let log = Logger::at(Some(directory.clone()));
    let sent = Arc::new(Mutex::new(vec![]));
    let port = FakePort {
        script: script.into(),
        pending: VecDeque::new(),
        current: None,
        sent: sent.clone(),
    };
    let (send, events) = mpsc::channel();
    let target = Target::bundled().unwrap();
    let snapshot = Snapshot {
        uid: "A".into(),
        role: 1,
        app_version: 1,
        product: target.product,
        hardware: target.hardware as u16,
        schema: target.schema,
        abi: target.abi,
        features: 0x1d00,
        modes: 0x4e,
        motor_state: 5,
        mode: 0,
        bus_v: Some(24.0),
        bus_a: Some(0.0),
        iq_a: Some(0.0),
        speed_rad_s: Some(0.0),
        identity_at: Some(Instant::now()),
        feedback_at: Some(Instant::now()),
        stop_confirmed: true,
        ..Default::default()
    };
    Fixture {
        session: Session {
            port: TraceTransport::new(port, log, serde_json::json!({"node":1,"uid":"A"})),
            sequence_file: directory.join("sequences.json"),
            node: 1,
            target,
            sequence: 100,
            manual_start: 100,
            bound_uid: "A".into(),
            expected_uid: "A".into(),
            high_rate: false,
            snapshot,
            send,
            started: Instant::now(),
            feedback_degraded: false,
            lease_supported: false,
            last_renewal: None,
            renewal_block: None,
        },
        events,
        sent,
        directory,
    }
}

fn running(f: &mut Fixture) {
    f.session.snapshot.motor_state = 6;
    f.session.snapshot.mode = 2;
    f.session.snapshot.motion_may_run = true;
    f.session.snapshot.stop_confirmed = false;
}

fn idle_payload() -> Vec<u8> {
    let mut data = super::tests::valid_feedback();
    data[43] = 5;
    data[44] = 0;
    data
}

fn logs(f: &Fixture) -> Vec<serde_json::Value> {
    f.session.port.log.flush().unwrap();
    let path = PathBuf::from(f.session.port.log.location());
    fs::read_to_string(path)
        .unwrap()
        .lines()
        .map(|line| serde_json::from_str(line).unwrap())
        .collect()
}

#[test]
fn busy_then_good_keeps_session_and_records_raw_evidence() {
    let mut f = fixture(vec![
        Reply::Busy,
        Reply::Data(super::tests::valid_feedback()),
    ]);
    running(&mut f);
    assert!(f.session.poll_feedback().is_ok());
    assert_eq!(*f.sent.lock().unwrap(), [108, 108]);
    assert!(f.session.snapshot.feedback_fresh());
    let records = logs(&f);
    assert!(records.iter().any(|row| row["event"] == "feedback_busy"
        && row["details"]["recent_frames"].as_array().unwrap().len() == 2));
    assert!(
        records
            .iter()
            .any(|row| row["event"] == "feedback_busy_recovered")
    );
}

#[test]
fn repeated_busy_never_refreshes_old_measurements_and_eventually_fails() {
    let mut f = fixture(vec![Reply::Busy, Reply::Busy, Reply::Busy, Reply::Busy]);
    running(&mut f);
    let previous = f.session.snapshot.feedback_at;
    assert!(f.session.poll_feedback().is_ok());
    assert_eq!(f.session.snapshot.feedback_at, previous);
    assert!(f.session.feedback_degraded);
    f.session.snapshot.feedback_at = Some(Instant::now() - Duration::from_millis(510));
    assert!(f.session.poll_feedback().is_err());
    assert_eq!(f.sent.lock().unwrap().len(), 3);
}

#[test]
fn single_lost_read_recovers_and_late_or_corrupt_frames_are_diagnosed() {
    let mut f = fixture(vec![
        Reply::Lost,
        Reply::DamagedThenGood(super::tests::valid_feedback()),
    ]);
    assert!(f.session.poll_feedback().is_ok());
    assert_eq!(*f.sent.lock().unwrap(), [108, 108]);
    assert!(
        logs(&f)
            .iter()
            .any(|row| row["event"] == "request_recovered"
                && row["details"]["transaction"]["ignored"] == 2)
    );
}

#[test]
fn local_enable_and_target_rejections_keep_connection_and_send_nothing() {
    let mut f = fixture(vec![]);
    f.session.snapshot.stop_confirmed = false;
    assert!(
        f.session
            .dispatch(Command::Start {
                mode: ControlMode::Speed
            })
            .unwrap()
    );
    assert!(
        f.session
            .dispatch(Command::SetTarget {
                mode: ControlMode::Speed,
                value: 1.0
            })
            .unwrap()
    );
    assert!(f.sent.lock().unwrap().is_empty());
    assert_eq!(
        f.events
            .try_iter()
            .filter(|event| matches!(event, Event::Rejected(_)))
            .count(),
        2
    );
}

#[test]
fn out_of_range_target_does_not_stop_an_existing_session() {
    let mut f = fixture(vec![]);
    running(&mut f);
    assert!(
        f.session
            .dispatch(Command::SetTarget {
                mode: ControlMode::Speed,
                value: 1000.0
            })
            .unwrap()
    );
    assert!(f.sent.lock().unwrap().is_empty());
    assert!(f.session.snapshot.motion_may_run);
}

#[test]
fn explicit_device_rejection_does_not_disconnect_or_retry_start() {
    let mut f = fixture(vec![Reply::Data(vec![5, 0, 5, 1])]);
    assert!(
        f.session
            .dispatch(Command::Start {
                mode: ControlMode::Speed
            })
            .unwrap()
    );
    assert_eq!(*f.sent.lock().unwrap(), [117]);
    assert!(!f.session.snapshot.motion_may_run);
    assert!(
        f.events
            .try_iter()
            .any(|event| matches!(event, Event::Rejected(_)))
    );
}

#[test]
fn ambiguous_start_timeout_is_not_replayed_and_stop_is_confirmed() {
    let mut f = fixture(vec![
        Reply::Lost,
        Reply::Data(vec![0; 4]),
        Reply::Data(idle_payload()),
    ]);
    let error = f
        .session
        .dispatch(Command::Start {
            mode: ControlMode::Speed,
        })
        .unwrap_err();
    assert!(f.session.snapshot.motion_may_run);
    let error = f.session.fail_after_stop(error);
    assert!(error.to_string().contains("已请求并回读 STOP"));
    assert_eq!(*f.sent.lock().unwrap(), [117, 110, 108]);
    assert!(f.session.snapshot.stop_confirmed);
    assert!(
        logs(&f)
            .iter()
            .any(|row| row["event"] == "failure_stop_result"
                && row["details"]["transaction"]["confirmed"] == true)
    );
}

#[test]
fn stop_failure_stays_unknown_and_is_logged() {
    let mut f = fixture(vec![Reply::Lost, Reply::Lost]);
    running(&mut f);
    let error = f.session.fail_after_stop(anyhow::anyhow!("原始通信错误"));
    assert!(error.to_string().contains("STOP 未获最终确认"));
    assert!(f.session.snapshot.motion_may_run);
    assert_eq!(*f.sent.lock().unwrap(), [110]);
    assert!(
        logs(&f)
            .iter()
            .any(|row| row["event"] == "failure_stop_result"
                && row["details"]["transaction"]["confirmed"] == false)
    );
}

#[test]
fn sustained_loss_respects_running_feedback_deadline() {
    let mut f = fixture(vec![Reply::Wait, Reply::Wait]);
    running(&mut f);
    f.session.snapshot.feedback_at = Some(Instant::now() - Duration::from_millis(450));
    let start = Instant::now();
    assert!(f.session.poll_feedback().is_err());
    assert!(start.elapsed() < Duration::from_millis(250));
    assert_eq!(*f.sent.lock().unwrap(), [108]);
}

#[test]
fn idle_link_has_bounded_recovery_and_a_success_clears_degraded_status() {
    let mut f = fixture(vec![Reply::Lost, Reply::Lost, Reply::Data(idle_payload())]);
    assert!(f.session.poll_feedback().is_ok());
    assert!(f.session.feedback_degraded);
    assert!(f.session.poll_feedback().is_ok());
    assert!(!f.session.feedback_degraded);
    let mut failed = fixture(vec![Reply::Lost, Reply::Lost]);
    failed.session.snapshot.feedback_at = Some(Instant::now() - Duration::from_secs(3));
    assert!(failed.session.poll_feedback().is_err());
}

#[test]
fn sdk_failure_and_invalid_payload_are_hard_errors() {
    let mut sdk = fixture(vec![Reply::Error]);
    assert!(sdk.session.poll_feedback().is_err());
    let mut bad = fixture(vec![Reply::Data(vec![0; 4])]);
    assert!(bad.session.poll_feedback().is_err());
    assert!(
        logs(&bad)
            .iter()
            .any(|row| row["event"] == "feedback_invalid")
    );
}

#[test]
fn fault_log_export_contains_device_transaction_and_raw_frames() {
    let mut f = fixture(vec![Reply::Lost, Reply::Lost]);
    assert!(f.session.poll_feedback().is_ok());
    let destination = f.directory.join("export.jsonl");
    f.session.port.log.export(destination.clone()).unwrap();
    let text = fs::read_to_string(destination).unwrap();
    let row: serde_json::Value = serde_json::from_str(text.lines().next().unwrap()).unwrap();
    assert_eq!(row["version"], env!("CARGO_PKG_VERSION"));
    assert_eq!(row["details"]["device"]["uid"], "A");
    assert_eq!(row["details"]["transaction"]["kind"], 108);
    assert!(
        !row["details"]["recent_frames"]
            .as_array()
            .unwrap()
            .is_empty()
    );
    let original = PathBuf::from(f.session.port.log.location());
    assert!(f.session.port.log.export(original).is_err());
}

#[test]
fn busy_with_no_snapshot_state_keeps_original_state_and_timestamp() {
    let data = hex::decode("060000000000000000000080000000800000008000000080000000800080008000800080008000800080010000").unwrap();
    let mut snapshot = Snapshot {
        motor_state: 6,
        mode: 3,
        position_rad: Some(1.0),
        feedback_at: Some(Instant::now()),
        ..Default::default()
    };
    let before = snapshot.feedback_at;
    let error = apply_feedback(&mut snapshot, &data, 1).unwrap_err();
    assert!(matches!(error.downcast_ref(), Some(SessionIssue::Busy)));
    assert_eq!(snapshot.feedback_at, before);
    assert_eq!(snapshot.motor_state, 6);
    assert_eq!(snapshot.position_rad, Some(1.0));
}

#[test]
fn repeated_isolated_busy_and_loss_recover_without_any_motion_command() {
    let mut script = vec![];
    for i in 0..2000 {
        if i % 10 == 0 {
            script.push(Reply::Busy);
        } else if i % 17 == 0 {
            script.push(Reply::Lost);
        }
        script.push(Reply::Data(super::tests::valid_feedback()));
    }
    let mut f = fixture(script);
    running(&mut f);
    for _ in 0..2000 {
        assert!(f.session.poll_feedback().is_ok());
        assert!(f.session.snapshot.feedback_fresh());
    }
    assert!(f.sent.lock().unwrap().iter().all(|kind| *kind == 108));
}

#[test]
fn all_state_changing_transactions_are_sent_once_when_reply_is_lost() {
    for kind in [110, 116, 117, 191, 192, 193] {
        let mut f = fixture(vec![Reply::Lost, Reply::Data(vec![0; 4])]);
        assert!(
            f.session
                .exchange_timed(kind, 100, &[], Duration::from_millis(500))
                .is_err()
        );
        assert_eq!(*f.sent.lock().unwrap(), [kind]);
    }
}

#[test]
fn incomplete_identity_refresh_does_not_remove_configuration_gates() {
    let mut page0 = vec![0; 32];
    page0[..12].fill(1);
    let mut page1 = vec![0; 32];
    page1[21] = 1;
    page1[22] = 1;
    let mut caps = vec![0; 16];
    caps[..4].copy_from_slice(&0x21d00u32.to_le_bytes());
    caps[6] = 64;
    let mut f = fixture(vec![
        Reply::Data(page0),
        Reply::Data(page1),
        Reply::Data(vec![0; 32]),
        Reply::Data(caps),
        Reply::Lost,
        Reply::Lost,
    ]);
    f.session.bound_uid = "01".repeat(12);
    f.session.expected_uid = f.session.bound_uid.clone();
    f.session.snapshot.active_configuration = Some(ConfigurationInfo {
        model: 3,
        node: 1,
        flags: 8,
        revision: 4,
        speed_rad_s: 1.0,
        thermal: [0.0; 7],
    });
    let previous = f.session.snapshot.identity_at;
    assert!(f.session.read_identity().is_err());
    assert_eq!(f.session.snapshot.identity_at, previous);
    assert!(
        f.session
            .snapshot
            .active_configuration
            .as_ref()
            .unwrap()
            .blocked()
    );
    assert_eq!(
        f.session
            .snapshot
            .active_configuration
            .as_ref()
            .unwrap()
            .revision,
        4
    );
}

#[test]
fn host_position_requires_new_firmware_and_explicit_session_start() {
    let mut old = fixture(vec![]);
    old.session.snapshot.position_rad = Some(10.0);
    assert!(
        old.session
            .dispatch(Command::Start {
                mode: ControlMode::Position
            })
            .unwrap()
    );
    assert!(old.sent.lock().unwrap().is_empty());
    let mut modern = fixture(vec![Reply::Data(vec![1, 0, 0, 0])]);
    modern.session.snapshot.position_rad = Some(10.0);
    modern.session.snapshot.features |= crate::control::HOST_POSITION_SESSION_FEATURE;
    assert!(
        modern
            .session
            .dispatch(Command::Start {
                mode: ControlMode::Position
            })
            .unwrap()
    );
    assert_eq!(*modern.sent.lock().unwrap(), [117]);
    assert!(
        logs(&modern)
            .iter()
            .any(|row| row["event"] == "command_requested"
                && row["details"]["transaction"]["payload"] == "03010000")
    );
}


#[test]
fn motion_renewal_only_for_owned_session_and_counter_survives_reconnect() {
    let mut f = fixture(vec![Reply::Data(vec![0; 4])]);
    running(&mut f);
    f.session.renew_owned_motion().unwrap();
    assert!(f.sent.lock().unwrap().is_empty());
    f.session.last_renewal = Some(Instant::now() - Duration::from_millis(150));
    f.session.renew_owned_motion().unwrap();
    assert_eq!(*f.sent.lock().unwrap(), [118]);
    assert_eq!(f.session.renewal_block.as_mut().unwrap().next(), Some(2));
    let mut next = reserve_renewals(&f.session.sequence_file, &f.session.bound_uid).unwrap();
    assert_eq!(next.next(), Some(257));
    f.session.snapshot.motor_state = 5;
    f.session.renew_owned_motion().unwrap();
    assert!(f.session.last_renewal.is_none());
}
