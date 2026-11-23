//! 参数事务运行在既有设备线程；小批次读取让 STOP 在批次间获得处理机会。
use super::*;
use crate::parameters::{self, Action, ReplyError};
use std::collections::BTreeMap;

impl<T: Transport> Session<T> {
    fn parameter_reply(
        &mut self,
        kind: u16,
        payload: &[u8],
        id: Option<u8>,
    ) -> Result<parameters::Value> {
        let sequence = self.next_sequence();
        let data = self.exchange(kind, sequence, payload)?;
        match parameters::parse(&data, id) {
            Ok(value) => Ok(value),
            Err(ReplyError::Rejected(result, detail)) => Err(SessionIssue::Rejected(format!(
                "参数 Type{kind} 被拒绝：{result}/{detail}；请重新读取设备值"
            ))
            .into()),
            Err(ReplyError::Malformed) => bail!("参数 Type{kind} 回复格式错误"),
        }
    }

    pub(super) fn parameter_action(&mut self, action: Action) -> Result<bool> {
        self.read_feedback(false)?;
        ensure!(
            self.snapshot.stop_confirmed
                && self.snapshot.safe_idle(&self.target, &self.bound_uid)
                && self.snapshot.features & parameters::FEATURE != 0,
            SessionIssue::Rejected("参数服务需要兼容 APP、关相空闲及新鲜回读".into())
        );
        let mut values = BTreeMap::new();
        let mut unsupported = vec![];
        let mut saved = false;
        let state = match action {
            Action::Read {
                ids,
                saved: read_saved,
            } => {
                ensure!(!ids.is_empty() && ids.len() <= 4, "参数读取批次须为 1…4 项");
                saved = read_saved;
                let deadline = Instant::now() + Duration::from_millis(500);
                let mut state = None;
                for id in ids {
                    ensure!(parameters::scalar(id).is_some(), "参数未登记");
                    let sequence = self.next_sequence();
                    let data = self.exchange_timed(
                        if saved { 123 } else { 120 },
                        sequence,
                        &[id, 0, 0, 0],
                        deadline
                            .saturating_duration_since(Instant::now())
                            .min(Duration::from_millis(200)),
                    )?;
                    match parameters::parse(&data, Some(id)) {
                        Ok(value) => {
                            ensure!(
                                state.is_none_or(|previous: parameters::Value| previous.revision
                                    == value.revision),
                                SessionIssue::Rejected("读取期间参数修订变化，请重新读取".into())
                            );
                            state = Some(value);
                            values.insert(id, value);
                        }
                        Err(ReplyError::Rejected(2, 1)) => unsupported.push(id),
                        Err(ReplyError::Rejected(result, detail)) => {
                            return Err(SessionIssue::Rejected(format!(
                                "参数 {id} 读取被拒绝：{result}/{detail}"
                            ))
                            .into());
                        }
                        Err(ReplyError::Malformed) => bail!("参数 {id} 回复格式错误"),
                    }
                }
                state
            }
            Action::Write { id, raw, revision } => {
                let payload = parameters::write_payload(id, raw, revision)?;
                let value = self.parameter_reply(121, &payload, Some(id))?;
                // 使用设备量化后的实际值，不能把请求值冒充回读。
                let readback = self.parameter_reply(120, &[id, 0, 0, 0], Some(id))?;
                ensure!(
                    value.raw == readback.raw && value.revision == readback.revision,
                    "参数写入后的回读不一致"
                );
                values.insert(id, readback);
                Some(readback)
            }
            Action::Save { revision } => {
                let value = self.parameter_reply(122, &revision.to_le_bytes(), None)?;
                ensure!(
                    value.revision == revision
                        && value.saved_revision == revision
                        && value.flags == 1,
                    "参数保存未得到持久化确认"
                );
                self.send
                    .send(Event::Status(format!(
                        "参数修订 {revision} 已保存并经设备确认"
                    )))
                    .ok();
                Some(value)
            }
        };
        self.send
            .send(Event::Parameters(parameters::Report {
                uid: self.bound_uid.clone(),
                saved,
                values,
                unsupported,
                state,
            }))
            .ok();
        Ok(true)
    }
}
