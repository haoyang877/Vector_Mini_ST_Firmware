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
