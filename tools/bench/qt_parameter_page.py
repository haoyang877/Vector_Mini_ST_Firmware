"""原生Qt参数表；按UID保存设备快照与编辑草稿，后台负责协议事务。"""

from can_parameters import CATALOG, GROUPS, encode_value, value_text
from PySide6.QtWidgets import (
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QPushButton,
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)
from qt_production_icons import ICON_SIZE, icon


class ParameterPage(QWidget):
    """四组原生表格，只更新有变化的设备值，保存须持有本组快照。"""

    def __init__(self, action, parent=None) -> None:
        super().__init__(parent)
        self.snapshots = {}
        self.entries = {}
        self.readbacks = {}
        self.value_states = {}
        self.buttons = []
        self.identity = None
        self.busy = False
        self.available = False
        layout = QVBoxLayout(self)
        top = QHBoxLayout()
        read = QPushButton("读取全部参数")
        read.setIcon(icon("refresh"))
        read.setIconSize(ICON_SIZE)
        read.clicked.connect(lambda: action("all", False))
        top.addWidget(read)
        top.addStretch()
        self.buttons.append((read, "all", False))
        layout.addLayout(top)
        self.message = QLabel("识别设备后读取参数；设备值与编辑值分别显示")
        self.message.setWordWrap(True)
        layout.addWidget(self.message)
        self.tabs = QTabWidget()
        layout.addWidget(self.tabs)
        for group, (title, fields) in GROUPS.items():
            panel = QWidget()
            body = QVBoxLayout(panel)
            actions = QHBoxLayout()
            for save, text in ((False, "读取本组"), (True, "写入并保存")):
                button = QPushButton(text)
                button.setIcon(icon("save" if save else "refresh"))
                button.setIconSize(ICON_SIZE)
                button.clicked.connect(lambda _=False, g=group, s=save: action(g, s))
                actions.addWidget(button)
                self.buttons.append((button, group, save))
            actions.addStretch()
            body.addLayout(actions)
            table = QTableWidget(len(fields), 5)
            table.setHorizontalHeaderLabels(["参数", "设备值", "编辑值", "单位", "保存状态"])
            table.verticalHeader().hide()
            table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
            for row, (identifier, label) in enumerate(fields):
                spec = CATALOG[identifier]
                table.setItem(row, 0, QTableWidgetItem(label))
                table.setItem(
                    row,
                    3,
                    QTableWidgetItem(
                        {"pole_pairs": "对", "encoder_q15_counts": "Q15", "bool": "0 / 1"}.get(
                            spec["unit"], spec["unit"]
                        )
                    ),
                )
                value, state = QLabel("—"), QLabel("未读取")
                entry = QLineEdit()
                entry.setMaximumWidth(180)
                entry.setReadOnly(True)
                entry.textEdited.connect(lambda _text, i=identifier: self.update_value_state(i))
                table.setCellWidget(row, 1, value)
                table.setCellWidget(row, 2, entry)
                table.setCellWidget(row, 4, state)
                self.entries[identifier] = entry
                self.readbacks[identifier] = value
                self.value_states[identifier] = state
            table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
            body.addWidget(table)
            self.tabs.addTab(panel, icon("parameters"), title)
        self.update_controls(False, False)

    def update_value_state(self, identifier: int) -> None:
        """设备值、编辑草稿、保存事实分别展示，不将草稿作为已生效参数。"""
        row = next(
            (
                snapshot["rows"][identifier]
                for snapshot in self.snapshots.values()
                if identifier in snapshot["rows"]
            ),
            None,
        )
        if row is None:
            self.value_states[identifier].setText("需重新读取")
            return
        try:
            changed = bool(CATALOG[identifier]["access"] & 2) and (
                encode_value(identifier, self.entries[identifier].text()) != row["raw"]
            )
        except ValueError:
            self.value_states[identifier].setText("编辑值无效")
            return
        flags = row.get("flags", 0)
        state = (
            "有未写入修改"
            if changed
            else "保存异常"
            if flags & 4
            else (
                "已保存"
                if flags & 1 and not flags & 2 and row.get("revision") == row.get("saved_revision")
                else "RAM值 / 保存未确认"
            )
        )
        self.value_states[identifier].setText(state)

    def reset(self, identity: tuple | None) -> None:
        if identity == self.identity:
            return
        self.identity = identity
        self.snapshots.clear()
        for identifier in self.entries:
            self.readbacks[identifier].setText("—")
            self.value_states[identifier].setText("未读取")
        for entry in self.entries.values():
            entry.setReadOnly(False)
            entry.clear()
            entry.setReadOnly(True)
        self.message.setText("先读取当前电机参数；不同电机的编辑值不沿用。")

    def update_controls(self, busy: bool, available: bool) -> None:
        self.busy, self.available = busy, available
        for button, group, save in self.buttons:
            allowed = not busy and available and (not save or group in self.snapshots)
            button.setEnabled(allowed)
        for group, (_, fields) in GROUPS.items():
            for identifier, _ in fields:
                editable = (
                    not busy
                    and available
                    and group in self.snapshots
                    and CATALOG[identifier]["access"] & 2
                    and self.snapshots[group]["rows"][identifier]["access"] & 2
                )
                self.entries[identifier].setReadOnly(not editable)
                self.update_value_state(identifier)

    def request(self, group: str, save: bool) -> dict:
        """提交编辑组与读取快照；只发送真正变化的可写字段。"""
        settings = dict(group=group, changes=None, snapshot=None)
        if save:
            snapshot = self.snapshots.get(group)
            if snapshot is None:
                raise ValueError("请先读取本组参数")
            changes = {}
            for identifier, _ in GROUPS[group][1]:
                if CATALOG[identifier]["access"] & 2:
                    raw = encode_value(identifier, self.entries[identifier].text())
                    if raw != snapshot["rows"][identifier]["raw"]:
                        changes[identifier] = raw
            settings.update(changes=changes, snapshot=snapshot)
        return settings

    def display(self, report: dict) -> None:
        """保存后其他组修订缓存失效；失败保留编辑文本但要求重新读取。"""
        if report.get("group") == "all":
            if not report.get("passed"):
                self.snapshots.clear()
                self.message.setText(report.get("fatal", "全部读取失败") + "；请重新读取。")
                self.update_controls(self.busy, self.available)
                return
            rows = {int(key): value for key, value in report["rows"].items()}
            for group, (_, fields) in GROUPS.items():
                self.display(dict(report, group=group, rows={i: rows[i] for i, _ in fields}))
            self.message.setText(
                f"已读取全部 {len(rows)} 项参数 · 活动修订 {report['revision']} · 未保存编辑保留"
            )
            return
        drafts = {}
        old = self.snapshots.get(report.get("group"))
        if report.get("action") == "read" and old:
            for identifier, row in old["rows"].items():
                text = self.entries[identifier].text()
                if CATALOG[identifier]["access"] & 2 and text != value_text(identifier, row["raw"]):
                    drafts[identifier] = text
        if report.get("action") == "save" or "group" not in report:
            self.snapshots.clear()
        if report.get("passed"):
            group = report["group"]
            rows = {int(key): value for key, value in report["rows"].items()}
            self.snapshots[group] = dict(
                node=report["node"], uid=report["uid"], revision=report["revision"], rows=rows
            )
            for identifier, row in rows.items():
                entry = self.entries[identifier]
                entry.setReadOnly(False)
                entry.clear()
                entry.setText(drafts.get(identifier, value_text(identifier, row["raw"])))
                self.readbacks[identifier].setText(value_text(identifier, row["raw"]))
            action = "已保存并回读验证" if report["action"] == "save" else "已读取"
            self.message.setText(
                f"{GROUPS[group][0]}{action} · 活动修订 {report['revision']}"
                + (" · 已保留未保存编辑" if drafts else "")
            )
        else:
            self.snapshots.pop(report.get("group"), None)
            self.message.setText(report.get("fatal", "参数操作失败") + "；请重新读取。")
        self.update_controls(self.busy, self.available)
