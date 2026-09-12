"""Connect page: adapter open/close, device info, node scan, event log."""

from __future__ import annotations

from typing import Any

from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtWidgets import (
    QCheckBox,
    QFileDialog,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QListWidget,
    QListWidgetItem,
    QPushButton,
    QSplitter,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from mdrive_core.transport.controlcanfd import ControlCanFdConfig

from mdrive_gui import theme
from mdrive_gui.app_state import AppState
from mdrive_gui.worker import CanWorker

_NODE_COLUMNS = ("节点", "协议", "模式", "故障", "延迟", "状态")


def _muted_label(text: str) -> QLabel:
    label = QLabel(text)
    label.setProperty("muted", True)
    return label


def _value_label(text: str = "—") -> QLabel:
    label = QLabel(text)
    label.setProperty("mono", True)
    label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
    return label


class ConnectPage(QWidget):
    """Tab 1: device card, node table with auto-refresh, recent events."""

    nodeActivated = Signal(int)  # double-clicked node; main window selects it and jumps to params

    def __init__(self, state: AppState, worker: CanWorker, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.state = state
        self.worker = worker

        self.dll_edit = QLineEdit(str(ControlCanFdConfig().dll_path))
        browse = QPushButton("浏览…")
        browse.clicked.connect(self._browse_dll)
        self.open_button = QPushButton("打开设备")
        self.open_button.setProperty("variant", "primary")
        self.open_button.clicked.connect(self._open_device)
        self.close_button = QPushButton("关闭")
        self.close_button.clicked.connect(lambda: self.worker.submit("close"))

        self.info_fields = {
            "serial": _value_label(),
            "hardware": _value_label(),
            "firmware": _value_label(),
            "channels": _value_label(),
            "baud": _value_label(),
            "dll_sha": _value_label(),
        }

        device_card = QWidget()
        device_card.setObjectName("card")
        card_layout = QVBoxLayout(device_card)
        title = QLabel("设备")
        title.setProperty("cardTitle", True)
        card_layout.addWidget(title)
        dll_row = QHBoxLayout()
        dll_row.addWidget(_muted_label("DLL:"))
        dll_row.addWidget(self.dll_edit, 1)
        dll_row.addWidget(browse)
        card_layout.addLayout(dll_row)
        button_row = QHBoxLayout()
        button_row.addWidget(self.open_button)
        button_row.addWidget(self.close_button)
        button_row.addStretch(1)
        card_layout.addLayout(button_row)
        grid = QGridLayout()
        grid.setHorizontalSpacing(12)
        rows = (
            ("序列号", "serial"),
            ("硬件", "hardware"),
            ("固件", "firmware"),
            ("通道数", "channels"),
            ("波特率", "baud"),
            ("DLL SHA256", "dll_sha"),
        )
        for row, (caption, key) in enumerate(rows):
            grid.addWidget(_muted_label(caption), row, 0)
            grid.addWidget(self.info_fields[key], row, 1)
        card_layout.addLayout(grid)
        card_layout.addStretch(1)

        self.scan_button = QPushButton("扫描")
        self.scan_button.clicked.connect(self._scan)
        self.auto_refresh = QCheckBox("自动刷新 2s")
        self.auto_refresh.setChecked(False)

        self.node_table = QTableWidget(8, len(_NODE_COLUMNS))
        self.node_table.setHorizontalHeaderLabels(_NODE_COLUMNS)
        self.node_table.verticalHeader().setVisible(False)
        self.node_table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self.node_table.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
        self.node_table.setAlternatingRowColors(True)
        self.node_table.horizontalHeader().setStretchLastSection(True)
        self.node_table.itemDoubleClicked.connect(self._node_double_clicked)
        self._reset_node_table()

        node_card = QWidget()
        node_card.setObjectName("card")
        node_layout = QVBoxLayout(node_card)
        node_head = QHBoxLayout()
        node_title = QLabel("节点列表")
        node_title.setProperty("cardTitle", True)
        node_head.addWidget(node_title)
        node_head.addStretch(1)
        node_head.addWidget(self.scan_button)
        node_head.addWidget(self.auto_refresh)
        node_layout.addLayout(node_head)
        node_layout.addWidget(self.node_table)

        self.event_list = QListWidget()
        event_card = QWidget()
        event_card.setObjectName("card")
        event_layout = QVBoxLayout(event_card)
        event_title = QLabel("最近事件")
        event_title.setProperty("cardTitle", True)
        event_layout.addWidget(event_title)
        event_layout.addWidget(self.event_list)

        top_split = QSplitter(Qt.Orientation.Horizontal)
        top_split.addWidget(device_card)
        top_split.addWidget(node_card)
        top_split.setStretchFactor(0, 1)
        top_split.setStretchFactor(1, 1)

        layout = QVBoxLayout(self)
        layout.addWidget(top_split, 3)
        layout.addWidget(event_card, 2)

        self.refresh_timer = QTimer(self)
        self.refresh_timer.setInterval(2000)
        self.refresh_timer.timeout.connect(self._refresh_nodes)
        self.auto_refresh.toggled.connect(self._toggle_refresh)

        state.connectionChanged.connect(self._on_connection_changed)
        state.deviceInfoChanged.connect(self._on_device_info)
        state.nodesChanged.connect(self._on_nodes)
        state.logEmitted.connect(self._on_log)
        worker.nodeStatus.connect(self._on_node_status)
        self._on_connection_changed(state.connected)

    # --------------------------------------------------------------- actions
    def _browse_dll(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "选择 ControlCANFD.dll", self.dll_edit.text(), "DLL (*.dll)")
        if path:
            self.dll_edit.setText(path)

    def _open_device(self) -> None:
        self.worker.submit("open", dll_path=self.dll_edit.text().strip())
        self.scan_button.setEnabled(False)

    def _scan(self) -> None:
        self.worker.submit("scan")

    def _toggle_refresh(self, checked: bool) -> None:
        if checked and self.state.connected:
            self.refresh_timer.start()
        else:
            self.refresh_timer.stop()

    def _refresh_nodes(self) -> None:
        if not self.state.connected:
            return
        for node in self.state.online_nodes():
            self.worker.submit("node_status", node=node)

    def _node_double_clicked(self, item: QTableWidgetItem) -> None:
        row = item.row()
        status = self.node_table.item(row, 5)
        if status is None or status.text() != "在线":
            return
        self.nodeActivated.emit(row)

    # ---------------------------------------------------------------- events
    def _on_connection_changed(self, connected: bool) -> None:
        self.open_button.setEnabled(not connected)
        self.close_button.setEnabled(connected)
        self.scan_button.setEnabled(connected)
        self.dll_edit.setEnabled(not connected)
        if not connected:
            self.refresh_timer.stop()
            self._reset_node_table()
        elif self.auto_refresh.isChecked():
            self.refresh_timer.start()

    def _on_device_info(self, info: dict[str, Any]) -> None:
        if not info:
            for field in self.info_fields.values():
                field.setText("—")
            return
        sha = str(info.get("dll_sha256", ""))
        self.info_fields["serial"].setText(str(info.get("serial", "—")))
        self.info_fields["hardware"].setText(str(info.get("hardware", "—")))
        self.info_fields["firmware"].setText(str(info.get("firmware_version", "—")))
        self.info_fields["channels"].setText(str(info.get("channels", "—")))
        self.info_fields["baud"].setText(f"{info.get('nominal_bps', 0) // 1000}k / {info.get('data_bps', 0) // 1000}k")
        self.info_fields["dll_sha"].setText(f"{sha[:8]}…{sha[-5:]}" if len(sha) > 13 else sha or "—")
        self.info_fields["dll_sha"].setToolTip(sha)

    def _on_nodes(self, nodes: list[dict[str, Any]]) -> None:
        self._reset_node_table()
        for entry in nodes:
            node = int(entry.get("node", 0))
            if not 0 <= node < 8:
                continue
            self._fill_node_row(node, entry)
        self.node_table.resizeColumnsToContents()
        self.node_table.horizontalHeader().setStretchLastSection(True)

    def _on_node_status(self, status: dict[str, Any]) -> None:
        node = int(status.get("node", 0))
        if 0 <= node < 8:
            self._fill_node_row(node, status)

    def _on_log(self, level: str, message: str) -> None:
        from datetime import datetime

        stamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        item = QListWidgetItem(f"{stamp}  {message}")
        if level == "ERROR":
            item.setForeground(theme.color(theme.ERROR))
        elif level == "WARN":
            item.setForeground(theme.color(theme.WARN))
        self.event_list.insertItem(0, item)
        while self.event_list.count() > 200:
            self.event_list.takeItem(self.event_list.count() - 1)

    # ------------------------------------------------------------ table fill
    def _reset_node_table(self) -> None:
        for row in range(8):
            self._fill_node_row(row, None)

    def _fill_node_row(self, row: int, entry: dict[str, Any] | None) -> None:
        online = entry is not None and entry.get("online", False)
        values = (
            str(row),
            str(entry.get("revision", "—")) if entry else "—",
            _fmt(entry, "mode") if entry else "—",
            _fmt(entry, "fault") if entry else "—",
            f"{entry.get('latency_ms', 0):.1f} ms" if entry and "latency_ms" in entry else "—",
            "在线" if online else "离线",
        )
        for column, text in enumerate(values):
            cell = self.node_table.item(row, column)
            if cell is None:
                cell = QTableWidgetItem()
                cell.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                self.node_table.setItem(row, column, cell)
            cell.setText(text)
            if column == 5:
                cell.setForeground(theme.color(theme.OK if online else theme.FAINT))
            elif not online:
                cell.setForeground(theme.color(theme.FAINT))
            else:
                cell.setForeground(theme.color(theme.TEXT))


def _fmt(entry: dict[str, Any], key: str) -> str:
    value = entry.get(key)
    if value is None:
        return "—"
    return f"{float(value):g}"
