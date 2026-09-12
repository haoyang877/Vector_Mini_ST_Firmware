"""Main window: menus, toolbar, five tabs, status bar — all bus I/O via CanWorker."""

from __future__ import annotations

from typing import Any

from PySide6.QtGui import QAction, QKeySequence
from PySide6.QtWidgets import (
    QComboBox,
    QLabel,
    QMainWindow,
    QMenu,
    QSizePolicy,
    QStatusBar,
    QTabWidget,
    QToolBar,
    QToolButton,
    QWidget,
)

from mdrive_gui import theme
from mdrive_gui.app_state import AppState
from mdrive_gui.pages.connect_page import ConnectPage
from mdrive_gui.pages.log_page import LogPage
from mdrive_gui.pages.params_page import ParamsPage
from mdrive_gui.pages.plots_page import PlotsPage
from mdrive_gui.pages.upgrade_page import UpgradePage
from mdrive_gui.worker import CanWorker

TAB_CONNECT, TAB_UPGRADE, TAB_PARAMS, TAB_PLOTS, TAB_LOG = range(5)


def _value_label(text: str = "—") -> QLabel:
    label = QLabel(text)
    label.setProperty("value", True)
    return label


class MainWindow(QMainWindow):
    """Single-window host app; pages subscribe to AppState, act via CanWorker."""

    def __init__(self, state: AppState | None = None, worker: CanWorker | None = None, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.state = state or AppState()
        self.worker = worker or CanWorker()
        self._owns_worker = worker is None

        self.setWindowTitle("电机驱动器上位机 v0.1 — USB-CAN 调试 / 参数 / 升级")
        self.setMinimumSize(1200, 760)

        self.connect_page = ConnectPage(self.state, self.worker)
        self.upgrade_page = UpgradePage(self.state, self.worker)
        self.params_page = ParamsPage(self.state, self.worker)
        self.plots_page = PlotsPage(self.state, self.worker)
        self.log_page = LogPage(self.state, self.worker)

        self.tabs = QTabWidget()
        self.tabs.addTab(self.connect_page, "连接")
        self.tabs.addTab(self.upgrade_page, "固件升级")
        self.tabs.addTab(self.params_page, "参数")
        self.tabs.addTab(self.plots_page, "曲线")
        self.tabs.addTab(self.log_page, "日志")
        self.setCentralWidget(self.tabs)

        self._build_menus()
        self._build_toolbar()
        self._build_statusbar()
        self._wire()

    # ------------------------------------------------------------------ menus
    def _build_menus(self) -> None:
        file_menu = self.menuBar().addMenu("文件(&F)")
        import_action = QAction("导入参数…", self)
        import_action.setShortcut(QKeySequence("Ctrl+O"))
        import_action.triggered.connect(lambda: self._goto_params("import"))
        export_action = QAction("导出参数…", self)
        export_action.triggered.connect(lambda: self._goto_params("export"))
        quit_action = QAction("退出", self)
        quit_action.setShortcut(QKeySequence("Ctrl+Q"))
        quit_action.triggered.connect(self.close)
        file_menu.addActions([import_action, export_action])
        file_menu.addSeparator()
        file_menu.addAction(quit_action)

        device_menu = self.menuBar().addMenu("设备(&D)")
        connect_action = QAction("连接", self)
        connect_action.triggered.connect(self.connect_page._open_device)
        disconnect_action = QAction("断开", self)
        disconnect_action.triggered.connect(self._disconnect)
        scan_action = QAction("扫描", self)
        scan_action.setShortcut(QKeySequence("F5"))
        scan_action.triggered.connect(self._scan)
        device_menu.addActions([connect_action, disconnect_action, scan_action])

        help_menu = self.menuBar().addMenu("帮助(&H)")
        about_action = QAction("关于", self)
        about_action.triggered.connect(
            lambda: self.state.log("INFO", "电机驱动器上位机 v0.1 · 协议 rev2 · CAN-FD 1M/1M")
        )
        help_menu.addAction(about_action)

    def _goto_params(self, action: str) -> None:
        self.tabs.setCurrentIndex(TAB_PARAMS)
        if action == "import":
            self.params_page._import()
        else:
            self.params_page._export()

    # ---------------------------------------------------------------- toolbar
    def _build_toolbar(self) -> None:
        toolbar = QToolBar("主工具栏")
        toolbar.setMovable(False)
        self.addToolBar(toolbar)

        self.connect_button = QToolButton()
        self.connect_button.setText("连接 ▾")
        self.connect_button.setProperty("variant", "primary")
        self.connect_button.setPopupMode(QToolButton.ToolButtonPopupMode.InstantPopup)
        menu = QMenu(self.connect_button)
        menu.addAction("打开设备", self.connect_page._open_device)
        menu.addAction("关闭设备", self._disconnect)
        self.connect_button.setMenu(menu)
        self.connect_button.clicked.connect(self.connect_page._open_device)
        toolbar.addWidget(self.connect_button)

        scan_button = QAction("扫描节点", self)
        scan_button.triggered.connect(self._scan)
        toolbar.addAction(scan_button)
        toolbar.addSeparator()

        toolbar.addWidget(QLabel("节点:"))
        self.node_combo = QComboBox()
        self.node_combo.addItems([str(node) for node in range(8)])
        self.node_combo.currentIndexChanged.connect(self.state.select_node)
        toolbar.addWidget(self.node_combo)
        toolbar.addSeparator()

        self.link_badge = QLabel("● 未连接")
        self.link_badge.setObjectName("linkBadge")
        toolbar.addWidget(self.link_badge)

        spacer = QWidget()
        spacer.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        toolbar.addWidget(spacer)

        estop_button = QToolButton()
        estop_button.setText("⏹ 紧急停止")
        estop_button.setProperty("variant", "danger")
        estop_button.clicked.connect(self._estop)
        toolbar.addWidget(estop_button)

    # -------------------------------------------------------------- statusbar
    def _build_statusbar(self) -> None:
        bar = QStatusBar()
        self.setStatusBar(bar)
        self.status_device = _value_label()
        self.status_node = _value_label()
        self.status_telemetry = _value_label()
        self.status_errors = _value_label("0")
        for caption, value in (
            ("设备", self.status_device),
            ("节点", self.status_node),
            ("遥测", self.status_telemetry),
            ("错误", self.status_errors),
        ):
            label = QLabel(caption)
            bar.addWidget(label)
            bar.addWidget(value)
        bar.addWidget(QWidget(), 1)

    # ------------------------------------------------------------------ wiring
    def _wire(self) -> None:
        worker = self.worker
        state = self.state

        worker.connected.connect(self._on_connected)
        worker.disconnected.connect(lambda: state.set_connected(False))
        worker.nodes.connect(state.set_nodes)
        worker.nodeStatus.connect(state.update_node)
        worker.failed.connect(self._on_failed)
        worker.logged.connect(state.log)

        state.connectionChanged.connect(self._on_connection_changed)
        state.nodesChanged.connect(self._refresh_statusbar)
        state.selectedNodeChanged.connect(self._refresh_statusbar)
        state.deviceInfoChanged.connect(self._refresh_statusbar)
        state.telemetryChanged.connect(self._refresh_statusbar)
        state.errorRaised.connect(lambda _msg: self._refresh_statusbar())
        state.selectedNodeChanged.connect(self.node_combo.setCurrentIndex)

        self.connect_page.nodeActivated.connect(self._on_node_activated)

        self._refresh_statusbar()

    # ------------------------------------------------------------------ slots
    def _on_connected(self, info: dict[str, Any]) -> None:
        self.state.set_connected(True)
        self.state.set_device_info(info)

    def _on_failed(self, message: str) -> None:
        self.state.raise_error(message)
        self.statusBar().showMessage(message, 8000)

    def _on_node_activated(self, node: int) -> None:
        self.state.select_node(node)
        self.tabs.setCurrentIndex(TAB_PARAMS)

    def _scan(self) -> None:
        if not self.state.connected:
            self.state.raise_error("CAN 设备未打开，请先连接")
            return
        self.worker.submit("scan")

    def _estop(self) -> None:
        if not self.state.connected:
            self.state.raise_error("紧急停止需要已打开的 CAN 设备")
            return
        self.worker.submit("estop", node=self.state.selected_node)

    def _on_connection_changed(self, connected: bool) -> None:
        if connected:
            nominal = self.state.device_info.get("nominal_bps", 0) // 1000
            data = self.state.device_info.get("data_bps", 0) // 1000
            self.link_badge.setText(f"● 链路 {nominal}k/{data}k FD · 已连接")
            self.link_badge.setStyleSheet(f"color: {theme.OK}")
        else:
            self.link_badge.setText("● 未连接")
            self.link_badge.setStyleSheet(f"color: {theme.MUTED}")
        self._refresh_statusbar()

    def _refresh_statusbar(self, *_args: Any) -> None:
        info = self.state.device_info
        serial = str(info.get("serial", ""))
        self.status_device.setText(f"…{serial[-4:]}" if serial else "未连接")
        node = self.state.selected_node
        online = next((entry for entry in self.state.nodes if entry.get("node") == node), None)
        if online:
            self.status_node.setText(f"{node} 在线 (rev{online.get('revision', '?')})")
        else:
            self.status_node.setText(f"{node} 离线")
        telemetry = self.state.telemetry
        vbus = telemetry.get("vbus")
        ibus = telemetry.get("ibus")
        temp = telemetry.get("temp")
        if vbus is None and ibus is None and temp is None:
            self.status_telemetry.setText("—")
        else:
            vbus_text = f"{vbus:.1f}" if vbus is not None else "—"
            ibus_text = f"{ibus:.1f}" if ibus is not None else "—"
            temp_text = f"{temp:.1f}" if temp is not None else "—"
            self.status_telemetry.setText(f"{vbus_text}V · {ibus_text}A · {temp_text}°C")
        self.status_errors.setText(str(self.state.error_count))

    # ------------------------------------------------------------------ close
    def _disconnect(self) -> None:
        """Stop any running stream first so the close task can execute."""
        self.worker.request_stream_stop()
        self.worker.submit("close")

    def closeEvent(self, event) -> None:  # noqa: N802 - Qt override
        if self._owns_worker:
            self.worker.stop()
        super().closeEvent(event)

    def start_worker(self) -> None:
        if self._owns_worker and not self.worker.isRunning():
            self.worker.start()
