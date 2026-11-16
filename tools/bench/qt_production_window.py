"""Qt生产工作台：原生控件与任务映射，不直接调用设备SDK。"""

import json
import os
import queue
import sys
import time
from datetime import datetime
from pathlib import Path

from can_calibration_curves import ARCHIVE_TYPES, calibration_archive, model_directory_name
from can_motor_operations import describe_faults, speed_value
from five_axis_can_stability import PER_NODE, encode_frames, version_text
from five_axis_jlink import FIRST_FIT_AXES
from hardware_check import empty_report
from production_backend import (
    AXES,
    AXIS_SPEED_LIMITS,
    CHECK_DEFAULTS,
    MODES,
    RATES,
    STATUS_REFRESH_S,
    hardware_check_fields,
    motor_speed_fields,
    parse_raw,
)
from production_controller import ProductionController
from production_flow import ProductionFlow
from production_report import build_report, failure_summary, report_rows
from PySide6.QtCore import QSignalBlocker, QSize, Qt, QTimer, QUrl
from PySide6.QtGui import QColor, QDesktopServices, QIcon, QPixmap
from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDialog,
    QFileDialog,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QListWidget,
    QListWidgetItem,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QProgressBar,
    QPushButton,
    QRadioButton,
    QScrollArea,
    QSplitter,
    QStackedWidget,
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)
from qt_parameter_page import ParameterPage
from qt_production_icons import ICON_SIZE, icon
from qt_production_theme import apply_theme
from qt_speed_plot import SpeedPlot

VERSION = "2.1.2"


def label(text: str, name: str = "", wrap: bool = False) -> QLabel:
    widget = QLabel(text)
    widget.setObjectName(name)
    widget.setWordWrap(wrap)
    widget.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
    return widget


def button(text, callback, primary=False, *, icon_name: str) -> QPushButton:
    widget = QPushButton(text)
    widget.setIcon(icon(icon_name, emphasis=primary))
    widget.setIconSize(ICON_SIZE)
    if primary:
        widget.setObjectName("primary")
    widget.clicked.connect(callback)
    return widget


def combo(values, selected=None) -> QComboBox:
    widget = QComboBox()
    widget.addItems(list(values))
    if selected is not None:
        widget.setCurrentText(selected)
    return widget


def table(headers: list[str]) -> QTableWidget:
    widget = QTableWidget(0, len(headers))
    widget.setHorizontalHeaderLabels(headers)
    widget.verticalHeader().hide()
    widget.setAlternatingRowColors(True)
    widget.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
    widget.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
    widget.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.ResizeToContents)
    widget.horizontalHeader().setStretchLastSection(True)
    return widget


def rows(widget: QTableWidget, values: list) -> None:
    widget.setRowCount(len(values))
    for row, items in enumerate(values):
        for col, value in enumerate(items):
            widget.setItem(row, col, QTableWidgetItem(str(value)))


class ProductionWindow(QMainWindow):
    """主线程维护当前设备与视图；设备动作交给独立控制器。"""

    def __init__(self, controller=None, *, auto_connect=True) -> None:
        super().__init__()
        self.setWindowTitle(f"电机生产测试上位机 v{VERSION} · Qt")
        self.resize(1440, 960)
        self.setMinimumSize(1060, 740)
        apply_theme(QApplication.instance())
        self.resource = Path(getattr(sys, "_MEIPASS", Path(__file__).resolve().parent))
        self.setWindowIcon(QIcon(str(self.resource / "assets/production_motor.svg")))
        self.root_log = Path.home() / "CAN-Link-Test"
        self.controller = controller or ProductionController(
            Path(os.environ.get("CONTROLCANFD_DLL", self.resource / "ControlCANFD.dll")), self
        )
        self.controller.event.connect(self.on_event)
        self.controller.completed.connect(self.on_completed)
        self.controller.busy_changed.connect(self.update_controls)
        self.controller.closed.connect(self.finish_close)
        self.devices = {}
        self.current_motor = None
        self.identity = None
        self.verified_uid = None
        self.flow = ProductionFlow()
        self.report_dir = self.report_path = self.curve_path = None
        self.records = []
        self.active_settings = None
        self.next_refresh = 0.0
        self.next_pending = self.close_ready = False
        self.pages, self.edits = {}, {}
        self.action_buttons = []
        self.build_ui()
        self.refresh_timer = QTimer(self)
        self.refresh_timer.setInterval(100)
        self.refresh_timer.timeout.connect(self.refresh_status)
        self.refresh_timer.start()
        self.update_controls()
        if controller is None and auto_connect:
            QTimer.singleShot(300, self.initial_scan)

    def initial_scan(self) -> None:
        """启动仅尝试一次只读连接扫描；失败或用户断开后不循环重连。"""
        if not self.close_ready and not self.controller.busy and self.auto_refresh.isChecked():
            self.launch(dict(profile="can_scan"), automatic=True)

    def read_selected_status(self) -> None:
        """扫描完成后读取实际状态，先核对UID，不依赖编辑焦点或J-Link。"""
        if (
            not self.close_ready
            and not self.controller.busy
            and self.controller.connected
            and self.current_motor
            and self.auto_refresh.isChecked()
        ):
            self.next_refresh = time.monotonic() + STATUS_REFRESH_S
            self.launch(dict(profile="device_status"), automatic=True)

    def build_ui(self) -> None:
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(22, 18, 22, 14)
        root.setSpacing(12)
        heading = QHBoxLayout()
        heading.addWidget(label("电机生产测试上位机", "title"))
        heading.addWidget(label(f"PRODUCTION CONSOLE  /  v{VERSION}", "subtitle"))
        heading.addStretch()
        self.next_button = button("归档并开始下一台", self.next_device, icon_name="next-device")
        heading.addWidget(self.next_button)
        root.addLayout(heading)
        header = QHBoxLayout()
        connection = QGroupBox("通信连接")
        form = QGridLayout(connection)
        self.channel = combo(("1", "2"))
        self.nominal = combo(("500K", "1M"), "1M")
        self.data_rate = combo(RATES, "5M")
        for col, (text, widget) in enumerate(
            (("通道", self.channel), ("仲裁段", self.nominal), ("数据段", self.data_rate))
        ):
            form.addWidget(label(text), 0, col)
            form.addWidget(widget, 1, col)
            widget.currentTextChanged.connect(self.connection_changed)
        self.auto_refresh = QCheckBox("自动刷新 · 1 秒")
        self.auto_refresh.setChecked(True)
        self.connection_label = label("CAN 未连接", "subtitle")
        self.disconnect_button = button(
            "断开", lambda: self.launch(dict(profile="can_release")), icon_name="disconnect"
        )
        form.addWidget(self.auto_refresh, 2, 0, 1, 2)
        form.addWidget(self.disconnect_button, 2, 2)
        connection_status = QHBoxLayout()
        self.connection_icon = QLabel()
        self.connection_icon.setFixedSize(20, 20)
        connection_status.addWidget(self.connection_icon)
        connection_status.addWidget(self.connection_label, 1)
        form.addLayout(connection_status, 3, 0, 1, 3)
        header.addWidget(connection, 3)
        current = QGroupBox("当前电机")
        form = QVBoxLayout(current)
        form.setSpacing(10)
        self.axis = combo(
            [f"{role} · {model} · 节点 {node}" for node, (role, model, _) in FIRST_FIT_AXES.items()]
        )
        self.axis.setCurrentIndex(3)
        self.axis.currentIndexChanged.connect(self.select_motor)
        form.addWidget(self.axis)
        self.uid_label = label("尚未识别设备", "subtitle", True)
        form.addWidget(self.uid_label)
        self.scan_button = button(
            "连接 / 扫描", lambda: self.launch(dict(profile="can_scan")), icon_name="scan"
        )
        form.addWidget(self.scan_button, 0, Qt.AlignmentFlag.AlignLeft)
        form.addStretch()
        header.addWidget(current, 3)
        device = QGroupBox("设备状态")
        form = QVBoxLayout(device)
        details = QGridLayout()
        details.setHorizontalSpacing(18)
        details.setVerticalSpacing(5)
        self.version_label = label("APP 固件：—", "statusValue", True)
        self.loader_label = label("Loader 固件：—", "statusValue", True)
        self.status_label = label("运行状态：未读取", "statusValue", True)
        self.mode_label = label("控制模式：未读取", "statusValue", True)
        self.fault_label = label("故障信息：未读取", "statusValue", True)
        details.addWidget(self.version_label, 0, 0)
        details.addWidget(self.loader_label, 1, 0)
        details.addWidget(self.status_label, 0, 1)
        details.addWidget(self.mode_label, 1, 1)
        fault_row = QHBoxLayout()
        self.fault_icon = QLabel()
        self.fault_icon.setFixedSize(20, 20)
        fault_row.addWidget(self.fault_icon)
        fault_row.addWidget(self.fault_label, 1)
        details.setColumnStretch(0, 2)
        details.setColumnStretch(1, 3)
        metrics = QHBoxLayout()
        self.voltage = label("— V", "metric")
        self.current = label("— A", "metric")
        self.temperature = label("— °C", "metric")
        for widget, caption in (
            (self.voltage, "母线电压"),
            (self.current, "母线电流"),
            (self.temperature, "电机温度"),
        ):
            tile = QWidget()
            tile.setObjectName("telemetryTile")
            metric = QVBoxLayout(tile)
            metric.setContentsMargins(10, 8, 10, 8)
            metric.setSpacing(6)
            caption_row = QHBoxLayout()
            badge = QLabel()
            badge.setPixmap(
                icon(
                    "temperature"
                    if caption == "电机温度"
                    else "voltage"
                    if caption == "母线电压"
                    else "connect"
                ).pixmap(18, 18)
            )
            caption_row.addWidget(badge)
            caption_row.addWidget(label(caption, "subtitle"))
            caption_row.addStretch()
            metric.addLayout(caption_row)
            metric.addWidget(widget)
            metrics.addWidget(tile, 1)
        form.addLayout(details)
        form.addLayout(fault_row)
        form.addLayout(metrics)
        foot = QHBoxLayout()
        self.last_read = label("等待设备连接", "subtitle")
        foot.addWidget(self.last_read, 1)
        self.status_button = button(
            "刷新", lambda: self.launch(dict(profile="device_status")), icon_name="refresh"
        )
        self.fault_button = button(
            "清故障", lambda: self.launch(dict(profile="clear_fault")), icon_name="clear-fault"
        )
        foot.addWidget(self.status_button)
        foot.addWidget(self.fault_button)
        form.addLayout(foot)
        header.addWidget(device, 6)
        root.addLayout(header)
        self.flow_label = label(self.flow.summary(None), "subtitle", True)
        root.addWidget(self.flow_label)
        body = QHBoxLayout()
        self.navigation = QListWidget()
        self.navigation.setObjectName("navigation")
        self.navigation.setFixedWidth(160)
        self.navigation.setIconSize(QSize(22, 22))
        for name, text in (
            ("maintenance", "设备维护"),
            ("hardware-test", "生产检测"),
            ("calibration", "电机标定"),
            ("parameters", "参数配置"),
            ("records", "测试记录"),
        ):
            self.navigation.addItem(QListWidgetItem(icon(name), text))
        self.navigation.currentRowChanged.connect(self.change_page)
        body.addWidget(self.navigation)
        self.splitter = QSplitter(Qt.Orientation.Vertical)
        self.stack = QStackedWidget()
        self.page_scroll = QScrollArea()
        self.page_scroll.setWidgetResizable(True)
        self.page_scroll.setWidget(self.stack)
        self.page_scroll.setMinimumHeight(240)
        self.splitter.addWidget(self.page_scroll)
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(2000)
        self.log.setPlaceholderText("运行记录 · 设备操作开始后显示原始进度与诊断信息")
        self.splitter.addWidget(self.log)
        self.splitter.setSizes([570, 145])
        body.addWidget(self.splitter, 1)
        root.addLayout(body, 1)
        self.build_maintenance()
        self.build_hardware()
        self.build_calibration()
        self.parameters = ParameterPage(self.parameter_action)
        self.stack.addWidget(self.parameters)
        self.build_records()
        footer = QHBoxLayout()
        self.progress = QProgressBar()
        self.progress.setRange(0, 100)
        self.progress.setTextVisible(False)
        self.progress.setMaximumWidth(230)
        footer.addWidget(self.progress)
        self.task_status = label("就绪", "subtitle", True)
        footer.addWidget(self.task_status, 1)
        self.open_log_button = button(
            "本次记录", lambda: self.open_path(self.report_dir), icon_name="folder"
        )
        footer.addWidget(self.open_log_button)
        footer.addWidget(
            button(
                "收起 / 展开日志",
                lambda: self.log.setVisible(not self.log.isVisible()),
                icon_name="records",
            )
        )
        self.stop_button = button("停止", self.controller.stop, icon_name="stop")
        self.stop_button.setObjectName("stop")
        footer.addWidget(self.stop_button)
        root.addLayout(footer)
        self.navigation.setCurrentRow(0)

    def tab_page(self, tabs, name, title):
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(20, 16, 20, 16)
        layout.setSpacing(12)
        self.pages[name] = page
        tabs.addTab(
            page,
            icon(
                {
                    "jlink": "flash",
                    "upgrade": "upgrade",
                    "communication": "communication",
                    "advanced": "maintenance",
                    "calibration": "calibration",
                    "speed": "chart",
                }[name]
            ),
            title,
        )
        return layout

    def add_action(self, layout, text, callback, primary=False, *, icon_name: str):
        widget = button(text, callback, primary, icon_name=icon_name)
        self.action_buttons.append(widget)
        layout.addWidget(widget)
        return widget

    def add_fields(self, layout, definitions):
        form = QFormLayout()
        form.setSpacing(10)
        for key, title, value in definitions:
            widget = QLineEdit(str(value))
            widget.setMaximumWidth(
                720 if key == "payload" else 200 if key in ("specimen", "serial") else 120
            )
            self.edits[key] = widget
            form.addRow(title, widget)
        layout.addLayout(form)

    def build_maintenance(self) -> None:
        self.maintenance = QTabWidget()
        self.stack.addWidget(self.maintenance)
        layout = self.tab_page(self.maintenance, "jlink", "整包烧录")
        layout.addWidget(label("新板首次烧录", "sectionTitle"))
        layout.addWidget(
            label(
                "01 识别板卡    →    02 确认电机类型    →    03 烧录整包    →    04 启动回读",
                "subtitle",
            )
        )
        self.install_label = label("连接 J-Link 与板卡电源，然后识别当前板卡。", wrap=True)
        layout.addWidget(self.install_label)
        content = QHBoxLayout()
        firmware = QGroupBox("本批固件")
        box = QVBoxLayout(firmware)
        box.addWidget(label("APP 1.8.16   +   Loader 2.3.4", "metric"))
        box.addWidget(label("AT32F456  ·  CAN FD 1M / 5M", "subtitle"))
        self.hex_path = self.resource / "firmware/five-axis.hex"
        if not self.hex_path.is_file():
            self.hex_path = None
        self.hex_label = label("内置统一整包" if self.hex_path else "尚未选择本批 HEX", wrap=True)
        box.addWidget(self.hex_label)
        self.add_action(box, "选择本批 HEX", self.choose_hex, icon_name="folder")
        box.addStretch()
        content.addWidget(firmware, 1)
        self.initial_box = QGroupBox("新板电机类型 · 必选")
        choices = QVBoxLayout(self.initial_box)
        self.initial_choices = {}
        for node, (role, model, _) in FIRST_FIT_AXES.items():
            radio = QRadioButton(f"{role}  /  {model}  /  节点 {node}")
            radio.toggled.connect(self.update_controls)
            choices.addWidget(radio)
            self.initial_choices[node] = radio
        content.addWidget(self.initial_box, 1)
        layout.addLayout(content)
        actions = QHBoxLayout()
        self.identify_button = self.add_action(
            actions,
            "1. 识别板卡",
            lambda: self.launch(dict(profile="jlink_identify")),
            icon_name="scan",
        )
        self.flash_button = self.add_action(
            actions,
            "3. 烧录整包并校验",
            lambda: self.launch(dict(profile="jlink_flash")),
            True,
            icon_name="flash",
        )
        self.go_detection = button(
            "进入生产检测  →", lambda: self.navigation.setCurrentRow(1), icon_name="hardware-test"
        )
        actions.addWidget(self.go_detection)
        actions.addStretch()
        layout.addLayout(actions)
        layout.addStretch()
        layout = self.tab_page(self.maintenance, "upgrade", "CAN 固件升级")
        layout.addWidget(label("CAN 固件升级", "sectionTitle"))
        upgrade_body = QHBoxLayout()
        settings = QGroupBox("升级操作")
        settings.setMinimumWidth(240)
        settings.setMaximumWidth(300)
        settings_layout = QVBoxLayout(settings)
        self.package_path = (
            self.resource / "can-app"
            if getattr(sys, "frozen", False)
            else self.resource.parents[1] / "outputs/AT32F456-FULL-v1.8.16/app-package"
        )
        self.package_label = label("", wrap=True)
        settings_layout.addWidget(self.package_label)
        self.add_action(settings_layout, "选择升级包目录", self.choose_package, icon_name="folder")
        self.add_action(
            settings_layout,
            "扫描在线设备",
            lambda: self.launch(dict(profile="can_scan")),
            icon_name="scan",
        )
        self.batch_upgrade = QCheckBox("批量升级已扫描的设备")
        settings_layout.addWidget(self.batch_upgrade)
        self.add_fields(settings_layout, [("rounds", "每台升级次数", "1")])
        self.edits["rounds"].setFixedWidth(72)
        self.single_frame = QCheckBox("兼容单帧写入")
        settings_layout.addWidget(self.single_frame)
        self.add_action(
            settings_layout,
            "开始 CAN 升级",
            lambda: self.launch(dict(profile="can_upgrade")),
            True,
            icon_name="upgrade",
        )
        settings_layout.addStretch()
        devices = QGroupBox("连接与固件信息")
        devices_layout = QVBoxLayout(devices)
        self.upgrade_package_info = label("", "subtitle", True)
        devices_layout.addWidget(self.upgrade_package_info)
        self.upgrade_table = table(
            ["选择", "节点 / 电机", "UID", "连接", "APP 固件", "Loader 固件", "升级状态"]
        )
        devices_layout.addWidget(self.upgrade_table, 1)
        upgrade_body.addWidget(settings)
        upgrade_body.addWidget(devices, 1)
        layout.addLayout(upgrade_body, 1)
        self.update_upgrade_package()
        layout = self.tab_page(self.maintenance, "communication", "通信测试")
        layout.addWidget(label("通信测试", "sectionTitle"))
        self.communication_tabs = QTabWidget()
        layout.addWidget(self.communication_tabs)
        page = QWidget()
        panel = QVBoxLayout(page)
        self.test_mode = combo(MODES)
        self.test_mode.setMaximumWidth(260)
        panel.addWidget(self.test_mode)
        self.multi_query = QCheckBox("多节点查询")
        panel.addWidget(self.multi_query)
        picks = QHBoxLayout()
        self.query_checks = {}
        for node, text in AXES.items():
            check = QCheckBox(f"{text} · {node}")
            check.setChecked(node != 8)
            picks.addWidget(check)
            self.query_checks[node] = check
        panel.addLayout(picks)
        self.add_fields(panel, [("count", "每节点笔数", "1000"), ("rate", "总笔 / 秒", "50")])
        self.add_action(panel, "开始通信测试", self.start_query, True, icon_name="communication")
        panel.addStretch()
        self.communication_tabs.addTab(page, icon("communication"), "通信压测")
        page = QWidget()
        panel = QVBoxLayout(page)
        self.add_fields(panel, [("can_id", "CAN ID · HEX", ""), ("payload", "数据 · HEX", "")])
        self.add_action(panel, "填入当前电机状态查询", self.fill_query, icon_name="refresh")
        self.add_action(panel, "发送一帧", self.send_frame, True, icon_name="communication")
        panel.addStretch()
        self.communication_tabs.addTab(page, icon("connect"), "协议发送")
        self.fill_query()
        layout = self.tab_page(self.maintenance, "advanced", "高级维护")
        layout.addWidget(label("维护操作", "sectionTitle"))
        self.target_nominal = combo(("500K", "1M"), "1M")
        self.target_data = combo(RATES, "5M")
        self.target_nominal.setMaximumWidth(120)
        self.target_data.setMaximumWidth(120)
        form = QFormLayout()
        form.addRow("目标仲裁速率", self.target_nominal)
        form.addRow("目标数据速率", self.target_data)
        layout.addLayout(form)
        actions = QHBoxLayout()
        self.add_action(
            actions,
            "读取波特率",
            lambda: self.launch(dict(profile="can_bitrate", set=None)),
            icon_name="refresh",
        )
        self.add_action(actions, "保存波特率", self.save_bitrate, icon_name="save")
        actions.addStretch()
        layout.addLayout(actions)
        layout.addWidget(
            label("擦除前必须识别当前芯片。操作会保存备份，擦除后需重新烧录。", wrap=True)
        )
        self.erase_app = self.add_action(
            layout, "擦除 APP · 保留参数和 Loader", lambda: self.erase("app"), icon_name="warning"
        )
        self.erase_all = self.add_action(
            layout, "整片擦除 · 含参数和身份", lambda: self.erase("all"), icon_name="warning"
        )
        layout.addStretch()

    def build_hardware(self) -> None:
        page = QWidget()
        self.stack.addWidget(page)
        layout = QVBoxLayout(page)
        layout.setContentsMargins(20, 16, 20, 16)
        layout.addWidget(label("生产检测", "sectionTitle"))
        self.phase_label = label("通信 → 静态 → 相电阻 → 双向动态 → 关相 / 结果", "subtitle", True)
        layout.addWidget(self.phase_label)
        splitter = QSplitter()
        settings = QGroupBox("检测设置")
        panel = QVBoxLayout(settings)
        self.check_mode = combo(
            ("关相静态", "静态＋相电阻", "静态和双向驱动"), CHECK_DEFAULTS["mode"]
        )
        self.check_mode.setMaximumWidth(260)
        panel.addWidget(self.check_mode)
        self.add_fields(
            panel,
            [
                ("ambient", "室温 °C · 可空", "20"),
                ("current", "相电流 A", "3"),
                ("frequency", "电频 Hz", "3"),
                ("leg", "单程 ms", "1500"),
                ("travel", "两侧可用行程 °", "360"),
            ],
        )
        self.hardware_start = self.add_action(
            panel, "开始本轮检测", self.start_hardware, True, icon_name="start"
        )
        self.open_report_button = button(
            "打开单台报告", lambda: self.open_path(self.report_path), icon_name="records"
        )
        panel.addWidget(self.open_report_button)
        panel.addStretch()
        splitter.addWidget(settings)
        result = QWidget()
        panel = QVBoxLayout(result)
        result_title = QHBoxLayout()
        self.result_icon = QLabel()
        self.result_icon.setPixmap(icon("hardware-test").pixmap(20, 20))
        result_title.addWidget(self.result_icon)
        result_title.addWidget(label("本轮检测结果", "statusValue"), 1)
        panel.addLayout(result_title)
        self.failure = QPlainTextEdit()
        self.failure.setReadOnly(True)
        self.failure.setMaximumHeight(110)
        self.failure.setPlainText("尚未开始本轮检测")
        panel.addWidget(self.failure)
        self.hardware_table = table(["测试项目", "实测结果", "判定标准", "结果"])
        self.hardware_table.horizontalHeader().setSectionResizeMode(
            1, QHeaderView.ResizeMode.Stretch
        )
        panel.addWidget(self.hardware_table, 1)
        splitter.addWidget(result)
        splitter.setSizes([290, 780])
        layout.addWidget(splitter, 1)

    def build_calibration(self) -> None:
        self.calibration = QTabWidget()
        self.stack.addWidget(self.calibration)
        layout = self.tab_page(self.calibration, "calibration", "角度与零位")
        layout.addWidget(label("电机标定", "sectionTitle"))
        self.add_fields(
            layout, [("specimen", "样机序列号", "0001"), ("serial", "测试序列号", "0001")]
        )
        actions = QHBoxLayout()
        self.add_action(
            actions,
            "编码器 / Hall 角度标定并保存",
            lambda: self.start_calibration(1),
            True,
            icon_name="calibration",
        )
        self.add_action(
            actions,
            "电角零位标定并保存",
            lambda: self.start_calibration(2),
            icon_name="calibration",
        )
        actions.addStretch()
        layout.addLayout(actions)
        layout.addWidget(
            label("完成后回读 Flash 保存状态；减速轴沿用固件的联合标定流程。", "subtitle", True)
        )
        self.calibration_result = label("等待标定", wrap=True)
        layout.addWidget(self.calibration_result)
        self.curve_button = button("查看补偿曲线 / 原码", self.show_curves, icon_name="chart")
        layout.addWidget(self.curve_button)
        layout.addStretch()
        layout = self.tab_page(self.calibration, "speed", "速度调试")
        layout.addWidget(label("速度闭环调试", "sectionTitle"))
        self.add_fields(
            layout, [("speed", "目标速度 rad/s", "3"), ("speed_limit", "速度上限 rad/s", "300")]
        )
        actions = QHBoxLayout()
        self.add_action(actions, "使能速度控制", self.start_speed, True, icon_name="start")
        self.speed_update = button("更新速度目标", self.update_speed, icon_name="refresh")
        actions.addWidget(self.speed_update)
        self.speed_stop = button("停止", self.controller.stop, icon_name="stop")
        actions.addWidget(self.speed_stop)
        actions.addStretch()
        layout.addLayout(actions)
        self.speed_label = label("等待速度反馈", "subtitle")
        layout.addWidget(self.speed_label)
        self.plot = SpeedPlot()
        layout.addWidget(self.plot, 1)

    def build_records(self) -> None:
        page = QWidget()
        self.stack.addWidget(page)
        layout = QVBoxLayout(page)
        layout.setContentsMargins(20, 16, 20, 16)
        title = QHBoxLayout()
        title.addWidget(label("测试记录", "sectionTitle"))
        title.addStretch()
        title.addWidget(
            button("打开历史记录目录", lambda: self.open_path(self.root_log), icon_name="folder")
        )
        layout.addLayout(title)
        self.record_table = table(["时间", "任务", "节点", "结果"])
        self.record_table.cellDoubleClicked.connect(
            lambda row, _col: self.open_path(self.records[row]["path"])
        )
        layout.addWidget(self.record_table)
        layout.addWidget(label("本次启动完成的操作；双击打开对应记录目录。", "subtitle"))

    def node(self) -> int:
        return list(FIRST_FIT_AXES)[self.axis.currentIndex()]

    def bus(self) -> dict:
        return dict(
            channel_index=int(self.channel.currentText()) - 1,
            nominal_bps=RATES[self.nominal.currentText()],
            data_bps=RATES[self.data_rate.currentText()],
        )

    def change_page(self, index: int) -> None:
        self.stack.setCurrentIndex(index)

    def connection_changed(self, *_args) -> None:
        self.devices.clear()
        self.identity = self.verified_uid = None
        self.select_motor()

    def select_motor(self, *_args) -> None:
        old = self.current_motor
        self.current_motor = self.devices.get(self.node())
        if not old or not self.current_motor or old["uid"] != self.current_motor["uid"]:
            self.parameters.reset(
                (self.node(), self.current_motor["uid"]) if self.current_motor else None
            )
            self.plot.clear()
            self.edits["speed_limit"].setText(str(AXIS_SPEED_LIMITS[self.node()]))
        if self.identity and (
            self.identity.get("node") not in (None, self.node())
            or self.current_motor
            and self.identity["uid"] != self.current_motor["uid"]
        ):
            self.identity = None
        self.render_device()
        self.update_controls()

    def render_device(self) -> None:
        motor = self.current_motor or {}
        self.connection_label.setText(
            "CAN 已连接 · ISO FD + BRS" if self.controller.connected else "CAN 未连接"
        )
        self.connection_icon.setPixmap(
            icon("connect" if self.controller.connected else "disconnect").pixmap(20, 20)
        )
        self.uid_label.setText("UID " + motor["uid"] if motor else "尚未识别设备")
        self.version_label.setText(
            "APP 固件："
            + ("v" + version_text(motor["app_version"]) if motor.get("app_version") else "未读取")
        )
        self.loader_label.setText(
            "Loader 固件："
            + (
                "v" + version_text(motor["loader_version"])
                if motor.get("loader_version")
                else "未读取"
            )
        )
        stamp = motor.get("checked_at")
        stale = bool(stamp and time.time() - stamp > 5)
        error = motor.get("status_error")
        state_code, mode_code = motor.get("state"), motor.get("mode")
        state = {3: "自检", 4: "标定", 5: "空闲", 6: "运行", 8: "故障"}.get(
            state_code, "未读取" if state_code is None else f"未知状态（{state_code}）"
        )
        if motor.get("role") == 2:
            state = "Loader · 等待升级"
        status = "读取失败" if error else "数据已过期" if stale else state
        self.status_label.setText("运行状态：" + status)
        modes = {
            0: "未使能",
            1: "电流闭环",
            2: "速度闭环",
            3: "位置闭环",
            6: "电压控制",
            9: "开环电压",
            10: "开环电流",
        }
        mode = modes.get(mode_code, "未知模式")
        mode = "未读取" if mode_code is None else f"{mode}（模式 {mode_code}）"
        historical = "（上次读取）" if stale or error else ""
        self.mode_label.setText("控制模式：" + mode + historical)
        faults = motor.get("faults")
        self.fault_label.setText(
            "故障信息："
            + (
                "未读取"
                if faults is None
                else describe_faults(faults, include_codes=False) + historical
            )
        )
        self.fault_label.setStyleSheet("color:#b42336;" if faults else "")
        self.fault_icon.setPixmap(
            icon(
                "warning" if faults is None or stale or error else "fail" if faults else "pass"
            ).pixmap(20, 20)
        )
        self.fault_icon.setVisible(faults is not None)
        self.status_label.setToolTip(str(error) if error else "")
        for widget, key, unit, digits in (
            (self.voltage, "bus_voltage_v", "V", 2),
            (self.current, "bus_current_a", "A", 3),
            (self.temperature, "motor_temperature_c", "°C", 2),
        ):
            value = motor.get(key) if not stale and not error and motor.get("role") == 1 else None
            widget.setText(f"{value:.{digits}f} {unit}" if value is not None else "— " + unit)
            widget.setProperty("unavailable", value is None)
            widget.style().unpolish(widget)
            widget.style().polish(widget)
        self.last_read.setText(
            "读取 " + datetime.fromtimestamp(stamp).strftime("%H:%M:%S")
            if stamp
            else "等待设备连接"
        )
        self.flow_label.setText(self.flow.summary(motor.get("uid")))

    def update_controls(self, *_args) -> None:
        busy = self.controller.busy
        automatic = busy and self.controller.automatic
        for widget in self.action_buttons:
            widget.setEnabled(not busy or automatic)
        for widget in (self.axis, self.channel, self.nominal, self.data_rate):
            widget.setEnabled(not busy)
        self.scan_button.setEnabled(not busy or automatic)
        self.navigation.setEnabled(not busy or automatic)
        for key, edit in self.edits.items():
            edit.setEnabled(
                not busy
                or automatic
                or (key == "speed" and self.controller.settings.get("profile") == "motor_speed")
            )
        for tabs in (self.maintenance, self.calibration, self.communication_tabs):
            for index in range(tabs.count()):
                tabs.setTabEnabled(index, not busy or automatic or index == tabs.currentIndex())
        blank = self.identity is not None and self.identity.get("node") is None
        self.initial_box.setVisible(blank)
        for choice in self.initial_choices.values():
            choice.setEnabled(
                not busy and (not self.identity or "resume_node" not in self.identity)
            )
        selected = any(choice.isChecked() for choice in self.initial_choices.values())
        self.flash_button.setEnabled(
            not busy
            and self.hex_path is not None
            and self.identity is not None
            and (not blank or selected)
        )
        self.go_detection.setEnabled(
            not busy
            and self.current_motor is not None
            and self.verified_uid == self.current_motor["uid"]
        )
        self.next_button.setEnabled(not busy or automatic)
        self.status_button.setEnabled((not busy or automatic) and bool(self.current_motor))
        self.fault_button.setEnabled(
            not busy and bool(self.current_motor and self.current_motor.get("role") == 1)
        )
        self.disconnect_button.setEnabled(not busy and self.controller.connected)
        self.batch_upgrade.setEnabled(not busy)
        self.multi_query.setEnabled(not busy)
        self.single_frame.setEnabled(not busy)
        self.test_mode.setEnabled(not busy)
        self.check_mode.setEnabled(not busy)
        self.upgrade_table.setEnabled(not busy)
        for check in self.query_checks.values():
            check.setEnabled(not busy)
        self.target_nominal.setEnabled(not busy)
        self.target_data.setEnabled(not busy)
        self.stop_button.setEnabled(
            busy and not self.controller.settings.get("profile", "").startswith("jlink_")
        )
        speed_active = busy and self.controller.settings.get("profile") == "motor_speed"
        self.speed_update.setEnabled(speed_active)
        self.speed_stop.setEnabled(speed_active)
        self.parameters.update_controls(
            busy and not automatic, bool(self.current_motor and self.current_motor.get("role") == 1)
        )
        self.open_log_button.setEnabled(self.report_dir is not None)
        self.open_report_button.setEnabled(self.report_path is not None)
        self.curve_button.setEnabled(self.curve_path is not None)
        self.erase_all.setEnabled(not busy and self.identity is not None)
        self.erase_app.setEnabled(
            not busy and self.identity is not None and self.identity.get("node") is not None
        )
        self.render_device()

    def refresh_status(self) -> None:
        self.render_device()
        if (
            self.controller.busy
            or not self.controller.connected
            or not self.current_motor
            or not self.auto_refresh.isChecked()
        ):
            return
        focus = QApplication.focusWidget()
        if isinstance(focus, (QLineEdit, QComboBox)):
            return
        if time.monotonic() < self.next_refresh:
            return
        self.next_refresh = time.monotonic() + STATUS_REFRESH_S
        self.launch(dict(profile="device_status"), automatic=True)

    def choose_hex(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "选择本批合并 HEX", "", "Intel HEX (*.hex)")
        if path:
            self.hex_path = Path(path)
            self.hex_label.setText(self.hex_path.name)
            self.update_controls()

    def choose_package(self) -> None:
        path = QFileDialog.getExistingDirectory(self, "选择包含 image.bin 和 manifest.bin 的目录")
        if path:
            self.package_path = Path(path)
            self.update_upgrade_package()

    def update_upgrade_package(self) -> None:
        """路径只显示末级目录，完整路径留在提示；版本来自实际升级包元数据。"""
        self.package_label.setText("升级包：" + self.package_path.name)
        self.package_label.setToolTip(str(self.package_path))
        try:
            metadata = json.loads((self.package_path / "package.json").read_text(encoding="utf-8"))
            app = version_text(int(metadata["image_version"]))
            loader = version_text(int(metadata["loader_version"]))
            self.upgrade_package_info.setText(f"目标固件：APP v{app}  ·  Loader v{loader}")
        except (OSError, ValueError, KeyError, TypeError):
            self.upgrade_package_info.setText("目标固件：未读取有效升级包，请选择目录")

    def fill_query(self) -> None:
        identifier, data = encode_frames(self.node(), 108, b"", 1)[0]
        self.edits["can_id"].setText(f"{identifier:08X}")
        self.edits["payload"].setText(data.hex(" ").upper())

    def send_frame(self) -> None:
        try:
            self.launch(
                dict(
                    profile="manual",
                    frame=parse_raw(self.edits["can_id"].text(), self.edits["payload"].text()),
                )
            )
        except ValueError as exc:
            self.error(str(exc))

    def start_query(self) -> None:
        try:
            profile = MODES[self.test_mode.currentText()]
            nodes = (
                [3, 4, 5, 6, 7]
                if profile == "acceptance"
                else [n for n, w in self.query_checks.items() if w.isChecked()]
                if self.multi_query.isChecked()
                else [self.node()]
            )
            if not nodes:
                raise ValueError("请至少选择一个查询节点")
            if profile == "acceptance" and (
                self.nominal.currentText() != "1M" or self.data_rate.currentText() != "5M"
            ):
                raise ValueError("五轴验收需要1M/5M")
            count = int(self.edits["count"].text()) if profile == "fixed" else PER_NODE
            rate = int(self.edits["rate"].text()) if profile == "fixed" else 50
            if not 1 <= count <= 1000000 or not 1 <= rate <= 10000:
                raise ValueError("每节点笔数1～1000000，总笔/秒1～10000")
            self.launch(
                dict(
                    profile=profile,
                    nodes=nodes,
                    count=count,
                    rate=rate,
                    node_uids={n: m["uid"] for n, m in self.devices.items()},
                )
            )
        except ValueError as exc:
            self.error(str(exc))

    def start_hardware(self) -> None:
        try:
            settings = hardware_check_fields(
                {
                    key: self.edits[key].text()
                    for key in ("ambient", "current", "frequency", "leg", "travel")
                },
                self.check_mode.currentText() == "静态和双向驱动",
            )
            self.launch(
                dict(
                    profile="hardware_check",
                    resistance_only=self.check_mode.currentText() == "静态＋相电阻",
                    **settings,
                )
            )
        except ValueError as exc:
            self.error(str(exc))

    def start_calibration(self, kind: int) -> None:
        try:
            archive = calibration_archive(
                ARCHIVE_TYPES[self.node()],
                self.edits["specimen"].text(),
                self.edits["serial"].text(),
            )
            self.launch(dict(profile="motor_calibration", calibration_kind=kind, archive=archive))
        except ValueError as exc:
            self.error(str(exc))

    def start_speed(self) -> None:
        try:
            settings = motor_speed_fields(
                dict(speed=self.edits["speed"].text(), limit=self.edits["speed_limit"].text())
            )
            self.launch(dict(profile="motor_speed", **settings))
        except ValueError as exc:
            self.error(str(exc))

    def update_speed(self) -> None:
        try:
            self.controller.update_speed(speed_value(self.edits["speed"].text()))
        except (ValueError, RuntimeError, queue.Full) as exc:
            self.error(str(exc) or "目标队列已满")

    def parameter_action(self, group: str, save: bool) -> None:
        try:
            self.launch(dict(profile="parameters", **self.parameters.request(group, save)))
        except ValueError as exc:
            self.error(str(exc))

    def save_bitrate(self) -> None:
        self.launch(
            dict(
                profile="can_bitrate",
                set=(
                    RATES[self.target_nominal.currentText()] // 1000,
                    RATES[self.target_data.currentText()] // 1000,
                ),
            )
        )

    def erase(self, scope: str) -> None:
        if not self.identity or self.controller.busy:
            return
        text = (
            "仅擦除 APP，保留 Loader 和参数"
            if scope == "app"
            else "整片擦除，包含 Loader、APP、参数和生产身份"
        )
        reply = QMessageBox.warning(
            self,
            "确认擦除",
            f"UID {self.identity['uid']}\n{text}\n擦除后需重新烧录。",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        if reply == QMessageBox.StandardButton.Yes:
            self.launch(dict(profile="jlink_erase", erase_scope=scope, erase_confirmed=True))

    def launch(self, settings: dict, *, automatic=False) -> None:
        """构造任务快照并锁定当前UID，动作前不修改设备。"""
        try:
            settings = dict(settings, bus=self.bus())
            profile = settings["profile"]
            swd = profile.startswith("jlink_")
            if swd:
                settings.update(
                    identity=self.identity,
                    hex=str(self.hex_path or ""),
                    restart_after_flash=True,
                    force_reflash=True,
                    first_fit_node=None,
                )
                if profile == "jlink_flash":
                    if not self.hex_path or not self.identity:
                        raise ValueError("请先选择固件并识别当前板卡")
                    if self.identity["node"] is None:
                        selected = next(
                            (n for n, w in self.initial_choices.items() if w.isChecked()), None
                        )
                        if selected is None:
                            raise ValueError("新板必须选择实际电机类型")
                        settings["first_fit_node"] = selected
                if profile == "jlink_erase" and (
                    not self.identity or not settings.get("erase_confirmed")
                ):
                    raise ValueError("擦除必须识别并明确确认")
            elif profile in (
                "parameters",
                "device_status",
                "clear_fault",
                "motor_speed",
                "motor_calibration",
                "hardware_check",
            ):
                if not self.current_motor:
                    raise ValueError("请先识别当前电机")
                if profile in ("parameters", "clear_fault") and self.current_motor.get("role") != 1:
                    raise ValueError("当前设备未运行 APP")
                settings.update(node=self.node(), expected_uid=self.current_motor["uid"])
            elif profile in ("can_upgrade", "can_bitrate"):
                targets = (
                    [m for m in self.devices.values() if self.upgrade_selected(m["node"])]
                    if self.batch_upgrade.isChecked()
                    else ([self.current_motor] if self.current_motor else [])
                )
                if not targets:
                    raise ValueError("请先扫描并选择在线设备")
                settings["targets"] = [dict(node=m["node"], uid=m["uid"]) for m in targets]
                if profile == "can_upgrade":
                    rounds = int(self.edits["rounds"].text())
                    if rounds < 1:
                        raise ValueError("升级次数须为正整数")
                    settings.update(
                        rounds=rounds,
                        package=str(self.package_path),
                        single_frame_writes=self.single_frame.isChecked(),
                    )
            directory = None
            if not automatic and profile not in ("can_release", "can_shutdown"):
                directory = self.root_log
                if profile == "motor_calibration":
                    directory = (
                        directory
                        / "calibration"
                        / model_directory_name(settings["archive"]["name"])
                        / ("测试-" + settings["archive"]["test_serial"])
                    )
                directory = directory / datetime.now().strftime("%Y%m%d-%H%M%S-%f")
                directory.mkdir(parents=True, exist_ok=False)
            if profile == "jlink_identify":
                self.identity = self.verified_uid = None
                self.select_motor()
            self.controller.start(settings, directory, automatic=automatic)
            if not automatic and profile not in ("can_release", "can_shutdown"):
                self.active_settings = settings
                self.report_dir, self.report_path = directory, None
                self.flow.begin(
                    self.current_motor["uid"]
                    if self.current_motor
                    else (self.identity or {}).get("uid"),
                    profile,
                    kind=settings.get("calibration_kind", 0),
                    full=bool(settings.get("drive")),
                )
                self.progress.setValue(0)
                self.log.clear()
                self.task_status.setText("任务已提交")
                if swd:
                    self.verified_uid = None
                if profile == "motor_speed":
                    self.plot.clear()
                if profile == "hardware_check":
                    self.result_icon.setPixmap(icon("hardware-test").pixmap(20, 20))
                    self.failure.setPlainText("本轮检测进行中，等待结果")
                    self.hardware_table.setRowCount(0)
                self.update_controls()
        except (ValueError, OSError, RuntimeError) as exc:
            if not automatic:
                self.error(str(exc))
            else:
                self.task_status.setText(str(exc))

    def upgrade_selected(self, node: int) -> bool:
        for row in range(self.upgrade_table.rowCount()):
            item = self.upgrade_table.item(row, 0)
            if item.data(Qt.ItemDataRole.UserRole) == node:
                return item.checkState() == Qt.CheckState.Checked
        return False

    def on_event(self, tag: str, value) -> None:
        if tag == "stage":
            self.task_status.setText(str(value))
            self.log.appendPlainText(str(value))
            if self.controller.settings and self.controller.settings["profile"].startswith(
                "jlink_"
            ):
                self.install_label.setText(str(value))
        elif tag == "hardware_phases":
            names = {
                "communication": "通信",
                "static": "静态",
                "resistance": "相电阻",
                "dynamic": "动态",
                "finalize": "关相 / 结果",
            }
            self.phase_label.setText("   ·   ".join(f"{names[k]} {v}" for k, v in value.items()))
            total = sum(v != "本轮不测" for v in value.values())
            self.progress.setValue(
                int(100 * sum(v == "已完成" for v in value.values()) / max(1, total))
            )
        elif tag == "hardware_static":
            self.display_hardware_rows(value)
            completed = value["static_completed"]
            names = ("Ia", "Ib", "Ic", "母线电流", "母线电压", "电机 NTC", "主编码器", "霍尔")
            self.phase_label.setText(
                f"静态检查 {completed}/8 · "
                + (f"正在检测：{names[completed]}" if completed < 8 else "静态项目已完成")
            )
        elif tag == "parameter_progress":
            done, total = value
            self.progress.setValue(int(100 * done / max(1, total)))
            self.task_status.setText(f"参数读取 {done}/{total}")
        elif tag == "progress":
            self.progress.setValue(int(100 * value["attempted"] / max(1, value["planned"])))
            self.task_status.setText(
                f"{value['stage']} · {value['attempted']}/{value['planned']} · 失败 {value['failures']}"
            )
        elif tag == "upgrade_progress":
            number, total, percent = value
            self.progress.setValue(int(100 * ((number - 1) + percent / 100) / max(1, total)))
        elif tag == "axis_status":
            node, text = value
            for row in range(self.upgrade_table.rowCount()):
                if self.upgrade_table.item(row, 0).data(Qt.ItemDataRole.UserRole) == node:
                    self.upgrade_table.item(row, 6).setText(text)
        elif tag == "motor_feedback":
            self.plot.append(value)
            self.speed_label.setText(
                f"目标 {value['target_rad_s']:+.3f} rad/s    实际 {value['actual_rad_s']:+.3f} rad/s"
            )
            if self.current_motor:
                self.current_motor.update(
                    {
                        k: value.get(k)
                        for k in (
                            "bus_voltage_v",
                            "bus_current_a",
                            "motor_temperature_c",
                            "state",
                            "mode",
                            "faults",
                        )
                    },
                    checked_at=time.time(),
                    status_error=None,
                )
                self.render_device()

    def display_hardware_rows(self, report: dict) -> None:
        """同轮逐项快照和终态报告共用数值、判定与颜色投影。"""
        rows(
            self.hardware_table,
            [[r[k] for k in ("item", "measured", "limit", "result")] for r in report_rows(report)],
        )
        for row, result in enumerate(report_rows(report)):
            color = {
                "passed": "#18765b",
                "failed": "#b03b42",
                "pending": "#64748b",
                "excluded": "#64748b",
            }[result["outcome"]]
            for col in range(4):
                self.hardware_table.item(row, col).setForeground(QColor(color))

    def on_completed(self, report: dict, directory: Path | None, automatic: bool) -> None:
        profile = report["profile"]
        if profile in ("can_release", "can_shutdown"):
            self.task_status.setText(
                "CAN 已释放" if report.get("passed") else report.get("fatal", "释放失败")
            )
            self.update_controls()
            if self.next_pending:
                QTimer.singleShot(0, self.next_device)
            return
        if profile == "hardware_check" and "items" not in report:
            report = {
                **empty_report((self.active_settings or {}).get("node", self.node())),
                **report,
            }
        self.flow.record(report, str(directory) if directory else "")
        if profile == "device_status":
            if report.get("device"):
                motor = report["device"]
                self.devices[motor["node"]] = dict(motor)
                if motor["node"] == self.node():
                    self.current_motor = self.devices[self.node()]
                if not automatic:
                    self.select_motor()
            elif self.current_motor:
                if "UID" in report.get("fatal", ""):
                    self.devices.pop(self.node(), None)
                    self.identity = None
                    self.select_motor()
                else:
                    self.current_motor["status_error"] = report.get("fatal", "读取失败")
                    self.task_status.setText(self.current_motor["status_error"])
            self.render_device()
        elif profile == "parameters":
            self.parameters.display(report)
            if report.get("passed") and 1 in report.get("rows", {}):
                from can_parameters import value_text

                self.edits["speed_limit"].setText(value_text(1, report["rows"][1]["raw"]))
        elif profile == "can_scan":
            self.devices.clear()
            self.upgrade_table.setRowCount(0)
            if not report.get("fatal") and not report.get("cancelled"):
                self.devices = {
                    m["node"]: dict(m, checked_at=time.time()) for m in report.get("motors", [])
                }
                self.populate_upgrade_table()
                if self.devices and (len(self.devices) == 1 or self.node() not in self.devices):
                    with QSignalBlocker(self.axis):
                        self.axis.setCurrentIndex(
                            list(FIRST_FIT_AXES).index(next(iter(self.devices)))
                        )
            self.select_motor()
            self.task_status.setText(
                report.get("fatal")
                or (
                    "扫描已取消"
                    if report.get("cancelled")
                    else f"发现 {len(self.devices)} 台在线设备"
                    if self.devices
                    else "CAN已连接，未发现节点3～7"
                )
            )
            if (
                self.devices
                and not self.next_pending
                and not getattr(self.controller, "pending", None)
            ):
                QTimer.singleShot(0, self.read_selected_status)
        elif profile.startswith("jlink_"):
            self.display_install(report)
        elif profile == "hardware_check":
            if report.get("production_report") and directory:
                self.report_path = directory / report["production_report"]
            data = build_report(report, tool_version=VERSION, created_at="")
            self.result_icon.setPixmap(
                icon("pass" if data["verdict"] == "本范围通过" else "fail").pixmap(20, 20)
            )
            self.failure.setPlainText(
                data["scope_name"] + " · " + data["verdict"] + "\n" + failure_summary(report)
            )
            self.display_hardware_rows(report)
            if report.get("phase_states"):
                self.on_event("hardware_phases", report["phase_states"])
        elif profile in ("motor_calibration", "motor_speed"):
            if report.get("curves") and directory:
                self.curve_path = directory / report["curves"]["png"]
            if profile == "motor_calibration":
                text = (
                    "标定完成并保存" if report.get("passed") else report.get("fatal", "标定未完成")
                )
                if report.get("curve_error"):
                    text += "；曲线未生成：" + report["curve_error"]
                self.calibration_result.setText(text)
            if (
                self.current_motor
                and self.current_motor["uid"] == report.get("uid")
                and report.get("final_status")
            ):
                self.current_motor.update(
                    report["final_status"], checked_at=time.time(), status_error=None
                )
        elif profile == "clear_fault":
            if report.get("identity_changed"):
                self.devices.pop(self.node(), None)
                self.identity = None
                self.select_motor()
            if (
                self.current_motor
                and self.current_motor["uid"] == report.get("uid")
                and report.get("after")
            ):
                self.current_motor.update(
                    report["after"], checked_at=time.time(), status_error=None
                )
        elif profile == "can_upgrade":
            for node, result in report.get("motors", {}).items():
                if result.get("passed"):
                    self.devices.pop(int(node), None)
                self.flow.begin(result.get("uid"), "jlink_flash")
            self.select_motor()
        if not automatic:
            self.next_refresh = time.monotonic() + STATUS_REFRESH_S
            success = bool(report.get("passed", report.get("success", report.get("pass", False))))
            text = (
                report.get("fatal")
                or report.get("note")
                or ("操作完成" if success else "操作已结束，查看本次结果")
            )
            if profile == "hardware_check":
                text = self.failure.toPlainText().splitlines()[0]
            elif profile == "can_scan":
                text = self.task_status.text()
            self.task_status.setText(text)
            self.log.appendPlainText(json.dumps(report, ensure_ascii=False, indent=2))
            if directory:
                old = [
                    [self.record_table.item(r, c).text() for c in range(4)]
                    for r in range(self.record_table.rowCount())
                ]
                self.records.insert(0, dict(path=directory, report=report))
                rows(
                    self.record_table,
                    [
                        [
                            datetime.now().strftime("%H:%M:%S"),
                            profile,
                            report.get("node", report.get("identity", {}).get("node", "—")),
                            text,
                        ]
                    ]
                    + old,
                )
        self.update_controls()
        if self.next_pending:
            QTimer.singleShot(0, self.next_device)

    def display_install(self, report: dict) -> None:
        self.verified_uid = None
        identity = report.get("identity")
        if report["profile"] == "jlink_erase":
            self.identity = None
            self.devices.clear()
            self.select_motor()
            self.install_label.setText(
                "擦除完成，请重新识别并烧录"
                if report.get("success")
                else report.get("fatal", "擦除未完成")
            )
            return
        if identity and (report.get("success") or report.get("protected_regions_verified")):
            self.identity = dict(identity)
            if identity.get("node") is None:
                for choice in self.initial_choices.values():
                    choice.setAutoExclusive(False)
                    choice.setChecked(False)
                    choice.setAutoExclusive(True)
                resume = identity.get("resume_node")
                if resume in self.initial_choices:
                    self.initial_choices[resume].setChecked(True)
                self.install_label.setText(
                    "已识别新板，请选择实际电机类型"
                    if not resume
                    else "初装中断恢复：沿用原电机类型"
                )
            else:
                node = identity["node"]
                versions = report.get("can_test", {}).get("device_versions", {})
                status = report.get("device_status", {})
                motor = dict(
                    node=node,
                    uid=identity["uid"],
                    role=status.get("role", 1 if report.get("app_started") else None),
                    app_version=versions.get("app", status.get("app_version")),
                    loader_version=versions.get("loader", status.get("loader_version")),
                    **{
                        k: v
                        for k, v in status.items()
                        if k not in ("node", "uid", "role", "app_version", "loader_version")
                    },
                )
                self.devices[node] = motor
                with QSignalBlocker(self.axis):
                    self.axis.setCurrentIndex(list(FIRST_FIT_AXES).index(node))
                self.select_motor()
                self.install_label.setText("已识别已配板；再次烧录保留参数和标定数据")
                if (
                    report.get("success")
                    and report.get("app_started")
                    and report.get("can_test", {}).get("version_verified")
                ):
                    self.verified_uid = identity["uid"]
                    self.install_label.setText("烧录、启动、版本回读通过。可进入生产检测。")
        if not report.get("success"):
            if not report.get("protected_regions_verified"):
                self.identity = None
                if report["profile"] != "jlink_identify":
                    self.devices.clear()
                self.select_motor()
            self.install_label.setText(report.get("fatal", "烧录未完成，请重新识别"))

    def populate_upgrade_table(self) -> None:
        rows(
            self.upgrade_table,
            [
                [
                    "",
                    f"{m['node']} · {AXES[m['node']]}",
                    m["uid"],
                    "在线 · Loader" if m.get("role") == 2 else "在线 · APP",
                    "v" + version_text(m["app_version"]) if m.get("app_version") else "未读取",
                    "v" + version_text(m["loader_version"])
                    if m.get("loader_version")
                    else "未读取",
                    "待升级",
                ]
                for m in self.devices.values()
            ],
        )
        for row, motor in enumerate(self.devices.values()):
            item = self.upgrade_table.item(row, 0)
            item.setFlags(item.flags() | Qt.ItemFlag.ItemIsUserCheckable)
            item.setCheckState(Qt.CheckState.Checked)
            item.setData(Qt.ItemDataRole.UserRole, motor["node"])

    def next_device(self) -> None:
        if getattr(self.controller, "pending", None) is not None:
            return
        if self.controller.busy:
            if self.controller.automatic:
                self.next_pending = True
                self.controller.stop()
            return
        self.next_pending = False
        if self.flow.devices:
            try:
                directory = self.root_log / "production-sessions"
                directory.mkdir(parents=True, exist_ok=True)
                path = directory / (datetime.now().strftime("%Y%m%d-%H%M%S-%f") + ".json")
                path.write_text(
                    json.dumps(
                        dict(
                            tool_version=VERSION,
                            devices=self.flow.devices,
                            summaries={uid: self.flow.summary(uid) for uid in self.flow.devices},
                        ),
                        ensure_ascii=False,
                        indent=2,
                    ),
                    encoding="utf-8",
                )
            except OSError as exc:
                self.error("归档失败，当前设备已保留：" + str(exc))
                return
        self.flow = ProductionFlow()
        self.devices.clear()
        self.identity = self.verified_uid = None
        self.report_dir = self.report_path = self.curve_path = None
        self.upgrade_table.setRowCount(0)
        self.hardware_table.setRowCount(0)
        self.result_icon.setPixmap(icon("hardware-test").pixmap(20, 20))
        self.failure.setPlainText("下一台尚未开始检测")
        self.calibration_result.setText("等待标定")
        self.phase_label.setText("通信 → 静态 → 相电阻 → 双向动态 → 关相 / 结果")
        self.install_label.setText("下一台待识别；连接 J-Link 后点击识别板卡。")
        self.progress.setValue(0)
        self.edits["specimen"].clear()
        self.edits["serial"].clear()
        self.select_motor()
        self.navigation.setCurrentRow(0)
        self.maintenance.setCurrentIndex(0)
        self.log.clear()
        self.task_status.setText("下一台待识别")
        self.update_controls()

    def show_curves(self) -> None:
        if not self.curve_path:
            return
        dialog = QDialog(self)
        dialog.setWindowTitle("补偿曲线与原始数据")
        dialog.resize(1120, 760)
        layout = QVBoxLayout(dialog)
        actions = QHBoxLayout()
        for filename, title in (
            (self.curve_path.name, "PNG"),
            ("angle-calibration.csv", "补偿CSV"),
            ("hall-raw.csv", "Hall原码"),
            ("calibration-values.csv", "标定参数"),
        ):
            path = self.curve_path.parent / filename
            if path.is_file():
                actions.addWidget(
                    button(title, lambda _=False, p=path: self.open_path(p), icon_name="chart")
                )
        layout.addLayout(actions)
        scroll = QScrollArea()
        image = QLabel()
        image.setPixmap(QPixmap(str(self.curve_path)))
        scroll.setWidget(image)
        layout.addWidget(scroll)
        self.curve_dialog = dialog
        dialog.show()

    def open_path(self, path: Path | None) -> None:
        if path is not None and path.exists():
            QDesktopServices.openUrl(QUrl.fromLocalFile(str(path.resolve())))

    def error(self, text: str) -> None:
        QMessageBox.warning(self, "操作未执行", text)

    def closeEvent(self, event) -> None:
        if self.close_ready:
            event.accept()
            return
        event.ignore()
        self.refresh_timer.stop()
        self.task_status.setText("等待设备操作安全结束…")
        self.controller.shutdown()

    def finish_close(self) -> None:
        self.close_ready = True
        QTimer.singleShot(0, self.close)
