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
from can_parameters import CATALOG, GROUPS, value_text
from five_axis_can_stability import PER_NODE, encode_frames, version_text
from five_axis_jlink import FIRST_FIT_AXES, firmware_versions
from hall_waveform_check import hall_waveform_criterion
from hardware_check import empty_report
from hardware_limits import default_path, read_limits
from production_backend import (
    AXES,
    AXIS_SPEED_LIMITS,
    MODES,
    RATES,
    STATUS_REFRESH_S,
    hardware_check_fields,
    motor_speed_fields,
    parse_raw,
)
from production_controller import ProductionController
from production_flow import ProductionFlow
from production_inspection import (
    INSPECTION_STAGES,
    PRODUCTION_VERSION,
    archive_inspection,
    initial_inspection,
    inspection_numbers,
)
from production_power_check import (
    DEFAULT_POWER_MAX_W,
    DEFAULT_POWER_MIN_W,
    SUGGESTED_OUTPUT_RPM,
    power_check_fields,
)
from production_report import build_report, failure_summary, inspection_overview, report_rows
from PySide6.QtCore import QSignalBlocker, QSize, Qt, QTimer, QUrl
from PySide6.QtGui import QDesktopServices, QIcon, QPixmap
from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDialog,
    QFileDialog,
    QFormLayout,
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
    QSizePolicy,
    QSplitter,
    QStackedWidget,
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)
from qt_calibration_preview import CalibrationPreview
from qt_inspection_table import InspectionTable
from qt_parameter_page import ParameterPage
from qt_production_icons import ICON_SIZE, icon
from qt_production_theme import apply_theme
from qt_speed_plot import SpeedPlot
from qt_waveform_view import WaveformView

VERSION = PRODUCTION_VERSION


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
        self.inspection_binding = None
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
        root.setContentsMargins(18, 12, 18, 10)
        root.setSpacing(8)
        heading = QHBoxLayout()
        heading.addWidget(label("电机生产测试上位机", "title"))
        heading.addWidget(label(f"PRODUCTION CONSOLE  /  v{VERSION}", "subtitle"))
        heading.addStretch()
        self.next_button = button("归档并开始下一台", self.next_device, icon_name="next-device")
        heading.addWidget(self.next_button)
        root.addLayout(heading)
        root.addWidget(self.build_device_header())
        self.flow_label = label(self.flow.summary(None, integrated=True), "subtitle", True)
        root.addWidget(self.flow_label)
        body = QHBoxLayout()
        self.navigation = QListWidget()
        self.navigation.setObjectName("navigation")
        self.navigation.setFixedWidth(150)
        self.navigation.setIconSize(QSize(22, 22))
        for name, text in (
            ("maintenance", "设备维护"),
            ("hardware-test", "生产检测"),
            ("chart", "电机运动"),
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
        self.build_motion()
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
        self.log_button = button("详细日志", self.toggle_log, icon_name="records")
        footer.addWidget(self.log_button)
        self.stop_button = button("停止", self.controller.stop, icon_name="stop")
        self.stop_button.setObjectName("stop")
        footer.addWidget(self.stop_button)
        root.addLayout(footer)
        self.navigation.setCurrentRow(0)

    def build_device_header(self) -> QWidget:
        header = QWidget()
        header.setObjectName("deviceHeader")
        layout = QVBoxLayout(header)
        layout.setContentsMargins(10, 6, 10, 6)
        layout.setSpacing(4)
        connection = QHBoxLayout()
        connection.setSpacing(9)
        self.axis = combo(
            [f"{role} · {model} · 节点 {node}" for node, (role, model, _) in FIRST_FIT_AXES.items()]
        )
        self.axis.setCurrentIndex(3)
        self.axis.setAccessibleName("当前电机")
        self.axis.currentIndexChanged.connect(self.select_motor)
        connection.addWidget(self.axis)
        self.connection_icon = QLabel()
        self.connection_icon.setFixedSize(16, 16)
        connection.addWidget(self.connection_icon)
        self.connection_label = label("CAN 未连接", "subtitle")
        connection.addWidget(self.connection_label)
        connection.addSpacing(5)
        self.version_label = label("APP 固件：—", "deviceVersion")
        self.loader_label = label("Loader 固件：—", "deviceVersion")
        for version in (self.version_label, self.loader_label):
            version.setToolTip("当前板端版本：来自 CAN 回读或本次 SWD 安装核验")
            connection.addWidget(version)
        connection.addStretch()
        self.build_connection_dialog()
        self.connection_settings = button(
            "连接设置", self.connection_dialog.show, icon_name="parameters"
        )
        connection.addWidget(self.connection_settings)
        layout.addLayout(connection)

        device = QHBoxLayout()
        device.setSpacing(8)
        self.status_label = label("运行状态：未读取", "statusValue")
        self.mode_label = label("控制模式：未读取", "subtitle")
        device.addWidget(self.status_label)
        device.addWidget(self.mode_label)
        self.fault_icon = QLabel()
        self.fault_icon.setFixedSize(16, 16)
        device.addWidget(self.fault_icon)
        self.fault_label = label("故障信息：未读取", "statusValue", True)
        self.fault_label.setMaximumWidth(230)
        device.addWidget(self.fault_label)
        self.fault_button = button(
            "清故障", lambda: self.launch(dict(profile="clear_fault")), icon_name="clear-fault"
        )
        self.fault_button.setObjectName("rowAction")
        device.addWidget(self.fault_button)
        device.addStretch()
        self.voltage = label("— V", "metric")
        self.current = label("— A", "metric")
        self.temperature = label("— °C", "metric")
        for widget, caption in (
            (self.voltage, "母线电压"),
            (self.current, "母线电流"),
            (self.temperature, "电机温度"),
        ):
            device.addWidget(label(caption, "subtitle"))
            device.addWidget(widget)
            device.addSpacing(6)
        layout.addLayout(device)
        return header

    def build_connection_dialog(self) -> None:
        self.connection_dialog = QDialog(self)
        self.connection_dialog.setWindowTitle("连接设置")
        layout = QVBoxLayout(self.connection_dialog)
        self.uid_label = label("尚未识别设备", "subtitle", True)
        layout.addWidget(self.uid_label)
        self.channel = combo(("1", "2"))
        self.nominal = combo(("500K", "1M"), "1M")
        self.data_rate = combo(RATES, "5M")
        rates = QFormLayout()
        for caption, widget in (
            ("通道", self.channel),
            ("仲裁段", self.nominal),
            ("数据段", self.data_rate),
        ):
            rates.addRow(caption, widget)
            widget.currentTextChanged.connect(self.connection_changed)
        layout.addLayout(rates)
        actions = QHBoxLayout()
        self.scan_button = button(
            "连接 / 扫描", lambda: self.launch(dict(profile="can_scan")), icon_name="scan"
        )
        self.disconnect_button = button(
            "断开", lambda: self.launch(dict(profile="can_release")), icon_name="disconnect"
        )
        actions.addWidget(self.scan_button)
        actions.addWidget(self.disconnect_button)
        actions.addStretch()
        layout.addLayout(actions)
        self.auto_refresh = QCheckBox("自动刷新 · 1 秒")
        self.auto_refresh.setChecked(True)
        layout.addWidget(self.auto_refresh)
        status_actions = QHBoxLayout()
        self.last_read = label("等待设备连接", "subtitle")
        status_actions.addWidget(self.last_read, 1)
        self.status_button = button(
            "刷新", lambda: self.launch(dict(profile="device_status")), icon_name="refresh"
        )
        status_actions.addWidget(self.status_button)
        layout.addLayout(status_actions)

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
                "按从左到右的顺序操作；烧录完成后自动启动并核验版本。",
                "subtitle",
            )
        )
        self.install_label = label("连接 J-Link 与板卡电源，然后识别当前板卡。", wrap=True)
        layout.addWidget(self.install_label)
        content = QHBoxLayout()
        content.setSpacing(8)
        self.identify_box = QGroupBox("①  识别板卡")
        self.identify_box.setObjectName("installStep")
        identify = QVBoxLayout(self.identify_box)
        identify.addWidget(label("选择目标 J-Link，再读取芯片身份。", "subtitle", True))
        identify.addWidget(label("J-Link 探针"))
        self.probe_selector = QComboBox()
        self.probe_selector.addItem("单探针自动选择", None)
        self.probe_selector.currentIndexChanged.connect(self.select_probe)
        identify.addWidget(self.probe_selector)
        self.add_action(
            identify,
            "扫描探针",
            lambda: self.launch(dict(profile="jlink_probes")),
            icon_name="scan",
        )
        identify.addStretch()
        self.identify_button = self.add_action(
            identify,
            "识别板卡",
            lambda: self.launch(dict(profile="jlink_identify")),
            icon_name="scan",
        )
        content.addWidget(self.identify_box, 1)
        content.addWidget(label("→", "stepArrow"))
        self.initial_box = QGroupBox("②  确认电机类型")
        self.initial_box.setObjectName("installStep")
        choices = QVBoxLayout(self.initial_box)
        self.initial_hint = label("新板烧录前选择实际电机类型，再识别板卡。", "subtitle", True)
        choices.addWidget(self.initial_hint)
        self.initial_choices = {}
        for node, (role, model, _) in FIRST_FIT_AXES.items():
            radio = QRadioButton(f"{role}  /  {model}  /  节点 {node}")
            radio.toggled.connect(self.update_controls)
            choices.addWidget(radio)
            self.initial_choices[node] = radio
        content.addWidget(self.initial_box, 1)
        content.addWidget(label("→", "stepArrow"))
        self.flash_box = QGroupBox("③  烧录并校验")
        self.flash_box.setObjectName("installStep")
        box = QVBoxLayout(self.flash_box)
        box.addWidget(label("本次烧录包", "subtitle"))
        self.firmware_label = label("固件版本未确认", "packageVersion", True)
        box.addWidget(self.firmware_label)
        box.addWidget(label("AT32F456  ·  CAN FD 1M / 5M", "subtitle"))
        self.hex_path = self.resource / "firmware/five-axis.hex"
        if not self.hex_path.is_file():
            self.hex_path = None
        self.hex_label = label("内置统一整包" if self.hex_path else "尚未选择本批 HEX", wrap=True)
        box.addWidget(self.hex_label)
        self.update_firmware_version()
        self.add_action(box, "选择本批 HEX", self.choose_hex, icon_name="folder")
        self.verify_can = QCheckBox("烧录后执行 CAN 验收")
        self.verify_can.setToolTip("仅接 J-Link 时取消勾选；仍会通过 SWD 核验安装版本。")
        self.verify_can.setChecked(True)
        box.addWidget(self.verify_can)
        box.addStretch()
        self.flash_button = self.add_action(
            box,
            "烧录整包并校验",
            lambda: self.launch(dict(profile="jlink_flash")),
            True,
            icon_name="flash",
        )
        content.addWidget(self.flash_box, 1)
        layout.addLayout(content)
        self.swd_version_label = label("板端 SWD 版本：尚未确认", "subtitle", True)
        layout.addWidget(self.swd_version_label)
        actions = QHBoxLayout()
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
        self.package_path = self.resource / "can-app"
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
        erase_box = QGroupBox("Flash 擦除 · J-Link / SWD")
        erase_box.setMaximumWidth(820)
        erase_layout = QVBoxLayout(erase_box)
        identify_row = QHBoxLayout()
        self.erase_identify = self.add_action(
            identify_row,
            "识别擦除目标（J-Link）",
            lambda: self.launch(dict(profile="jlink_identify")),
            icon_name="scan",
        )
        self.erase_status = label("", "subtitle", True)
        identify_row.addWidget(self.erase_status, 1)
        erase_layout.addLayout(identify_row)
        erase_layout.addWidget(label("操作前保存备份；擦除后需重新烧录。", wrap=True))
        erase_actions = QHBoxLayout()
        self.erase_app = self.add_action(
            erase_actions,
            "擦除 APP · 保留参数和 Loader",
            lambda: self.erase("app"),
            icon_name="warning",
        )
        self.erase_all = self.add_action(
            erase_actions, "整片擦除 · 含参数和身份", lambda: self.erase("all"), icon_name="warning"
        )
        erase_actions.addStretch()
        erase_layout.addLayout(erase_actions)
        layout.addWidget(erase_box)
        layout.addStretch()

    def build_hardware(self) -> None:
        page = QWidget()
        page.setObjectName("inspectionPage")
        self.stack.addWidget(page)
        layout = QVBoxLayout(page)
        layout.setContentsMargins(14, 10, 14, 10)
        layout.setSpacing(6)
        title = QHBoxLayout()
        title.addWidget(label("生产检测", "sectionTitle"), 1)
        self.build_inspection_settings()
        self.inspection_settings_button = button(
            "检测设置", self.inspection_settings.show, icon_name="parameters"
        )
        title.addWidget(self.inspection_settings_button)
        layout.addLayout(title)
        execution = QHBoxLayout()
        execution.setSpacing(6)
        self.number_fields = QWidget()
        number_row = QHBoxLayout(self.number_fields)
        number_row.setContentsMargins(0, 0, 0, 0)
        number_row.setSpacing(6)
        for caption, key in (("电机序列号", "motor_serial"), ("测试需求号", "test_requirement")):
            number_row.addWidget(label(caption))
            edit = self.edits[key] = QLineEdit()
            edit.setAccessibleName(caption)
            edit.setMaxLength(40)
            edit.setFixedWidth(112)
            edit.textChanged.connect(self.update_controls)
            number_row.addWidget(edit)
        execution.addWidget(self.number_fields)
        self.bound_numbers = label("", "statusValue", True)
        self.bound_numbers.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Preferred)
        self.bound_numbers.hide()
        execution.addWidget(self.bound_numbers, 1)
        self.confirm_numbers_button = button(
            "确认编号", self.confirm_inspection_numbers, icon_name="pass"
        )
        execution.addWidget(self.confirm_numbers_button)
        execution.addStretch(0)
        self.hardware_start = self.add_action(
            execution, "开始检测", self.start_hardware, True, icon_name="start"
        )
        self.hardware_start.setToolTip("整轮检测包含标定和一次高速检测，高速项使用可连续转动的工装")
        self.inspection_stop = button("停止", self.controller.stop, icon_name="stop")
        self.inspection_stop.setAccessibleName("停止检测")
        self.inspection_stop.setObjectName("stop")
        execution.addWidget(self.inspection_stop)
        layout.addLayout(execution)

        result_title = QHBoxLayout()
        result_title.addWidget(label("本轮检测结果", "resultTitle"))
        self.result_verdict = label("待开始", "resultVerdict")
        result_title.addWidget(self.result_verdict)
        self.result_counts = label("", "subtitle")
        result_title.addWidget(self.result_counts, 1)
        self.result_detail_button = button(
            "结果详情", self.toggle_result_details, icon_name="records"
        )
        self.result_detail_button.hide()
        result_title.addWidget(self.result_detail_button)
        self.waveform_overview_button = button(
            "波形总览",
            lambda: self.show_inspection_curve("overview", "生产检测波形总览"),
            icon_name="chart",
        )
        result_title.addWidget(self.waveform_overview_button)
        self.open_report_button = button(
            "查看报告", lambda: self.open_path(self.report_path), icon_name="records"
        )
        result_title.addWidget(self.open_report_button)
        layout.addLayout(result_title)
        self.phase_label = label("等待确认编号", "inspectionProgress")
        self.phase_label.setTextFormat(Qt.TextFormat.PlainText)
        self.phase_label.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Fixed)
        layout.addWidget(self.phase_label)
        self.failure = QPlainTextEdit()
        self.failure.setReadOnly(True)
        self.failure.setMaximumHeight(100)
        self.failure.hide()
        layout.addWidget(self.failure)
        self.hardware_table = InspectionTable(self.show_inspection_curve)
        self.inspection_plot = self.hardware_table.plot
        layout.addWidget(self.hardware_table, 1)
        self.reset_result_view()

    def build_inspection_settings(self) -> None:
        self.inspection_settings = QDialog(self)
        self.inspection_settings.setWindowTitle("生产检测设置")
        panel = QVBoxLayout(self.inspection_settings)
        self.add_action(
            panel,
            "打开检测阈值配置",
            lambda: self.open_path(default_path()),
            icon_name="parameters",
        )
        self.add_fields(
            panel,
            [
                ("ambient", "室温 °C · 可空", "20"),
            ],
        )
        self.add_fields(
            panel,
            [
                ("power_rpm", "高速目标 · 输出轴 rpm", ""),
                ("power_min_w", "母线输入功率下限 W", ""),
                ("power_max_w", "母线输入功率上限 W", ""),
            ],
        )
        self.edits["power_rpm"].setPlaceholderText(f"预设 {SUGGESTED_OUTPUT_RPM[self.node()]} rpm")
        self.edits["power_min_w"].setPlaceholderText(f"默认 {DEFAULT_POWER_MIN_W:g} W")
        self.edits["power_max_w"].setPlaceholderText(f"默认 {DEFAULT_POWER_MAX_W:g} W")

    def build_calibration(self) -> None:
        layout = self.tab_page(self.maintenance, "calibration", "标定维护")
        columns = QHBoxLayout()
        columns.setSpacing(16)
        settings = QGroupBox("标定信息")
        settings.setMinimumWidth(330)
        settings.setMaximumWidth(380)
        controls = QVBoxLayout(settings)
        controls.setSpacing(14)
        controls.addWidget(label("电机标定", "sectionTitle"))
        self.add_fields(
            controls, [("specimen", "样机序列号", "0001"), ("serial", "测试序列号", "0001")]
        )
        self.add_action(
            controls,
            "编码器 / Hall 角度标定并保存",
            lambda: self.start_calibration(1),
            True,
            icon_name="calibration",
        )
        self.add_action(
            controls,
            "电角零位标定并保存",
            lambda: self.start_calibration(2),
            icon_name="calibration",
        )
        self.add_action(
            controls,
            "读取已保存标定曲线",
            lambda: self.launch(
                dict(profile="calibration_readback", probe_serial=self.probe_selector.currentData())
            ),
            icon_name="refresh",
        )
        controls.addWidget(
            label("完成后回读 Flash 保存状态；减速轴沿用固件的联合标定流程。", "subtitle", True)
        )
        self.calibration_result = label("等待标定", wrap=True)
        controls.addWidget(self.calibration_result)
        controls.addStretch()
        columns.addWidget(settings)
        chart = QGroupBox("补偿曲线")
        chart_layout = QVBoxLayout(chart)
        self.curve_button = button("查看原图 / 原码", self.show_curves, icon_name="chart")
        chart_actions = QHBoxLayout()
        chart_actions.addStretch()
        chart_actions.addWidget(self.curve_button)
        chart_layout.addLayout(chart_actions)
        self.calibration_preview = CalibrationPreview()
        chart_layout.addWidget(self.calibration_preview, 1)
        columns.addWidget(chart, 1)
        layout.addLayout(columns, 1)

    def build_motion(self) -> None:
        page = QWidget()
        self.stack.addWidget(page)
        layout = QVBoxLayout(page)
        layout.setContentsMargins(20, 16, 20, 16)
        layout.addWidget(label("电机运动", "sectionTitle"))
        columns = QHBoxLayout()
        columns.setSpacing(16)
        controls = QGroupBox("速度模式")
        settings = QVBoxLayout(controls)
        settings.setSpacing(14)
        self.add_fields(
            settings, [("speed", "目标速度 rad/s", "3"), ("speed_limit", "电机轴上限 rad/s", "300")]
        )
        self.add_action(settings, "使能速度控制", self.start_speed, True, icon_name="start")
        self.speed_update = button("更新速度目标", self.update_speed, icon_name="refresh")
        settings.addWidget(self.speed_update)
        self.speed_stop = button("停止", self.controller.stop, icon_name="stop")
        settings.addWidget(self.speed_stop)
        self.speed_label = label("等待速度反馈", "subtitle", True)
        settings.addWidget(self.speed_label)
        parameters = QGroupBox("速度环参数 · 设备读回")
        parameter_layout = QVBoxLayout(parameters)
        self.add_action(
            parameter_layout,
            "读取速度环参数",
            lambda: self.parameter_action("speed", False),
            icon_name="refresh",
        )
        self.speed_parameter_status = label("尚未读取", "subtitle", True)
        parameter_layout.addWidget(self.speed_parameter_status)
        fields = QFormLayout()
        fields.setVerticalSpacing(3)
        self.speed_parameter_values = {}
        for identifier, title in GROUPS["speed"][1]:
            value = label("—")
            fields.addRow(title, value)
            self.speed_parameter_values[identifier] = value
        parameter_layout.addLayout(fields)
        settings.addWidget(parameters)
        settings.addStretch()
        control_scroll = QScrollArea()
        control_scroll.setWidgetResizable(True)
        control_scroll.setMinimumWidth(310)
        control_scroll.setMaximumWidth(370)
        control_scroll.setWidget(controls)
        columns.addWidget(control_scroll)
        chart = QGroupBox("实时曲线")
        chart.setMinimumWidth(360)
        chart_layout = QVBoxLayout(chart)
        self.plot = SpeedPlot()
        chart_layout.addWidget(self.plot)
        columns.addWidget(chart, 1)
        layout.addLayout(columns, 1)

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
        # 滚动区只采用当前页的最小尺寸，维护页的宽度不挤出检测表。
        self.stack.setMinimumSize(
            self.stack.currentWidget().minimumSizeHint().expandedTo(QSize(1, 1))
        )
        self.progress.setVisible(index != 1)
        self.stop_button.setVisible(
            index != 1 or self.stop_button.isEnabled() and not self.inspection_stop.isEnabled()
        )
        self.set_log_visible(index != 1)

    def set_log_visible(self, visible: bool) -> None:
        self.log.setVisible(visible)
        self.log_button.setText("收起日志" if visible else "详细日志")
        if visible and self.splitter.sizes()[1] < 100:
            self.splitter.setSizes([max(240, self.splitter.height() - 140), 140])

    def toggle_log(self) -> None:
        self.set_log_visible(self.log.isHidden())

    def connection_changed(self, *_args) -> None:
        self.devices.clear()
        self.identity = self.verified_uid = None
        self.swd_version_label.setText("板端 SWD 版本：尚未确认")
        self.select_motor()

    def select_motor(self, *_args) -> None:
        old = self.current_motor
        self.current_motor = self.devices.get(self.node())
        if not old or not self.current_motor or old["uid"] != self.current_motor["uid"]:
            self.reset_inspection()
            for key in ("power_rpm", "power_min_w", "power_max_w"):
                self.edits[key].clear()
            self.edits["power_rpm"].setPlaceholderText(
                f"预设 {SUGGESTED_OUTPUT_RPM[self.node()]} rpm"
            )
            self.parameters.reset(
                (self.node(), self.current_motor["uid"]) if self.current_motor else None
            )
            self.plot.clear()
            self.curve_path = None
            self.calibration_preview.display(None)
            self.calibration_result.setText("等待标定")
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
            icon("connect" if self.controller.connected else "disconnect").pixmap(16, 16)
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
        for widget, key in (
            (self.version_label, "app_version"),
            (self.loader_label, "loader_version"),
        ):
            widget.setProperty("unavailable", not motor.get(key))
            widget.style().unpolish(widget)
            widget.style().polish(widget)
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
        self.status_label.setStyleSheet(
            "color:#b42336;" if error or stale or state_code == 8 else ""
        )
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
        self.fault_label.setToolTip(
            f"{describe_faults(faults)}（0x{faults:08X}）{historical}"
            if faults
            else self.fault_label.text()
        )
        self.fault_icon.setPixmap(
            icon(
                "warning" if faults is None or stale or error else "fail" if faults else "pass"
            ).pixmap(16, 16)
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
        self.flow_label.setText(self.flow.summary(motor.get("uid"), integrated=True))

    def update_controls(self, *_args) -> None:
        # 搭建编号区时可能已经触发输入信号，等待其余控件就绪。
        if not hasattr(self, "erase_status") or not hasattr(self, "parameters"):
            return
        busy = self.controller.busy
        automatic = busy and self.controller.automatic
        for widget in self.action_buttons:
            widget.setEnabled(not busy or automatic)
        for widget in (self.axis, self.channel, self.nominal, self.data_rate):
            widget.setEnabled(not busy)
        self.scan_button.setEnabled(not busy or automatic)
        self.connection_settings.setEnabled(not busy or automatic)
        self.inspection_settings_button.setEnabled(not busy or automatic)
        self.navigation.setEnabled(not busy or automatic)
        for key, edit in self.edits.items():
            edit.setEnabled(
                not busy
                or automatic
                or (key == "speed" and self.controller.settings.get("profile") == "motor_speed")
            )
        for tabs in (self.maintenance, self.communication_tabs):
            for index in range(tabs.count()):
                tabs.setTabEnabled(index, not busy or automatic or index == tabs.currentIndex())
        blank = self.identity is not None and self.identity.get("node") is None
        configured = self.identity is not None and self.identity.get("node") is not None
        self.initial_hint.setText(
            "已配板：沿用以下配置，保留参数和标定数据。"
            if configured
            else "初装恢复：沿用原电机类型。"
            if self.identity and "resume_node" in self.identity
            else "新板必选：按实际电机和轴用途选择，识别后才能烧录。"
        )
        for choice in self.initial_choices.values():
            choice.setEnabled(
                not busy
                and not configured
                and (not self.identity or "resume_node" not in self.identity)
            )
        selected = any(choice.isChecked() for choice in self.initial_choices.values())
        self.flash_button.setEnabled(
            not busy
            and self.hex_path is not None
            and self.identity is not None
            and (not blank or selected)
        )
        self.probe_selector.setEnabled(not busy)
        self.verify_can.setEnabled(not busy)
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
        self.render_speed_parameters()
        self.open_log_button.setEnabled(self.report_dir is not None)
        self.open_report_button.setEnabled(self.report_path is not None)
        self.waveform_overview_button.setEnabled("overview" in self.inspection_curve_paths)
        available = (not busy or automatic) and bool(
            self.current_motor and self.current_motor.get("role") == 1
        )
        confirmed = self.inspection_binding is not None
        self.confirm_numbers_button.setEnabled(
            available
            and (
                confirmed
                or all(
                    self.edits[key].text().strip() for key in ("motor_serial", "test_requirement")
                )
            )
        )
        self.confirm_numbers_button.setText("修改编号" if confirmed else "确认编号")
        self.number_fields.setVisible(not confirmed)
        self.bound_numbers.setVisible(confirmed)
        for key in ("motor_serial", "test_requirement"):
            self.edits[key].setReadOnly(confirmed)
        self.hardware_start.setEnabled(available and confirmed)
        pending = getattr(self.controller, "pending", None)
        self.inspection_stop.setEnabled(
            busy
            and (
                self.controller.settings.get("profile") == "production_inspection"
                or pending is not None
                and pending[0]["profile"] == "production_inspection"
            )
        )
        self.stop_button.setVisible(
            self.navigation.currentRow() != 1
            or self.stop_button.isEnabled()
            and not self.inspection_stop.isEnabled()
        )
        if self.result_verdict.text() == "待开始":
            self.set_inspection_progress("等待开始检测" if confirmed else "等待确认编号")
        self.curve_button.setEnabled(self.curve_path is not None)
        self.erase_all.setEnabled(not busy and self.identity is not None)
        self.erase_app.setEnabled(
            not busy and self.identity is not None and self.identity.get("node") is not None
        )
        self.erase_status.setText(
            "当前任务进行中，结束后可识别或擦除。"
            if busy
            else "请在整包烧录页选择 J-Link 并识别目标。CAN连接不能代替SWD识别。"
            if not self.identity
            else f"已识别 UID {self.identity['uid']} · "
            + (
                "空白板：可整片擦除，无可擦除的APP配置。"
                if self.identity.get("node") is None
                else f"节点 {self.identity['node']}，可选择擦除范围。"
            )
        )
        self.erase_app.setToolTip(self.erase_status.text())
        self.erase_all.setToolTip(self.erase_status.text())
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
            self.update_firmware_version()
            self.update_controls()

    def update_firmware_version(self) -> None:
        """选择 HEX 后重新读取版本；失败清除旧文本，不访问设备。"""
        if self.hex_path is None:
            self.firmware_label.setText("固件版本未确认")
            self.firmware_label.setToolTip("请选择本批合并 HEX")
            return
        try:
            app, loader = firmware_versions(self.hex_path, self.resource)
            self.firmware_label.setText(
                f"APP {version_text(app)}   +   Loader {version_text(loader)}"
            )
            self.firmware_label.setToolTip(str(self.hex_path))
        except (OSError, ValueError, KeyError, TypeError) as exc:
            self.firmware_label.setText("固件版本未确认")
            self.firmware_label.setToolTip(str(exc))

    def select_probe(self) -> None:
        """更换探针后丢弃原板身份与版本，下一次写入必须重新识别。"""
        self.identity = self.verified_uid = None
        self.devices.clear()
        self.swd_version_label.setText("板端 SWD 版本：尚未确认")
        self.install_label.setText("探针已更换，请重新识别板卡。")
        self.select_motor()

    def display_probes(self, serials: list[int]) -> None:
        """保留仍存在的选择；多探针不猜测目标。"""
        previous = self.probe_selector.currentData()
        selected = previous if previous in serials else serials[0] if len(serials) == 1 else None
        with QSignalBlocker(self.probe_selector):
            self.probe_selector.clear()
            self.probe_selector.addItem("请选择探针" if serials else "未发现探针", None)
            for serial in serials:
                self.probe_selector.addItem(f"J-Link · {serial}", serial)
            self.probe_selector.setCurrentIndex(self.probe_selector.findData(selected))
        if selected != previous or (self.identity and self.identity["serial"] != selected):
            self.select_probe()

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
            if (
                not self.inspection_binding
                or not self.current_motor
                or self.inspection_binding["uid"] != self.current_motor["uid"]
                or self.inspection_binding["bus"] != self.bus()
            ):
                raise ValueError("请先确认当前电机的序列号和测试需求号")
            read_limits()
            settings = hardware_check_fields({"ambient": self.edits["ambient"].text()}, False)
            self.launch(
                dict(
                    profile="production_inspection",
                    numbers=self.inspection_binding["numbers"].copy(),
                    probe_serial=self.probe_selector.currentData(),
                    power_check=power_check_fields(
                        self.node(),
                        rpm=self.edits["power_rpm"].text(),
                        min_w=self.edits["power_min_w"].text(),
                        max_w=self.edits["power_max_w"].text(),
                    ),
                    **settings,
                )
            )
        except ValueError as exc:
            self.error(str(exc))

    def confirm_inspection_numbers(self) -> None:
        """确认本轮编号并绑定当前设备及CAN设置。"""
        if self.inspection_binding:
            self.reset_inspection()
        else:
            try:
                if not self.current_motor or self.current_motor.get("role") != 1:
                    raise ValueError("请先扫描并选择运行 APP 的电机")
                numbers = inspection_numbers(
                    self.edits["motor_serial"].text(), self.edits["test_requirement"].text()
                )
                self.inspection_binding = dict(
                    uid=self.current_motor["uid"], bus=self.bus(), numbers=numbers
                )
                summary = f"电机序列号 {numbers['motor_serial']} · 测试需求号 {numbers['test_requirement']}"
                self.bound_numbers.setText(summary)
                self.bound_numbers.setToolTip(summary)
            except ValueError as exc:
                self.error(str(exc))
        self.update_controls()

    def reset_inspection(self) -> None:
        self.inspection_binding = None
        if (self.active_settings or {}).get("profile") == "production_inspection":
            self.report_dir = None
        self.reset_result_view()

    def reset_result_view(self, *, running: bool = False) -> None:
        self.report_path = None
        self.inspection_curve_paths: dict[str, Path] = {}
        self.waveform_overview_button.setEnabled(False)
        self.waveform_overview_button.setToolTip("检测结束后查看本轮波形")
        self.hardware_table.clear_angles()
        self.result_counts.setText(self.hardware_table.display(inspection_overview({})))
        self.set_result_verdict("检测中" if running else "待开始", "running" if running else "")
        self.set_inspection_progress("任务已提交" if running else "等待确认编号")
        self.failure.clear()
        self.failure.hide()
        self.result_detail_button.hide()
        self.result_detail_button.setText("结果详情")

    def set_result_verdict(self, text: str, outcome: str) -> None:
        self.result_verdict.setText(text)
        self.result_verdict.setProperty("outcome", outcome)
        self.result_verdict.style().unpolish(self.result_verdict)
        self.result_verdict.style().polish(self.result_verdict)

    def set_inspection_progress(self, text: str) -> None:
        if self.phase_label.text() != text:
            self.phase_label.setText(text)
            self.phase_label.setToolTip(text)

    def toggle_result_details(self) -> None:
        visible = self.failure.isHidden()
        self.failure.setVisible(visible)
        self.result_detail_button.setText("收起详情" if visible else "结果详情")

    def start_calibration(self, kind: int) -> None:
        try:
            archive = calibration_archive(
                ARCHIVE_TYPES[self.node()],
                self.edits["specimen"].text(),
                self.edits["serial"].text(),
            )
            self.launch(
                dict(
                    profile="motor_calibration",
                    calibration_kind=kind,
                    archive=archive,
                    probe_serial=self.probe_selector.currentData(),
                )
            )
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

    def render_speed_parameters(self) -> None:
        snapshot = self.parameters.snapshots.get("speed")
        self.speed_parameter_status.setText(
            f"已读回 · 修订 {snapshot['revision']}" if snapshot else "尚未读取或已失效"
        )
        for identifier, label_widget in self.speed_parameter_values.items():
            row = snapshot["rows"].get(identifier) if snapshot else None
            text = "—"
            if row:
                text = value_text(identifier, row["raw"])
                unit = CATALOG[identifier]["unit"]
                if unit == "bool":
                    text = "启用" if row["raw"] else "关闭"
                elif unit not in ("1", ""):
                    text += f" {unit}"
            label_widget.setText(text)

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
                    probe_serial=self.probe_selector.currentData(),
                    hex=str(self.hex_path or ""),
                    restart_after_flash=True,
                    force_reflash=True,
                    first_fit_node=None,
                )
                if not self.verify_can.isChecked():
                    settings["bus"] = None
                if profile == "jlink_identify":
                    settings["identity"] = None
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
                "calibration_readback",
                "hardware_check",
                "production_inspection",
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
                elif profile == "production_inspection":
                    numbers = inspection_numbers(**settings["numbers"])
                    settings["numbers"] = numbers
                    directory = (
                        directory
                        / "production"
                        / numbers["motor_serial"]
                        / numbers["test_requirement"]
                    )
                directory = directory / datetime.now().strftime("%Y%m%d-%H%M%S-%f")
                directory.mkdir(parents=True, exist_ok=False)
            if profile == "jlink_identify":
                self.identity = self.verified_uid = None
                self.select_motor()
            self.controller.start(settings, directory, automatic=automatic)
            if profile in ("jlink_identify", "jlink_flash", "jlink_erase"):
                self.swd_version_label.setText("板端 SWD 版本：正在操作，等待本次确认")
                if self.current_motor:
                    self.current_motor.pop("app_version", None)
                    self.current_motor.pop("loader_version", None)
            if not automatic and profile not in ("can_release", "can_shutdown"):
                self.active_settings = settings
                self.report_dir = directory
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
                if profile in ("motor_calibration", "calibration_readback"):
                    self.curve_path = None
                    self.calibration_preview.display(None)
                    self.calibration_result.setText(
                        "正在只读回读保存记录"
                        if profile == "calibration_readback"
                        else "标定进行中，等待保存及曲线导出"
                    )
                if profile in ("hardware_check", "production_inspection"):
                    self.reset_result_view(running=True)
                if profile == "production_inspection":
                    self.curve_path = None
                    self.calibration_preview.display(None)
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
        elif tag == "inspection_phases":
            completed = sum(
                status in ("已确认", "通过", "待复核", "已记录", "已归档")
                for status in value.values()
            )
            self.progress.setValue(round(100 * completed / len(INSPECTION_STAGES)))
            current = next(
                (
                    (key, value[key])
                    for key in reversed(INSPECTION_STAGES)
                    if value.get(key, "未测") != "未测"
                ),
                None,
            )
            if current:
                key, status = current
                self.set_inspection_progress(
                    f"第 {list(INSPECTION_STAGES).index(key) + 1}/{len(INSPECTION_STAGES)} 步 · "
                    f"{INSPECTION_STAGES[key]} · {status}"
                )
        elif tag == "inspection_results":
            self.result_counts.setText(self.hardware_table.display(value))
        elif tag == "inspection_angle":
            self.hardware_table.append_angle(value)
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
        integrated = report.get("profile") == "production_inspection"
        results = inspection_overview(report) if integrated else report_rows(report)
        self.result_counts.setText(
            self.hardware_table.display(
                results,
                available_curves=frozenset(self.inspection_curve_paths)
                if integrated
                else frozenset(),
            )
        )

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
        if profile == "production_inspection" and "phase_states" not in report:
            settings = self.active_settings or {}
            if settings.get("profile") == profile and directory is not None:
                report = {**initial_inspection(settings), **report}
                archive_inspection(report, directory)
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
            if "probes" in report:
                self.display_probes(report["probes"])
            if profile != "jlink_probes":
                self.display_install(report)
            else:
                self.install_label.setText(
                    report.get("fatal") or "扫描完成，请选择探针并识别板卡。"
                )
        elif profile in ("hardware_check", "production_inspection"):
            self.report_path = (
                directory / report["production_report"]
                if report.get("production_report") and directory
                else None
            )
            data = build_report(report, tool_version=VERSION, created_at="")
            verdict = data["verdict"]
            outcome = {"本范围通过": "passed", "本轮通过": "passed", "待复核": "review"}.get(
                verdict, "" if report.get("cancelled") else "failed"
            )
            self.set_result_verdict(verdict, outcome)
            self.failure.setPlainText(
                data["scope_name"] + " · " + verdict + "\n" + failure_summary(report)
            )
            self.result_detail_button.show()
            if report.get("phase_states"):
                self.on_event(
                    "inspection_phases"
                    if profile == "production_inspection"
                    else "hardware_phases",
                    report["phase_states"],
                )
            if profile == "production_inspection":
                self.hardware_table.set_curve_expanded(False)
                calibration = report.get("calibration", {})
                combined = calibration.get("curves", {}).get("png")
                encoder = calibration.get("saved_calibration_fault", {}).get("encoder_curves", {})
                power_check = report.get("power_check", {})
                paths = {
                    kind: Path("calibration") / filename
                    for kind, filename in (
                        ("encoder", encoder.get("png") or combined),
                        ("hall", calibration.get("hall_raw_png") or combined),
                    )
                    if filename
                }
                paths.update(power_check.get("curves", {}))
                if report.get("waveform_overview"):
                    paths["overview"] = Path(report["waveform_overview"])
                if report.get("waveform_overview_error"):
                    self.waveform_overview_button.setToolTip(report["waveform_overview_error"])
                self.inspection_curve_paths = {
                    kind: directory / filename
                    for kind, filename in paths.items()
                    if directory and (directory / filename).is_file()
                }
                self.curve_path = self.inspection_curve_paths.get(
                    "encoder", self.inspection_curve_paths.get("hall")
                )
                self.calibration_preview.display(self.curve_path)
                if self.current_motor and self.current_motor["uid"] == report.get("uid"):
                    for step in (
                        power_check.get("run", {}),
                        report.get("electrical_zero", {}),
                        calibration,
                    ):
                        if step.get("final_status"):
                            self.current_motor.update(step["final_status"])
                            break
                detail = (
                    report.get("report_error")
                    or report.get("fatal")
                    or power_check.get("curve_error")
                    or report.get("note")
                )
                if not detail:
                    detail = "已确认关相" if report.get("stop_confirmed") else "关相未确认"
                    if self.report_path:
                        detail += " · 报告已归档"
                self.set_inspection_progress(f"检测结束 · {detail}")
            self.display_hardware_rows(report)
        elif profile in ("motor_calibration", "motor_speed", "calibration_readback"):
            curve = report.get("curves", {}).get("png") or report.get("hall_raw_png")
            if curve or profile == "motor_calibration":
                self.curve_path = directory / curve if directory and curve else None
                self.calibration_preview.display(self.curve_path)
            if profile == "motor_calibration":
                text = (
                    "标定完成并保存" if report.get("passed") else report.get("fatal", "标定未完成")
                )
                if report.get("curve_error"):
                    text += "；曲线未生成：" + report["curve_error"]
                raw = report.get("raw_capture", {})
                if raw.get("error"):
                    text += "；Hall 原码采集异常：" + raw["error"]
                elif raw.get("samples"):
                    text += f"；已采集 {raw['samples']} 组 Hall 原码"
                waveform = report.get("hall_waveform")
                if waveform:
                    status = waveform["status"]
                    if status == "detected":
                        phases = []
                        for sector in waveform["sectors"]:
                            start = sector["start_deg"]
                            end = start + sector["width_deg"]
                            phases.append(f"{start}～{end if end <= 360 else end % 360}°")
                        text += (
                            "\nHall 波形形状：检出重复凹陷（Hall 相位 " + "、".join(phases) + "）"
                        )
                    elif status == "clear":
                        text += "\nHall 波形形状：未检出重复凹陷"
                    else:
                        text += "\nHall 波形形状：未测 · " + waveform["reason"]
                    if status in ("detected", "clear"):
                        text += "\n试验判线：" + hall_waveform_criterion(waveform)
                self.calibration_result.setText(text)
            elif profile == "calibration_readback":
                self.calibration_result.setText(
                    "已回读保存的标定曲线；未重新标定"
                    if report.get("passed")
                    else report.get("fatal", "标定曲线回读失败")
                )
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
            if profile in ("hardware_check", "production_inspection"):
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
        versions = report.get("swd_versions")
        self.swd_version_label.setText(
            f"板端 SWD 已确认：APP v{version_text(versions['app'])} · "
            f"Loader v{version_text(versions['loader'])}（配套镜像回读一致）"
            if versions
            else "板端 SWD 版本：尚未确认"
        )
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
                if node in self.initial_choices:
                    self.initial_choices[node].setChecked(True)
                versions = report.get("can_test", {}).get("device_versions", {}) or report.get(
                    "swd_versions", {}
                )
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
                    and report.get("swd_versions")
                ):
                    self.install_label.setText("烧录、启动及 SWD 版本确认通过；CAN 通信未验证。")
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
        self.calibration_result.setText("等待标定")
        self.reset_inspection()
        self.install_label.setText("下一台待识别；连接 J-Link 后点击识别板卡。")
        self.swd_version_label.setText("板端 SWD 版本：尚未确认")
        self.progress.setValue(0)
        self.edits["specimen"].clear()
        self.edits["serial"].clear()
        self.edits["motor_serial"].clear()
        self.edits["test_requirement"].clear()
        self.select_motor()
        self.navigation.setCurrentRow(0)
        self.maintenance.setCurrentIndex(0)
        self.log.clear()
        self.task_status.setText("下一台待识别")
        self.update_controls()

    def show_curves(self) -> None:
        self.show_curve_file(self.curve_path)

    def show_inspection_curve(self, kind: str, title: str) -> None:
        path = self.inspection_curve_paths.get(kind)
        if kind == "overview":
            self.show_curve_file(path, title=title, data_files=(), overview=True)
            return
        if kind == "hall":
            self.show_curve_file(path)
            return
        if kind == "encoder":
            self.show_curve_file(
                path,
                title="编码器标定 · 补偿曲线",
                data_files=(
                    ("angle-calibration.csv", "补偿CSV"),
                    ("calibration-values.csv", "标定参数"),
                ),
            )
            return
        self.show_curve_file(
            path,
            title=title + " · 曲线",
            data_files=(("speed.csv", "高速遥测 CSV"),),
        )

    def show_curve_file(
        self,
        curve_path: Path | None,
        *,
        title: str | None = None,
        data_files: tuple[tuple[str, str], ...] = (
            ("angle-calibration.csv", "补偿CSV"),
            ("hall-raw.csv", "Hall原码"),
            ("calibration-values.csv", "标定参数"),
        ),
        overview: bool = False,
    ) -> None:
        if not curve_path:
            return
        pixmap = QPixmap(str(curve_path))
        if pixmap.isNull():
            self.error(f"无法读取波形图片：{curve_path.name}")
            return
        dialog = QDialog(self)
        dialog.setWindowTitle(
            title
            or (
                "Hall 原码波形与 XY 轨迹"
                if curve_path.name == "hall-raw.png"
                else "补偿曲线与原始数据"
            )
        )
        dialog.resize(QSize(1280, 840) if overview else QSize(1120, 760))
        layout = QVBoxLayout(dialog)
        actions = QHBoxLayout()
        if overview:
            dialog.setWindowFlag(Qt.WindowType.WindowMaximizeButtonHint)
            dialog.setAttribute(Qt.WidgetAttribute.WA_DeleteOnClose)
            scroll = WaveformView(pixmap)
            actions.addWidget(button("适应宽度", lambda: scroll.set_scale(None), icon_name="chart"))
            smaller = QPushButton("−")
            smaller.setAccessibleName("缩小")
            smaller.clicked.connect(lambda: scroll.zoom(0.8))
            actions.addWidget(smaller)
            scale = QLabel()
            scale.setMinimumWidth(48)
            scale.setAlignment(Qt.AlignmentFlag.AlignCenter)
            scroll.scale_changed.connect(lambda value: scale.setText(f"{value:.0%}"))
            actions.addWidget(scale)
            larger = QPushButton("+")
            larger.setAccessibleName("放大")
            larger.clicked.connect(lambda: scroll.zoom(1.25))
            actions.addWidget(larger)
            actions.addWidget(button("原始像素", lambda: scroll.set_scale(1.0), icon_name="chart"))
            actions.addStretch(1)
        else:
            scroll = QScrollArea()
            image = QLabel()
            if pixmap.width() > 1060:
                pixmap = pixmap.scaledToWidth(1060, Qt.TransformationMode.SmoothTransformation)
            image.setPixmap(pixmap)
            scroll.setWidget(image)
        for filename, label in ((curve_path.name, "PNG"), *data_files):
            path = curve_path.parent / filename
            if path.is_file():
                actions.addWidget(
                    button(label, lambda _=False, p=path: self.open_path(p), icon_name="chart")
                )
        layout.addLayout(actions)
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
