"""真实Qt离屏控件与模拟任务验证；不连接USB-CAN和探针。"""

import os
import queue
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools/bench"))

from can_parameters import parameter_ids
from can_session import SessionTask
from hardware_check import direction_results, empty_report
from production_controller import ProductionController
from production_report import inspection_overview
from PySide6.QtCore import QObject, Qt, Signal
from PySide6.QtGui import QColor, QIcon, QPalette, QPixmap
from PySide6.QtWidgets import (
    QApplication,
    QLabel,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QTabWidget,
)
from qt_production_icons import icon
from qt_production_theme import apply_theme
from qt_production_window import ProductionWindow
from qt_speed_plot import SpeedPlot


class FakeController(QObject):
    event = Signal(str, object)
    completed = Signal(object, object, bool)
    busy_changed = Signal(bool)
    closed = Signal()

    def __init__(self):
        super().__init__()
        self.busy = False
        self.connected = False
        self.automatic = False
        self.settings = {}
        self.calls = []
        self.stops = 0
        self.speeds = []

    def start(self, settings, directory, *, automatic=False):
        self.calls.append((settings, directory, automatic))
        self.settings = settings
        self.automatic = automatic
        self.busy = True
        self.busy_changed.emit(True)

    def stop(self):
        self.stops += 1

    def update_speed(self, value):
        self.speeds.append(value)

    def shutdown(self):
        self.closed.emit()

    def done(self, report):
        self.busy = False
        self.completed.emit(report, self.calls[-1][1] if self.calls else None, self.automatic)
        self.busy_changed.emit(False)


class QtProductionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.controller = FakeController()
        self.window = ProductionWindow(self.controller)
        self.window.refresh_timer.stop()
        self.temp = tempfile.TemporaryDirectory()
        self.window.root_log = Path(self.temp.name)
        self.addCleanup(self.temp.cleanup)
        self.addCleanup(self.window.deleteLater)
        self.error = patch.object(self.window, "error").start()
        self.addCleanup(patch.stopall)
        self.uid = "01" * 12

    def bind(self, node=6, uid=None):
        self.window.devices[node] = dict(
            node=node,
            uid=uid or self.uid,
            role=1,
            state=5,
            mode=0,
            faults=0,
            app_version=0x01080C,
            loader_version=0x020304,
            checked_at=time.time(),
        )
        self.window.axis.setCurrentIndex(node - 3)
        self.window.select_motor()

    def test_status_fields_expand_mode_and_fault_details(self):
        self.bind()
        w = self.window
        self.assertEqual(w.version_label.text(), "APP 固件：v1.8.12")
        self.assertEqual(w.loader_label.text(), "Loader 固件：v2.3.4")
        self.assertEqual(w.status_label.text(), "运行状态：空闲")
        self.assertIn("未使能（模式 0）", w.mode_label.text())
        self.assertEqual(w.fault_label.text(), "故障信息：无故障")
        w.current_motor.update(mode=2, state=8, faults=33)
        w.render_device()
        self.assertIn("速度闭环", w.mode_label.text())
        self.assertIn("过流", w.fault_label.text())
        self.assertIn("编码器", w.fault_label.text())
        self.assertNotIn("0x", w.fault_label.text())
        w.current_motor.update(faults=1 << 31)
        w.render_device()
        self.assertEqual(w.fault_label.text(), "故障信息：未识别故障")
        w.current_motor.update(mode=99, state=99, checked_at=time.time() - 6)
        w.render_device()
        self.assertIn("已过期", w.status_label.text())
        self.assertIn("未知模式", w.mode_label.text())
        self.assertIn("上次读取", w.fault_label.text())

    def test_local_firmware_version_refreshes_and_clears_stale_text(self):
        w = self.window
        w.hex_path = None
        w.update_firmware_version()
        self.assertEqual(w.firmware_label.text(), "固件版本未确认")
        with (
            patch("qt_production_window.QFileDialog.getOpenFileName", return_value=("new.hex", "")),
            patch("qt_production_window.firmware_versions", return_value=(0x010819, 0x020304)),
        ):
            w.choose_hex()
        self.assertEqual(w.firmware_label.text(), "APP 1.8.25   +   Loader 2.3.4")
        with patch("qt_production_window.firmware_versions", return_value=(0x010817, 0x020305)):
            w.update_firmware_version()
        self.assertEqual(w.firmware_label.text(), "APP 1.8.23   +   Loader 2.3.5")
        for error in (OSError("文件缺失"), ValueError("HEX 损坏"), KeyError("loader_version")):
            with patch("qt_production_window.firmware_versions", side_effect=error):
                w.update_firmware_version()
            self.assertEqual(w.firmware_label.text(), "固件版本未确认")
            self.assertNotIn("APP 1.8.23", w.firmware_label.text())
            self.assertTrue(w.firmware_label.toolTip())

    def test_input_widths_follow_field_content_across_pages(self):
        w = self.window
        for key, entry in w.edits.items():
            names = ("specimen", "serial", "motor_serial", "test_requirement")
            limit = 720 if key == "payload" else 200 if key in names else 120
            self.assertLessEqual(entry.maximumWidth(), limit, key)
        self.assertGreater(w.edits["payload"].maximumWidth(), w.edits["can_id"].maximumWidth())
        for entry in w.parameters.entries.values():
            self.assertLessEqual(entry.maximumWidth(), 180)
        for combo in (w.test_mode, w.target_data, w.target_nominal):
            self.assertLessEqual(combo.maximumWidth(), 260)

    def test_probe_selection_is_explicit_and_invalidates_previous_board(self):
        w = self.window
        w.display_probes([123, 456])
        self.assertIsNone(w.probe_selector.currentData())
        w.probe_selector.setCurrentIndex(w.probe_selector.findData(456))
        w.launch(dict(profile="jlink_identify"))
        self.assertEqual(self.controller.calls[-1][0]["probe_serial"], 456)
        self.controller.done(
            dict(
                profile="jlink_identify",
                success=True,
                probes=[123, 456],
                identity=dict(uid=self.uid, serial=456, node=6),
            )
        )
        self.assertEqual(w.identity["serial"], 456)
        w.display_probes([123, 456])
        self.assertEqual(w.identity["serial"], 456)
        w.probe_selector.setCurrentIndex(w.probe_selector.findData(123))
        self.assertIsNone(w.identity)
        self.assertIsNone(w.current_motor)
        self.assertFalse(w.flash_button.isEnabled())

    def test_swd_only_result_shows_versions_without_claiming_can_passed(self):
        w = self.window
        w.verify_can.setChecked(False)
        w.launch(dict(profile="jlink_identify"))
        self.assertIsNone(self.controller.calls[-1][0]["bus"])
        self.controller.busy = False
        w.display_install(
            dict(
                profile="jlink_flash",
                success=True,
                app_started=True,
                identity=dict(uid=self.uid, serial=123, node=6),
                swd_versions=dict(app=0x010819, loader=0x020304),
            )
        )
        w.update_controls()
        self.assertIn("v1.8.25", w.version_label.text())
        self.assertIn("v2.3.4", w.loader_label.text())
        self.assertIn("SWD 已确认", w.swd_version_label.text())
        self.assertIn("CAN 通信未验证", w.install_label.text())
        self.assertFalse(w.go_detection.isEnabled())
        w.display_install(dict(profile="jlink_erase", success=True))
        self.assertIn("尚未确认", w.swd_version_label.text())

    def test_new_flash_invalidates_previous_version_as_soon_as_job_starts(self):
        w = self.window
        w.display_install(
            dict(
                profile="jlink_flash",
                success=True,
                app_started=True,
                identity=dict(uid=self.uid, serial=123, node=6),
                swd_versions=dict(app=0x010819, loader=0x020304),
            )
        )
        w.hex_path = Path("test.hex")
        w.launch(dict(profile="jlink_flash"))
        self.assertIn("等待本次确认", w.swd_version_label.text())
        self.assertNotIn("v1.8.25", w.version_label.text())
        self.assertNotIn("v2.3.4", w.loader_label.text())

    def test_header_keeps_status_in_two_rows_and_settings_in_dialog(self):
        w = self.window
        w.show()
        self.app.processEvents()
        header = w.axis.parentWidget()
        self.assertIs(w.channel.parentWidget(), w.connection_dialog)
        self.assertTrue(w.connection_dialog.isHidden())
        self.assertLessEqual(header.height(), 90)
        self.assertIs(w.version_label.parentWidget(), header)
        self.assertAlmostEqual(
            w.axis.geometry().center().y(), w.version_label.geometry().center().y(), delta=3
        )
        self.assertIs(w.fault_button.parentWidget(), header)
        self.assertLess(w.fault_label.geometry().right(), w.fault_button.geometry().left())
        self.assertFalse(w.fault_button.isEnabled())
        self.assertTrue(w.fault_icon.isHidden())
        self.assertTrue(w.voltage.property("unavailable"))
        self.assertEqual(w.last_read.text(), "等待设备连接")
        self.bind()
        w.current_motor.update(bus_voltage_v=28.35, bus_current_a=0.01, motor_temperature_c=25)
        w.render_device()
        self.app.processEvents()
        centers = [
            widget.geometry().center().y()
            for widget in (w.voltage, w.current, w.temperature, w.fault_label)
        ]
        self.assertLessEqual(max(centers) - min(centers), 3)
        self.assertGreater(centers[0], w.axis.geometry().bottom())
        self.assertFalse(w.voltage.property("unavailable"))
        self.assertFalse(w.fault_icon.isHidden())
        self.assertTrue(w.fault_button.isEnabled())
        w.connection_settings.click()
        self.assertTrue(w.connection_dialog.isVisible())
        w.connection_dialog.close()
        w.fault_button.click()
        self.assertEqual(self.controller.calls[-1][0]["profile"], "clear_fault")
        self.assertEqual(self.controller.calls[-1][0]["expected_uid"], self.uid)
        w.hide()

    def test_inspection_results_are_primary_and_curve_is_inline(self):
        w = self.window
        self.bind(node=7)
        w.resize(1060, 740)
        w.navigation.setCurrentRow(1)
        w.show()
        self.app.processEvents()
        self.assertTrue(w.log.isHidden())
        self.assertTrue(w.failure.isHidden())
        self.assertTrue(w.inspection_settings.isHidden())
        self.assertTrue(w.hardware_start.isVisible())
        self.assertGreater(w.hardware_table.height(), 320)
        self.assertEqual(w.page_scroll.horizontalScrollBar().maximum(), 0)
        self.assertEqual(w.hardware_table.horizontalHeaderItem(0).text(), "序号")
        self.assertEqual(w.hardware_table.item(0, 0).text(), "1")
        self.assertEqual(w.hardware_table.item(0, 1).text(), "CAN 通信")
        self.assertEqual(w.hardware_table.item(0, 4).text(), "未测")
        static_names = (
            "Ia",
            "Ib",
            "Ic",
            "母线电流",
            "母线电压",
            "电机 NTC",
            "电机角静态",
            "霍尔静态",
        )
        for row, name in enumerate(static_names, 2):
            self.assertEqual(w.hardware_table.item(row, 0).text(), f"2.{row - 1}")
            self.assertEqual(w.hardware_table.item(row, 1).text(), name)
            self.assertEqual(w.hardware_table.item(row, 4).text(), "未测")
            self.assertFalse(w.hardware_table.isRowHidden(row))
        self.assertEqual(w.hardware_table.item(10, 0).text(), "3")

        static = empty_report(7)
        static.update(
            scope="static_resistance", static_completed=8, board_items=[1] * 7 + [3, 0, 0]
        )
        static["measurements"] = {"0": dict(samples=20, values=[2048, 1, None, None])}
        direction_results(static)
        w.on_event(
            "inspection_results",
            inspection_overview({"phase_states": {"preflight": "进行中"}}, static_snapshot=static),
        )
        self.assertIn("7/7", w.hardware_table.item(1, 2).text())
        self.assertIn("2048", w.hardware_table.item(2, 2).text())
        self.assertIn("RMS", w.hardware_table.item(2, 3).text())
        self.assertEqual(w.hardware_table.item(9, 4).text(), "未装")
        self.assertIn("通过 7", w.result_counts.text())

        running = dict(profile="production_inspection", phase_states={"power": "进行中"})
        w.on_event("inspection_phases", running["phase_states"])
        w.on_event("inspection_results", inspection_overview(running))
        w.on_event(
            "inspection_angle",
            dict(elapsed_s=0.1, reference_source="motor_encoder", reference_deg=7),
        )
        plot_row = w.hardware_table.curve_row
        self.assertFalse(w.hardware_table.isRowHidden(plot_row))
        self.assertEqual(w.hardware_table.item(plot_row, 0).text(), "")
        self.assertEqual(w.hardware_table.item(11, 1).text(), "编码器标定")
        self.assertEqual(w.hardware_table.item(12, 1).text(), "Hall 标定")
        self.assertEqual(w.hardware_table.item(plot_row - 2, 0).text(), "5.3")
        self.assertEqual(w.hardware_table.item(plot_row - 1, 0).text(), "6")
        self.assertEqual(w.hardware_table.item(plot_row + 1, 0).text(), "7")
        self.assertEqual(w.hardware_table.item(plot_row + 1, 1).text(), "输出轴转速稳定性")
        self.assertIn("±5%", w.hardware_table.item(plot_row + 1, 3).text())
        self.assertIn("10 秒", w.hardware_table.item(plot_row + 1, 3).text())
        self.assertEqual(w.hardware_table.item(plot_row + 2, 1).text(), "高速输入功率稳定性")
        self.assertIn("[-1000, 1000] W", w.hardware_table.item(plot_row + 2, 3).text())
        self.assertIn("高速角度、转速及功率检测", w.phase_label.text())
        w.hardware_table.curve_button.click()
        w.on_event("inspection_results", inspection_overview(running))
        self.assertTrue(w.hardware_table.isRowHidden(plot_row))
        self.assertEqual(len(w.inspection_plot.samples), 1)
        complete = dict(
            profile="production_inspection",
            phase_states={"power": "未通过", "archive": "已归档"},
            verdict="未通过 / 未完成",
            power_check=dict(status="failed", reason="高速检测通信失败"),
            production_report="production-report.html",
            stop_confirmed=True,
        )
        w.on_completed(complete, w.root_log, False)
        self.assertEqual(w.result_verdict.property("outcome"), "failed")
        self.assertIn("失败 1", w.result_counts.text())
        self.assertIn("高速检测通信失败", w.hardware_table.item(plot_row + 2, 2).text())
        self.assertNotEqual(w.hardware_table.item(plot_row + 2, 1).background().color(), QColor())
        self.assertTrue(w.open_report_button.isEnabled())
        self.assertTrue(w.hardware_table.isRowHidden(plot_row))
        w.hardware_table.curve_button.click()
        self.assertFalse(w.hardware_table.isRowHidden(plot_row))
        self.assertEqual(len(w.inspection_plot.samples), 1)
        w.hide()

    def test_hall_calibration_failure_opens_hall_waveform_and_saved_encoder_curve(self):
        self.bind(node=7)
        w = self.window
        folder = w.root_log / "calibration"
        folder.mkdir()
        pixmap = QPixmap(45, 20)
        pixmap.fill(QColor("white"))
        self.assertTrue(pixmap.save(str(folder / "hall-raw.png")))
        encoder = QPixmap(55, 20)
        encoder.fill(QColor("white"))
        self.assertTrue(encoder.save(str(folder / "angle-calibration.png")))
        (folder / "hall-raw.csv").touch()
        (folder / "angle-calibration.csv").touch()
        w.on_completed(
            dict(
                profile="production_inspection",
                uid=self.uid,
                phase_states={"calibration": "未通过"},
                fatal="联合电角零位 · Hall sin 最大值过高",
                calibration=dict(
                    hall_raw_png="hall-raw.png",
                    final_status=dict(state=8, mode=0, faults=0x4000),
                    hall_waveform=dict(status="not_measured", reason="原码触及 ADC 端点"),
                    saved_calibration_fault=dict(
                        calibration_flags=3,
                        saved_revision=8,
                        details=["Hall sin 最大值过高"],
                        encoder_curves=dict(png="angle-calibration.png"),
                    ),
                ),
            ),
            w.root_log,
            False,
        )
        self.assertIn("0x00004000", w.fault_label.toolTip())
        self.assertIn("Hall sin 最大值过高", w.phase_label.text())
        table = w.hardware_table
        for name, title, width, csv_label in (
            ("Hall 波形凹坑", "Hall 原码波形与 XY 轨迹", 45, "Hall原码"),
            ("编码器标定", "编码器标定 · 补偿曲线", 55, "补偿CSV"),
        ):
            item = table.findItems(name, Qt.MatchFlag.MatchExactly)[0]
            action = table.cellWidget(item.row(), 2).findChild(QPushButton, "rowAction")
            action.click()
            self.assertEqual(w.curve_dialog.windowTitle(), title)
            self.assertEqual(w.curve_dialog.findChild(QScrollArea).widget().pixmap().width(), width)
            self.assertIn(
                csv_label, [button.text() for button in w.curve_dialog.findChildren(QPushButton)]
            )
            w.curve_dialog.close()

    def test_high_speed_curve_buttons_open_matching_files_and_reset_each_run(self):
        w = self.window
        work = w.root_log / "high-speed"
        work.mkdir()
        curves = {"speed": "high-speed/output-speed.png", "power": "high-speed/input-power.png"}
        for name, width in zip(curves.values(), (30, 40), strict=True):
            image = QPixmap(width, 20)
            image.fill(QColor("white"))
            self.assertTrue(image.save(str(w.root_log / name)))
        (work / "speed.csv").touch()
        overview = QPixmap(3900, 4500)
        overview.fill(QColor("white"))
        self.assertTrue(overview.save(str(w.root_log / "waveform-overview.png")))
        w.on_completed(
            dict(
                profile="production_inspection",
                power_check=dict(curves=curves),
                waveform_overview="waveform-overview.png",
            ),
            w.root_log,
            False,
        )
        table = w.hardware_table
        for title, width in (("输出轴转速稳定性", 30), ("高速输入功率稳定性", 40)):
            item = next(
                cell
                for cell in table.findItems(title, Qt.MatchFlag.MatchExactly)
                if cell.column() == 1
            )
            row_button = table.cellWidget(item.row(), 2).findChild(QPushButton, "rowAction")
            self.assertEqual(row_button.text(), "查看曲线")
            row_button.click()
            self.assertIn(title, w.curve_dialog.windowTitle())
            preview = w.curve_dialog.findChild(QScrollArea).widget()
            self.assertEqual(preview.pixmap().width(), width)
            self.assertIn(
                "高速遥测 CSV", [b.text() for b in w.curve_dialog.findChildren(QPushButton)]
            )
            w.curve_dialog.close()
        self.assertTrue(w.waveform_overview_button.isEnabled())
        w.waveform_overview_button.click()
        self.assertEqual(w.curve_dialog.windowTitle(), "生产检测波形总览")
        self.app.processEvents()
        scroll = w.curve_dialog.findChild(QScrollArea)
        self.assertEqual(scroll.widget().pixmap.width(), 3900)
        self.assertEqual(scroll.widget().width(), scroll.viewport().width())
        w.curve_dialog.resize(1400, 800)
        self.app.processEvents()
        self.assertEqual(scroll.widget().width(), scroll.viewport().width())
        self.assertGreater(scroll.widget().width(), 1060)
        buttons = {b.text(): b for b in w.curve_dialog.findChildren(QPushButton)}
        buttons["原始像素"].click()
        self.app.processEvents()
        pixel_ratio = scroll.devicePixelRatioF()
        self.assertEqual(scroll.widget().width(), round(3900 / pixel_ratio))
        self.assertIn("100%", [label.text() for label in w.curve_dialog.findChildren(QLabel)])
        self.assertGreater(scroll.horizontalScrollBar().maximum(), 0)
        buttons["+"].click()
        self.assertEqual(scroll.widget().width(), round(3900 * 1.25 / pixel_ratio))
        buttons["−"].click()
        self.assertEqual(scroll.widget().width(), round(3900 / pixel_ratio))
        buttons["适应宽度"].click()
        self.app.processEvents()
        self.assertEqual(scroll.widget().width(), scroll.viewport().width())
        self.assertEqual(scroll.widget().pixmap.width(), 3900)
        w.curve_dialog.close()
        w.reset_result_view(running=True)
        self.assertEqual(w.inspection_curve_paths, {})
        self.assertFalse(w.waveform_overview_button.isEnabled())
        for title in ("输出轴转速稳定性", "高速输入功率稳定性"):
            item = next(
                cell
                for cell in table.findItems(title, Qt.MatchFlag.MatchExactly)
                if cell.column() == 1
            )
            self.assertIsNone(table.cellWidget(item.row(), 2))

    def test_install_steps_group_actions_and_prioritize_device_versions(self):
        w = self.window
        self.bind()
        w.resize(1380, 1050)
        w.show()
        self.app.processEvents()
        self.assertIs(w.identify_button.parentWidget(), w.identify_box)
        self.assertIs(w.probe_selector.parentWidget(), w.identify_box)
        self.assertIs(w.flash_button.parentWidget(), w.flash_box)
        self.assertIs(w.firmware_label.parentWidget(), w.flash_box)
        self.assertLess(w.identify_box.x(), w.initial_box.x())
        self.assertLess(w.initial_box.x(), w.flash_box.x())
        self.assertGreater(w.version_label.font().pixelSize(), w.firmware_label.font().pixelSize())
        self.assertFalse(w.version_label.property("unavailable"))
        w.devices.clear()
        w.select_motor()
        self.assertTrue(w.version_label.property("unavailable"))
        w.hide()

    def test_light_palette_replaces_dark_system_colors(self):
        dark = QPalette()
        dark.setColor(QPalette.ColorRole.Window, QColor("#101010"))
        dark.setColor(QPalette.ColorRole.Base, QColor("#000000"))
        self.app.setProperty("production_theme", None)
        self.app.setPalette(dark)
        apply_theme(self.app)
        self.assertEqual(self.app.palette().color(QPalette.ColorRole.Base).name(), "#ffffff")
        self.assertEqual(self.app.palette().color(QPalette.ColorRole.Window).name(), "#f2f5f9")
        self.assertEqual(self.app.palette().color(QPalette.ColorRole.Shadow).name(), "#cdd8e4")

    def test_scan_selects_only_online_motor_then_reads_status(self):
        w = self.window
        self.controller.connected = True
        motor = dict(
            node=7,
            uid=self.uid,
            role=1,
            state=5,
            faults=0,
            app_version=0x01080E,
            loader_version=0x020304,
        )
        with patch("qt_production_window.QTimer.singleShot") as scheduled:
            w.on_completed(dict(profile="can_scan", motors=[motor], issues=[]), None, True)
        self.assertEqual(w.node(), 7)
        self.assertEqual(w.current_motor["uid"], self.uid)
        self.assertIn("v1.8.14", w.version_label.text())
        scheduled.assert_called_once()
        w.read_selected_status()
        request = self.controller.calls[-1][0]
        self.assertEqual(request["profile"], "device_status")
        self.assertEqual(request["node"], 7)
        self.assertEqual(request["expected_uid"], self.uid)

    def test_multiple_scan_keeps_selected_online_node(self):
        w = self.window
        motors = [dict(node=n, uid=str(n) * 24, role=1) for n in (4, 6, 7)]
        with patch("qt_production_window.QTimer.singleShot"):
            w.on_completed(dict(profile="can_scan", motors=motors), None, True)
        self.assertEqual(w.node(), 6)
        self.assertIsNotNone(w.current_motor)
        w.on_completed(dict(profile="can_scan", motors=[]), None, True)
        self.assertIsNone(w.current_motor)
        self.assertFalse(self.controller.calls)

    def test_missing_jlink_does_not_discard_can_device(self):
        self.bind(7)
        self.window.launch(dict(profile="jlink_identify"))
        self.controller.busy = False
        self.window.display_install(
            dict(profile="jlink_identify", success=False, fatal="没有J-Link")
        )
        self.assertEqual(self.window.current_motor["uid"], self.uid)
        self.assertIsNone(self.window.identity)

    def test_initial_scan_once_and_explicit_disconnect_no_retry(self):
        w = self.window
        w.initial_scan()
        self.assertEqual(self.controller.calls[-1][0]["profile"], "can_scan")
        self.assertTrue(self.controller.calls[-1][2])
        self.controller.busy = False
        self.controller.connected = False
        self.controller.calls.clear()
        w.refresh_status()
        self.assertFalse(self.controller.calls)

    def test_every_action_navigation_and_tab_has_a_valid_icon(self):
        w = self.window
        self.assertFalse(w.windowIcon().isNull())
        for button in w.findChildren(QPushButton):
            with self.subTest(button=button.text()):
                self.assertFalse(button.icon().isNull())
                self.assertFalse(button.icon().pixmap(20, 20).isNull())
        for index in range(w.navigation.count()):
            self.assertFalse(w.navigation.item(index).icon().isNull())
        for tabs in w.findChildren(QTabWidget):
            for index in range(tabs.count()):
                self.assertFalse(tabs.tabIcon(index).isNull())
        assets = Path(__file__).resolve().parents[2] / "tools/bench/assets/icons"
        self.assertEqual(len(list(assets.glob("*.svg"))), 24)
        for path in assets.glob("*.svg"):
            self.assertFalse(icon(path.stem).pixmap(16, 16).isNull())

    def test_primary_and_disabled_icons_have_distinct_visible_colors(self):
        active = icon("start", emphasis=True).pixmap(20, 20, QIcon.Mode.Normal).toImage()
        disabled = icon("start", emphasis=True).pixmap(20, 20, QIcon.Mode.Disabled).toImage()
        active_colors = {
            active.pixelColor(x, y).name()
            for x in range(20)
            for y in range(20)
            if active.pixelColor(x, y).alpha() > 200
        }
        disabled_colors = {
            disabled.pixelColor(x, y).name()
            for x in range(20)
            for y in range(20)
            if disabled.pixelColor(x, y).alpha() > 200
        }
        self.assertIn("#ffffff", active_colors)
        self.assertIn("#8798a9", disabled_colors)

    def test_native_pages_routes_and_removed_controls(self):
        w = self.window
        self.assertEqual(w.stack.count(), 5)
        self.assertEqual(w.navigation.currentRow(), 0)
        self.assertEqual(w.maintenance.count(), 5)
        self.assertEqual(w.maintenance.tabText(2), "通信测试")
        self.assertEqual(w.maintenance.tabText(4), "标定维护")
        self.assertEqual(w.communication_tabs.count(), 2)
        self.assertEqual(len(w.parameters.entries), 30)
        self.assertNotIn("lease", w.edits)
        self.assertNotIn("target", w.edits)
        self.assertFalse(self.controller.calls)
        for i in range(5):
            w.navigation.setCurrentRow(i)
            self.assertEqual(w.stack.currentIndex(), i)
        self.assertFalse(self.controller.calls)

    def test_first_fit_requires_type_and_resume_is_locked(self):
        w = self.window
        w.hex_path = Path("test.hex")
        w.launch(dict(profile="jlink_flash"))
        self.error.assert_called_once()
        self.assertFalse(self.controller.calls)
        w.display_install(
            dict(
                profile="jlink_identify",
                success=True,
                identity=dict(uid=self.uid, serial=123, node=None),
            )
        )
        w.update_controls()
        self.assertFalse(w.flash_button.isEnabled())
        w.initial_choices[5].setChecked(True)
        self.assertTrue(w.flash_button.isEnabled())
        w.launch(dict(profile="jlink_flash"))
        request = self.controller.calls[-1][0]
        self.assertEqual(request["first_fit_node"], 5)
        self.assertTrue(request["force_reflash"])
        self.assertTrue(request["restart_after_flash"])
        self.controller.busy = False
        w.display_install(
            dict(
                profile="jlink_identify",
                success=True,
                identity=dict(uid=self.uid, serial=123, node=None, resume_node=7),
            )
        )
        w.update_controls()
        self.assertTrue(w.initial_choices[7].isChecked())
        self.assertTrue(all(not c.isEnabled() for c in w.initial_choices.values()))

    def test_motor_type_is_visible_before_identification_and_keeps_preselection(self):
        w = self.window
        w.show()
        self.app.processEvents()
        self.assertTrue(w.initial_box.isVisible())
        w.initial_choices[5].setChecked(True)
        w.display_install(
            dict(
                profile="jlink_identify",
                success=True,
                identity=dict(uid=self.uid, serial=123, node=None),
            )
        )
        w.update_controls()
        self.assertTrue(w.initial_choices[5].isChecked())
        w.display_install(
            dict(
                profile="jlink_identify",
                success=True,
                identity=dict(uid=self.uid, serial=123, node=7),
            )
        )
        w.update_controls()
        self.assertTrue(w.initial_box.isVisible())
        self.assertTrue(w.initial_choices[7].isChecked())
        self.assertTrue(all(not choice.isEnabled() for choice in w.initial_choices.values()))
        w.hide()

    def test_flash_readback_gate_and_existing_parameter_preservation(self):
        w = self.window
        w.hex_path = Path("test.hex")
        report = dict(
            profile="jlink_flash",
            success=True,
            app_started=True,
            identity=dict(uid=self.uid, serial=123, node=6),
        )
        w.display_install(report)
        w.update_controls()
        self.assertFalse(w.go_detection.isEnabled())
        report["can_test"] = dict(
            version_verified=True, device_versions=dict(app=0x01080C, loader=0x020304)
        )
        w.display_install(report)
        w.update_controls()
        self.assertTrue(w.go_detection.isEnabled())
        w.launch(dict(profile="jlink_flash"))
        self.assertIsNone(self.controller.calls[-1][0]["first_fit_node"])
        self.assertEqual(self.controller.calls[-1][0]["identity"]["uid"], self.uid)

    def test_production_check_always_includes_motion_and_validates_inputs(self):
        self.bind()
        w = self.window
        self.assertFalse(hasattr(w, "check_mode"))
        self.assertFalse(self.controller.calls)
        w.start_hardware()
        self.assertFalse(self.controller.calls)
        w.edits["motor_serial"].setText("YAW-0001")
        w.edits["test_requirement"].setText("REQ-017")
        w.confirm_inspection_numbers()
        self.assertTrue(w.edits["motor_serial"].isReadOnly())
        self.assertFalse(hasattr(w, "inspection_fixture"))
        self.assertTrue(w.hardware_start.isEnabled())
        w.start_hardware()
        request, directory, _ = self.controller.calls[-1]
        self.assertEqual(request["profile"], "production_inspection")
        self.assertIsNone(request["drive"])
        self.assertEqual(request["power_check"]["output_rpm"], 50)
        self.assertNotIn("travel_deg", request["power_check"])
        self.assertNotIn("power_travel", w.edits)
        self.assertEqual(request["expected_uid"], self.uid)
        self.assertEqual(directory.parent, w.root_log / "production" / "YAW-0001" / "REQ-017")
        self.assertNotIn("fixture_confirmed", request)
        self.controller.busy = False
        count = len(self.controller.calls)
        w.edits["power_rpm"].setText("99")
        w.start_hardware()
        self.assertEqual(len(self.controller.calls), count)
        self.error.assert_called()

    def test_inspection_angle_updates_and_change_of_motor_invalidates_numbers(self):
        self.bind()
        w = self.window
        w.edits["motor_serial"].setText("S-1")
        w.edits["test_requirement"].setText("T-1")
        w.confirm_inspection_numbers()
        w.on_event(
            "inspection_angle",
            dict(elapsed_s=0.2, reference_source="motor_encoder", reference_deg=5.2, hall_deg=None),
        )
        self.assertEqual(w.inspection_plot.samples[-1]["reference_deg"], 5.2)
        self.assertIsNone(w.inspection_plot.samples[-1]["hall_deg"])
        self.bind(uid="02" * 12)
        self.assertIsNone(w.inspection_binding)
        self.assertFalse(w.inspection_plot.samples)
        self.assertEqual(w.navigation.item(2).text(), "电机运动")

    def test_measurements_refresh_every_second_without_draft_reset(self):
        self.bind()
        w = self.window
        self.controller.connected = True
        w.parameters.entries[11].setText("1.234")
        with (
            patch("qt_production_window.time.monotonic", return_value=10),
            patch.object(QApplication, "focusWidget", return_value=None),
        ):
            w.refresh_status()
        self.assertEqual(w.next_refresh, 11)
        self.assertTrue(self.controller.calls[-1][2])
        device = dict(
            w.current_motor, bus_voltage_v=28.32, bus_current_a=0.001, motor_temperature_c=-60.44
        )
        self.controller.done(dict(profile="device_status", passed=True, device=device))
        self.assertEqual(w.parameters.entries[11].text(), "1.234")
        self.assertEqual(w.voltage.text(), "28.32 V")
        self.assertEqual(w.current.text(), "0.001 A")
        self.assertEqual(w.temperature.text(), "-60.44 °C")
        self.assertEqual(w.next_refresh, 11)
        device["checked_at"] -= 6
        w.current_motor = device
        w.render_device()
        self.assertEqual(w.voltage.text(), "— V")

    def test_edit_focus_defers_automatic_query(self):
        self.bind()
        self.controller.connected = True
        with patch.object(QApplication, "focusWidget", return_value=self.window.edits["speed"]):
            self.window.refresh_status()
        self.assertFalse(self.controller.calls)

    def test_parameter_read_draft_save_and_uid_reset(self):
        self.bind()
        w = self.window
        data = {
            i: dict(raw=1000, access=3, revision=1, saved_revision=1, flags=1)
            for i in parameter_ids("all")
        }
        report = dict(
            profile="parameters",
            action="read",
            group="all",
            node=6,
            uid=self.uid,
            passed=True,
            revision=1,
            rows=data,
        )
        w.on_completed(report, None, False)
        self.assertEqual(len(w.parameters.snapshots), 4)
        w.parameters.entries[11].setText("1.5")
        w.on_completed(report, None, False)
        self.assertEqual(w.parameters.entries[11].text(), "1.5")
        w.parameter_action("current", True)
        self.assertEqual(self.controller.calls[-1][0]["changes"], {11: 1500})
        self.controller.busy = False
        self.bind(uid="02" * 12)
        self.assertFalse(w.parameters.snapshots)
        self.assertEqual(w.parameters.entries[11].text(), "")

    def test_hardware_failure_values_and_missing_scope(self):
        report = empty_report(6)
        report.update(
            uid=self.uid,
            scope="static",
            communication="正常",
            stage=4,
            board_items=[1, 1, 1, 1, 2, 1, 1, 3, 0, 0],
            measurements_supported=True,
        )
        report["measurements"] = {"4": dict(samples=20, values=[26.8, 28, 1, 0])}
        direction_results(report)
        self.window.on_completed(report, None, False)
        self.assertIn("26.80", self.window.failure.toPlainText())
        self.assertIn("27～29", self.window.failure.toPlainText())
        self.assertEqual(self.window.hardware_table.rowCount(), 11)
        self.assertIn("不在本轮范围", self.window.hardware_table.item(9, 4).text())

    def test_sequential_static_updates_rows_before_completion(self):
        report = empty_report(7)
        report.update(
            scope="full",
            stage=1,
            board_items=[1] + [0] * 9,
            measurements_supported=True,
            static_completed=1,
        )
        report["measurements"] = {"0": dict(samples=2000, values=[2048, 1, None, None])}
        direction_results(report)
        self.window.on_event("hardware_static", report)
        self.assertEqual(self.window.hardware_table.item(0, 4).text(), "正常")
        self.assertEqual(self.window.hardware_table.item(1, 4).text(), "未测")
        self.assertIn("正在检测：Ib", self.window.phase_label.text())
        self.assertIn("2048", self.window.hardware_table.item(0, 2).text())

    def test_worker_exit_before_hardware_report_remains_visible(self):
        self.bind()
        self.window.on_completed(dict(profile="hardware_check", fatal="进程退出"), None, False)
        self.assertIn("进程退出", self.window.failure.toPlainText())
        self.assertEqual(self.window.hardware_table.rowCount(), 11)

    def test_hardware_colors_distinguish_pass_failure_and_incomplete(self):
        report = empty_report(7)
        report.update(
            scope="full",
            communication="正常",
            stage=5,
            board_items=[1, 0, 2, 0, 0, 0, 0, 0, 0, 0],
            measurements_supported=True,
        )
        report["measurements"] = {"1": dict(samples=1, values=[2033.75, 0, None, None])}
        direction_results(report)
        self.window.on_completed(report, None, False)
        for row, expected in ((0, "#18765b"), (1, "#64748b"), (2, "#b03b42")):
            for column in range(self.window.hardware_table.columnCount()):
                self.assertEqual(
                    self.window.hardware_table.item(row, column).foreground().color().name(),
                    expected,
                )

    def test_query_modes_raw_and_upgrade_targets(self):
        self.bind(7)
        w = self.window
        for text, mode in (
            ("常规查询", "fixed"),
            ("阶梯压测", "ladder"),
            ("五轴稳定性验收", "acceptance"),
        ):
            self.controller.busy = False
            w.test_mode.setCurrentText(text)
            w.start_query()
            self.assertEqual(self.controller.calls[-1][0]["profile"], mode)
        self.controller.busy = False
        w.fill_query()
        w.send_frame()
        self.assertEqual(self.controller.calls[-1][0]["profile"], "manual")
        self.controller.busy = False
        w.populate_upgrade_table()
        w.batch_upgrade.setChecked(True)
        w.launch(dict(profile="can_upgrade"))
        request = self.controller.calls[-1][0]
        self.assertEqual(request["targets"], [dict(node=7, uid=self.uid)])
        self.assertFalse(request["single_frame_writes"])

    def test_upgrade_page_keeps_compact_controls_and_separate_device_details(self):
        self.bind(7)
        w = self.window
        w.navigation.setCurrentRow(0)
        w.maintenance.setCurrentWidget(w.pages["upgrade"])
        w.populate_upgrade_table()
        w.resize(1440, 960)
        w.show()
        self.app.processEvents()
        self.assertLessEqual(w.edits["rounds"].width(), 80)
        self.assertLess(
            w.edits["rounds"].mapToGlobal(w.edits["rounds"].rect().topLeft()).x(),
            w.upgrade_table.mapToGlobal(w.upgrade_table.rect().topLeft()).x(),
        )
        self.assertEqual(w.upgrade_table.item(0, 3).text(), "在线 · APP")
        self.assertEqual(w.upgrade_table.item(0, 4).text(), "v1.8.12")
        self.assertEqual(w.upgrade_table.item(0, 5).text(), "v2.3.4")
        w.on_event("axis_status", (7, "APP 已持久确认"))
        self.assertEqual(w.upgrade_table.item(0, 6).text(), "APP 已持久确认")
        self.assertEqual(w.upgrade_table.item(0, 4).text(), "v1.8.12")
        package = Path(self.temp.name) / "can-app"
        package.mkdir()
        (package / "package.json").write_text(
            '{"image_version":67600,"loader_version":131844}', encoding="utf-8"
        )
        w.package_path = package
        w.update_upgrade_package()
        self.assertEqual(w.package_label.text(), "升级包：can-app")
        self.assertEqual(w.package_label.toolTip(), str(package))
        self.assertIn("APP v1.8.16", w.upgrade_package_info.text())
        w.hide()

    def test_runtime_parameters_use_device_snapshot_and_clear_for_next_motor(self):
        self.bind()
        w = self.window
        rows = {
            i: dict(raw=500, access=3, flags=1, revision=2, saved_revision=2)
            for i in parameter_ids("speed")
        }
        w.on_completed(
            dict(
                profile="parameters",
                group="speed",
                action="read",
                node=6,
                uid=self.uid,
                passed=True,
                revision=2,
                rows=rows,
            ),
            None,
            False,
        )
        self.assertIn("0.0005", w.speed_parameter_values[16].text())
        w.parameters.entries[16].setText("9")
        w.update_controls()
        self.assertIn("0.0005", w.speed_parameter_values[16].text())
        self.assertIn("修订 2", w.speed_parameter_status.text())
        self.bind(uid="02" * 12)
        self.assertEqual(w.speed_parameter_values[16].text(), "—")

    def test_calibration_embeds_curve_and_clears_previous_motor(self):
        self.bind()
        w = self.window
        picture = QPixmap(1000, 1500)
        picture.fill(QColor("white"))
        path = Path(self.temp.name) / "curve.png"
        self.assertTrue(picture.save(str(path)))
        w.navigation.setCurrentRow(0)
        w.maintenance.setCurrentWidget(w.pages["calibration"])
        w.resize(1280, 900)
        w.show()
        self.app.processEvents()
        w.on_completed(
            dict(
                profile="motor_calibration", passed=True, uid=self.uid, curves=dict(png=path.name)
            ),
            path.parent,
            False,
        )
        self.app.processEvents()
        self.assertFalse(w.calibration_preview.image.isNull())
        self.assertTrue(w.curve_button.isEnabled())
        self.assertLessEqual(
            w.calibration_preview.label.width(), w.calibration_preview.viewport().width()
        )
        self.assertGreater(w.calibration_preview.verticalScrollBar().maximum(), 0)
        self.bind(uid="02" * 12)
        self.assertTrue(w.calibration_preview.image.isNull())
        self.assertIsNone(w.curve_path)
        w.calibration_preview.display(path.parent / "missing.png")
        self.assertIn("无法读取", w.calibration_preview.label.text())
        w.hide()

    def test_calibration_archive_speed_and_stop(self):
        self.bind()
        w = self.window
        w.display_probes([123, 456])
        w.probe_selector.setCurrentIndex(w.probe_selector.findData(456))
        self.bind()
        w.start_calibration(2)
        request = self.controller.calls[-1][0]
        self.assertEqual(request["probe_serial"], 456)
        self.assertEqual(request["calibration_kind"], 2)
        self.assertEqual(request["archive"]["test_serial"], "0001")

        self.controller.busy = False
        w.start_speed()
        self.assertEqual(self.controller.calls[-1][0]["speed_limit"], 300)
        w.edits["speed"].setText("-2")
        w.update_speed()
        self.assertEqual(self.controller.speeds, [-2])
        w.stop_button.click()
        self.assertEqual(self.controller.stops, 1)

    def test_saved_calibration_readback_does_not_start_calibration(self):
        self.bind()
        self.window.launch(dict(profile="calibration_readback", probe_serial=None))
        request = self.controller.calls[-1][0]
        self.assertEqual(request["profile"], "calibration_readback")
        self.assertEqual(request["expected_uid"], self.uid)
        self.assertNotIn("calibration_kind", request)

    def test_calibration_displays_raw_capture_result_separately(self):
        self.bind()
        report = dict(profile="motor_calibration", passed=True, uid=self.uid)
        self.window.on_completed(dict(report, raw_capture=dict(error="未采集原码")), None, False)
        self.assertIn("标定完成并保存", self.window.calibration_result.text())
        self.assertIn("Hall 原码采集异常：未采集原码", self.window.calibration_result.text())
        self.window.on_completed(dict(report, raw_capture=dict(samples=123)), None, False)
        self.assertIn("123 组 Hall 原码", self.window.calibration_result.text())

    def test_hall_waveform_result_displays_phase_ranges(self):
        self.bind()
        report = dict(profile="motor_calibration", passed=True, uid=self.uid)
        waveform = dict(
            status="detected",
            radius_limit=0.90,
            sectors=[dict(start_deg=40, width_deg=10), dict(start_deg=350, width_deg=20)],
        )
        self.window.on_completed(dict(report, hall_waveform=waveform), None, False)
        text = self.window.calibration_result.text()
        self.assertIn("标定完成并保存", text)
        self.assertIn("检出重复凹陷", text)
        self.assertIn("40～50°、350～10°", text)
        self.assertIn("试验判线", text)

    def test_busy_disables_nonmotion_controls_but_allows_speed_target(self):
        self.bind()
        self.window.navigation.setCurrentRow(2)
        self.window.start_speed()
        self.assertFalse(self.window.navigation.isEnabled())
        self.assertFalse(self.window.axis.isEnabled())
        self.assertTrue(self.window.edits["speed"].isEnabled())
        self.assertFalse(self.window.edits["speed_limit"].isEnabled())

    def test_next_board_archives_preserves_batch_and_drops_results(self):
        self.bind(7)
        w = self.window
        w.flow.record(dict(profile="jlink_flash", uid=self.uid, success=False), "old")
        w.edits["ambient"].setText("23")
        w.edits["power_rpm"].setText("47")
        w.next_device()
        self.assertIsNone(w.current_motor)
        self.assertFalse(w.flow.devices)
        self.assertEqual(w.node(), 7)
        self.assertEqual(w.edits["ambient"].text(), "23")
        self.assertEqual(w.edits["power_rpm"].text(), "")
        self.assertEqual(len(list(w.root_log.rglob("production-sessions/*.json"))), 1)
        self.assertIsNone(w.report_dir)

    def test_archive_failure_keeps_device_and_hardware_task_cannot_switch(self):
        self.bind()
        w = self.window
        w.flow.record(dict(profile="jlink_flash", uid=self.uid, success=False), "old")
        with patch.object(Path, "write_text", side_effect=OSError("disk full")):
            w.next_device()
        self.assertEqual(w.current_motor["uid"], self.uid)
        self.controller.busy = True
        w.next_device()
        self.assertEqual(w.current_motor["uid"], self.uid)
        self.assertEqual(self.controller.stops, 0)

    def test_erase_page_identifies_swd_target_independently_of_can(self):
        self.bind()
        w = self.window
        self.assertFalse(w.erase_all.isEnabled())
        self.assertIn("J-Link", w.erase_status.text())
        w.erase_identify.click()
        self.assertEqual(self.controller.calls[-1][0]["profile"], "jlink_identify")
        self.assertNotIn("erase_confirmed", self.controller.calls[-1][0])
        self.controller.done(
            dict(
                profile="jlink_identify",
                success=True,
                identity=dict(uid=self.uid, node=6, serial=123),
            )
        )
        self.assertTrue(w.erase_app.isEnabled())
        self.assertTrue(w.erase_all.isEnabled())
        self.assertIn(self.uid, w.erase_status.text())
        self.controller.done(
            dict(
                profile="jlink_identify",
                success=True,
                identity=dict(uid=self.uid, node=None, serial=123),
            )
        )
        self.assertFalse(w.erase_app.isEnabled())
        self.assertTrue(w.erase_all.isEnabled())

    def test_erase_requires_explicit_confirmation(self):
        w = self.window
        w.identity = dict(uid=self.uid, node=6, serial=123)
        with patch.object(QMessageBox, "warning", return_value=QMessageBox.StandardButton.No):
            w.erase("all")
        self.assertFalse(self.controller.calls)
        with patch.object(QMessageBox, "warning", return_value=QMessageBox.StandardButton.Yes):
            w.erase("app")
        self.assertTrue(self.controller.calls[-1][0]["erase_confirmed"])

    def test_idle_close_waits_for_next_qt_event(self):
        w = self.window
        w.show()
        self.app.processEvents()
        w.close()
        self.assertTrue(w.close_ready)
        self.app.processEvents()
        self.assertFalse(w.isVisible())

    def test_plot_bounded_and_missing_values_render(self):
        plot = SpeedPlot()
        for i in range(3000):
            plot.append(dict(elapsed_s=i / 100, iq_a=0.1, reference_iq_a=None))
        self.assertEqual(len(plot.samples), 1200)
        plot.append(dict(elapsed_s=100, iq_a=0.2))
        self.assertEqual(len(plot.samples), 1)
        plot.resize(600, 300)
        self.assertFalse(plot.grab().isNull())
        plot.deleteLater()


class QtControllerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def controller(self):
        controller = ProductionController(Path("unused.dll"))
        self.addCleanup(controller.deleteLater)
        self.addCleanup(controller.timer.stop)
        return controller

    def attach(self, controller, profile="device_status"):
        endpoint = SimpleNamespace(
            process=SimpleNamespace(is_alive=lambda: True),
            events=queue.Queue(),
            active=True,
            connected=True,
        )
        task = SessionTask(endpoint, 1)
        controller.session = endpoint
        controller.job = task
        controller.events = task
        controller.cancel = threading.Event()
        controller.settings = dict(profile=profile)
        controller.automatic = True
        return endpoint

    def test_manual_request_waits_for_current_auto_read(self):
        c = self.controller()
        endpoint = self.attach(c)
        c.start(dict(profile="hardware_check"), Path("unused"))
        self.assertTrue(c.cancel.is_set())
        endpoint.events.put((1, "done", dict(profile="device_status", passed=True)))
        with patch.object(c, "start") as start:
            c.poll()
        start.assert_called_once_with(
            dict(profile="hardware_check"), Path("unused"), automatic=False
        )

    def test_release_failure_never_launches_waiting_write(self):
        c = self.controller()
        endpoint = self.attach(c, "can_release")
        c.pending = (dict(profile="jlink_flash"), None, False)
        endpoint.events.put((1, "done", dict(profile="can_release", passed=False)))
        with patch.object(c, "start") as start:
            c.poll()
        start.assert_not_called()
        self.assertIsNone(c.pending)

    def test_swd_stop_ignored_and_speed_stop_signals_cancel(self):
        c = self.controller()
        self.attach(c, "jlink_flash")
        c.stop()
        self.assertFalse(c.cancel.is_set())
        c.settings["profile"] = "motor_speed"
        c.stop()
        self.assertTrue(c.cancel.is_set())

    def test_done_message_waits_for_worker_exit(self):
        c = self.controller()
        events = queue.Queue()
        events.close = lambda: None
        events.put(("done", dict(profile="hardware_check", passed=True)))
        c.events = events
        c.settings = dict(profile="hardware_check")
        job = SimpleNamespace(is_alive=lambda: True, join=lambda: None, close=lambda: None)
        c.job = job
        c.poll()
        self.assertIs(c.job, job)
        job.is_alive = lambda: False
        c.poll()
        self.assertIsNone(c.job)
