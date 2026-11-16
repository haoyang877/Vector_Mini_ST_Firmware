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
from PySide6.QtCore import QObject, Signal
from PySide6.QtGui import QColor, QIcon, QPalette, QPixmap
from PySide6.QtWidgets import QApplication, QMessageBox, QPushButton, QTabWidget
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

    def test_input_widths_follow_field_content_across_pages(self):
        w = self.window
        for key, entry in w.edits.items():
            limit = 720 if key == "payload" else 200 if key in ("specimen", "serial") else 120
            self.assertLessEqual(entry.maximumWidth(), limit, key)
        self.assertGreater(w.edits["payload"].maximumWidth(), w.edits["can_id"].maximumWidth())
        for entry in w.parameters.entries.values():
            self.assertLessEqual(entry.maximumWidth(), 180)
        for combo in (w.check_mode, w.test_mode, w.target_data, w.target_nominal):
            self.assertLessEqual(combo.maximumWidth(), 260)

    def test_header_groups_faults_and_equal_measurement_tiles(self):
        w = self.window
        w.show()
        self.app.processEvents()
        self.assertIs(w.axis.parentWidget(), w.channel.parentWidget())
        self.assertLess(w.axis.parentWidget().height(), 190)
        self.assertTrue(w.fault_icon.isHidden())
        self.assertTrue(w.voltage.property("unavailable"))
        self.assertEqual(w.last_read.text(), "等待设备连接")
        self.bind()
        w.current_motor.update(bus_voltage_v=28.35, bus_current_a=0.01, motor_temperature_c=25)
        w.render_device()
        self.app.processEvents()
        tiles = [widget.parentWidget() for widget in (w.voltage, w.current, w.temperature)]
        self.assertLessEqual(max(t.width() for t in tiles) - min(t.width() for t in tiles), 1)
        self.assertFalse(w.voltage.property("unavailable"))
        self.assertFalse(w.fault_icon.isHidden())
        self.assertGreater(w.fault_label.y(), w.mode_label.y())
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
        self.assertEqual(w.maintenance.count(), 4)
        self.assertEqual(w.maintenance.tabText(2), "通信测试")
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

    def test_three_detection_modes_and_blank_validation(self):
        self.bind()
        w = self.window
        for mode, powered, resistance in (
            ("关相静态", False, False),
            ("静态＋相电阻", False, True),
            ("静态和双向驱动", True, False),
        ):
            self.controller.busy = False
            w.check_mode.setCurrentText(mode)
            w.start_hardware()
            request = self.controller.calls[-1][0]
            self.assertEqual(bool(request["drive"]), powered)
            self.assertEqual(request["resistance_only"], resistance)
            self.assertEqual(request["expected_uid"], self.uid)
        self.controller.busy = False
        count = len(self.controller.calls)
        w.edits["travel"].clear()
        w.start_hardware()
        self.assertEqual(len(self.controller.calls), count)
        self.error.assert_called()

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
        self.assertIn("不在本轮范围", self.window.hardware_table.item(9, 3).text())

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
        self.assertEqual(self.window.hardware_table.item(0, 3).text(), "正常")
        self.assertEqual(self.window.hardware_table.item(1, 3).text(), "未测")
        self.assertIn("正在检测：Ib", self.window.phase_label.text())
        self.assertIn("2048", self.window.hardware_table.item(0, 1).text())

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
            for column in range(4):
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
        w.navigation.setCurrentRow(2)
        w.calibration.setCurrentIndex(0)
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
        w.start_calibration(2)
        request = self.controller.calls[-1][0]
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

    def test_busy_disables_nonmotion_controls_but_allows_speed_target(self):
        self.bind()
        self.window.navigation.setCurrentRow(2)
        self.window.calibration.setCurrentIndex(1)
        self.window.start_speed()
        self.assertFalse(self.window.navigation.isEnabled())
        self.assertFalse(self.window.axis.isEnabled())
        self.assertTrue(self.window.edits["speed"].isEnabled())
        self.assertFalse(self.window.edits["speed_limit"].isEnabled())

    def test_next_board_archives_preserves_batch_and_drops_results(self):
        self.bind(7)
        w = self.window
        w.flow.record(dict(profile="jlink_flash", uid=self.uid, success=False), "old")
        w.edits["current"].setText("2")
        w.next_device()
        self.assertIsNone(w.current_motor)
        self.assertFalse(w.flow.devices)
        self.assertEqual(w.node(), 7)
        self.assertEqual(w.edits["current"].text(), "2")
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
