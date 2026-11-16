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
from PySide6.QtGui import QColor, QPalette
from PySide6.QtWidgets import QApplication, QMessageBox
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
        self.assertIn("无故障（0x00000000）", w.fault_label.text())
        w.current_motor.update(mode=2, state=8, faults=33)
        w.render_device()
        self.assertIn("速度闭环", w.mode_label.text())
        self.assertIn("过流", w.fault_label.text())
        self.assertIn("编码器", w.fault_label.text())
        self.assertIn("0x00000021", w.fault_label.text())
        w.current_motor.update(mode=99, state=99, checked_at=time.time() - 6)
        w.render_device()
        self.assertIn("已过期", w.status_label.text())
        self.assertIn("未知模式", w.mode_label.text())
        self.assertIn("上次读取", w.fault_label.text())

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

    def test_worker_exit_before_hardware_report_remains_visible(self):
        self.bind()
        self.window.on_completed(dict(profile="hardware_check", fatal="进程退出"), None, False)
        self.assertIn("进程退出", self.window.failure.toPlainText())
        self.assertEqual(self.window.hardware_table.rowCount(), 11)

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
