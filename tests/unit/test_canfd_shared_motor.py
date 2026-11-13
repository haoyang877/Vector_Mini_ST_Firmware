"""验证共用电机状态、页面重排和任务参数；不连接硬件。"""

import queue
import sys
import tempfile
import threading
import tkinter as tk
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools/bench"))
import canfd_desktop as desktop
from hardware_check import direction_results


class SharedMotorTests(unittest.TestCase):
    def setUp(self):
        try:
            self.root = tk.Tk()
        except tk.TclError as exc:
            self.skipTest(f"本环境未提供Tk显示：{exc}")
        self.root.withdraw()
        self.addCleanup(self.root.destroy)
        self.window = desktop.CanTestWindow(self.root)
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.window.report_dir = Path(self.temp.name)
        self.uid = "01" * 12

    def capture_start(self, page, **options):
        window = self.window
        window.update_controls()
        window.tabs.select(window.pages[page])
        launched = []

        def process(**kwargs):
            job = SimpleNamespace(start=lambda: launched.append(kwargs))
            return job

        context = SimpleNamespace(
            Queue=lambda **_: queue.Queue(), Event=threading.Event, Process=process
        )
        with (
            patch.object(desktop.mp, "get_context", return_value=context),
            patch.object(desktop.Path, "home", return_value=Path(self.temp.name)),
            patch.object(self.root, "after"),
        ):
            window.start(**options)
        self.assertEqual(len(launched), 1)
        return launched[0]["args"][0]

    def bind(self, node=7):
        self.window.set_current_motor(node, self.uid, "测试识别", role=1)

    def test_fault_clear_uses_current_motor_from_any_page(self):
        w = self.window
        self.assertTrue(w.clear_fault_button.instate(["disabled"]))
        self.bind(6)
        self.assertFalse(w.clear_fault_button.instate(["disabled"]))
        settings = self.capture_start("calibration", clear_fault=True)
        self.assertEqual(settings["profile"], "clear_fault")
        self.assertEqual(settings["node"], 6)
        self.assertEqual(settings["expected_uid"], self.uid)
        self.assertTrue(w.clear_fault_button.instate(["disabled"]))

    def test_fault_names_display_and_clear_readback_update_identity(self):
        w = self.window
        w.set_current_motor(6, self.uid, "扫描", role=1, state=8, faults=32)
        self.assertIn("编码器故障", w.current_motor_label.get())
        w.display_report(
            dict(
                profile="clear_fault",
                node=6,
                uid=self.uid,
                passed=True,
                cleared=True,
                before=dict(state=8, mode=0, faults=32),
                after=dict(state=5, mode=0, faults=0),
                result=0,
                detail=0,
            )
        )
        self.assertIn("无故障", w.current_motor_label.get())
        self.assertIn("清除前：编码器故障", w.output.get("1.0", "end"))
        self.assertEqual(w.current_motor["faults"], 0)

    def test_hardware_summary_exposes_reverse_direction(self):
        report = desktop.empty_report(6)
        report.update(stage=4, communication="正常", directions=0x96)
        report["board_items"] = [1] * 8 + [2, 1]
        direction_results(report)
        self.window.display_report(report)
        self.assertIn("编码器方向反向", self.window.state.get())
        text = self.window.output.get("1.0", "end")
        self.assertIn("电机角动态　异常", text)
        self.assertIn("正向命令→编码器反转", text)
        self.assertIn("反向命令→编码器正转", text)

    def test_page_order_and_action_routing(self):
        w = self.window
        self.assertEqual(
            [w.tabs.tab(tab, "text") for tab in w.tabs.tabs()],
            ["J-Link 烧录", "硬件测试", "通信测试", "角度标定", "升级压测", "速度控制"],
        )
        expected = dict(
            jlink="jlink_identify",
            hardware="hardware_check",
            communication="fixed",
            calibration="motor_calibration",
            upgrade="can_upgrade",
            speed="motor_speed",
        )
        for page, mode in expected.items():
            w.tabs.select(w.pages[page])
            self.assertEqual(w.operation_mode(), mode)
            self.assertEqual(w.operation_mode(scan=True), "can_scan")
        w.tabs.select(w.pages["communication"])
        w.communication_tabs.select(w.raw_tab)
        self.assertEqual(w.operation_mode(), "manual")

    def test_flash_result_updates_single_motor_for_all_pages(self):
        w = self.window
        w.display_report(
            dict(
                profile="jlink_flash",
                success=True,
                identity=dict(node=7, uid=self.uid, serial=1),
                app_started=True,
                steps=[],
                elapsed_s=1,
            )
        )
        self.assertEqual(w.current_node(), 7)
        self.assertEqual(w.current_motor["uid"], self.uid)
        self.assertIn("YAW", w.motor_axis.get())
        for obsolete in (
            "check_node",
            "calibration_node",
            "speed_node",
            "archive_type",
            "new_axis",
        ):
            self.assertFalse(hasattr(w, obsolete))
        self.assertEqual(w.selected_upgrade_motors()[0]["node"], 7)

    def test_motion_and_calibration_use_shared_uid_and_axis(self):
        for page, profile in (
            ("hardware", "hardware_check"),
            ("calibration", "motor_calibration"),
            ("speed", "motor_speed"),
        ):
            with self.subTest(page=page):
                self.bind()
                settings = self.capture_start(page)
                self.assertEqual((settings["node"], settings["expected_uid"]), (7, self.uid))
                self.assertEqual(settings["profile"], profile)
                if page == "calibration":
                    self.assertEqual(settings["archive"]["type"], "Y轴电机")
                self.window.process = None
                self.window.commands = None

    def test_query_and_single_upgrade_need_no_page_selection(self):
        self.bind()
        settings = self.capture_start("communication")
        self.assertEqual(settings["nodes"], [7])
        self.assertEqual(settings["node_uids"], {7: self.uid})
        self.window.process = None
        settings = self.capture_start("upgrade")
        self.assertEqual(settings["targets"], [{"node": 7, "uid": self.uid}])

    def test_switching_axis_invalidates_old_swd_identity(self):
        self.bind()
        w = self.window
        w.identity = dict(node=7, uid=self.uid)
        w.motor_axis.set(next(label for label, n in w.axis_choices.items() if n == 6))
        self.assertIsNone(w.current_motor)
        self.assertIsNone(w.identity)
        self.assertIn("尚未识别", w.current_motor_label.get())
        self.assertIn("06", w.identifier.get())

    def test_blank_board_requires_one_explicit_global_type_choice(self):
        w = self.window
        w.display_report(
            dict(
                profile="jlink_identify",
                success=True,
                identity=dict(node=None, uid=self.uid, serial=1),
                steps=[],
                elapsed_s=1,
            )
        )
        self.assertFalse(w.first_fit_selected)
        w.motor_axis.set(next(label for label, n in w.axis_choices.items() if n == 7))
        w.select_first_fit()
        self.assertTrue(w.first_fit_selected)
        self.assertEqual(self.capture_start("jlink")["first_fit_node"], 7)

    def test_single_scan_auto_selects_axis_and_failed_identification_clears_it(self):
        w = self.window
        motor = dict(
            node=7,
            uid=self.uid,
            role=1,
            boot_state=1,
            state=5,
            faults=0,
            app_version=0x010804,
            loader_version=0x020304,
            model="YQ-5708",
        )
        w.display_report(dict(profile="can_scan", motors=[motor], issues=[], cancelled=False))
        self.assertEqual(w.current_node(), 7)
        self.assertEqual(w.current_motor["uid"], self.uid)
        w.display_report(
            dict(profile="jlink_identify", success=False, fatal="连接失败", steps=[], elapsed_s=1)
        )
        self.assertIsNone(w.current_motor)

    def test_batch_features_remain_explicit(self):
        w = self.window
        self.bind()
        w.batch_query.set(True)
        for n, selected in w.nodes.items():
            selected.set(n in (3, 7))
        self.assertEqual(self.capture_start("communication")["nodes"], [3, 7])
        w.process = None
        w.upgrade_targets = {3: dict(node=3, uid="02" * 12, role=1), 7: w.current_motor}
        w.batch_upgrade.set(True)
        w.upgrade_selected[3].set(True)
        w.upgrade_selected[7].set(True)
        self.assertEqual([m["node"] for m in self.capture_start("upgrade")["targets"]], [3, 7])

    def test_busy_state_keeps_only_active_page_enabled(self):
        w = self.window
        w.tabs.select(w.pages["hardware"])
        w.process = object()
        w.update_controls()
        for key, page in w.pages.items():
            self.assertEqual(
                w.tabs.tab(page, "state"), "normal" if key == "hardware" else "disabled"
            )


class CurrentMotorIdentityTests(unittest.TestCase):
    def test_replaced_device_rejected_before_motion(self):
        with patch.object(desktop, "Link") as link:
            link.return_value.request.return_value = bytes(32)
            with self.assertRaisesRegex(ValueError, "UID已改变"):
                desktop.check_current_motor(None, dict(node=7, expected_uid="01" * 12))
