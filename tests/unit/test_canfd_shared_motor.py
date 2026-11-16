"""验证共用电机状态、页面重排和任务参数；不连接硬件。"""

import queue
import sys
import tempfile
import threading
import time
import tkinter as tk
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

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
        dll_patch = patch.object(desktop, "controlcan_dll_path", return_value=Path("mock.dll"))
        dll_patch.start()
        self.addCleanup(dll_patch.stop)

    def capture_start(self, page, **options):
        window = self.window
        window.update_controls()
        window.navigate(page)
        launched = []

        def process(**kwargs):
            job = SimpleNamespace(start=lambda: launched.append(kwargs))
            return job

        context = SimpleNamespace(
            Queue=lambda **_: queue.Queue(), Event=threading.Event, Process=process
        )
        with (
            patch.object(desktop.mp, "get_context", return_value=context),
            patch.object(
                desktop,
                "CanSession",
                return_value=SimpleNamespace(
                    process=SimpleNamespace(is_alive=lambda: True),
                    cancel=threading.Event(),
                    connected=False,
                    submit=lambda settings, directory: (
                        launched.append(dict(args=(settings, directory))),
                        SimpleNamespace(),
                    )[1],
                ),
            ),
            patch.object(desktop.Path, "home", return_value=Path(self.temp.name)),
            patch.object(self.root, "after"),
        ):
            window.start(**options)
        self.assertEqual(len(launched), 1)
        return launched[0]["args"][0]

    def bind(self, node=7):
        self.window.set_current_motor(node, self.uid, "测试识别", role=1)

    def test_install_requires_identity_and_selection_before_page_flash(self):
        w = self.window
        w.hex_path = Path("mock.hex")
        w.update_controls()
        self.assertTrue(w.install_flash.instate(["disabled"]))
        w.identity = dict(node=None, uid=self.uid, serial=123)
        w.update_controls()
        self.assertTrue(w.install_flash.instate(["disabled"]))
        w.initial_axis.set(next(label for label, node in w.axis_choices.items() if node == 5))
        w.select_initial_axis()
        self.assertFalse(w.install_flash.instate(["disabled"]))
        self.assertEqual(self.capture_start("jlink")["first_fit_node"], 5)
        self.assertIsNone(w.install_verified_uid)

    def test_install_navigation_needs_successful_version_readback(self):
        w = self.window
        report = dict(
            profile="jlink_flash",
            success=True,
            app_started=True,
            identity=dict(node=4, uid=self.uid, serial=123),
            steps=[],
            elapsed_s=1,
        )
        w.display_report(report)
        self.assertIsNone(w.install_verified_uid)
        self.assertTrue(w.go_detection.instate(["disabled"]))
        report["can_test"] = dict(
            version_verified=True, device_versions=dict(app=0x01080C, loader=0x020304)
        )
        w.display_report(report)
        w.update_controls()
        self.assertEqual(w.install_verified_uid, self.uid)
        self.assertFalse(w.go_detection.instate(["disabled"]))
        self.assertIn("回读通过", w.install_phase.get())
        w.install_verified_uid = "another-device"
        w.update_controls()
        self.assertTrue(w.go_detection.instate(["disabled"]))

    def test_hardware_phase_progress_and_summary_reset(self):
        w = self.window
        w.show_hardware_phases(
            dict(communication="已完成", static="进行中", resistance="本轮不测", dynamic="本轮不测")
        )
        self.assertEqual(w.progress.get(), "1/3 阶段")
        self.assertIn("进行中", w.hardware_phases["static"][1].get())
        w.hardware_failure.set("上轮失败")
        self.bind()
        self.capture_start("hardware")
        self.assertNotIn("上轮", w.hardware_failure.get())
        self.assertIn("等待本轮结果", w.hardware_failure_label.get("1.0", "end"))

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

    def test_device_summary_uses_observed_versions_and_invalidates_changed_uid(self):
        w = self.window
        w.display_report(
            dict(
                profile="device_status",
                device=dict(
                    node=6,
                    uid=self.uid,
                    role=1,
                    app_version=0x010806,
                    loader_version=0x020304,
                    state=8,
                    mode=0,
                    faults=32,
                    checked_at=1,
                ),
            )
        )
        self.assertIn("APP：v1.8.6", w.device_versions.get())
        self.assertIn("Loader：v2.3.4", w.device_versions.get())
        self.assertIn("编码器", w.device_state.get())
        self.assertEqual(w.operation_mode(status_only=True), "device_status")
        w.display_report(dict(profile="device_status", fatal="设备UID已改变，请重新识别"))
        self.assertIsNone(w.current_motor)
        self.assertEqual(w.device_versions.get(), "APP：—    Loader：—")

    def test_same_node_with_changed_uid_clears_old_curve(self):
        self.bind(6)
        self.window.speed_plot.append(dict(elapsed_s=0, iq_a=0.5))
        self.window.set_current_motor(6, "02" * 12, "新板")
        self.assertFalse(self.window.speed_plot.samples)

    def test_fault_names_display_and_clear_readback_update_identity(self):
        w = self.window
        w.set_current_motor(6, self.uid, "扫描", role=1, state=8, faults=32)
        self.assertIn("编码器故障", w.device_state.get())
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
        self.assertIn("无故障", w.device_state.get())
        self.assertIn("清除前：编码器故障", w.output.get("1.0", "end"))
        self.assertEqual(w.current_motor["faults"], 0)

    def test_hardware_summary_exposes_reverse_direction(self):
        report = desktop.empty_report(6)
        report.update(stage=4, communication="正常", directions=0x96)
        report["board_items"] = [1] * 8 + [2, 1]
        direction_results(report)
        self.window.display_report(report)
        self.assertIn("编码器方向反向", self.window.state.get())
        rows = [
            self.window.hardware_table.item(i, "values")
            for i in self.window.hardware_table.get_children()
        ]
        self.assertEqual(rows[9][0], "电机角动态")
        self.assertEqual(rows[9][3], "异常")
        text = self.window.hardware_result_note.get()
        self.assertIn("正向命令→编码器反转", text)
        self.assertIn("反向命令→编码器正转", text)

    def test_hardware_result_table_has_values_limits_and_missing_states(self):
        report = desktop.empty_report(6)
        report.update(stage=4, communication="正常", measurements_supported=True)
        report["board_items"] = [1] * 8 + [0, 3]
        report["measurements"] = {
            "0": {"samples": 2000, "values": [2050, 4, None, None]},
            "4": {"samples": 2000, "values": [26.99, 28.2, 1, 0]},
        }
        direction_results(report)
        self.window.display_report(report)
        rows = [
            self.window.hardware_table.item(i, "values")
            for i in self.window.hardware_table.get_children()
        ]
        self.assertEqual(len(rows), 11)
        self.assertIn("2050.00", rows[0][1])
        self.assertIn("RMS <10", rows[0][2])
        self.assertIn("26.99", rows[4][1])
        self.assertIn("1/2000", rows[4][1])
        self.assertEqual(rows[9][1], "未采集")
        self.assertEqual(rows[10][1], "未装")
        self.window.show("下一项任务")
        self.assertEqual(self.window.hardware_results.winfo_manager(), "")

    def test_page_order_and_action_routing(self):
        w = self.window
        self.assertEqual(
            [w.tabs.tab(tab, "text") for tab in w.tabs.tabs()],
            ["设备维护", "生产检测", "电机标定", "参数配置", "测试记录"],
        )
        expected = dict(
            jlink="jlink_flash",
            hardware="hardware_check",
            communication="fixed",
            calibration="motor_calibration",
            upgrade="can_upgrade",
            speed="motor_speed",
            parameters="parameters",
        )
        for page, mode in expected.items():
            w.navigate(page)
            self.assertEqual(w.operation_mode(), mode)
            self.assertEqual(w.operation_mode(scan=True), "can_scan")
        w.navigate("communication")
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

    def test_flash_can_repeat_and_displays_queried_versions(self):
        w = self.window
        identity = dict(node=6, uid=self.uid, serial=123, resume_node=6)
        w.display_report(
            dict(
                profile="jlink_flash",
                success=True,
                identity=identity,
                app_started=True,
                protected_regions_verified=True,
                steps=[],
                elapsed_s=1,
                can_test=dict(
                    device_versions=dict(app=0x010804, loader=0x020304), version_verified=True
                ),
            )
        )
        w.hex_path = Path(self.temp.name) / "firmware.hex"
        self.assertEqual(w.identity, dict(node=6, uid=self.uid, serial=123))
        self.assertIn("APP v1.8.4 / Loader v2.3.4", w.firmware_version.get())
        self.assertIn("可直接再次烧录", w.motor_label.get())
        settings = self.capture_start("jlink")
        self.assertEqual(settings["profile"], "jlink_flash")
        self.assertTrue(settings["force_reflash"])
        self.assertTrue(settings["restart_after_flash"])
        self.assertIsNone(settings["first_fit_node"])

    def test_flash_query_failure_keeps_verified_identity_without_fake_version(self):
        w = self.window
        identity = dict(node=6, uid=self.uid, serial=123)
        w.display_report(
            dict(
                profile="jlink_flash",
                success=False,
                identity=identity,
                app_started=True,
                protected_regions_verified=True,
                steps=[],
                elapsed_s=1,
                fatal="CAN timeout",
                can_test=dict(queries=[]),
            )
        )
        self.assertEqual(w.identity, identity)
        self.assertIn("本次未查询到", w.firmware_version.get())
        self.assertEqual(w.operation_mode(), "jlink_flash")

    def test_erase_requires_confirmation_and_uses_bound_identity(self):
        w = self.window
        self.assertTrue(w.erase_buttons["all"].instate(["disabled"]))
        w.identity = dict(node=6, uid=self.uid, serial=123)
        w.update_controls()
        self.assertFalse(w.erase_buttons["app"].instate(["disabled"]))
        with (
            patch.object(desktop.messagebox, "askyesno", return_value=False) as ask,
            patch.object(w, "start") as start,
        ):
            w.confirm_erase("all")
            start.assert_not_called()
            self.assertIn(self.uid, ask.call_args.args[1])
            self.assertIn("256 KiB", ask.call_args.args[1])
            self.assertEqual(ask.call_args.kwargs["default"], "no")
        with (
            patch.object(desktop.messagebox, "askyesno", return_value=True),
            patch.object(w, "start") as start,
        ):
            w.confirm_erase("app")
            start.assert_called_once_with(erase_scope="app", erase_confirmed=True)
        settings = self.capture_start("jlink", erase_scope="app", erase_confirmed=True)
        self.assertEqual(settings["profile"], "jlink_erase")
        self.assertEqual(settings["identity"]["uid"], self.uid)
        self.assertTrue(w.erase_buttons["all"].instate(["disabled"]))

    def test_erase_report_resets_versions_and_keeps_correct_reflash_path(self):
        w = self.window
        for scope in ("app", "all"):
            w.display_report(
                dict(
                    profile="jlink_erase",
                    success=True,
                    identity=dict(node=6, uid=self.uid, serial=123),
                    erase_scope=scope,
                    steps=[],
                )
            )
            self.assertIn("等待重新烧录", w.firmware_version.get())
            self.assertEqual(w.identity["node"], 6 if scope == "app" else None)
            self.assertFalse(w.first_fit_selected)
            if scope == "all":
                self.assertIsNone(w.current_motor)
            else:
                self.assertEqual(w.current_motor["source"], "APP已擦除")

    def test_failed_erase_invalidates_previous_identity(self):
        w = self.window
        w.identity = dict(node=6, uid=self.uid, serial=123)
        self.bind(6)
        w.display_report(
            dict(profile="jlink_erase", success=False, fatal="readback failed", steps=[])
        )
        self.assertIsNone(w.identity)
        self.assertIsNone(w.current_motor)
        self.assertIn("未完成", w.state.get())

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

    def test_speed_page_defaults_follow_motor_axis_without_travel_stop(self):
        w = self.window
        self.assertNotIn("travel", w.motor_fields)
        for node, limit in ((3, 30), (4, 30), (5, 30), (6, 300), (7, 300)):
            self.bind(node)
            self.assertEqual(float(w.motor_fields["limit"].get()), limit)
        settings = self.capture_start("speed")
        self.assertEqual(settings["speed"], 3)
        self.assertEqual(settings["speed_limit"], 300)
        self.assertIsNone(settings["travel"])

    def test_speed_actions_move_and_counters_remain_automatic(self):
        w = self.window
        self.bind(6)
        self.assertEqual(set(w.motor_fields), {"speed", "limit"})
        w.navigate("speed")
        w.update_controls()
        self.assertEqual(w.start_button.winfo_manager(), "")
        self.assertEqual(w.stop_button.winfo_manager(), "")
        self.assertEqual(w.speed_start.winfo_manager(), "pack")
        settings = self.capture_start("speed")
        self.assertEqual((settings["last_lease"], settings["last_target"]), (0, 0))
        w.process = None
        w.update_controls()
        w.navigate("hardware")
        w.update_controls()
        self.assertEqual(w.start_button.winfo_manager(), "pack")

    def test_parameter_task_keeps_ui_responsive_until_done(self):
        self.bind(6)
        w = self.window
        w.navigate("parameters")
        events = queue.Queue()
        events.close = lambda: None
        alive = [True]
        task = SimpleNamespace(is_alive=lambda: alive[0], close=lambda: None, join=lambda: None)
        session = SimpleNamespace(
            process=SimpleNamespace(is_alive=lambda: True),
            cancel=threading.Event(),
            connected=True,
            submit=lambda settings, directory: task,
        )
        with (
            patch.object(desktop, "CanSession", return_value=session),
            patch.object(desktop.Path, "home", return_value=Path(self.temp.name)),
        ):
            w.start(parameter_action=("current", False))
            w.events = events
            w.start(parameter_action=("current", False))
            self.assertIs(w.process, task)
            ticks = []
            self.root.after(1, lambda: ticks.append(True))
            deadline = time.monotonic() + 0.2
            while not ticks and time.monotonic() < deadline:
                self.root.update()
                time.sleep(0.001)
            self.assertTrue(ticks)
            w.stop()
            self.assertTrue(session.cancel.is_set())
            self.assertIs(w.process, task)
            events.put(("done", dict(profile="parameters", fatal="测试停止")))
            alive[0] = False
            deadline = time.monotonic() + 1
            while w.process is not None and time.monotonic() < deadline:
                self.root.update()
                time.sleep(0.001)
            self.assertIsNone(w.process)
            self.assertIn("测试停止", w.parameter_page.message.get())
            w.can_session = None

    def test_refresh_preserves_parameter_draft(self):
        self.bind(6)
        report = dict(
            profile="parameters",
            action="read",
            group="current",
            node=6,
            uid=self.uid,
            passed=True,
            revision=7,
            rows={i: dict(raw=1000, access=3) for i in (11, 12, 13, 14)},
        )
        self.window.display_report(report)
        entry = self.window.parameter_page.entries[11]
        entry.delete(0, "end")
        entry.insert(0, "1.5")
        report["revision"] = 8
        report["rows"][11] = dict(raw=1200, access=3)
        self.window.display_report(report)
        self.assertEqual(entry.get(), "1.5")
        self.assertEqual(self.window.parameter_page.snapshots["current"]["rows"][11]["raw"], 1200)
        self.assertIn("保留", self.window.parameter_page.message.get())

    def test_legacy_task_waits_for_can_release(self):
        self.bind(6)
        w = self.window
        endpoint = SimpleNamespace(
            process=SimpleNamespace(is_alive=lambda: True),
            connected=True,
            active=True,
            cancel=threading.Event(),
            events=queue.Queue(),
        )
        task = desktop.SessionTask(endpoint, 1)
        endpoint.submit = lambda settings, directory: task
        w.can_session = endpoint
        with patch.object(self.root, "after"):
            w.start(identify_only=True)
        self.assertIs(w.process, task)
        self.assertEqual(w.active_profile, "can_release")
        self.assertTrue(w.pending_start["identify_only"])
        endpoint.events.put((1, "done", dict(profile="can_release", passed=True)))
        with patch.object(w, "start") as start:
            w.poll()
            start.assert_called_once_with(
                identify_only=True,
                scan=False,
                bitrate_action=None,
                calibration_kind=1,
                clear_fault=False,
                erase_scope=None,
                erase_confirmed=False,
                status_only=False,
                parameter_action=None,
            )
        w.can_session = None

    def test_hardware_hint_and_upgrade_use_current_device(self):
        self.window.set_current_motor(4, self.uid, "测试", role=1, app_version=0x010808)
        self.assertIn("当前设备 APP v1.8.8", self.window.hardware_hint.get())
        self.assertIn("不支持", self.window.hardware_hint.get())
        self.window.open_current_upgrade()
        self.assertEqual(self.window.current_page(), "upgrade")
        self.assertEqual(self.window.current_node(), 4)
        self.assertFalse(self.window.batch_upgrade.get())
        self.assertIsNone(self.window.process)
        self.window.set_current_motor(4, self.uid, "测试", role=1, app_version=0x01080C)
        self.assertIn("1A/2A", self.window.hardware_hint.get())

    def test_all_parameters_distribute_snapshots_and_keep_draft(self):
        self.bind(6)
        from can_parameters import parameter_ids

        rows = {i: dict(raw=1000, access=3) for i in parameter_ids("all")}
        report = dict(
            profile="parameters",
            action="read",
            group="all",
            node=6,
            uid=self.uid,
            passed=True,
            revision=7,
            rows=rows,
        )
        self.window.display_report(report)
        entry = self.window.parameter_page.entries[11]
        entry.delete(0, "end")
        entry.insert(0, "1.7")
        self.window.display_report(dict(report, revision=8))
        self.assertEqual(entry.get(), "1.7")
        self.assertEqual(len(self.window.parameter_page.snapshots), 4)
        self.assertIn("30", self.window.parameter_page.message.get())
        self.window.display_report(
            dict(profile="parameters", group="all", passed=False, fatal="超时")
        )
        self.assertFalse(self.window.parameter_page.snapshots)

    def test_stale_status_no_longer_looks_normal(self):
        self.window.set_current_motor(
            6, self.uid, "测试", role=1, state=5, mode=0, faults=0, checked_at=time.time() - 6
        )
        self.assertIn("过期", self.window.device_state.get())
        self.assertNotIn("空闲", self.window.device_state.get())

    def test_heartbeat_only_uses_connected_idle_session(self):
        self.bind(6)
        w = self.window
        w.cancel_status_timer()
        submitted = []
        w.can_session = SimpleNamespace(
            connected=True,
            cancel=threading.Event(),
            process=SimpleNamespace(is_alive=lambda: True),
            submit=lambda settings, out: submitted.append((settings, out)) or object(),
        )
        with patch.object(w.root, "after"):
            w.status_heartbeat()
            self.assertEqual(len(submitted), 1)
            self.assertEqual(submitted[0][0]["profile"], "device_status")
            self.assertIsNone(submitted[0][1])
            self.assertTrue(w.auto_status_task)
            w.status_heartbeat()
            self.assertEqual(len(submitted), 1)
            w.process = None
            w.can_session.connected = False
            w.status_heartbeat()
            self.assertEqual(len(submitted), 1)
        w.status_timer = None
        w.can_session = None

    def test_resistance_mode_does_not_require_dynamic_fields(self):
        self.bind(6)
        w = self.window
        w.check_mode.set("静态＋相电阻")
        w.check_fields["travel"].delete(0, "end")
        settings = self.capture_start("hardware")
        self.assertTrue(settings["resistance_only"])
        self.assertIsNone(settings["drive"])

    def test_parameter_rows_distinguish_device_draft_and_persistence(self):
        self.bind(6)
        page = self.window.parameter_page
        rows = {
            i: dict(raw=1000, access=3, revision=7, saved_revision=7, flags=1)
            for i in (11, 12, 13, 14)
        }
        report = dict(
            profile="parameters",
            action="read",
            group="current",
            node=6,
            uid=self.uid,
            passed=True,
            revision=7,
            rows=rows,
        )
        self.window.display_report(report)
        self.assertEqual(page.readbacks[11].get(), "1")
        self.assertEqual(page.value_states[11].get(), "已保存")
        page.entries[11].delete(0, "end")
        page.entries[11].insert(0, "1.5")
        page.update_value_state(11)
        self.assertEqual(page.readbacks[11].get(), "1")
        self.assertEqual(page.value_states[11].get(), "有未写入修改")
        page.entries[11].delete(0, "end")
        page.entries[11].insert(0, "1")
        rows[11].update(flags=3, saved_revision=6)
        page.update_value_state(11)
        self.assertEqual(page.value_states[11].get(), "RAM值 / 保存未确认")

    def test_production_title_measurements_and_adjustable_record(self):
        w = self.window
        self.assertIn("电机生产测试上位机", self.root.title())
        w.set_current_motor(
            6,
            self.uid,
            "测试",
            role=1,
            state=5,
            mode=0,
            faults=0,
            checked_at=time.time(),
            bus_voltage_v=28.12,
            bus_current_a=1.23,
            motor_temperature_c=23.56,
        )
        self.assertIn("28.12 V", w.device_measurements.get())
        self.assertIn("1.230 A", w.device_measurements.get())
        self.assertIn("23.56 °C", w.device_measurements.get())
        w.current_motor["checked_at"] -= 6
        w.update_device_summary()
        self.assertNotIn("28.12", w.device_measurements.get())
        self.assertIn("—", w.device_measurements.get())
        self.assertEqual(len(w.workspace.panes()), 2)
        self.assertEqual(int(w.output.cget("height")), 12)
        self.assertEqual(w.clear_fault_button.master.master.cget("text"), "3. 设备状态")
        self.assertEqual(w.tabs.master, w.task_area)
        self.assertEqual(w.output.master, w.text_record)
        self.assertTrue(w.output.cget("yscrollcommand"))
        w.show("测试记录")
        self.assertEqual(w.text_record.winfo_manager(), "pack")
        self.assertEqual(w.hardware_results.winfo_manager(), "")

    def test_auto_status_does_not_rewrite_selection_controls_or_draft(self):
        self.bind(6)
        w = self.window
        w.auto_status_task = True
        entry = w.parameter_page.entries[11]
        entry.configure(state="normal")
        entry.insert(0, "1.234")
        with (
            patch.object(w, "set_current_motor") as select,
            patch.object(w, "update_controls") as controls,
        ):
            w.display_report(
                dict(
                    profile="device_status",
                    device=dict(
                        node=6,
                        uid=self.uid,
                        role=1,
                        state=5,
                        mode=0,
                        faults=0,
                        checked_at=time.time(),
                        bus_voltage_v=28,
                    ),
                )
            )
        select.assert_not_called()
        controls.assert_not_called()
        self.assertEqual(entry.get(), "1.234")
        self.assertIn("28.00", w.device_measurements.get())

    def test_edit_focus_defers_automatic_query(self):
        self.bind(6)
        w = self.window
        w.cancel_status_timer()
        session = SimpleNamespace(
            connected=True, process=SimpleNamespace(is_alive=lambda: True), submit=Mock()
        )
        w.can_session = session
        with (
            patch.object(w.root, "focus_get", return_value=w.parameter_page.entries[11]),
            patch.object(w.root, "after"),
        ):
            w.status_heartbeat()
        session.submit.assert_not_called()
        w.can_session = None
        w.status_timer = None

    def test_manual_request_waits_for_active_automatic_read(self):
        self.bind(6)
        w = self.window
        w.auto_status_task = True
        w.process = object()
        w.stop_signal = threading.Event()
        w.start(parameter_action=("current", False))
        self.assertEqual(w.pending_start["parameter_action"], ("current", False))
        self.assertTrue(w.stop_signal.is_set())
        w.process = None

    def test_hardware_report_is_exported_only_to_this_job_directory(self):
        w = self.window
        report = desktop.empty_report(6)
        report.update(scope="static", uid=self.uid, stage=4, communication="正常")
        from production_report import export_report

        export_report(report, w.report_dir, tool_version="test")
        report["production_report"] = "production-report.html"
        w.display_report(report)
        self.assertTrue(w.production_report_path.is_file())
        self.assertEqual(w.production_report_path.parent, w.report_dir)
        self.assertTrue((w.report_dir / "production-report.csv").is_file())
        self.assertTrue((w.report_dir / "production-report.json").is_file())
        w.update_controls()
        self.assertFalse(w.production_report_button.instate(["disabled"]))

    def test_maintenance_default_and_nested_routes_keep_all_tools(self):
        w = self.window
        self.assertEqual(w.current_page(), "jlink")
        self.assertEqual(
            [w.maintenance_tabs.tab(t, "text") for t in w.maintenance_tabs.tabs()],
            ["整包烧录", "CAN 固件升级", "通信测试", "高级维护"],
        )
        w.navigate("communication")
        self.assertEqual(w.operation_mode(), "fixed")
        w.communication_tabs.select(w.raw_tab)
        self.assertEqual(w.operation_mode(), "manual")
        w.navigate("advanced")
        self.assertEqual(w.operation_mode(), "none")
        self.assertEqual(w.operation_mode(bitrate_action="read"), "can_bitrate")
        self.assertEqual(w.operation_mode(erase_scope="app"), "jlink_erase")
        self.assertEqual(w.erase_row.master, w.pages["advanced"])
        w.navigate("speed")
        self.assertEqual(w.operation_mode(), "motor_speed")
        w.navigate("records")
        self.assertEqual(w.operation_mode(), "none")
        w.navigate("hardware")
        self.assertIsNone(w.process, "导航不可启动设备操作")

    def test_log_fold_preserves_text_and_detection_is_in_main_page(self):
        w = self.window
        w.show("保留日志")
        w.toggle_log()
        self.assertEqual(len(w.workspace.panes()), 1)
        self.assertIn("保留日志", w.output.get("1.0", "end"))
        w.toggle_log()
        self.assertEqual(len(w.workspace.panes()), 2)
        self.assertEqual(w.hardware_results.master, w.pages["hardware"])

    def test_records_capture_result_without_starting_devices(self):
        w = self.window
        w.display_report(
            dict(profile="parameters", group="current", passed=False, fatal="测试", node=4)
        )
        rows = w.record_table.get_children()
        self.assertEqual(len(rows), 1)
        self.assertEqual(w.record_paths[rows[0]], w.report_dir)
        w.navigate("records")
        self.assertIsNone(w.process)
        w.start()
        self.assertIsNone(w.process)

    def test_parameter_snapshot_edit_and_changed_uid(self):
        w = self.window
        self.bind(6)
        rows = {i: dict(raw=1000, access=3) for i in (11, 12, 13, 14)}
        report = dict(
            profile="parameters",
            action="read",
            group="current",
            node=6,
            uid=self.uid,
            passed=True,
            revision=7,
            rows=rows,
        )
        w.display_report(report)
        entry = w.parameter_page.entries[11]
        entry.delete(0, "end")
        entry.insert(0, "1.5")
        settings = self.capture_start("parameters", parameter_action=("current", True))
        self.assertEqual(settings["changes"], {11: 1500})
        self.assertEqual(settings["snapshot"]["revision"], 7)
        w.process = None
        w.set_current_motor(6, "02" * 12, "换板", role=1)
        self.assertFalse(w.parameter_page.snapshots)
        self.assertEqual(entry.get(), "")

    def test_parameter_adapter_failure_invalidates_snapshot_without_crashing(self):
        self.bind(6)
        self.window.display_report(dict(profile="parameters", fatal="未找到USB CAN"))
        self.assertFalse(self.window.parameter_page.snapshots)
        self.assertIn("未找到USB CAN", self.window.parameter_page.message.get())

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

    def test_flash_without_prior_identification_and_refresh_are_separate_actions(self):
        w = self.window
        w.hex_path = Path(self.temp.name) / "firmware.hex"
        settings = self.capture_start("jlink")
        self.assertEqual(settings["profile"], "jlink_flash")
        self.assertIsNone(settings["identity"])
        self.assertIsNone(settings["first_fit_node"])
        w.process = None
        with patch.object(w, "start") as start:
            w.identify_again()
            start.assert_called_once_with(identify_only=True)
        self.assertEqual(w.operation_mode(identify_only=True), "jlink_identify")

    def test_new_board_shows_unselected_types_then_uses_explicit_choice(self):
        w = self.window
        w.hex_path = Path(self.temp.name) / "firmware.hex"
        w.display_report(
            dict(
                profile="jlink_flash",
                success=True,
                needs_initial_setup=True,
                identity=dict(node=None, uid=self.uid, serial=123),
                steps=[],
                elapsed_s=1,
            )
        )
        w.update_controls()
        self.assertEqual(w.initial_axis.get(), "")
        self.assertTrue(w.start_button.instate(["disabled"]))
        self.assertEqual(w.initial_setup.winfo_manager(), "pack")
        # 点击与顶部当前值相同的选项，也应完成明确选择。
        w.initial_axis.set(w.motor_axis.get())
        w.select_initial_axis()
        self.assertFalse(w.start_button.instate(["disabled"]))
        self.assertIn("初始化并烧录", w.start_button["text"])
        self.assertEqual(self.capture_start("jlink")["first_fit_node"], 6)

    def test_interrupted_initial_setup_locks_original_type(self):
        w = self.window
        w.display_report(
            dict(
                profile="jlink_identify",
                success=True,
                identity=dict(node=None, uid=self.uid, serial=123, resume_node=7),
                steps=[],
                elapsed_s=1,
            )
        )
        w.update_controls()
        self.assertTrue(w.first_fit_selected)
        self.assertEqual(w.current_node(), 7)
        self.assertTrue(all(button.instate(["disabled"]) for button in w.initial_buttons))

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
        w.navigate("hardware")
        w.process = object()
        w.update_controls()
        for key, page in w.sections.items():
            self.assertEqual(
                w.tabs.tab(page, "state"), "normal" if key == "production" else "disabled"
            )


class CurrentMotorIdentityTests(unittest.TestCase):
    def test_replaced_device_rejected_before_motion(self):
        with patch.object(desktop, "Link") as link:
            link.return_value.request.return_value = bytes(32)
            with self.assertRaisesRegex(ValueError, "UID已改变"):
                desktop.check_current_motor(None, dict(node=7, expected_uid="01" * 12))
