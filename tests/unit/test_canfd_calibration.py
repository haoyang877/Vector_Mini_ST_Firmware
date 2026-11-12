"""USB-CAN 标定页离线验收，模拟设备应答及工装零位确认。"""

import json
import struct
import sys
import tempfile
import unittest
from pathlib import Path
from queue import SimpleQueue
from threading import Event
from unittest.mock import MagicMock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools/bench"))
import canfd_desktop as desktop
from motor_calibration_can import JOB, STATUS


class CalibrationDesktopTests(unittest.TestCase):
    def setUp(self):
        self.settings = dict(profile="calibration", node=6, uid="ab" * 12, kinds=[1, 2, 6], bus={})
        self.zero_ready = Event()
        self.stop = Event()
        self.zero_ready.set()
        self.kinds = []
        self.calls = []
        self.fail_kind = None
        self.cancel_kind = None
        self.faults = 0
        self.lookup = self.enterContext(
            patch.object(
                desktop, "read_node_uids", return_value={bytes.fromhex(self.settings["uid"])}
            )
        )
        self.enterContext(patch.object(desktop, "ControlCanFd"))
        self.enterContext(patch.object(desktop.os, "add_dll_directory", create=True))
        self.enterContext(
            patch.object(desktop, "controlcan_dll_path", return_value=Path("test.dll"))
        )
        self.enterContext(patch("time.sleep"))
        self.client = self.enterContext(patch.object(desktop, "UpgradeClient")).return_value
        self.client.node = 6
        self.client.request.side_effect = self.reply

    def reply(self, command, payload=b""):
        self.calls.append(command)
        if command == 2:
            return struct.pack("<4I", 1 << 15, 0, 0, 0)
        if command == 108:
            return STATUS.pack(
                0, 0, self.faults, *([0] * 4), 0, *([0] * 8), 6, 8 if self.faults else 5, 0
            )
        if command == 110:
            return bytes(4)
        if command == 160:
            kind, reserved = struct.unpack("<HH", payload)
            self.assertEqual(reserved, 0)
            self.kinds.append(kind)
            if kind == self.cancel_kind:
                self.stop.set()
            return JOB.pack(1, 0, len(self.kinds), kind, 1, 1)
        kind = self.kinds[-1]
        self.assertEqual(struct.unpack("<I", payload), (len(self.kinds),))
        if command == 162:
            return JOB.pack(0, 0, len(self.kinds), kind, 3, 1)
        self.assertEqual(command, 161)
        if kind == self.cancel_kind:
            return JOB.pack(0, 0, len(self.kinds), kind, 4, 1)
        if kind == self.fail_kind:
            self.faults = 16
            return JOB.pack(0, 8, len(self.kinds), kind, 5, 2)
        return JOB.pack(0, 0, len(self.kinds), kind, 2, 2)

    def execute(self):
        with tempfile.TemporaryDirectory() as tmp:
            events = SimpleQueue()
            desktop.acquire(self.settings, Path(tmp), events, self.stop, self.zero_ready)
            report = json.loads((Path(tmp) / "summary.json").read_text(encoding="utf8"))
            while True:
                tag, data = events.get_nowait()
                if tag == "done":
                    self.assertEqual(data, report)
                    return report

    def test_only_selected_jobs_start_in_order(self):
        for kinds in ([1, 2, 6], [2, 6], [6]):
            with self.subTest(kinds=kinds):
                self.kinds.clear()
                self.settings["kinds"] = kinds
                report = self.execute()
                self.assertEqual(self.kinds, kinds)
                self.assertEqual(report["status"], "全部通过")
                self.assertTrue(
                    all(r["status"] == "已通过" and r["ticket"] for r in report["results"])
                )

    def test_failed_flash_keeps_prior_success_and_stops_next_job(self):
        self.fail_kind = 2
        report = self.execute()
        self.assertEqual(self.kinds, [1, 2])
        self.assertEqual(report["status"], "标定异常")
        self.assertEqual([r["status"] for r in report["results"]], ["已通过", "标定异常"])
        self.assertIn("Flash 保存失败", report["results"][1]["reason"])
        self.assertEqual(report["results"][1]["faults"], 16)

    def test_stop_cancels_active_ticket_and_confirms_terminal(self):
        self.cancel_kind = 2
        report = self.execute()
        self.assertEqual(self.kinds, [1, 2])
        self.assertEqual(report["results"][-1]["status"], "已取消")
        self.assertEqual(self.calls[-4:], [162, 161, 110, 108])

    def test_wait_for_mechanical_reference_can_be_stopped_before_start(self):
        self.settings["kinds"] = [6]
        self.zero_ready = MagicMock()

        def wait(_):
            self.stop.set()
            return False

        self.zero_ready.wait.side_effect = wait
        report = self.execute()
        self.assertEqual(report["status"], "已停止")
        self.assertFalse(self.calls)

    def test_identity_fault_and_missing_capability_prevent_motion(self):
        self.lookup.return_value = set()
        self.assertIn("UID", self.execute()["results"][0]["reason"])
        self.lookup.return_value = {bytes.fromhex(self.settings["uid"])}
        self.faults = 64
        self.assertEqual(self.execute()["results"][0]["faults"], 64)
        self.client.request.side_effect = (
            lambda cmd, payload=b"": bytes(16) if cmd == 2 else bytes(45)
        )
        self.assertIn("标定票据能力", self.execute()["results"][0]["reason"])
        self.assertFalse(self.kinds)

    def test_timed_out_start_already_terminal_cannot_pass_as_new(self):
        self.client.trace = [{"attempt": 1}]

        def reply(command, payload=b""):
            if command == 160:
                self.kinds.append(1)
                return JOB.pack(0, 0, 9, 1, 2, 2)
            return self.reply(command, payload)

        self.client.request.side_effect = reply
        report = self.execute()
        self.assertEqual(report["status"], "标定异常")
        self.assertIn("无法确认本次标定", report["results"][0]["reason"])
        self.assertEqual(self.kinds, [1])

    def test_gui_selection_and_fault_message(self):
        try:
            window = desktop.tk.Tk()
        except desktop.tk.TclError as exc:
            self.skipTest(str(exc))
        window.withdraw()
        self.addCleanup(window.destroy)
        ui = desktop.CanTestWindow(window)
        ui.tabs.select(4)
        ui.update_controls()
        self.assertTrue(ui.start_button.instate(["disabled"]))
        ui.display_report(
            dict(
                profile="calibration_scan", motors=[dict(node=6, role=1, uid=self.settings["uid"])]
            )
        )
        ui.cal_kinds[2].set(True)
        ui.update_controls()
        self.assertFalse(ui.start_button.instate(["disabled"]))
        with (
            tempfile.TemporaryDirectory() as tmp,
            patch.object(desktop.Path, "home", return_value=Path(tmp)),
            patch.object(desktop.mp, "get_context") as ctx,
        ):
            ui.start()
            job = ctx.return_value.Process.call_args.kwargs
            self.assertIs(job["target"], desktop.acquire)
            self.assertEqual(job["args"][0]["kinds"], [2])
            self.assertEqual(job["args"][0]["uid"], self.settings["uid"])
        self.fail_kind = 2
        ui.display_report(self.execute())
        self.assertEqual(ui.state.get(), "标定异常")
        self.assertIn("Flash 保存失败", ui.cal_error.cget("text"))
        self.assertIn("机械零位 · 未执行", ui.output.get("1.0", "end"))
