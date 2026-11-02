"""统一台架协议、主机准入及报告的离线回归。"""

import argparse
import hashlib
import json
import struct
import tempfile
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

import numpy as np

from host_app.upgrade_client import encode_frames
from tools.bench.unified_motor_test import (
    Client,
    assess_record,
    evaluate,
    plan_template,
    render_report,
    run_plan,
    save_json,
    validate_plan,
    write_samples,
)

ROOT = Path(__file__).resolve().parents[2]


class BenchProtocolTests(unittest.TestCase):
    def test_golden_requests(self):
        """线路头、负载与两层CRC保持确定，未调用设备端口。"""
        vectors = []
        for version in ("v1", "v1_1"):
            data = json.loads(
                (ROOT / f"docs/protocols/unified_motor_bench_{version}_vectors.json").read_text(
                    encoding="utf-8"
                )
            )
            vectors.extend(data["vectors"])
        for vector in vectors:
            with self.subTest(vector["name"]):
                frames = encode_frames(3, 0x7F00, bytes.fromhex(vector["request_payload"]), 1)
                self.assertEqual(len(frames), 1)
                self.assertEqual(frames[0][0], 0x0CEF0302)
                self.assertEqual(frames[0][1].hex(), vector["request_frame"])

    def test_unreviewed_plan_cannot_open_hardware(self):
        """默认模板即使提供文件也不得通过执行准入。"""
        with self.assertRaisesRegex(ValueError, "尚未批准"):
            validate_plan(plan_template(), Path("not-opened.bin"))

    def test_pulse_deadline_is_required(self):
        """主机也拒绝超过5ms的Iq脉冲，不能只依靠板端拒绝。"""
        with tempfile.TemporaryDirectory() as temporary:
            firmware = Path(temporary) / "firmware.bin"
            firmware.write_bytes(b"offline-fixture")
            plan = plan_template()
            plan.update(
                reviewed=True,
                expected_info_hex="00" * 32,
                test_speed_rad_s=1.0,
                firmware_sha256=hashlib.sha256(firmware.read_bytes()).hexdigest(),
            )
            plan["limits"] = {
                "current_a": 1.0,
                "speed_rad_s": 5.0,
                "travel_rad": 10.0,
                "duration_ms": 30000,
                "lease_ms": 500,
            }
            plan["steps"] = [
                {
                    "id": "pulse",
                    "case": "TC04",
                    "group": "dq",
                    "stride": 1,
                    "drive": {"kind": "iq_pulse", "fraction": 0.1, "duration_ms": 6},
                }
            ]
            with self.assertRaisesRegex(ValueError, "5ms"):
                validate_plan(plan, firmware)

    def test_timeout_does_not_repeat_action(self):
        """应答丢失只发送一次RUN，防止完成后重触发。"""

        class SilentBus:
            def __init__(self):
                self.sent = []

            def send(self, identifier, data):
                self.sent.append((identifier, data))

            def receive(self, timeout_s):
                return None

        with tempfile.TemporaryDirectory() as temporary:
            bus = SilentBus()
            client = Client(bus, 3, Path(temporary) / "frames.jsonl")
            with self.assertRaises(TimeoutError):
                client.request(0x7F00, struct.pack("<HH", 2, 1), timeout=0)
            self.assertEqual(len(bus.sent), 1)


class BenchReportTests(unittest.TestCase):
    """合成数据只用于离线软件回归，不作为实测输出。"""

    def setUp(self):
        self.plan = plan_template()
        self.plan["limits"]["current_a"] = 1.0
        self.step = {"id": "TC04_id", "case": "TC04", "group": "dq", "drive": {"kind": "id"}}
        self.rows = np.zeros((128, 8))
        self.rows[:, 0] = np.arange(128)
        self.rows[32:, 1] = 0.1
        self.rows[32:, 3] = 0.1 * (1 - np.exp(-np.arange(96) / 3))

    def test_step_metrics_use_board_sample_time(self):
        status, metrics, _ = evaluate(self.step, self.rows, self.plan)
        self.assertEqual(status, "PASS")
        self.assertLess(metrics["rise_ms"], 1.0)
        self.assertLess(metrics["settle_ms"], 3.0)

    def test_negative_step_and_missing_settle_window(self):
        step = {**self.step, "drive": {"kind": "iq_pulse"}}
        rows = np.zeros_like(self.rows)
        rows[:, 0] = self.rows[:, 0]
        rows[:, 2] = -self.rows[:, 1]
        rows[:, 4] = -self.rows[:, 3]
        self.assertEqual(evaluate(step, rows, self.plan)[0], "PASS")
        self.assertEqual(evaluate(self.step, self.rows[:40], self.plan)[0], "BLOCKED")

    def test_invalid_data_and_insufficient_cycles_are_blocked(self):
        invalid = self.rows.copy()
        invalid[50, 3] = np.nan
        self.assertEqual(evaluate(self.step, invalid, self.plan)[0], "BLOCKED")
        phase = np.zeros_like(self.rows)
        phase[:, 0] = self.rows[:, 0]
        phase[:, 1:4] = 2048
        phase[:, 7] = np.linspace(0, 2 * np.pi, 128)
        step = {"case": "TC01", "group": "phase", "drive": {"kind": "speed"}}
        self.assertEqual(evaluate(step, phase, self.plan)[0], "BLOCKED")

    def test_flat_adc_rail_is_not_accepted_as_low_noise(self):
        rows = np.zeros_like(self.rows)
        rows[:, 0] = self.rows[:, 0]
        rows[:, 1:4] = 2048
        step = {"case": "TC01", "group": "phase"}
        self.assertEqual(evaluate(step, rows, self.plan)[0], "PASS")
        rows[:, 1] = 4095
        self.assertEqual(evaluate(step, rows, self.plan)[0], "FAIL")

    def test_circular_angle_origin_alignment(self):
        rows = np.zeros_like(self.rows)
        rows[:, 0] = np.arange(128) * 100
        theta = np.linspace(0, 7 * np.pi, 128)
        rows[:, 3] = (theta % (2 * np.pi)) * 65536 / (2 * np.pi)
        rows[:, 4] = theta - 0.7
        status, metrics, _ = evaluate({"case": "TC06", "group": "angle"}, rows, self.plan)
        self.assertEqual(status, "PASS")
        self.assertLess(metrics["rmse"], 1e-8)

    def test_calibration_requires_real_completion_not_existing_flags(self):
        """已有标志或超时停止不能替代完成；对齐段不当作标定后精度。"""
        with tempfile.TemporaryDirectory() as temporary:
            raw = Path(temporary)
            step = {"id": "lut", "case": "TC06", "group": "angle", "drive": {"kind": "lut"}}
            write_samples(raw / "lut.csv", "angle", list(self.rows))
            info = {
                "run_kind": 4,
                "run_state": 3,
                "reason": 0,
                "fault": 0,
                "calibration_flags": 1,
                "invalid": 0,
            }
            record = {"status": "RECORDED", "step": step, "info": info}
            self.assertEqual(assess_record(record, raw, self.plan)[0], "PASS")
            info["reason"] = 5
            self.assertEqual(assess_record(record, raw, self.plan)[0], "FAIL")
            info["reason"] = 0
            info["calibration_flags"] = 0
            self.assertEqual(assess_record(record, raw, self.plan)[0], "FAIL")
            info["calibration_flags"] = 1
            info["invalid"] = 1
            self.assertEqual(assess_record(record, raw, self.plan)[0], "FAIL")
            info.update(invalid=0, run_kind=5)
            step["drive"]["kind"] = "zero"
            self.assertEqual(assess_record(record, raw, self.plan)[0], "FAIL")
            info["calibration_flags"] = 3
            self.assertEqual(assess_record(record, raw, self.plan)[0], "PASS")

    def test_report_preserves_failure_and_marks_unrun_cases(self):
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)
            raw = output / "raw"
            raw.mkdir()
            save_json(output / "plan.json", self.plan)
            write_samples(raw / "TC04_id.csv", "dq", list(self.rows))
            save_json(
                raw / "TC04_id.json",
                {
                    "step": self.step,
                    "status": "FAIL",
                    "final": {"fault": 0, "invalid": 0},
                },
            )
            render_report(output)
            results = json.loads((output / "results.json").read_text(encoding="utf-8"))
            self.assertEqual(results["TC04"]["status"], "FAIL")
            self.assertEqual(results["TC01"]["status"], "NOT_RUN")
            self.assertTrue((output / "plots/TC04_id.png").is_file())
            self.assertIn("TC04_id / FAIL", (output / "report.html").read_text(encoding="utf-8"))

    def test_missing_scope_cannot_be_accepted_as_full_pass(self):
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)
            raw = output / "raw"
            raw.mkdir()
            save_json(output / "plan.json", self.plan)
            save_json(
                raw / "TC07_negative.json",
                {
                    "step": {"id": "TC07_negative", "case": "TC07", "operation": "protocol"},
                    "status": "RECORDED",
                    "checks": [],
                    "scope": "synthetic software fixture",
                },
            )
            render_report(output)
            results = json.loads((output / "results.json").read_text(encoding="utf-8"))
            self.assertEqual(results["TC07"]["status"], "BLOCKED")
            self.assertIn("TC07_lifecycle", results["TC07"]["missing_steps"])

    def test_failed_metric_stops_before_next_excitation(self):
        """板端正常结束但指标失败，也必须停止后续激励；全程使用内存端口。"""
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            firmware, dll = root / "image.bin", root / "sdk.bin"
            firmware.write_bytes(b"offline firmware")
            dll.write_bytes(b"offline sdk")
            plan = self.plan.copy()
            plan.update(
                reviewed=True,
                firmware_sha256=hashlib.sha256(firmware.read_bytes()).hexdigest(),
                expected_info_hex="00" * 32,
                test_speed_rad_s=0.5,
            )
            plan["limits"] = {
                "current_a": 1.0,
                "speed_rad_s": 2.0,
                "travel_rad": 1.0,
                "duration_ms": 1000,
                "lease_ms": 500,
            }
            plan["steps"] = [
                {
                    "id": name,
                    "case": "TC04",
                    "group": "dq",
                    "stride": 1,
                    "drive": {"kind": "id", "fraction": 0.1, "duration_ms": 200},
                }
                for name in ("first", "must_not_run")
            ]
            plan_path = root / "plan.json"
            save_json(plan_path, plan)
            slow = self.rows.copy()
            slow[32:, 3] = 0.1 * (1 - np.exp(-np.arange(96) / 20))

            def capture(client, selected_plan, step, directory):
                write_samples(directory / (step["id"] + ".csv"), "dq", list(slow))
                return {"step": step, "status": "RECORDED", "final": {"fault": 0, "invalid": 0}}

            client = MagicMock()
            client.request.return_value = bytes(32)
            client.info.return_value = {"flags": 0}
            with (
                patch("tools.bench.unified_motor_test.ControlCanFd"),
                patch("tools.bench.unified_motor_test.Client", return_value=client),
                patch("tools.bench.unified_motor_test.acquire", side_effect=capture) as calls,
                patch("tools.bench.unified_motor_test.render_report"),
            ):
                with self.assertRaisesRegex(RuntimeError, "指标"):
                    run_plan(
                        argparse.Namespace(
                            execute=True,
                            plan=plan_path,
                            firmware=firmware,
                            dll=dll,
                            node=1,
                            device_index=0,
                            output=root / "run",
                        )
                    )
                self.assertEqual(calls.call_count, 1)
                self.assertTrue(client.stop.called)
                manifest = json.loads((root / "run/manifest.json").read_text(encoding="utf-8"))
                self.assertEqual(manifest["status"], "FAILED")


if __name__ == "__main__":
    unittest.main()
