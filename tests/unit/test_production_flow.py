"""生产流程只承认本次UID、完整范围及真实保存完成的证据。"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools/bench"))
from hardware_check import direction_results, empty_report
from production_flow import ProductionFlow


class ProductionFlowTests(unittest.TestCase):
    def hardware(self, uid="a", scope="full"):
        report = empty_report(4)
        report.update(
            uid=uid,
            scope=scope,
            stage=4,
            communication="正常",
            stop_confirmed=True,
            board_items=[1] * 10,
            directions=0x99,
            measurements_supported=True,
            subset_passed=True,
            resistance=dict(
                result=1,
                failure=0,
                nominal_ohm=2,
                phase_ohm=[2] * 3,
                line_ohm=[4] * 3,
                open_phases=0,
                low_phases=0,
                high_phases=0,
                open_lines=0,
            ),
        )
        report["measurements"] = {
            str(i): dict(samples=20, values=[1, 2, None, None]) for i in range(11)
        }
        direction_results(report)
        return report

    def calibration(self, flow, kind, uid="a", passed=True):
        flow.record(
            dict(
                profile="motor_calibration",
                uid=uid,
                kind=kind,
                passed=passed,
                stop_confirmed=passed,
            ),
            "calibration",
        )

    def flash(self, flow, uid="a"):
        flow.record(
            dict(
                profile="jlink_flash",
                uid=uid,
                success=True,
                app_started=True,
                can_test=dict(version_verified=True),
            ),
            "flash",
        )

    def test_full_sequence_and_uid_isolation(self):
        flow = ProductionFlow()
        self.flash(flow)
        flow.record(self.hardware(), "test")
        self.calibration(flow, 1)
        self.calibration(flow, 2)
        self.assertIn("本台流程未完成", flow.summary("a"))
        flow.record(self.hardware(), "recheck")
        self.assertIn("本台流程完成", flow.summary("a"))
        self.assertIn("本台流程未完成", flow.summary("b"))
        self.assertEqual(len(flow.devices["a"]["records"]), 5)

    def test_combined_zero_requires_verified_saved_flag(self):
        flow = ProductionFlow()
        self.calibration(flow, 1)
        self.assertNotIn("zero", flow.devices["a"]["steps"])
        flow.record(
            dict(
                profile="motor_calibration",
                uid="a",
                kind=1,
                passed=True,
                stop_confirmed=True,
                curves=dict(calibration_flags=3),
            ),
            "combined",
        )
        self.assertTrue(flow.devices["a"]["steps"]["zero"])

    def test_partial_test_never_counts_as_complete_hardware(self):
        flow = ProductionFlow()
        for scope in ("static", "static_resistance"):
            flow.record(self.hardware(scope=scope), "partial")
        self.assertNotIn("hardware", flow.devices["a"]["steps"])

    def test_restart_and_failed_attempt_invalidate_old_success(self):
        flow = ProductionFlow()
        self.flash(flow)
        flow.record(self.hardware(), "test")
        self.calibration(flow, 1)
        self.calibration(flow, 2)
        flow.record(self.hardware(), "recheck")
        flow.begin("a", "motor_calibration", kind=1)
        self.assertNotIn("recheck", flow.devices["a"]["steps"])
        self.assertNotIn("zero", flow.devices["a"]["steps"])
        self.calibration(flow, 1, passed=False)
        self.assertIn("角度标定：未通过", flow.summary("a"))
        flow.begin("a", "jlink_flash")
        self.assertFalse(flow.devices["a"]["steps"])

    def test_failed_recheck_overwrites_success_and_parameter_write_invalidates(self):
        flow = ProductionFlow()
        self.flash(flow)
        flow.record(self.hardware(), "test")
        self.calibration(flow, 1)
        self.calibration(flow, 2)
        flow.record(self.hardware(), "recheck")
        report = self.hardware()
        report["communication"] = "异常"
        flow.record(report, "failed")
        self.assertIn("完整复检：未通过", flow.summary("a"))
        flow.record(dict(profile="parameters", uid="a", save_sent=True), "parameters")
        self.assertNotIn("hardware", flow.devices["a"]["steps"])
        self.assertNotIn("recheck", flow.devices["a"]["steps"])

    def test_identify_and_unrelated_actions_do_not_complete_flow(self):
        flow = ProductionFlow()
        flow.begin("a", "device_status")
        flow.record(dict(profile="jlink_flash", uid="a", needs_initial_setup=True), "identify")
        self.assertFalse(flow.devices)
