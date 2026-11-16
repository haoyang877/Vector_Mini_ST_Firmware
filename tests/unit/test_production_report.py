"""生产报告保留轮次、原判据和范围，未测不得误判通过。"""

import csv
import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools/bench"))
from hardware_check import direction_results, empty_report
from production_report import build_report, export_report, report_rows


class ReportTests(unittest.TestCase):
    def report(self):
        report = empty_report(4)
        report.update(
            uid="0123456789abcdef01234567",
            app_version=0x01080C,
            loader_version=0x020304,
            scope="static",
            stage=4,
            communication="正常",
            board_items=[1] * 8 + [0, 0],
            run_id=12,
            measurements_supported=True,
        )
        report["measurements"] = {
            str(i): dict(samples=20, values=[1, 2, None, None]) for i in range(8)
        }
        direction_results(report)
        return report

    def test_scope_and_incomplete_results_never_claim_full_pass(self):
        report = self.report()
        data = build_report(report, tool_version="1.20.0", created_at="test")
        self.assertEqual(data["verdict"], "本范围通过")
        self.assertEqual(data["scope_name"], "关相静态")
        self.assertEqual(data["rows"][-1]["result"], "不在本轮范围")
        report["scope"] = "full"
        self.assertEqual(
            build_report(report, tool_version="x", created_at="x")["verdict"], "未通过 / 未完成"
        )
        report["scope"] = "static"
        report["board_items"][0] = 0
        direction_results(report)
        self.assertEqual(
            build_report(report, tool_version="x", created_at="x")["verdict"], "未通过 / 未完成"
        )

    def test_no_measurements_cannot_claim_production_pass(self):
        report = self.report()
        report["measurements"] = {}
        self.assertEqual(
            build_report(report, tool_version="x", created_at="x")["verdict"], "未通过 / 未完成"
        )

    def test_failure_keeps_measured_value_limits_and_suggestion(self):
        report = self.report()
        report["board_items"][4] = 2
        report["measurements"] = {"4": dict(samples=20, values=[26.8, 28, 1, 0])}
        direction_results(report)
        row = next(row for row in report_rows(report) if row["item"] == "母线电压")
        self.assertTrue(row["attention"])
        self.assertIn("26.80", row["measured"])
        self.assertIn("27～29", row["limit"])
        self.assertIn("电源", row["suggestion"])

    def test_exports_same_uid_and_rows_escapes_html(self):
        report = self.report()
        report["note"] = "<script>alert(1)</script>"
        with tempfile.TemporaryDirectory() as folder:
            path = export_report(report, Path(folder), tool_version="1.20.0")
            text = path.read_text(encoding="utf-8")
            self.assertNotIn("<script>", text)
            self.assertIn("&lt;script&gt;", text)
            data = json.loads(path.with_suffix(".json").read_text(encoding="utf-8"))
            with path.with_suffix(".csv").open(encoding="utf-8-sig", newline="") as stream:
                rows = list(csv.reader(stream))
            self.assertEqual(len(rows), 12)
            self.assertEqual(rows[1][0], report["uid"])
            self.assertEqual(data["raw"]["run_id"], 12)
            self.assertEqual(data["verdict"], "未通过 / 未完成")
