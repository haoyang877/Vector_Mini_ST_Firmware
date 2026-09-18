"""Project harness 回归检查；所有用例均保持离线且不接触硬件。"""

import sys as _sys
from pathlib import Path as _Path

_sys.path.insert(0, str(_Path(__file__).resolve().parents[2] / "tools"))
from project_paths import ROOT

_sys.path.insert(0, str(ROOT / "tools/harness"))

import json
import tempfile
import unittest
from unittest.mock import patch

import _style
import check_architecture
import check_docs
import check_hygiene
import doctor


class HarnessTests(unittest.TestCase):
    def test_quick_doctor_never_contacts_hardware(self):
        result = doctor.collect("quick")
        self.assertFalse(result["hardware_contacted"])
        self.assertIn("python", {check["name"] for check in result["checks"]})

    def test_current_architecture_matches_debt_baseline(self):
        _, new, _ = check_architecture.evaluate()
        self.assertEqual(new, [])

    def test_architecture_ratchet_rejects_a_new_violation(self):
        violation = {
            "id": "firmware/new.c -> firmware/app/private.h",
            "from_layer": "motor",
            "to_layer": "app",
            "include": "private.h",
        }
        with tempfile.TemporaryDirectory() as folder:
            debt = _Path(folder) / "debt.json"
            debt.write_text(json.dumps({"schema_version": 1, "violations": []}))
            with (
                patch.object(check_architecture, "DEBT", debt),
                patch.object(check_architecture, "scan", return_value=[violation]),
            ):
                _, new, _ = check_architecture.evaluate()
        self.assertEqual(new, [violation])

    def test_docs_and_hygiene_have_no_unwaived_problem(self):
        self.assertEqual(check_docs.problems(), [])
        self.assertEqual(check_hygiene.problems(), [])

    def test_all_product_modules_share_the_root_style_configuration(self):
        self.assertEqual(
            set(_style.module_names()),
            {"firmware", "loader", "host_app", "shared", "tooling", "templates"},
        )

    def test_checked_templates_have_no_style_problem(self):
        paths = _style.managed_files("templates")
        self.assertTrue(paths)
        self.assertEqual(_style.collect_format_issues(paths), [])
        self.assertEqual(_style.collect_lint_issues(paths), [])

    def test_nontrivial_new_source_requires_a_chinese_comment(self):
        with tempfile.TemporaryDirectory() as folder:
            source = _Path(folder) / "example.py"
            source.write_text("\n".join(["value = 1"] * 12), encoding="utf-8")
            issues = _style._comment_issues(source)
        self.assertEqual(len(issues), 1)
        self.assertEqual(issues[0]["check"], "comment-language")

    def test_style_debt_is_invalid_after_file_content_changes(self):
        source = ROOT / "templates/python_tool.py"
        name = source.relative_to(ROOT).as_posix()
        valid = {
            "schema_version": 1,
            "files": {name: {"sha256": _style.digest(source), "checks": ["python-format"]}},
        }
        changed = {
            "schema_version": 1,
            "files": {name: {"sha256": "0" * 64, "checks": ["python-format"]}},
        }
        self.assertTrue(_style.debt_allows(source, "python-format", valid))
        self.assertFalse(_style.debt_allows(source, "python-format", changed))


if __name__ == "__main__":
    unittest.main()
