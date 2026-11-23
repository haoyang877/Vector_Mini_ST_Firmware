"""桌面检测输入在访问设备前校验，覆盖空白行程及静态/双向的不同要求。"""

import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools/bench"))
from hardware_check import reserve_counters
from production_backend import CHECK_DEFAULTS, hardware_check_fields


class HardwareCheckFieldsTests(unittest.TestCase):
    def test_default_mode_has_complete_bench_drive_settings(self):
        self.assertEqual(CHECK_DEFAULTS["node"], "6")
        self.assertEqual(CHECK_DEFAULTS["mode"], "静态和双向驱动")
        self.assertEqual(
            hardware_check_fields(CHECK_DEFAULTS, True),
            dict(drive=(3, 3, 1500, 360), ambient=20, last_lease=0, last_target=0),
        )

    def test_zero_defaults_do_not_reset_saved_uid_counters(self):
        settings = hardware_check_fields(CHECK_DEFAULTS, True)
        with tempfile.TemporaryDirectory() as folder:
            database = Path(folder) / "counter.sqlite3"
            self.assertEqual(reserve_counters(database, "bench", 0, 0), (0, 0))
            self.assertEqual(
                reserve_counters(
                    database, "bench", settings["last_lease"], settings["last_target"]
                ),
                (512, 16),
            )

    def setUp(self):
        self.fields = dict(
            current="3", frequency="3", leg="1500", travel="", ambient="", lease="", target=""
        )

    def test_empty_travel_reports_required_field_before_start(self):
        with self.assertRaisesRegex(ValueError, "双向测试前请填写两侧可用行程"):
            hardware_check_fields(self.fields, True)

    def test_static_ignores_incomplete_or_invalid_drive_fields(self):
        self.fields.update(current="", frequency="abc", leg="NaN")
        self.assertEqual(
            hardware_check_fields(self.fields, False),
            dict(drive=None, ambient=None, last_lease=None, last_target=None),
        )

    def test_valid_drive_keeps_optional_fields_empty_and_limits(self):
        self.fields["travel"] = " 360 "
        result = hardware_check_fields(self.fields, True)
        self.assertEqual(result["drive"], (3, 3, 1500, 360))
        self.assertIsNone(result["last_lease"])
        self.fields["current"] = "3.1"
        with self.assertRaisesRegex(ValueError, "限流"):
            hardware_check_fields(self.fields, True)

    def test_invalid_numbers_identify_the_field(self):
        self.fields["travel"] = "360"
        for key, value, message in (
            ("current", "abc", "相电流.*有效数值"),
            ("travel", "NaN", "可用行程.*有限数值"),
            ("frequency", "inf", "电频.*有限数值"),
            ("leg", "1500.5", "单程时间.*整数"),
            ("ambient", "130", "室温.*-40～120"),
            ("lease", "-1", "续租计数.*0～"),
            ("target", "65536", "目标序号.*0～65535"),
        ):
            with self.subTest(key=key, value=value):
                fields = {**self.fields, key: value}
                with self.assertRaisesRegex(ValueError, message):
                    hardware_check_fields(fields, True)
