"""检测配置文件、固定点量化及确认帧边界。"""

import json
import struct
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools/bench"))
from hardware_limits import confirm_start, default_path, read_limits, start_payload


class LimitsTests(unittest.TestCase):
    def test_missing_file_reports_configuration_error(self):
        with tempfile.TemporaryDirectory() as tmp, self.assertRaisesRegex(ValueError, "无法读取"):
            read_limits(Path(tmp) / "missing.json")

    def test_defaults_round_trip(self):
        config = read_limits()
        payload = start_payload(config, 3, 8)
        self.assertEqual(len(payload), 44)
        self.assertEqual(config["wire_values"][12:14], [2000, 400])
        reply = struct.pack("<HHI16HI", 1, 0, config["revision"], *config["wire_values"], 9)
        self.assertEqual(confirm_start(reply, config, 8), 9)
        with self.assertRaises(ValueError):
            confirm_start(reply[:-1], config, 8)
        with self.assertRaises(ValueError):
            confirm_start(reply, config, 7)

    def test_invalid_configuration_is_not_silently_defaulted(self):
        original = json.loads(default_path().read_text(encoding="utf-8"))
        for field, value in (
            ("hall_trend_min_ratio", 1),
            ("phase_rms_counts", 0),
            ("bus_min_v", 30),
            ("hall_raw_min", 4001),
            ("temperature_max_c", -1),
            ("phase_rms_counts", True),
        ):
            with self.subTest(field=field, value=value), tempfile.TemporaryDirectory() as tmp:
                config = json.loads(json.dumps(original))
                config["limits"][field] = value
                path = Path(tmp) / "limits.json"
                path.write_text(json.dumps(config), encoding="utf-8")
                with self.assertRaises(ValueError):
                    read_limits(path)


if __name__ == "__main__":
    unittest.main()
