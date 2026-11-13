"""补偿表的来源绑定、CRC、曲线符号和导出失败隔离；不接触硬件。"""

import csv
import hashlib
import json
import queue
import struct
import sys
import tempfile
import unittest
import zlib
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools/bench"))
import can_calibration_curves as curves
import canfd_desktop as desktop


def record():
    data = bytearray(16384)
    struct.pack_into("<f", data, 0, 6)
    data[40] = 3
    struct.pack_into("<h", data, 44, 128)
    struct.pack_into("<11f", data, 2092, 0.6, 3200, 0.7, 3300, 80, 90, 0.05, 0.2, 10, 20, 3)
    struct.pack_into("<3f", data, 2144, 1.6, 3, 30)
    struct.pack_into("<II", data, 2160, 14, 0x454E4332)
    struct.pack_into("<I", data, 2204, 0x48410180)
    crc = zlib.crc32(data[2144:2288], zlib.crc32(data[:2140]))
    struct.pack_into("<I", data, 2140, crc)
    struct.pack_into("<IIfffiII", data, 2288, 0x334E454F, crc, 40, 0, 0, 1, 128, 128)
    struct.pack_into("<1024h", data, 2320, *([256] * 1024))
    struct.pack_into("<I", data, 4368, zlib.crc32(data[2288:4368]))
    return bytes(data)


class Link:
    def __init__(self, crc):
        self.node, self.timeouts, self.crc = 6, {}, crc
        self.saved_reads = 0
        self.wrong_uid = self.changed = False

    def status(self):
        data = bytearray(45)
        data[42:] = bytes((6, 5, 0))
        return bytes(data)

    def request(self, kind, payload=b"", *, page=None):
        if kind == 1:
            return bytes([2 if self.wrong_uid else 1]) * 12 + bytes(20)
        if kind == 123:
            self.saved_reads += 1
            revision = 3 if self.changed and self.saved_reads > 1 else 2
            return struct.pack("<HHBBHIII", 0, 0, 0, 2, 1, revision, revision, revision)
        if kind == 190:
            data = bytearray(44)
            data[4:8] = bytes((1, 3, 6, 7))
            struct.pack_into("<I", data, 8, self.crc)
            return bytes(data)
        raise AssertionError(kind)


class CalibrationCurvesTests(unittest.TestCase):
    def setUp(self):
        self.record = record()
        self.link = Link(struct.unpack_from("<I", self.record, 2140)[0])
        self.report = dict(
            passed=True,
            stop_confirmed=True,
            node=6,
            uid="01" * 12,
            kind=1,
            saved_revision={"saved_revision": 2},
        )
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.out = Path(self.temp.name)

    def export(self, data=None):
        with (
            patch.object(curves, "Link", return_value=self.link),
            patch.object(
                curves, "_read_record", return_value=self.record if data is None else data
            ),
        ):
            return curves.export_calibration_curves(None, self.report, self.out)

    def test_saved_table_exports_1024_points_and_correct_signs(self):
        result = self.export()
        self.assertTrue((self.out / result["png"]).read_bytes().startswith(b"\x89PNG"))
        with (self.out / result["csv"]).open(newline="") as f:
            rows = list(csv.DictReader(f))
        self.assertEqual(len(rows), 1024)
        self.assertEqual(float(rows[0]["motor_raw_minus_corrected_deg"]), 128 * 360 / 65536)
        self.assertEqual(float(rows[0]["hall_raw_minus_corrected_deg"]), -256 * 360 / 65536)
        self.assertEqual(int(rows[0]["motor_lut_q15"]), 128)
        self.assertEqual(int(rows[0]["hall_lut_q15"]), 256)
        self.assertTrue(result["table_updated"])
        self.assertEqual((self.out / "calibration-parameters.bin").read_bytes(), self.record)
        config = json.loads((self.out / "motor-configuration.json").read_text(encoding="utf-8"))
        self.assertEqual(config["parameters"]["limits"]["speed_limit_rad_s"], 30)
        self.assertEqual(config["parameters"]["control"]["iq_ki"], 3300)
        self.assertEqual(config["firmware_model"], "YQ-5740")
        manifest = json.loads((self.out / result["manifest"]).read_text(encoding="utf-8"))
        for name, info in manifest["files"].items():
            self.assertEqual(
                info["sha256"], hashlib.sha256((self.out / name).read_bytes()).hexdigest()
            )

    def test_archive_type_and_serial_are_separate_from_hardware_identity(self):
        self.assertEqual(curves.calibration_archive_name("P轴电机", "0001"), "P轴电机-0001")
        self.assertEqual(len(curves.ARCHIVE_TYPES), 5)
        for bad in ("", "../1", "a/b", "1:2"):
            with self.assertRaises(ValueError):
                curves.calibration_archive_name("P轴电机", bad)
        with self.assertRaises(ValueError):
            curves.calibration_archive_name("unknown", "0001")

    def test_archive_directory_cannot_escape_root(self):
        for raw in ("a/b", "..\\..\\x", "CON", "设备:123"):
            name = curves.model_directory_name(raw)
            self.assertNotIn("/", name)
            self.assertNotIn("\\", name)
            self.assertNotIn(":", name)
        with self.assertRaises(ValueError):
            curves.model_directory_name(" .. ")

    def test_zero_only_job_marks_table_as_existing(self):
        self.report["kind"] = 2
        self.assertFalse(self.export()["table_updated"])

    def test_uid_or_revision_change_rejects_curve(self):
        for flag in ("wrong_uid", "changed"):
            with self.subTest(flag=flag):
                setattr(self.link, flag, True)
                with self.assertRaises(ValueError):
                    self.export()
                setattr(self.link, flag, False)
                self.link.saved_reads = 0
        self.assertFalse((self.out / "angle-calibration.png").exists())

    def test_corrupt_base_or_hall_table_is_not_plotted(self):
        for offset in (44, 2292, 2320, 4368):
            with self.subTest(offset=offset):
                data = bytearray(self.record)
                data[offset] ^= 1
                with self.assertRaises(ValueError):
                    self.export(bytes(data))
        self.assertFalse((self.out / "angle-calibration.png").exists())

    def test_unsaved_or_not_stopped_never_reads_probe(self):
        for field in ("passed", "stop_confirmed"):
            with self.subTest(field=field), patch.object(curves, "_read_record") as read:
                with self.assertRaises(ValueError):
                    curves.export_calibration_curves(None, {**self.report, field: False}, self.out)
                read.assert_not_called()

    def test_curve_error_preserves_successful_calibration(self):
        report = {**self.report, "profile": "motor_calibration"}
        with (
            patch.object(desktop, "controlcan_dll_path", return_value=Path("test.dll")),
            patch.object(desktop.os, "add_dll_directory", create=True),
            patch.object(desktop, "ControlCanFd"),
            patch.object(desktop, "run_angle_calibration", return_value=report),
            patch.object(
                desktop, "export_calibration_curves", side_effect=OSError("J-Link absent")
            ),
        ):
            desktop.acquire(
                dict(profile="motor_calibration", node=6, calibration_kind=1, bus={}),
                self.out,
                queue.Queue(),
                SimpleNamespace(is_set=lambda: False),
            )
        saved = json.loads((self.out / "summary.json").read_text(encoding="utf-8"))
        self.assertTrue(saved["passed"])
        self.assertEqual(saved["curve_error"], "J-Link absent")
