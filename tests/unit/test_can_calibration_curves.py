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
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools/bench"))
import can_calibration_curves as curves
import production_backend as desktop
import production_inspection as inspection
from can_calibration_capture import HallRawCapture


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
        if kind == 2:
            return bytes(16)
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
        self.out.joinpath("hall-raw.csv").write_text(
            "sample_index,nominal_elapsed_s,hall_angle_mrad,hall_sin_counts,hall_cos_counts\n"
            "0,0,0,100,4000\n1,0.0005,1,3900,200\n",
            encoding="utf-8",
        )
        self.report["raw_capture"] = {"samples": 2, "csv": "hall-raw.csv"}
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
        self.assertEqual(config["calibration_values"]["electrical_zero_q15"], 0)
        self.assertTrue(config["calibration_values"]["electrical_zero_valid"])
        self.assertTrue((self.out / "calibration-values.csv").is_file())
        manifest = json.loads((self.out / result["manifest"]).read_text(encoding="utf-8"))
        for name, info in manifest["files"].items():
            self.assertEqual(
                info["sha256"], hashlib.sha256((self.out / name).read_bytes()).hexdigest()
            )

    def test_archive_type_and_serial_are_separate_from_hardware_identity(self):
        first = curves.calibration_archive("P轴电机", "0001", "T0001")
        second = curves.calibration_archive("P轴电机", "0001", "T0002")
        self.assertEqual(first["name"], "P轴电机-0001")
        self.assertEqual(first["specimen_serial"], second["specimen_serial"])
        self.assertNotEqual(first["test_serial"], second["test_serial"])
        self.report["archive"] = first
        self.export()
        for name in ("motor-configuration.json", "calibration-manifest.json"):
            saved = json.loads((self.out / name).read_text(encoding="utf-8"))
            self.assertEqual(saved["archive"], first)
        self.assertEqual(len(curves.ARCHIVE_TYPES), 5)
        for bad in ("", "../1", "a/b", "1:2"):
            for specimen, test in ((bad, "0001"), ("0001", bad)):
                with self.assertRaises(ValueError):
                    curves.calibration_archive("P轴电机", specimen, test)
        with self.assertRaises(ValueError):
            curves.calibration_archive("unknown", "0001", "0001")

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

    def test_selected_probe_is_forwarded_to_flash_read(self):
        with (
            patch.object(curves, "Link", return_value=self.link),
            patch.object(curves, "_read_record", return_value=self.record) as read,
        ):
            curves.export_calibration_curves(None, self.report, self.out, probe_serial=456)
        read.assert_called_once_with(self.report["uid"], 456)

    def test_can_record_reassembly_and_changed_block_rejection(self):
        data = self.record[:4372]
        crc = zlib.crc32(data)
        damaged = False

        def respond(kind, payload):
            self.assertEqual(kind, 194)
            revision, expected_crc, offset, count, version = struct.unpack("<IIHBB", payload)
            self.assertEqual(version, 1)
            if count:
                self.assertEqual((revision, expected_crc), (2, crc))
            return struct.pack("<HHIIHHBB", 0, 0, 2, crc, 4372, offset, count, 1) + bytes(
                value ^ int(damaged) for value in data[offset : offset + count]
            )

        link = SimpleNamespace(request=respond)
        self.assertEqual(curves._read_record_can(link, 2), data)
        with self.assertRaisesRegex(ValueError, "修订"):
            curves._read_record_can(link, 3)
        damaged = True
        with self.assertRaisesRegex(ValueError, "完整性"):
            curves._read_record_can(link, 2)

    def test_committed_hall_fault_decodes_board_flags_and_rejects_old_revision(self):
        failed = bytearray(self.record[:4372])
        struct.pack_into("<I", failed, 2204, 0x48410106)
        crc = zlib.crc32(failed[2144:2288], zlib.crc32(failed[:2140]))
        struct.pack_into("<I", failed, 2140, crc)
        report = dict(self.report, before_revision=dict(saved_revision=1))
        with (
            patch.object(curves, "Link", return_value=self.link),
            patch.object(curves, "_read_record_can", return_value=bytes(failed)) as read,
        ):
            result = curves.read_committed_calibration_fault(None, report, self.out)
        read.assert_called_once_with(self.link, 2)
        self.assertEqual(result["hall_fault_mask"], 6)
        self.assertEqual(result["calibration_flags"], 3)
        self.assertEqual(result["details"], ["Hall sin 最小值过低", "Hall sin 最大值过高"])
        self.assertEqual((self.out / result["parameters_bin"]).read_bytes(), failed)
        exported = result["encoder_curves"]
        self.assertTrue((self.out / exported["png"]).is_file())
        self.assertEqual(exported["points_per_curve"], 1024)
        with (self.out / exported["csv"]).open(encoding="utf-8", newline="") as stream:
            rows = list(csv.DictReader(stream))
        self.assertEqual(len(rows), 1024)
        self.assertEqual(rows[0]["motor_lut_q15"], "128")
        self.assertNotIn("hall_lut_q15", rows[0])
        with (
            patch.object(curves, "Link", return_value=self.link),
            patch.object(curves, "_read_record_can", return_value=bytes(failed)),
            patch.object(curves, "export_curves", side_effect=RuntimeError("绘图失败")),
        ):
            result = curves.read_committed_calibration_fault(None, report, self.out)
        self.assertEqual(result["encoder_curve_error"], "绘图失败")
        self.assertEqual(result["calibration_flags"], 3)
        self.assertEqual(result["hall_fault_mask"], 6)
        report["before_revision"]["saved_revision"] = 2
        with (
            patch.object(curves, "Link", return_value=self.link),
            patch.object(curves, "_read_record_can") as read,
            self.assertRaisesRegex(ValueError, "新保存修订"),
        ):
            curves.read_committed_calibration_fault(None, report, self.out)
        read.assert_not_called()

    def test_committed_clipped_hall_exports_diagnostic_only_after_crc_validation(self):
        failed = bytearray(self.record[:4372])
        struct.pack_into("<I", failed, 2204, 0x48410106)
        crc = zlib.crc32(failed[2144:2288], zlib.crc32(failed[:2140]))
        struct.pack_into("<I", failed, 2140, crc)
        struct.pack_into("<I", failed, 2292, crc)
        struct.pack_into("<II", failed, 2312, 2, 3)
        struct.pack_into("<I", failed, 4368, zlib.crc32(failed[2288:4368]))
        report = dict(self.report, before_revision=dict(saved_revision=1), stop_faults=0x4000)
        with (
            patch.object(curves, "Link", return_value=self.link),
            patch.object(curves, "_read_record_can", return_value=bytes(failed)),
        ):
            result = curves.read_committed_calibration_fault(None, report, self.out)
        self.assertTrue(result["hall_diagnostic_curve"])
        self.assertEqual(result["hall_fault_mask"], 6)
        csv_path = self.out / result["encoder_curves"]["csv"]
        self.assertIn("hall_raw_minus_corrected_deg", csv_path.read_text(encoding="utf-8"))
        failed[4368] ^= 1
        with (
            patch.object(curves, "Link", return_value=self.link),
            patch.object(curves, "_read_record_can", return_value=bytes(failed)),
        ):
            result = curves.read_committed_calibration_fault(None, report, self.out)
        self.assertNotIn("hall_diagnostic_curve", result)
        self.assertIn("CRC", result["hall_diagnostic_error"])
        self.assertNotIn("hall_raw_minus_corrected_deg", csv_path.read_text(encoding="utf-8"))

    def test_can_only_saved_readback_exports_without_opening_jlink(self):
        original_request = self.link.request
        data = self.record[:4372]
        crc = zlib.crc32(data)

        def respond(kind, payload=b"", *, page=None):
            if kind == 2:
                return struct.pack("<I", 1 << 24) + bytes(12)
            if kind == 194:
                _, _, offset, count, version = struct.unpack("<IIHBB", payload)
                return (
                    struct.pack("<HHIIHHBB", 0, 0, 2, crc, len(data), offset, count, version)
                    + data[offset : offset + count]
                )
            return original_request(kind, payload, page=page)

        self.link.request = respond
        with (
            patch.object(curves, "Link", return_value=self.link),
            patch.object(curves, "calibration_probe") as probe,
        ):
            report = curves.read_saved_calibration(None, 6, self.report["uid"], self.out)
        probe.assert_not_called()
        self.assertEqual((self.out / "calibration-parameters.bin").read_bytes(), data)
        self.assertFalse(report["curves"]["table_updated"])
        self.assertIn("CAN", report["curves"]["source"])

    def test_saved_output_pairing_fault_names_coverage_or_mapping(self):
        data = bytearray(self.record[:4372])
        for lut, text in ((1, "相位覆盖不足"), (2, "映射或相位误差超限")):
            with self.subTest(lut=lut):
                struct.pack_into("<II", data, 2312, 3, lut)
                struct.pack_into("<I", data, 4368, zlib.crc32(data[2288:4368]))
                failure = curves._saved_output_fault(data)
                self.assertIn(text, failure["details"][0])

    def test_can_hall_counter_wrap_and_failure_preserve_partial_samples(self):
        capture = HallRawCapture(self.out)
        capture.start(dict(app_version=0x1081B, capabilities=1 << 25))
        samples = iter([(0xFFFFFFFE, 0xFFFFFFFE), (3, 3), (2, 4)])

        def respond(_):
            counter, millis = next(samples)
            return struct.pack("<4H2I2H", 0, 0, 1, 1, counter, millis, 123, 4090)

        link = SimpleNamespace(request=respond)
        for _ in range(3):
            capture.sample(link)
        result = capture.finish()
        self.assertEqual(result["samples"], 2)
        self.assertFalse(result["complete"])
        self.assertIn("回退", result["error"])
        with (self.out / "hall-raw.csv").open(newline="") as stream:
            rows = list(csv.DictReader(stream))
        self.assertEqual(float(rows[1]["nominal_elapsed_s"]), 0.005)

    def test_can_hall_capture_uses_real_raw_values_and_skips_duplicate_counter(self):
        capture = HallRawCapture(self.out)
        capture.start(dict(app_version=0x1081B, capabilities=1 << 25))
        link = SimpleNamespace(
            request=lambda _: struct.pack("<4H2I2H", 0, 0, 1, 1, 200, 10, 100, 3900)
        )
        capture.sample(link)
        capture.sample(link)
        result = capture.finish()
        self.assertEqual(result["samples"], 1)
        self.assertEqual(result["source"], "CAN Hall snapshots")
        with (self.out / "hall-raw.csv").open(newline="") as stream:
            row = next(csv.DictReader(stream))
        self.assertEqual((row["hall_sin_counts"], row["hall_cos_counts"]), ("100", "3900"))
        self.assertEqual(row["sample_counter"], "200")

    def test_multiple_probes_require_selection_and_validate_uid(self):
        probe = Mock()
        probe.connected_emulators.return_value = [Mock(SerialNumber=123), Mock(SerialNumber=456)]
        probe.halted.return_value = False
        pylink = Mock()
        pylink.JLink.return_value = probe
        with (
            patch.dict(sys.modules, {"pylink": pylink}),
            patch.object(curves.board.maintenance, "check_identity") as identity,
            patch.object(curves.board.maintenance, "check_outputs_off"),
            patch.object(curves.board.maintenance, "release_reset_catch"),
        ):
            with self.assertRaisesRegex(ValueError, "选择"):
                with curves.calibration_probe(self.report["uid"]):
                    self.fail("未选择时不得连接")
            probe.open.assert_not_called()
            with self.assertRaisesRegex(ValueError, "已断开"):
                with curves.calibration_probe(self.report["uid"], 789):
                    self.fail("失效的选择不得回退")
            probe.open.assert_not_called()
            with curves.calibration_probe(self.report["uid"], 456):
                pass
            probe.open.assert_called_once_with(serial_no=456)
            identity.assert_called_once_with(
                probe,
                curves.board.maintenance.load_target("yg_esc_v1_at32f456"),
                bytes.fromhex(self.report["uid"]),
            )
            identity.side_effect = ValueError("UID 不符")
            with self.assertRaisesRegex(ValueError, "UID 不符"):
                with curves.calibration_probe(self.report["uid"], 456):
                    self.fail("错误芯片不得读取曲线")

    def test_curve_error_preserves_successful_calibration(self):
        report = {**self.report, "profile": "motor_calibration"}
        with (
            patch.object(desktop, "controlcan_dll_path", return_value=Path("test.dll")),
            patch.object(desktop.os, "add_dll_directory", create=True),
            patch.object(desktop, "ControlCanFd"),
            patch.object(inspection, "run_angle_calibration", return_value=report),
            patch.object(
                inspection, "export_calibration_curves", side_effect=OSError("J-Link absent")
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
        self.assertEqual(saved["hall_waveform"]["status"], "not_measured")

    def test_raw_capture_preserves_integer_samples_and_separates_receive_time(self):
        rows = [(0, 0, 0, 0, 0, 3141, 123, 4090), (0, 0, 0, 0, 0, 3142, 321, 12)]
        data = b"".join(struct.pack("<8h", *row) for row in rows)
        (self.out / "hall-raw.bin").write_bytes(data)
        capture = HallRawCapture(self.out)
        result = capture.finish()
        self.assertEqual(result["samples"], 2)
        self.assertFalse(result["loss_detection_available"])
        self.assertEqual(result["observed_extrema_counts"]["sin"], {"min": 123, "max": 321})
        self.assertEqual((self.out / "hall-raw.bin").read_bytes(), data)
        with (self.out / "hall-raw.csv").open(newline="") as stream:
            saved = list(csv.DictReader(stream))
        self.assertEqual(saved[1]["hall_cos_counts"], "12")
        self.assertEqual(float(saved[1]["nominal_elapsed_s"]), 0.0005)
        (self.out / "hall-raw.bin").write_bytes(b"\xab\xcd" + data + b"\x01")
        shifted = HallRawCapture(self.out).finish()
        self.assertEqual(shifted["samples"], 2)
        self.assertEqual((shifted["prefix_bytes"], shifted["suffix_bytes"]), (2, 1))

    def test_resistance_firmware_retains_raw_capture_format(self):
        for version in (0x01080B, 0x01080C):
            capture = HallRawCapture(self.out)
            capture.ready.set()
            with patch("can_calibration_capture.threading.Thread") as thread:
                capture.start({"app_version": version, "uid": "test"})
            thread.return_value.start.assert_called_once()
            self.assertNotIn("error", capture.result)

    def test_raw_capture_rejects_partial_frames_and_unknown_firmware(self):
        (self.out / "hall-raw.bin").write_bytes(b"123")
        self.assertIn("error", HallRawCapture(self.out).finish())
        capture = HallRawCapture(self.out)
        capture.start({"app_version": 1})
        self.assertIsNone(capture.thread)
        self.assertIn("error", capture.result)
