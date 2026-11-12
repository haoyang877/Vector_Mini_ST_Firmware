"""电机工装回归。模拟固件票据，不操作设备。"""

import csv
import struct
import sys
import unittest
import zlib
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools/bench"))
import commission_motor as tool


class Jobs:
    node = 6
    sequence = 800

    def __init__(self, replies):
        self.replies = iter(replies)
        self.calls = []
        self.trace = [{"attempt": 1}]

    def request(self, kind, payload=b""):
        self.calls.append((kind, payload))
        if kind == 108:
            return tool.STATUS.pack(0, 0, 0, *([0] * 4), 0, *([0] * 8), self.node, 5, 0)
        if kind == 110:
            return struct.pack("<HH", 0, 0)
        reply = next(self.replies)
        if isinstance(reply, Exception):
            raise reply
        return tool.JOB.pack(*reply)


class CommissionTests(unittest.TestCase):
    def setUp(self):
        self.enterContext(patch.object(tool.time, "sleep"))

    def test_custom_speed_uses_packaged_heat_without_prompt(self):
        axis = tool.board.parse_axis("pitch")
        with TemporaryDirectory() as tmp:
            kit = Path(tmp)
            (kit / "parameters").mkdir()
            preset = bytearray(2288)
            struct.pack_into("<7f", preset, 2240, *tool.board.BENCH_THERMAL)
            (kit / "parameters" / tool.parameter_filename(axis)).write_bytes(preset)
            with (
                patch.object(tool, "saved_parameters", return_value={"calibration_flag": 0}),
                patch.object(tool, "new_record") as generate,
                patch("builtins.input", side_effect=["30.1", "0.5"]) as prompt,
            ):
                tool.configure(Mock(speed=None, thermal=None), kit, axis, True)
            self.assertEqual(prompt.call_count, 2)
            self.assertTrue(all("限速" in call.args[0] for call in prompt.call_args_list))
            generate.assert_called_once_with(
                kit, axis, 0.5, struct.unpack_from("<7f", preset, 2240)
            )

    def test_curve_exports_match_saved_firmware_signs(self):
        for name in ("pitch", "roll"):
            axis = tool.board.parse_axis(name)
            record = bytearray(16384)
            struct.pack_into("<h", record, 44, 128)
            struct.pack_into("<h", record, 2320, 256)
            geared = name == "pitch"
            with TemporaryDirectory() as tmp:
                run = Path(tmp)
                curves = tool.export_curves(record, axis, geared, bytes(range(12)), run)
                self.assertTrue((run / curves["png"]).read_bytes().startswith(b"\x89PNG"))
                with (run / curves["csv"]).open(newline="") as stream:
                    rows = list(csv.DictReader(stream))
                self.assertEqual(len(rows), 1024)
                self.assertEqual(float(rows[-1]["raw_angle_deg"]), 1023 * 360 / 1024)
                self.assertEqual(float(rows[0]["motor_raw_minus_corrected_deg"]), 128 * 360 / 65536)
                self.assertEqual("hall_raw_minus_corrected_deg" in rows[0], geared)
                if geared:
                    self.assertEqual(
                        float(rows[0]["hall_raw_minus_corrected_deg"]), -256 * 360 / 65536
                    )

    def test_saved_ticket_stops_and_verifies_idle(self):
        client = Jobs([(1, 0, 91, 1, 1, 1), (0, 0, 91, 1, 1, 1), (0, 0, 91, 1, 2, 2)])
        self.assertEqual(tool.run_job(client, 1), 91)
        self.assertEqual([kind for kind, _ in client.calls], [108, 160, 161, 161, 110, 108])
        self.assertEqual(client.calls[2][1], struct.pack("<I", 91))

    def test_saved_bit_alone_and_other_tickets_cannot_complete(self):
        for response in [
            (0, 0, 13, 1, 2, 1),
            (0, 0, 14, 1, 2, 2),
            (0, 8, 13, 1, 5, 2),
            (0, 0, 13, 2, 2, 2),
            TimeoutError(),
        ]:
            with self.subTest(response=response):
                client = Jobs([(1, 0, 13, 1, 1, 1), response])
                with self.assertRaises((RuntimeError, TimeoutError)):
                    tool.run_job(client, 1)
                self.assertEqual(client.calls[-2:], [(110, b""), (108, b"")])

    def test_stale_start_and_lost_start_reply(self):
        client = Jobs([(7, 5, 0, 0, 0, 0), (1, 0, 24, 2, 1, 1), (0, 0, 24, 2, 2, 2)])
        self.assertEqual(tool.run_job(client, 2), 24)
        self.assertNotEqual(client.sequence, 800)
        client = Jobs([TimeoutError()])
        with self.assertRaises(TimeoutError):
            tool.run_job(client, 1)
        self.assertEqual(client.calls[-2:], [(110, b""), (108, b"")])

    def test_reverse_board_rejected_before_any_calibration_command(self):
        axis = tool.board.parse_axis("pitch")
        data = bytearray(16384)
        data[41] = 1
        struct.pack_into("<f", data, 0, axis.node)
        struct.pack_into("<I", data, 2136, 1)
        struct.pack_into("<II", data, 2160, 14, 0x454E4332)
        struct.pack_into("<I", data, 2216, axis.joint)
        struct.pack_into("<I", data, 2140, zlib.crc32(data[:2140] + data[2144:2288]))
        identity = bytearray(32)
        identity[22:24] = bytes((1, 1))
        client = Mock()
        client.identify.return_value = (bytes(32), identity, ())
        bundle = Mock(image=bytes(128), target={"constants": {"TARGET_DESCRIPTOR_OFFSET": "0U"}})
        with (
            TemporaryDirectory() as tmp,
            patch.object(tool, "UpgradeClient", return_value=client),
            patch.object(tool.board.maintenance, "read_twice", return_value=bytes(data)),
            self.assertRaisesRegex(ValueError, "encoder_reverse"),
        ):
            tool.calibrate(None, None, bundle, axis, bytes(range(12)), Path(tmp))
        client.request.assert_not_called()

    def test_calibration_paths_read_saved_record_and_check_fault_free_stop(self):
        def seal(record):
            crc = zlib.crc32(record[:2140] + record[2144:2288])
            struct.pack_into("<I", record, 2140, crc)
            return crc

        for axis in tool.board.AXES:
            with self.subTest(node=axis.node), TemporaryDirectory() as tmp:
                raw = bytearray(16384)
                struct.pack_into("<f", raw, 0, axis.node)
                struct.pack_into("<I", raw, 2136, 1)
                struct.pack_into("<II", raw, 2160, 14, 0x454E4332)
                struct.pack_into("<I", raw, 2216, axis.joint)

                seal(raw)
                before = bytes(raw)
                raw[40] = 3
                struct.pack_into("<I", raw, 2204, 0x48410180)
                struct.pack_into(
                    "<IIfffiII", raw, 2288, 0x334E454F, seal(raw), 40, 0, 0, 1, 128, 128
                )
                struct.pack_into("<I", raw, 4368, zlib.crc32(raw[2288:4368]))
                image = bytes(128)
                identity = bytearray(32)
                identity[22:24] = bytes((1, 1))
                client = Mock(node=axis.node)
                client.identify.return_value = (bytes(32), identity, ())
                client.request.return_value = tool.STATUS.pack(
                    0, 0, 0, *([0] * 4), 0, *([0] * 8), axis.node, 5, 0
                )
                bundle = Mock(
                    image=image,
                    debug=image,
                    target={"constants": {"TARGET_DESCRIPTOR_OFFSET": "0U"}},
                )
                with (
                    patch.object(tool, "UpgradeClient", return_value=client),
                    patch.object(tool.board, "verify_boot"),
                    patch.object(tool.board.maintenance, "check_outputs_off"),
                    patch.object(tool.board.maintenance, "check_identity"),
                    patch.object(
                        tool.board.maintenance, "read_twice", side_effect=[before, bytes(raw)]
                    ),
                    patch.object(tool, "run_job", side_effect=[41, 42]) as start,
                ):
                    result = tool.calibrate(None, None, bundle, axis, bytes(range(12)), Path(tmp))
                kinds = [call.args[1] for call in start.call_args_list]
                self.assertEqual(kinds, [1] if axis.node in (6, 7, 8) else [1, 2])
                self.assertEqual(result["calibration_flag"], 3)
                self.assertEqual(
                    (Path(tmp) / tool.parameter_filename(axis, "calibrated")).read_bytes(), raw
                )
