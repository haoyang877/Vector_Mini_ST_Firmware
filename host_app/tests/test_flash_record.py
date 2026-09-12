"""Offline tests for flash record readback (0x69 / 0x6A) and parsing."""

import struct
import sys
import unittest
import zlib
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from mdrive_core.protocol.flash_record import (  # noqa: E402
    FlashRecordError,
    OP_CHUNK,
    OP_INFO,
    read_info,
    read_record,
)
from mdrive_core.schema.flash_params import (  # noqa: E402
    FlashRecordParseError,
    lut_angle_error_degrees,
    lut_statistics,
    parse_record,
)
from mdrive_core.transport.frame import CanFrame  # noqa: E402


def build_blob() -> bytes:
    blob = bytearray(2240)
    struct.pack_into("<f", blob, 0x000, 1.0)  # node_id
    struct.pack_into("<f", blob, 0x004, 0.0012)  # currentoffset_a
    struct.pack_into("<f", blob, 0x010, 7.0)  # motor_pole_pairs
    struct.pack_into("<B", blob, 0x029, 1)  # encoder_reverse
    for index in range(1024):
        struct.pack_into("<h", blob, 0x2C + index * 2, (index % 100) - 50)
    struct.pack_into("<I", blob, 0x870, 10)  # schema_version
    struct.pack_into("<I", blob, 0x874, 0x454E4332)  # magic_word
    struct.pack_into("<II", blob, 0x8A0, 0x32535841, 2)  # AXS2 + version
    struct.pack_into("<II", blob, 0x8A8, 2, 1)  # joint_type, config revision
    struct.pack_into("<fff", blob, 0x8B0, -1.0, 1.0, 5.0)
    struct.pack_into("<I", blob, 0x8BC, 0xDEADBEEF)
    return bytes(blob)


class FakeBus:
    """Answer 0x69/0x6A from a prepared blob; optional dropped/corrupted replies."""

    def __init__(self, blob: bytes, fail_first_info: bool = False, corrupt_crc: bool = False) -> None:
        self.blob = blob
        self.fail_first_info = fail_first_info
        self.corrupt_crc = corrupt_crc
        self.pending: list[CanFrame] = []

    def send(self, channel: int, identifier: int, data: bytes) -> None:
        op = identifier & 0xFF
        if op == OP_INFO:
            if self.fail_first_info:
                self.fail_first_info = False
                return
            crc = zlib.crc32(self.blob) & 0xFFFFFFFF
            if self.corrupt_crc:
                crc ^= 0xFFFF
            payload = struct.pack(">IIII", 0x454E4332, 10, len(self.blob), crc)
            self.pending.append(CanFrame(identifier, payload, 1, None))
        elif op == OP_CHUNK:
            offset = int(struct.unpack(">f", data)[0])
            chunk = self.blob[offset : offset + 56]
            payload = struct.pack(">IB", offset, len(chunk)) + chunk
            self.pending.append(CanFrame(identifier, payload, 1, None))

    def receive(self, channel: int, timeout: float) -> list[CanFrame]:
        out, self.pending = self.pending, []
        return out


class SilentBus:
    def send(self, channel: int, identifier: int, data: bytes) -> None:
        pass

    def receive(self, channel: int, timeout: float) -> list[CanFrame]:
        return []


class FlashRecordTest(unittest.TestCase):
    def test_parse_fields(self) -> None:
        parsed = parse_record(build_blob())
        self.assertEqual(parsed["node_id"], 1.0)
        self.assertEqual(parsed["motor_pole_pairs"], 7.0)
        self.assertEqual(parsed["encoder_reverse"], 1)
        self.assertEqual(parsed["schema_version"], 10)
        self.assertEqual(parsed["magic_word"], 0x454E4332)
        profile = parsed["axis_profile"]
        self.assertTrue(profile["configured"])
        self.assertEqual(profile["joint_type"], 2)
        self.assertAlmostEqual(profile["maximum_speed_rad_s"], 5.0)

    def test_lut_formula_and_stats(self) -> None:
        blob = build_blob()
        errors = lut_angle_error_degrees(blob)
        self.assertEqual(len(errors), 1024)
        self.assertAlmostEqual(errors[0], -50 * 360.0 / 65536.0, places=9)
        stats = lut_statistics(blob)
        self.assertAlmostEqual(stats["max_deg"], 49 * 360.0 / 65536.0, places=9)
        self.assertAlmostEqual(stats["min_deg"], -50 * 360.0 / 65536.0, places=9)
        self.assertEqual(stats["max_angle_deg"], 99 * 64.0 * 360.0 / 65536.0)

    def test_parse_rejects_short_blob(self) -> None:
        with self.assertRaises(FlashRecordParseError):
            parse_record(b"\x00" * 100)

    def test_roundtrip(self) -> None:
        blob = build_blob()
        bus = FakeBus(blob)
        info = read_info(bus)
        self.assertEqual(info.size, len(blob))
        self.assertEqual(info.schema, 10)
        record = read_record(bus, info=info)
        self.assertEqual(record, blob)

    def test_retry_after_dropped_reply(self) -> None:
        bus = FakeBus(build_blob(), fail_first_info=True)
        info = read_info(bus, retries=3)
        self.assertEqual(info.size, 2240)

    def test_crc_mismatch_detected(self) -> None:
        bus = FakeBus(build_blob(), corrupt_crc=True)
        with self.assertRaises(FlashRecordError):
            read_record(bus)

    def test_silent_firmware_reports_unsupported(self) -> None:
        with self.assertRaises(FlashRecordError) as ctx:
            read_info(SilentBus(), retries=1, timeout=0.01)
        self.assertIn("0x69", str(ctx.exception))


if __name__ == "__main__":
    unittest.main()
