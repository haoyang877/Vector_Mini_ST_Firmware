"""Offline parameter codec regression tests."""

import math
from pathlib import Path
import struct
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from mdrive_core.protocol.param import decode, encode  # noqa: E402


class ParameterCodecTests(unittest.TestCase):
    def test_fixed_and_float_wire_vectors_roundtrip(self) -> None:
        vectors = (
            (0x02, 0x03, -1.25, "fb1e"),
            (0x06, 0x07, -1.25, "fffffb1e"),
            (0x04, 0x05, 6.28, "00000274"),
            (0x64, 0x65, 20.0, "41a00000"),
        )
        for set_id, get_id, value, wire in vectors:
            with self.subTest(set_id=set_id):
                encoded = encode(set_id, value)
                self.assertEqual(encoded, bytes.fromhex(wire))
                self.assertAlmostEqual(decode(get_id, encoded), value)

    def test_position_conversion_truncates_toward_zero(self) -> None:
        self.assertEqual(encode(0x06, math.radians(5)), bytes.fromhex("00000057"))

    def test_sentinels_and_invalid_values_are_rejected(self) -> None:
        invalid_replies = (
            (0x03, bytes.fromhex("8000")),
            (0x07, bytes.fromhex("80000000")),
            (0x05, bytes.fromhex("80000000")),
            (0x01, struct.pack(">f", math.nan)),
            (0x03, bytes(4)),
            (0x07, bytes(2)),
        )
        for param_id, payload in invalid_replies:
            with self.subTest(param_id=param_id), self.assertRaises(ValueError):
                decode(param_id, payload)
        for value in (math.nan, math.inf, 33.0):
            with self.subTest(value=value), self.assertRaises(ValueError):
                encode(0x02, value)


if __name__ == "__main__":
    unittest.main()
