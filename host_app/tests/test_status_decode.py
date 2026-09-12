"""Offline status stream decoder regression tests."""

from pathlib import Path
import struct
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from mdrive_core.protocol.status import FORMAT, decode  # noqa: E402


class StatusDecodeTests(unittest.TestCase):
    def test_known_status_values_decode_to_si_units(self) -> None:
        payload = struct.pack(
            FORMAT,
            1,
            3,
            1234,
            1000,
            -250,
            -225,
            500,
            -250,
            1200,
            -200,
            3050,
            2400,
        )

        result = decode(0x7F2, payload)

        self.assertEqual(result["node"], 2)
        self.assertEqual(result["fault"], 1)
        self.assertEqual(result["mode"], 3)
        self.assertEqual(result["position_target_rad"], 1.234)
        self.assertEqual(result["speed_target_rad_s"], -2.5)
        self.assertEqual(result["iq_reference_A"], 0.5)
        self.assertEqual(result["iq_feedback_A"], -0.25)
        self.assertEqual(result["bus_voltage_V"], 24.0)
        self.assertEqual(result["temperature_C"], 30.5)
        self.assertEqual(result["invalid_fields"], [])

    def test_invalid_sentinel_becomes_none_and_is_reported(self) -> None:
        payload = bytearray(48)
        payload[4] = 0x80
        payload[20] = 0x80
        payload[32] = 0x80
        payload[34] = 0x80

        result = decode(0x7F3, bytes(payload))

        self.assertIsNone(result["position_target_rad"])
        self.assertIsNone(result["iq_reference_A"])
        self.assertIsNone(result["temperature_C"])
        self.assertIsNone(result["bus_voltage_V"])
        self.assertEqual(
            result["invalid_fields"],
            ["position_target_rad", "iq_reference_A", "temperature_C", "bus_voltage_V"],
        )


if __name__ == "__main__":
    unittest.main()
