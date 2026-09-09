import unittest
from motor_axis_record import encode, decode, prepare


class MotorAxisRecordTests(unittest.TestCase):
    def test_roundtrip_and_corruption(self):
        for name, angle in (("roll", 90), ("pitch", 30)):
            record = encode(name, -angle, angle, 45)
            result = decode(record)
            self.assertEqual(result["name"], name)
            self.assertAlmostEqual(result["maximum_deg"], angle, places=5)
            for i in range(32):
                bad = bytearray(record); bad[i] ^= 1
                with self.assertRaises(ValueError): decode(bad)

    def test_patch_preserves_calibration_and_unrelated_bytes(self):
        source = bytes(range(256)) * 16 + bytes([255]) * 4096
        offsets = {"axis_profile": 4096, "pos_maxspeed": 4000}
        profile = {"name": "roll", "motion": {"minimum_deg": -90, "maximum_deg": 90, "cruise_deg_s": 45}}
        result = prepare(source, offsets, profile)
        allowed = set(range(4096, 4128)) | set(range(4000, 4004))
        self.assertTrue(all(a == b or i in allowed for i, (a, b) in enumerate(zip(source, result))))
        bad = bytearray(source); bad[4096] = 0
        with self.assertRaises(ValueError): prepare(bad, offsets, profile)
        with self.assertRaises(ValueError): prepare(source, {"axis_profile": 9000, "pos_maxspeed": 4000}, profile)


if __name__ == "__main__": unittest.main()
