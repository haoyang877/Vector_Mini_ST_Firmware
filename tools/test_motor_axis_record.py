import unittest
from motor_axis_record import encode, encode_joint, decode, prepare


class MotorAxisRecordTests(unittest.TestCase):
    def test_numeric_identity_and_configuration(self):
        for joint_type, name in enumerate(("unknown", "roll", "pitch", "yaw", "wheel_right", "wheel_left")):
            record = encode_joint(joint_type, 0)
            self.assertEqual(decode(record)["name"], name)
            self.assertEqual(decode(record)["joint_type_id"], joint_type)
            self.assertEqual(decode(record)["config_revision"], 0)
            for i in range(32):
                bad = bytearray(record); bad[i] ^= 1
                with self.assertRaises(ValueError): decode(bad)
        self.assertEqual(decode(encode_joint(1, 1, -90, 90, 45))["format_version"], 2)
        for args in ((6, 0), (True, 0), (3, 1, -1, 1, 1), (1, 2),
                     (2, 1, -30, 30, 45), (1, 1, -90, 90, 46), (1, 0, -1, 1, 1)):
            with self.assertRaises(ValueError): encode_joint(*args)

    def test_numeric_patch_preserves_legacy_offsets_and_calibration(self):
        source = bytearray(bytes(range(256)) * 32)
        source[4096:4128] = encode("pitch", -17.188733853924695, 51.56620156177409, 45)
        profile = {"name": "pitch", "joint_type_id": 2, "config_revision": 1,
                   "motion": {"minimum_deg": -17.188733853924695, "maximum_deg": 51.56620156177409,
                              "cruise_deg_s": 45}}
        result = prepare(source, {"axis_profile": 4096, "pos_maxspeed": 4000}, profile)
        self.assertEqual(decode(result[4096:4128])["joint_type_id"], 2)
        self.assertEqual(len(result), len(source))
        allowed = set(range(4096, 4128)) | set(range(4000, 4004))
        self.assertTrue(all(a == b or i in allowed for i, (a, b) in enumerate(zip(source, result))))
        profile["joint_type_id"] = 1
        with self.assertRaises(ValueError): prepare(source, {"axis_profile": 4096, "pos_maxspeed": 4000}, profile)

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
