import unittest
import numpy as np
from rtt_control_frame import decode


class RttFrameDecoderTests(unittest.TestCase):
    def test_v2_order_units_and_continuous_angles(self):
        raw = np.array([[20000, 19000, 18000, 1000, -2500, -2000,
                         1500, 100, 1400, 1400, 300, 0x4080],
                        [-20000, -19000, -18000, -1000, 2500, 2000,
                         -1500, -100, -1400, -1400, -300, 0x4086]])
        d = decode(raw)
        self.assertEqual(d['version'], 2)
        np.testing.assert_equal(d['target_deg'], [200, -200])
        np.testing.assert_equal(d['reference_deg'], [190, -190])
        np.testing.assert_equal(d['position_deg'], [180, -180])
        np.testing.assert_equal(d['error_deg'], [10, -10])
        np.testing.assert_allclose(d['speed_reference_rad_s'], np.radians([-25, 25]))
        np.testing.assert_allclose(d['speed_feedback_rad_s'], np.radians([-20, 20]))
        np.testing.assert_equal(d['iq_reference_A'], [1.5, -1.5])
        np.testing.assert_equal(d['feedforward_A'], [.1, -.1])
        np.testing.assert_equal(d['iq_feedback_A'], [1.4, -1.4])
        np.testing.assert_equal(d['integral_A'], [.3, -.3])
        self.assertTrue(d['valid'].all())

    def test_legacy_units_saturation_and_version_rejection(self):
        raw = np.zeros((2, 12))
        raw[:, 11] = 0x186
        raw[:, 0] = 16384
        raw[:, 2] = 10000
        raw[:, 5] = 10000
        raw[:, 7] = 123
        d = decode(raw)
        np.testing.assert_equal(d['reference_deg'], [90, 90])
        np.testing.assert_allclose(d['error_deg'], np.degrees([1, 1]))
        np.testing.assert_equal(d['speed_feedback_rad_s'], [1, 1])
        np.testing.assert_equal(d['iq_feedback_A'], [.123, .123])
        raw[0, 0] = 32767
        self.assertTrue(decode(raw)['encoding_saturated'][0])
        for tag in (0x4180, 0x80):
            raw[:, 11] = tag
            with self.assertRaises(ValueError):
                decode(raw)
        raw[:, 11] = [0x180, 0x4080]
        with self.assertRaises(ValueError):
            decode(raw)


if __name__ == '__main__':
    unittest.main()
