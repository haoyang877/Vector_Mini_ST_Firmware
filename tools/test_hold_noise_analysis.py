import tempfile
import unittest
from pathlib import Path

import numpy as np

from analyze_hold_noise import analyze


class HoldNoiseAnalysisTests(unittest.TestCase):
    def capture(self, data):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        path = Path(directory.name) / "capture.tsv"
        np.savetxt(path, data, delimiter="\t", header="raw RTT", comments="")
        return path

    def test_known_noise_and_tracking_error(self):
        data = np.zeros((8, 12))
        data[:, 11] = 0x186
        data[:, 5] = [100, -100] * 4
        data[:, 6] = [-20, 20] * 4
        data[:, 7] = data[:, 6] + 5
        result = analyze(self.capture(data), 2, 8)["intervals"][0]
        self.assertAlmostEqual(result["velocity_iq_reference_correlation"], -1)
        self.assertAlmostEqual(result["iq_reference_ac_rms_A"], .02)
        self.assertAlmostEqual(result["iq_tracking_rms_A"], .005)

    def test_invalid_frames_split_intervals_and_constant_signal_has_no_correlation(self):
        for flag in (8, 32, 64):
            data = np.zeros((8, 12))
            data[:, 11] = 0x186
            data[3, 11] += flag
            result = analyze(self.capture(data), 2, 2)["intervals"]
            self.assertEqual([(x["first_sample_zero_based"], x["stop_sample_exclusive"])
                              for x in result], [(1, 3), (6, 8)])
            self.assertIsNone(result[0]["velocity_iq_reference_correlation"])
        data[:, 11] = 0x182  # HOLD without reached qualification
        self.assertEqual(analyze(self.capture(data), 2, 2)["intervals"], [])

    def test_reject_wrong_shape_and_nonfinite_input(self):
        for data in (np.zeros((2, 11)), np.full((2, 12), np.nan)):
            with self.assertRaises(ValueError):
                analyze(self.capture(data))

    def test_v2_hold_units_and_current_order(self):
        data = np.zeros((8, 12))
        data[:, 11] = 0x4086
        data[:, 3] = [1, -1] * 4
        data[:, 5] = [100, -100] * 4
        data[:, 6] = [-20, 20] * 4
        data[:, 7] = 123
        data[:, 8] = data[:, 6] + 5
        result = analyze(self.capture(data), 2, 8)['intervals'][0]
        self.assertAlmostEqual(result['position_error_peak_to_peak_deg'], .02)
        self.assertAlmostEqual(result['speed_ac_rms_rad_s'], np.pi / 180)
        self.assertAlmostEqual(result['iq_tracking_rms_A'], .005)
        self.assertAlmostEqual(result['feedforward_peak_A'], .123)


if __name__ == "__main__":
    unittest.main()
