import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from analyze_motion_noise import analyze


class MotionNoiseAnalysisTests(unittest.TestCase):
    def fixture(self, flags=0x180):
        folder = tempfile.TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        path = Path(folder.name)
        x = np.zeros((4096, 12))
        sine = np.sin(2 * np.pi * 125 * np.arange(len(x)) / 2000)
        x[:, 3] = 2000
        x[:, 5] = np.round(100 * sine)
        x[:, 6] = -np.round(100 * sine)
        x[:, 7] = x[:, 6] + 5
        x[:, 11] = flags
        np.savetxt(path/'capture.tsv', x, header='RTT', comments='')
        (path/'trial.json').write_text(json.dumps(dict(events=[dict(opcode=3, frame=0)],
            runtime_parameters={}, failure=None, shutdown_verified=True)))
        return path

    def test_band_rms_and_closed_loop_sign(self):
        result = analyze(self.fixture())
        band = result['bands'][-1]
        self.assertAlmostEqual(band['iq_reference_rms_A'], .1/np.sqrt(2), delta=.001)
        self.assertAlmostEqual(band['iq_tracking_rms_A'], 0, places=10)
        self.assertAlmostEqual(band['speed_iq_reference_band_correlation'], -1)

    def test_v2_band_rms_and_feedback_channel(self):
        path = self.fixture()
        old = np.loadtxt(path/'capture.tsv', skiprows=1)
        new = np.zeros_like(old)
        new[:, 4] = np.round(np.degrees(old[:, 3] / 10000) * 100)
        new[:, 5] = np.round(np.degrees(old[:, 5] / 10000) * 100)
        new[:, 6] = old[:, 6]
        new[:, 7] = 1000  # deliberately distinct from measured Iq
        new[:, 8] = old[:, 7]
        new[:, 11] = 0x4080
        np.savetxt(path/'capture.tsv', new, header='RTT', comments='')
        band = analyze(path)['bands'][-1]
        self.assertAlmostEqual(band['iq_feedback_rms_A'], .1/np.sqrt(2), delta=.001)
        self.assertAlmostEqual(band['iq_tracking_rms_A'], 0, places=10)
        self.assertAlmostEqual(band['speed_rms_rad_s'], .01/np.sqrt(2), delta=.0001)

    def test_excludes_hold_and_bad_windows(self):
        for flags in (0x186, 0x180 | 64, 0x180 | 8, 0x180 | 32, 0):
            with self.assertRaises(ValueError):
                analyze(self.fixture(flags))


if __name__ == '__main__':
    unittest.main()
