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

    def test_excludes_hold_and_bad_windows(self):
        for flags in (0x186, 0x180 | 64, 0x180 | 8, 0x180 | 32, 0):
            with self.assertRaises(ValueError):
                analyze(self.fixture(flags))


if __name__ == '__main__':
    unittest.main()
