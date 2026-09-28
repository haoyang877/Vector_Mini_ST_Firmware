"""校验频响拟合和相位裕度模型，避免将趋势、半拍或低频交点误判为带宽。"""

import sys
from pathlib import Path
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools/analysis"))
from compare_speed_estimators import bandwidth_model, phasor, response


class SpeedEstimatorComparison(unittest.TestCase):
    def test_fit_rejects_rotation_trend_on_partial_cycles(self):
        t = np.arange(3187) / 2000
        expected = 0.2 * np.exp(-0.4j)
        value = 7 + 3 * t + np.real(expected * np.exp(2j * np.pi * 37 * t))
        self.assertLess(abs(phasor(t, value, 37) - expected), 1e-10)

    def test_two_ms_window_measures_one_ms_phase_delay(self):
        h = response(100)
        self.assertAlmostEqual(np.angle(h, deg=True), -36)
        self.assertAlmostEqual(abs(h), np.sinc(0.2))

    def test_margin_uses_high_frequency_crossing_and_slow_loop_hold(self):
        slow = bandwidth_model(600, 2000, 0.00015)
        fast = bandwidth_model(600, 4000, 0.00015)
        self.assertAlmostEqual(slow["pm_deg"], 60, delta=0.05)
        self.assertGreater(slow["crossover_hz"], 80)
        self.assertLess(slow["crossover_hz"], 100)
        self.assertGreater(fast["crossover_hz"], slow["crossover_hz"])


if __name__ == "__main__":
    unittest.main()
