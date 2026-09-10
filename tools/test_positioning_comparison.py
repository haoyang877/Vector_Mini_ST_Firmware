import unittest

import numpy as np

from analyze_positioning_comparison import settle_time


class SettlingTimeTests(unittest.TestCase):
    def test_requires_staying_inside_window_not_first_crossing(self):
        self.assertAlmostEqual(settle_time(np.arange(5), np.array([1, .1, .5, .2, .1]), .3, .5), 2.5)

    def test_never_settled_and_already_inside(self):
        self.assertIsNone(settle_time(np.arange(3), np.array([1, .1, .4]), .3, 0))
        self.assertEqual(settle_time(np.arange(3), np.array([.1, .2, .1]), .3, 0), 0)


if __name__ == '__main__':
    unittest.main()
