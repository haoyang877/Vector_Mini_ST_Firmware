"""实时曲线的有效位、缓冲上限与缺失段；不接触电机或通信SDK。"""

import struct
import sys
import tkinter as tk
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools/bench"))
from can_motor_operations import speed_feedback
from can_speed_plot import SpeedPlot


class SpeedFeedbackTests(unittest.TestCase):
    def test_all_six_channels_use_protocol_fields_and_validity(self):
        status = bytearray(45)
        struct.pack_into("<4i", status, 8, 1200, 1100, -3000, -2900)
        struct.pack_into("<H", status, 24, 0xCF)
        struct.pack_into("<2h", status, 30, 500, 450)
        status[43:] = bytes((6, 2))
        row = speed_feedback(status, -3, 1.5)
        self.assertEqual(
            [
                row[key]
                for key in (
                    "reference_position_rad",
                    "position_rad",
                    "reference_speed_rad_s",
                    "actual_rad_s",
                    "reference_iq_a",
                    "iq_a",
                )
            ],
            [1.2, 1.1, -3, -2.9, 0.5, 0.45],
        )
        struct.pack_into("<i", status, 8, -2147483648)
        struct.pack_into("<H", status, 24, 0x8B)
        row = speed_feedback(status, -3, 2)
        self.assertIsNone(row["reference_position_rad"])
        self.assertIsNone(row["reference_speed_rad_s"])
        self.assertIsNone(row["reference_iq_a"])
        self.assertEqual(row["iq_a"], 0.45)


class SpeedPlotTests(unittest.TestCase):
    def setUp(self):
        try:
            self.root = tk.Tk()
        except tk.TclError as exc:
            self.skipTest(str(exc))
        self.root.withdraw()
        self.addCleanup(self.root.destroy)
        self.plot = SpeedPlot(self.root)

    def test_missing_reference_is_a_gap_and_feedback_remains(self):
        for stamp, value in ((0, 1), (1, None), (2, 2)):
            self.plot.append(
                dict(
                    elapsed_s=stamp,
                    reference_position_rad=value,
                    position_rad=stamp,
                    reference_speed_rad_s=None,
                )
            )
        self.plot.redraw()
        ids = self.plot.canvas.find_withtag("reference_position_rad")
        self.assertEqual(len(ids), 2)
        self.assertTrue(all(self.plot.canvas.type(item) == "oval" for item in ids))
        self.assertEqual(len(self.plot.canvas.find_withtag("position_rad")), 1)

    def test_rolling_window_and_new_session_clear_old_samples(self):
        for number in range(3000):
            self.plot.append(dict(elapsed_s=number / 100, iq_a=0.1))
        self.assertEqual(len(self.plot.samples), 1200)
        self.plot.append(dict(elapsed_s=100, iq_a=0.2))
        self.assertEqual(len(self.plot.samples), 1)
        self.plot.append(dict(elapsed_s=0, iq_a=0.3))
        self.assertEqual(len(self.plot.samples), 1)
        self.plot.clear()
        self.assertFalse(self.plot.samples)
