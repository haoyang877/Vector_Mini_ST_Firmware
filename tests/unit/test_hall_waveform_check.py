"""离线验证原码覆盖、相位跨零和凹陷复现判定。"""

import csv
import math
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools/bench"))
from hall_waveform_check import inspect_hall_waveform


class HallWaveformCheckTests(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.directory = Path(tmp.name)

    def measure(
        self, angles, *, dent_repeats=0, centers=(0,), phase_error=0, missing=None, **capture
    ):
        rows = []
        for i, angle in enumerate(angles):
            if missing and missing(angle % 360):
                continue
            radius = 1.0
            if i < 360 * dent_repeats:
                radius -= 0.18 * sum(
                    math.exp(-0.5 * (math.remainder(angle - center, 360) / 6) ** 2)
                    for center in centers
                )
            rows.append(
                (
                    round(1990 + 1650 * radius * math.sin(math.radians(angle))),
                    round(2040 + 1550 * radius * math.cos(math.radians(angle + phase_error))),
                )
            )
        with (self.directory / "hall-raw.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(["hall_sin_counts", "hall_cos_counts"])
            writer.writerows(rows)
        return inspect_hall_waveform(
            self.directory,
            {
                "source": "CAN Hall snapshots",
                "samples": len(rows),
                "csv": "hall-raw.csv",
                **capture,
            },
        )

    def test_offset_and_gain_mismatch_without_dent(self):
        self.assertEqual(self.measure(range(24, 900))["status"], "clear")

    def test_smooth_ellipse_with_quadrature_error_is_clear(self):
        self.assertEqual(self.measure(range(24, 995), phase_error=14)["status"], "clear")

    def test_two_local_dents_remain_separate(self):
        result = self.measure(range(24, 995), centers=(43, 319), dent_repeats=3)
        self.assertEqual(result["status"], "detected")
        self.assertEqual(len(result["sectors"]), 2)
        for sector, center in zip(result["sectors"], (43, 319), strict=True):
            self.assertLessEqual(sector["start_deg"], center)
            self.assertGreater(sector["start_deg"] + sector["width_deg"], center)

    def test_dent_at_phase_wrap_with_both_directions(self):
        for angles in (range(-23, 875), range(28, -870, -1)):
            with self.subTest(angles=angles):
                result = self.measure(angles, dent_repeats=3)
                self.assertEqual(result["status"], "detected")
                self.assertEqual(len(result["sectors"]), 1)
                sector = result["sectors"][0]
                self.assertGreater(sector["start_deg"], 330)
                self.assertGreater(sector["start_deg"] + sector["width_deg"], 360)
                self.assertGreaterEqual(sector["max_chord_depth"], result["min_chord_depth"])

    def test_one_passage_dent_is_not_repeatable(self):
        self.assertEqual(self.measure(range(-23, 875), dent_repeats=1)["status"], "clear")

    def test_incomplete_capture_or_angle_coverage_is_unmeasured(self):
        for result in (
            self.measure(range(24, 420)),
            self.measure(range(24, 900), missing=lambda angle: 90 <= angle < 95),
            self.measure(range(24, 900), complete=False),
            self.measure(range(24, 900), samples=9999),
        ):
            self.assertEqual(result["status"], "not_measured")
            self.assertIn("reason", result)

    def test_repeated_partial_sweeps_do_not_make_a_full_turn(self):
        back_and_forth = (list(range(-45, 56)) + list(range(55, -46, -1))) * 18
        self.assertEqual(self.measure(back_and_forth)["status"], "not_measured")
