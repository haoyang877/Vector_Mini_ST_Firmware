"""Offline regression of plan bounds and acceptance failure paths; no J-Link import."""
import copy
import csv
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import run_mode3_validation as validation


class Mode3ValidationTests(unittest.TestCase):
    def setUp(self):
        self.profile = validation.read_json(validation.DEFAULT_PROFILE)

    def test_baseline_sequence_and_effective_settings(self):
        cases = {c["id"]: c for c in validation.motion_cases(self.profile)}
        self.assertEqual(cases["range"]["targets_deg"], [85, -85, 0, 5, -5, 0])
        self.assertEqual(cases["medium"]["targets_deg"], [20, 0, -20, 20, 0])
        with tempfile.TemporaryDirectory() as folder:
            out = Path(folder) / "plan"
            with patch.object(validation, "metadata", return_value={}):
                report = validation.make_plan(validation.DEFAULT_PROFILE, out)
            self.assertEqual(report["effective_deceleration_deg_s2"], 30)
            self.assertEqual(report["jerk_limit_deg_s3"], 225)
            self.assertEqual(report["actual_speed_pi_kp"], 3)
            script = (out / "run_bench.ps1").read_text(encoding="utf-8-sig")
            self.assertIn("--keep-parameters", script)
            self.assertIn("--case $Case", script)
            self.assertNotIn("servo_hil_flash", script)
            with self.assertRaises(FileExistsError):
                validation.make_plan(validation.DEFAULT_PROFILE, out)

    def test_narrow_and_asymmetric_ranges_never_escape_margin(self):
        for lo, hi, margin in [(-2, 2, .5), (10, 40, 2), (-45, 45, 5)]:
            p = copy.deepcopy(self.profile)
            p["motion"].update(minimum_deg=lo, maximum_deg=hi, target_margin_deg=margin)
            for case in validation.motion_cases(p):
                self.assertTrue(all(lo + margin <= x <= hi - margin for x in case["targets_deg"]))

    def test_changed_bench_does_not_receive_fixed_hardware_script(self):
        self.profile["motion"]["maximum_deg"] = 45
        with tempfile.TemporaryDirectory() as folder:
            profile = Path(folder) / "profile.json"
            validation.write_json(profile, self.profile)
            with patch.object(validation, "metadata", return_value={}), \
                 patch.object(validation, "DEFAULT_PROFILE", profile):
                report = validation.make_plan(profile, Path(folder) / "plan")
            self.assertFalse(report["local_bench_script_available"])
            self.assertFalse((Path(folder) / "plan/run_bench.ps1").exists())

    def test_unknown_or_invalid_parameters_are_rejected(self):
        template = validation.read_json(validation.ROOT / "tests/mode3/new_motor.template.json")
        with self.assertRaises(ValueError): validation.validate_profile(template)
        for section, key, value in [("motion", "cruise_deg_s", float("nan")),
                                    ("runtime", "current_limit_A", 0),
                                    ("runtime", "cascade_pos_Kp", 51),
                                    ("core", "hold_exit_deg", .31),
                                    ("acceptance", "maximum_iq_A", True),
                                    ("acceptance", "maximum_drop_samples", .5),
                                    ("dwell_seconds", "small", 2)]:
            p = copy.deepcopy(self.profile); p[section][key] = value
            with self.subTest(key=key), self.assertRaises(ValueError): validation.validate_profile(p)

    def fixture(self, folder, mutate=None, rows_mutate=None):
        path = Path(folder)
        trial = {"arguments": {"targets": "0", "seconds": 3}, "failure": None,
                 "runtime_parameters": {"cascade_pos_Kp": 8, "cascade_pos_Kd": 2,
                                        "speed_Kp": .5, "speed_Ki": 1, "pos_maxspeed": math.pi / 4},
                 "shutdown_verified": True, "shutdown_error": None,
                 "image": {"hex_sha256": "synthetic-fixture-not-a-firmware-image"},
                 "events": [{"opcode": 3, "frame": 0, "value": 0, "time": 0,
                             "result": 0, "error": 0, "mode": 3}],
                 "polls": [{"frame": 0, "time": 0}, {"frame": 60, "time": 3}]}
        rows = [[0] * 11 + [130] for _ in range(60)]
        if mutate: mutate(trial)
        if rows_mutate: rows_mutate(rows)
        validation.write_json(path / "trial.json", trial)
        with (path / "capture.tsv").open("w", newline="", encoding="utf-8") as stream:
            w = csv.writer(stream, delimiter="\t")
            w.writerow([f"rtt_channel1.data{i}" for i in range(12)]); w.writerows(rows)
        return path

    def test_good_record_passes_only_automatic_checks(self):
        with tempfile.TemporaryDirectory() as folder:
            r = validation.analyze_trial(validation.DEFAULT_PROFILE, self.fixture(folder))
            self.assertTrue(r["automatic_pass"], r["errors"])
            self.assertIn("NOT_EVALUATED", r["mechanical_vibration"])

    def test_stop_failure_missing_commands_and_image_cannot_pass(self):
        mutations = [lambda t: t.update(shutdown_verified=False),
                     lambda t: t.update(failure="heartbeat expired"),
                     lambda t: t.update(events=[]), lambda t: t.update(image=None),
                     lambda t: t["events"][0].update(result=1),
                     lambda t: t["arguments"].update(targets="5"),
                     lambda t: t.update(runtime_parameters={}),
                     lambda t: t["runtime_parameters"].update(speed_Kp=.1)]
        for mutate in mutations:
            with tempfile.TemporaryDirectory() as folder:
                r = validation.analyze_trial(validation.DEFAULT_PROFILE, self.fixture(folder, mutate))
                self.assertFalse(r["automatic_pass"])

    def test_invalid_tail_drop_current_and_hold_are_failures(self):
        for channel, value in [(11, 2), (11, 194), (11, 138), (11, 128), (7, 4000), (1, 100)]:
            with self.subTest(channel=channel, value=value), tempfile.TemporaryDirectory() as folder:
                def change(rows): rows[-1][channel] = value
                r = validation.analyze_trial(validation.DEFAULT_PROFILE, self.fixture(folder, rows_mutate=change))
                self.assertFalse(r["automatic_pass"])

    def test_truncated_data_and_bad_anchors_do_not_pass(self):
        with tempfile.TemporaryDirectory() as folder:
            path = self.fixture(folder, lambda t: t["polls"][-1].update(time=1))
            self.assertFalse(validation.analyze_trial(validation.DEFAULT_PROFILE, path)["automatic_pass"])
        with tempfile.TemporaryDirectory() as folder:
            path = self.fixture(folder, lambda t: t["polls"][-1].update(frame=61))
            with self.assertRaises(ValueError): validation.analyze_trial(validation.DEFAULT_PROFILE, path)

    def test_requested_case_must_match_recorded_sequence(self):
        with tempfile.TemporaryDirectory() as folder:
            result = validation.analyze_trial(validation.DEFAULT_PROFILE, self.fixture(folder), "range")
            self.assertFalse(result["automatic_pass"])


if __name__ == "__main__":
    unittest.main()
