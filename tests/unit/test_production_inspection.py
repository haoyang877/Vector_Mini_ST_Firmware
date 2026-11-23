"""整轮检测的门禁、报告归档和角度采集；不访问硬件。"""

import json
import math
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools/bench"))
import production_inspection as inspection
import production_power_check as power_check
from hall_waveform_check import hall_signal_limits
from hardware_check import direction_results, empty_report
from high_speed_curves import save_high_speed_curves
from inspection_angle_trace import angle_preview, rotor_continuity
from production_power_check import power_check_fields, power_window


class ProductionInspectionTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.out = Path(directory.name)
        self.uid = "01" * 12
        self.settings = dict(
            node=7,
            expected_uid=self.uid,
            numbers=inspection.inspection_numbers("YAW-0001", "REQ-7"),
            power_check=power_check_fields(7),
            ambient=20,
            last_lease=0,
            last_target=0,
        )
        self.report = inspection.initial_inspection(self.settings)
        self.events = []

    def preflight(self, *, hall=True, passed=True):
        return dict(
            empty_report(7),
            uid=self.uid,
            app_version=0x1081B,
            loader_version=0x20304,
            subset_passed=passed,
            stop_confirmed=True,
            communication_reliability_passed=True,
            output_coordinates=False,
            scope="static_resistance",
            board_items=[1] * 7 + [1 if hall else 3, 0, 0],
        )

    def run_round(self, *, stopped=lambda: False):
        inspection.run_inspection(
            None,
            self.settings,
            self.out,
            self.report,
            limits=dict(limits=dict(hall_raw_min=95, hall_raw_max=4000)),
            stopped=stopped,
            emit=lambda tag, value: self.events.append((tag, value)),
        )

    def test_preflight_failure_cannot_start_calibration(self):
        with (
            patch.object(
                inspection, "run_check", return_value=self.preflight(passed=False)
            ) as check,
            patch.object(inspection, "run_motor_calibration") as calibrate,
        ):
            self.run_round()
        check.assert_called_once()
        calibrate.assert_not_called()
        inspection.archive_inspection(self.report, self.out)
        saved = json.loads((self.out / "summary.json").read_text(encoding="utf-8"))
        self.assertEqual(saved["phase_states"]["preflight"], "未通过")
        self.assertEqual(saved["phase_states"]["calibration"], "未测")
        self.assertEqual(saved["motor_serial"], "YAW-0001")

    def test_preflight_publishes_each_completed_static_sample_once(self):
        def check(_bus, _node, *, static_progress, **_kwargs):
            snapshot = empty_report(7)
            snapshot.update(static_completed=1, board_items=[2, 1] + [0] * 8)
            snapshot["measurements"] = {"0": dict(samples=100, values=[2048, 12, None, None])}
            direction_results(snapshot)
            static_progress(snapshot)
            static_progress(snapshot)
            return self.preflight(passed=False)

        with patch.object(inspection, "run_check", side_effect=check):
            self.run_round()
        partial = [
            rows
            for tag, rows in self.events
            if tag == "inspection_results" and rows[3]["result"] == "进行中"
        ]
        self.assertEqual(len(partial), 1)
        self.assertEqual(partial[0][2]["item"], "Ia")
        self.assertEqual(partial[0][2]["outcome"], "failed")
        self.assertIn("12.00", partial[0][2]["measured"])
        self.assertEqual(partial[0][3]["measured"], "—")

    def test_calibration_without_confirmed_stop_blocks_high_speed_test(self):
        with (
            patch.object(inspection, "run_check", return_value=self.preflight()) as check,
            patch.object(
                inspection,
                "run_motor_calibration",
                return_value=dict(passed=True, stop_confirmed=False),
            ),
        ):
            self.run_round()
        check.assert_called_once()
        self.assertEqual(self.report["phase_states"]["calibration"], "未通过")
        self.assertEqual(self.report["phase_states"]["power"], "未测")
        self.assertIs(self.report["stop_confirmed"], False)

    def test_hall_dent_allows_high_speed_and_keeps_round_for_review(self):
        calibration = dict(
            passed=True,
            stop_confirmed=True,
            communication_reliability_passed=True,
            curves=dict(calibration_flags=3, saved_revision=7),
            hall_waveform=dict(
                status="detected",
                extrema_counts=dict(sin=[319, 3834], cos=[365, 3848]),
                sectors=[dict(start_deg=35, width_deg=15, repeat_count=3, max_chord_depth=0.0600)],
            ),
        )
        with (
            patch.object(inspection, "run_check", return_value=self.preflight()) as check,
            patch.object(
                inspection, "run_motor_calibration", return_value=calibration
            ) as calibrate,
            patch.object(
                inspection,
                "run_power_check",
                return_value=dict(
                    status="passed",
                    speed_stability=dict(status="passed"),
                    rotor_continuity=dict(status="clear"),
                    run=dict(enable_attempted=True),
                    stop_confirmed=True,
                ),
            ) as speed_test,
        ):
            self.run_round()
        calibrate.assert_called_once()
        check.assert_called_once()
        speed_test.assert_called_once()
        self.assertEqual(speed_test.call_args.kwargs["saved_revision"], 7)
        self.assertEqual(self.report["verdict"], "待复核")
        self.assertFalse(self.report["passed"])
        self.assertEqual(self.report["phase_states"]["calibration"], "待复核")
        self.assertEqual(self.report["phase_states"]["power"], "通过")
        rows = {row["item"]: row for row in inspection.inspection_overview(self.report)}
        self.assertEqual(rows["Hall 波形凹坑"]["result"], "待复核")
        self.assertEqual(rows["输出轴转速稳定性"]["result"], "通过")
        self.assertEqual(rows["高速输入功率稳定性"]["result"], "通过")
        inspection.archive_inspection(self.report, self.out)
        html = (self.out / "production-report.html").read_text(encoding="utf-8")
        self.assertIn("35～50°（3次，深度 0.0600）", html)
        self.assertIn("局部弦深度均 ≥ 0.020", html)
        self.assertTrue(self.report["stop_confirmed"])

    def test_verified_hall_failure_keeps_raw_evidence_and_runs_speed(self):
        failure = dict(
            passed=False,
            last_calibration_stage=0x1004,
            phase=5,
            effects=2,
            stop_confirmed=True,
            stop_faults=0x4000,
            communication_reliability_passed=True,
            capabilities=1 << 24,
            fatal="已关相，电流校零/角度标定故障（0x00004000）",
        )
        diag = dict(
            details=["Hall sin 最小值过低", "Hall sin 最大值过高"],
            hall_fault_mask=6,
            calibration_flags=3,
            saved_revision=7,
        )

        def record_failure(*_args, **_kwargs):
            (self.out / "calibration/hall-raw.csv").write_text(
                "nominal_elapsed_s,hall_sin_counts,hall_cos_counts\n0,4,74\n0.01,4095,4021\n",
                encoding="utf-8",
            )
            return failure

        with (
            patch.object(inspection, "run_check", return_value=self.preflight()),
            patch.object(inspection, "run_angle_calibration", side_effect=record_failure),
            patch.object(inspection, "HallRawCapture") as capture,
            patch.object(inspection, "read_committed_calibration_fault", return_value=diag) as read,
            patch.object(inspection, "export_calibration_curves") as compensation,
            patch.object(
                inspection,
                "run_power_check",
                return_value=dict(
                    status="passed",
                    speed_stability=dict(status="passed"),
                    rotor_continuity=dict(status="clear"),
                    run=dict(enable_attempted=True),
                    stop_confirmed=True,
                ),
            ) as high_speed,
        ):
            capture.return_value.finish.return_value = dict(
                source="CAN Hall snapshots", samples=2, csv="hall-raw.csv"
            )
            self.run_round()
        read.assert_called_once()
        compensation.assert_not_called()
        high_speed.assert_called_once()
        self.assertEqual(high_speed.call_args.kwargs["hall_fault"], 0x4000)
        self.assertEqual(high_speed.call_args.kwargs["saved_revision"], 7)
        self.assertFalse(self.report["passed"])
        self.assertEqual(self.report["verdict"], "未通过")
        self.assertEqual(self.report["phase_states"]["power"], "通过")
        self.assertIn("联合电角零位", self.report["fatal"])
        self.assertIn("板端回读 Hall sin 最小值过低、Hall sin 最大值过高", self.report["fatal"])
        self.assertEqual(
            self.report["hall_signal"]["peak_to_peak_counts"], {"sin": 4091, "cos": 3947}
        )
        rows = {row["item"]: row for row in inspection.inspection_overview(self.report)}
        self.assertEqual(rows["编码器标定"]["result"], "已保存")
        self.assertNotIn("Hall", rows["编码器标定"]["measured"])
        self.assertEqual(rows["Hall 标定"]["result"], "未通过")
        self.assertIn("Hall sin 最小值过低", rows["Hall 标定"]["measured"])
        self.assertEqual(rows["Hall 最大值 / 最小值"]["result"], "未通过")
        self.assertEqual(rows["Hall 峰峰值"]["result"], "通过")
        self.assertIn("ADC 端点", rows["Hall 波形凹坑"]["measured"])
        self.assertTrue((self.out / "calibration/hall-raw.png").is_file())
        inspection.archive_inspection(self.report, self.out)
        html = (self.out / "production-report.html").read_text(encoding="utf-8")
        self.assertIn('href="calibration/hall-raw.png"', html)
        self.assertIn("<td>编码器标定</td>", html)
        self.assertIn("<td>Hall 标定</td>", html)

    def test_calibration_rows_preserve_stage_and_unattributed_fault(self):
        report = dict(
            profile="production_inspection",
            phase_states={"calibration": "未通过"},
            fatal="共用故障位 0x00004000",
            calibration=dict(last_calibration_stage=0x1004),
        )
        rows = {row["item"]: row for row in inspection.inspection_overview(report)}
        self.assertEqual(rows["编码器标定"]["result"], "待复核")
        self.assertIn("角度补偿已完成", rows["编码器标定"]["measured"])
        self.assertEqual(rows["Hall 标定"]["result"], "待复核")
        self.assertIn("Hall 标定结果未确认", rows["Hall 标定"]["measured"])
        report["calibration"] = dict(last_calibration_stage=0x1001, fatal="编码器补偿表生成失败")
        rows = {row["item"]: row for row in inspection.inspection_overview(report)}
        self.assertEqual(rows["编码器标定"]["result"], "未通过")
        self.assertEqual(rows["编码器标定"]["measured"], "编码器补偿表生成失败")
        self.assertEqual(rows["Hall 标定"]["result"], "未测")

    def test_hall_out_of_range_or_missing_waveform_does_not_block_speed(self):
        cases = {
            "out-of-range": (
                dict(status="detected", extrema_counts=dict(sin=[50, 3800], cos=[220, 3750])),
                "未通过",
                "未通过",
            ),
            "not-measured": (dict(status="not_measured"), "待复核", "待复核"),
        }
        for name, (waveform, stage, verdict) in cases.items():
            with self.subTest(name=name):
                out = self.out / name
                out.mkdir()
                calibration = dict(
                    passed=True,
                    stop_confirmed=True,
                    communication_reliability_passed=True,
                    curves=dict(calibration_flags=3, saved_revision=7),
                    hall_waveform=waveform,
                )
                with (
                    patch.object(self, "out", out),
                    patch.object(self, "report", inspection.initial_inspection(self.settings)),
                    patch.object(inspection, "run_check", return_value=self.preflight()),
                    patch.object(inspection, "run_motor_calibration", return_value=calibration),
                    patch.object(
                        inspection,
                        "run_power_check",
                        return_value=dict(
                            status="passed",
                            speed_stability=dict(status="passed"),
                            rotor_continuity=dict(status="clear"),
                            run=dict(enable_attempted=True),
                            stop_confirmed=True,
                        ),
                    ) as speed_test,
                ):
                    self.run_round()
                    speed_test.assert_called_once()
                    self.assertEqual(speed_test.call_args.kwargs["hall_fault"], 0)
                    self.assertEqual(self.report["phase_states"]["calibration"], stage)
                    self.assertEqual(self.report["phase_states"]["power"], "通过")
                    self.assertEqual(self.report["verdict"], verdict)

    def test_hall_failure_without_confirmed_encoder_zero_or_fault_detail_still_blocks_speed(self):
        for diagnostic in (
            dict(calibration_flags=1, saved_revision=7, hall_fault_mask=6),
            dict(calibration_flags=3, saved_revision=7),
        ):
            with self.subTest(diagnostic=diagnostic), tempfile.TemporaryDirectory() as directory:
                calibration = dict(
                    passed=False,
                    stop_confirmed=True,
                    communication_reliability_passed=True,
                    phase=5,
                    effects=2,
                    stop_faults=0x4000,
                    saved_calibration_fault=diagnostic,
                )
                with (
                    patch.object(self, "out", Path(directory)),
                    patch.object(self, "report", inspection.initial_inspection(self.settings)),
                    patch.object(inspection, "run_check", return_value=self.preflight()),
                    patch.object(inspection, "run_motor_calibration", return_value=calibration),
                    patch.object(inspection, "run_power_check") as speed_test,
                ):
                    self.run_round()
                    speed_test.assert_not_called()
                    self.assertEqual(self.report["phase_states"]["calibration"], "未通过")

    def test_wheel_runs_independent_zero_then_accepts_no_hall(self):
        calibration = dict(
            passed=True,
            stop_confirmed=True,
            communication_reliability_passed=True,
            curves=dict(calibration_flags=1, saved_revision=7),
            hall_waveform=dict(status="not_measured"),
        )
        zero = dict(calibration, curves=dict(calibration_flags=3, saved_revision=8))
        with (
            patch.object(inspection, "run_check", return_value=self.preflight(hall=False)),
            patch.object(
                inspection, "run_motor_calibration", side_effect=[calibration, zero]
            ) as calibrate,
            patch.object(
                inspection,
                "run_power_check",
                return_value=dict(
                    status="incomplete",
                    run=dict(enable_attempted=True),
                    stop_confirmed=True,
                ),
            ) as speed_test,
        ):
            self.run_round()
        self.assertEqual(
            [call.args[1]["calibration_kind"] for call in calibrate.call_args_list], [1, 2]
        )
        self.assertEqual(speed_test.call_args.kwargs["saved_revision"], 8)
        rows = {row["item"]: row for row in inspection.inspection_overview(self.report)}
        self.assertEqual(rows["编码器标定"]["result"], "已保存")
        self.assertEqual(rows["Hall 标定"]["result"], "未装")
        self.assertEqual(self.report["verdict"], "待复核")  # 本例未提供任何动态角度。

    def test_production_uses_one_timed_speed_run_for_angles_and_stability(self):
        def speed_run(
            _bus,
            node,
            target,
            travel,
            _commands,
            out,
            *,
            output_test,
            expected_uid,
            feedback,
            **_kwargs,
        ):
            self.assertEqual(node, 7)
            self.assertEqual(expected_uid, self.uid)
            self.assertIsNone(travel)
            self.assertEqual(output_test.duration_s, 10)
            self.assertEqual(output_test.settling.hold_s, 0.5)
            self.assertEqual(output_test.calibration_revision, 6)
            self.assertAlmostEqual(target * 60 / math.tau, 50)
            motor_speed = target * 40
            for i in range(260):
                feedback(
                    dict(
                        elapsed_s=i / 20,
                        motion_coordinate="motor_shaft",
                        target_rad_s=motor_speed,
                        position_rad=motor_speed * i / 20,
                        actual_rad_s=motor_speed * min(1, i / 50),
                        bus_voltage_v=28,
                        bus_current_a=1,
                        output_rpm=motor_speed / 40 * 60 / math.tau * min(1, i / 50),
                        bus_input_power_w=28,
                    )
                )
            (out / "speed.csv").write_text(
                "elapsed_s,output_rpm,bus_input_power_w\n2.95,49,25\n3.05,50,19\n",
                encoding="utf-8",
            )
            return dict(
                passed=True,
                stop_confirmed=True,
                enable_attempted=True,
                communication_reliability_passed=True,
                motion_coordinate="motor_shaft",
                gear_ratio=40,
                steady_window_s=(3, 13),
            )

        calibration = dict(
            passed=True,
            stop_confirmed=True,
            communication_reliability_passed=True,
            curves=dict(calibration_flags=3, saved_revision=6),
            hall_waveform=dict(
                status="clear", extrema_counts=dict(sin=[200, 3800], cos=[220, 3750])
            ),
        )
        with (
            patch.object(inspection, "run_check", return_value=self.preflight()) as check,
            patch.object(inspection, "run_motor_calibration", return_value=calibration),
            patch.object(power_check, "run_speed_control", side_effect=speed_run) as speed_test,
        ):
            self.run_round()
        check.assert_called_once()
        self.assertTrue(check.call_args.kwargs["resistance_only"])
        speed_test.assert_called_once()
        self.assertNotIn("recheck", self.report)
        self.assertEqual(self.report["power_check"]["window_s"], 10)
        self.assertEqual(self.report["power_check"]["rotor_continuity"]["status"], "clear")
        self.assertEqual(self.report["power_check"]["steady_window_s"], (3, 13))
        self.assertEqual(sum(tag == "inspection_angle" for tag, _ in self.events), 260)
        self.assertEqual(self.report["phase_states"]["power"], "通过")
        self.assertEqual(self.report["phase_states"]["archive"], "关相已确认")
        self.assertTrue(self.report["passed"])
        self.report["calibration"]["raw_capture"] = dict(
            source="CAN Hall snapshots", csv="hall-raw.csv", samples=2
        )
        calibration_dir = self.out / "calibration"
        (calibration_dir / "hall-raw.csv").write_text(
            "nominal_elapsed_s,hall_sin_counts,hall_cos_counts\n0,100,4000\n1,3900,200\n",
            encoding="utf-8",
        )
        (calibration_dir / "angle-calibration.csv").write_text(
            "raw_angle_deg,motor_raw_minus_corrected_deg\n0,0.1\n180,0.3\n",
            encoding="utf-8",
        )
        figures = {}

        def capture(canvas, path):
            figures[path.name] = canvas.figure
            path.touch()

        with patch("matplotlib.backends.backend_agg.FigureCanvasAgg.print_png", new=capture):
            inspection.archive_inspection(self.report, self.out)
        filename = self.report["waveform_overview"]
        self.assertEqual(figures[filename].canvas.get_width_height(), (3900, 4500))
        raw, xy, hall, motor, full, steady, power = figures[filename].axes
        sin, cos = raw.lines
        self.assertEqual(list(sin.get_xdata()), [0, 1])
        self.assertEqual(list(cos.get_xdata()), [0, 1])
        self.assertEqual(list(sin.get_ydata()), [100, 3900])
        self.assertEqual(list(cos.get_ydata()), [4000, 200])
        self.assertNotEqual(sin.get_color(), cos.get_color())
        self.assertEqual(
            [label.get_text() for label in raw.get_legend().get_texts()],
            ["sin  最小 100 · 最大 3900 cnt", "cos  最小 200 · 最大 4000 cnt"],
        )
        self.assertEqual(list(xy.lines[0].get_ydata()), [4000, 200])
        self.assertIn("本轮没有有效的 Hall 补偿表", [text.get_text() for text in hall.texts])
        self.assertEqual(list(motor.lines[0].get_ydata()), [0.1, 0.3, 0.1])
        self.assertEqual(list(full.lines[0].get_ydata()), [49, 50])
        self.assertEqual(steady.get_xlim(), (3, 13))
        self.assertEqual(list(power.lines[1].get_ydata()), [22, 22])
        html = (self.out / "production-report.html").read_text(encoding="utf-8")
        self.assertIn('href="high-speed/speed.csv"', html)
        self.assertIn(f'src="{filename}"', html)
        self.assertEqual(
            self.report["power_check"]["curves"],
            {"speed": "high-speed/output-speed.png", "power": "high-speed/input-power.png"},
        )
        for filename in self.report["power_check"]["curves"].values():
            self.assertIn(f'href="{filename}"', html)
            self.assertTrue((self.out / filename).read_bytes().startswith(b"\x89PNG\r\n\x1a\n"))

    def test_power_filter_respects_sample_times_and_gaps_and_speed_zoom_shows_outliers(self):
        samples = [
            dict(elapsed_s=time, bus_input_power_w=value)
            for time, value in (
                (0, 10),
                (0.1, 20),
                (0.26, 30),
                (0.51, 100),
                (0.61, None),
                (0.71, 200),
            )
        ] + [
            dict(elapsed_s=time, output_rpm=value)
            for time, value in ((1.9, 120), (2, 200), (2.1, 201), (2.2, 218), (2.3, 184))
        ]
        figures = {}

        def capture(canvas, path):
            figures[path.name] = canvas.figure

        with patch("matplotlib.backends.backend_agg.FigureCanvasAgg.print_png", new=capture):
            save_high_speed_curves(
                samples,
                self.out,
                power_check_fields(3),
                window=(2, 12),
                max_gap_s=0.2,
                speed_error_fraction=0.05,
            )
        raw, filtered = figures["input-power.png"].axes[0].lines
        self.assertEqual(list(raw.get_ydata()[:3]), [10, 20, 30])
        self.assertEqual(list(filtered.get_ydata()[:3]), [15, 20, 25])
        self.assertTrue(math.isnan(filtered.get_ydata()[3]))  # 时间间隔过长
        self.assertEqual(filtered.get_ydata()[4], 100)
        self.assertTrue(math.isnan(filtered.get_ydata()[5]))  # 缺少功率值
        self.assertEqual(filtered.get_ydata()[6], 200)

        full, zoom = figures["output-speed.png"].axes
        self.assertIn(120, full.lines[0].get_ydata())
        self.assertEqual(zoom.get_xlim(), (2, 12))
        self.assertEqual(zoom.get_ylim(), (185, 215))
        self.assertEqual([line.get_ydata()[0] for line in zoom.lines[:3]], [190, 200, 210])
        self.assertEqual(list(zoom.lines[3].get_ydata()), [200, 201, 218, 184])
        self.assertEqual([marker.get_offsets()[0, 0] for marker in zoom.collections], [2.3, 2.2])
        self.assertIn("Peak to peak 34.00 rpm", zoom.get_title(loc="left"))

    def test_stopped_run_keeps_sampled_speed_without_inventing_missing_power(self):
        def speed_run(*_args, feedback, **_kwargs):
            for elapsed in (0.0, 0.3):
                feedback(
                    dict(
                        elapsed_s=elapsed,
                        position_rad=0,
                        output_rpm=35,
                        motion_coordinate="motor_shaft",
                    )
                )
            return dict(stop_confirmed=True)

        with patch.object(power_check, "run_speed_control", side_effect=speed_run):
            result = power_check.run_power_check(
                None,
                self.settings,
                self.out,
                saved_revision=6,
                stopped=lambda: True,
                progress=lambda _message: None,
                angle_sample=lambda _sample: None,
            )
        self.assertEqual(result["status"], "cancelled")
        self.assertEqual(result["curves"], {"speed": "high-speed/output-speed.png"})
        self.assertTrue((self.out / result["curves"]["speed"]).is_file())

    def test_invalid_archive_components_are_rejected(self):
        for value in ("", "../X", "A/B", "CON", "COM¹", "a.b"):
            with self.subTest(value=value), self.assertRaises(ValueError):
                inspection.inspection_numbers("SN", value)

    def test_hall_range_and_half_adc_span_are_independent(self):
        waveform = dict(extrema_counts=dict(sin=[50, 2098], cos=[1000, 3047]))
        result = hall_signal_limits(waveform, dict(hall_raw_min=95, hall_raw_max=4000))
        self.assertFalse(result["range_passed"])
        self.assertFalse(result["amplitude_passed"])
        self.assertEqual(result["peak_to_peak_counts"], dict(sin=2048, cos=2047))
        waveform["extrema_counts"] = dict(sin=[95, 2143], cos=[1000, 3048])
        result = hall_signal_limits(waveform, dict(hall_raw_min=95, hall_raw_max=4000))
        self.assertTrue(result["range_passed"] and result["amplitude_passed"])

    def test_output_speed_recipe_and_stable_power_require_complete_samples(self):
        self.assertEqual(
            power_check_fields(3),
            dict(
                output_rpm=200,
                power_min_w=-1000,
                power_max_w=1000,
            ),
        )
        self.assertEqual(power_check_fields(7)["output_rpm"], 50)
        with self.assertRaisesRegex(ValueError, "下限"):
            power_check_fields(7, rpm="50", min_w="100", max_w="10")
        with self.assertRaisesRegex(ValueError, "限"):
            power_check_fields(7, rpm="63")
        feedback = [
            dict(
                elapsed_s=i / 20,
                bus_voltage_v=28,
                bus_current_a=1,
                actual_rad_s=50 * math.tau / 60 * 40,
            )
            for i in range(40, 240)
        ]
        result = power_window(feedback, 1 / 40, 50, window=(2, 12), min_w=27, max_w=30)
        self.assertEqual(result["status"], "passed")
        self.assertEqual(result["speed_stability"]["status"], "passed")
        self.assertEqual(
            power_window(feedback[:60], 1 / 40, 50, window=(2, 12), min_w=27, max_w=30)[
                "speed_stability"
            ]["status"],
            "incomplete",
        )
        self.assertAlmostEqual(result["mean_w"], 28)
        feedback[-1]["bus_current_a"] = 1.2
        feedback[-2]["bus_current_a"] = 0.5
        result = power_window(feedback, 1 / 40, 50, window=(2, 12), min_w=27, max_w=30)
        self.assertEqual(result["status"], "failed")
        self.assertEqual(result["speed_stability"]["status"], "passed")
        self.assertEqual(result["reason"], "最小功率低于下限、最大功率超过上限")
        feedback[-1]["bus_current_a"] = None
        result = power_window(feedback, 1 / 40, 50, window=(2, 12), min_w=27, max_w=30)
        self.assertEqual(result["status"], "incomplete")
        self.assertEqual(result["speed_stability"]["status"], "passed")

    def test_output_speed_stability_use_all_samples_and_include_five_percent_boundaries(self):
        feedback = [
            dict(
                elapsed_s=i / 20,
                bus_voltage_v=28,
                bus_current_a=1,
                actual_rad_s=200 * math.tau / 60,
            )
            for i in range(40, 240)
        ]
        feedback[0]["actual_rad_s"] = 190 * math.tau / 60
        feedback[-1]["actual_rad_s"] = 210 * math.tau / 60
        result = power_window(feedback, 1.0, 200, window=(2, 12), min_w=26, max_w=30)
        self.assertEqual(result["speed_stability"]["status"], "passed")
        self.assertAlmostEqual(result["speed_stability"]["max_error_pct"], 5.0)
        feedback[-1]["actual_rad_s"] = 210.02 * math.tau / 60
        result = power_window(feedback, 1.0, 200, window=(2, 12), min_w=26, max_w=30)
        self.assertEqual(result["speed_stability"]["status"], "failed")
        self.assertAlmostEqual(result["speed_stability"]["max_rpm"], 210.02)
        self.assertEqual(result["status"], "incomplete")
        self.assertAlmostEqual(result["max_w"], 28)

    def test_speed_settling_timeout_reports_speed_failure_and_leaves_power_unmeasured(self):
        result = dict(
            settling_timed_out=True,
            stop_confirmed=True,
            communication_reliability_passed=True,
            fatal="10 秒内转速未连续 0.5 秒进入目标 ±5%",
        )
        with patch.object(power_check, "run_speed_control", return_value=result):
            measured = power_check.run_power_check(
                None,
                self.settings,
                self.out,
                saved_revision=6,
                stopped=lambda: False,
                progress=lambda _text: None,
                angle_sample=lambda _sample: None,
            )
        self.assertEqual(measured["status"], "not_measured")
        self.assertEqual(measured["speed_stability"]["status"], "failed")
        self.assertEqual(measured["speed_stability"]["reason"], result["fatal"])
        self.assertNotIn("steady_window_s", measured)

    def test_rotor_continuity_detects_jump_and_keeps_gaps(self):
        samples = [
            dict(
                elapsed_s=2 + i * 0.05,
                motion_coordinate="motor_shaft",
                position_rad=i * 0.15,
                target_rad_s=3,
            )
            for i in range(4)
        ]
        self.assertEqual(rotor_continuity(samples, covered=True, max_gap_s=0.2)["status"], "clear")
        samples[-1]["position_rad"] = 3
        self.assertEqual(rotor_continuity(samples, covered=True, max_gap_s=0.2)["jumps"], 1)
        samples[-1]["position_rad"] = None
        self.assertEqual(
            rotor_continuity(samples, covered=True, max_gap_s=0.2)["status"], "incomplete"
        )

    def test_output_angle_is_displayed_but_does_not_prove_motor_continuity(self):
        sample = dict(elapsed_s=2, motion_coordinate="output_shaft", position_rad=math.pi)
        self.assertEqual(
            angle_preview(sample, 0),
            dict(elapsed_s=2, reference_source="output_tracking", reference_deg=180),
        )
        self.assertEqual(
            rotor_continuity([sample], covered=True, max_gap_s=0.2)["status"], "not_measured"
        )
