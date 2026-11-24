"""标定票据和速度会话的协议/故障回归，替身不访问SDK或硬件。"""

import itertools
import queue
import struct
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools/bench"))
import can_motor_operations as ops
import hardware_check as hardware
import production_backend as desktop
from production_backend import motor_speed_fields


class Endpoint:
    def __init__(self):
        self.node, self.sequence = 6, 100
        self.timeouts, self.commands, self.requests = {}, [], []
        self.state, self.mode, self.actual_speed = 5, 0, 0
        self.phases, self.revision = [(1, 1), (2, 2)], 1
        self.starts, self.cancelled = 0, False
        self.cal_kind = 1
        self.fault = ""
        self.fault_bits = 0
        self.status_losses = 0
        self.speed_limit = 4000
        self.features = 0
        self.position = 1000
        self.position_after_target = None
        self.status_coordinates = []
        self.invalid_output = False

    def status(self, *, motor_axis=False):
        self.status_coordinates.append(motor_axis)
        data = bytearray(45)
        struct.pack_into("<I", data, 4, self.fault_bits)
        struct.pack_into("<i", data, 12, self.position)
        struct.pack_into("<i", data, 20, round(self.actual_speed * 1000))
        valid = 0x29A
        if (self.invalid_output and not motor_axis) or (
            motor_axis and self.fault == "motor_feedback"
        ):
            valid &= ~10
        struct.pack_into("<H", data, 24, valid)
        struct.pack_into("<h", data, 32, 50)
        data[42:] = bytes((6, self.state, self.mode))
        return bytes(data)

    def command(self, kind, payload=b"", *, sequence=None):
        self.commands.append((kind, payload, sequence))
        if kind == 117:
            self.state, self.mode = 6, 2
            if self.fault == "start_ack":
                self.timeouts[117] = 1
                raise TimeoutError("start ACK lost")
        elif kind == 116:
            mode, position, speed, iq, vq = struct.unpack("<B3x4i", payload)
            if (mode, position, iq, vq) != (2, 0, 0, 0):
                raise ValueError("速度目标线路字段不符")
            self.actual_speed = speed / 1000
            if self.position_after_target is not None:
                self.position = self.position_after_target
        elif kind == 118 and self.fault == "lease_ack":
            self.timeouts[118] = 1
            raise TimeoutError("lease ACK lost")
        elif kind == 110:
            if self.fault == "stop_pending":
                return 1
            self.state, self.mode = 5, 0
        return 0

    def request(self, kind, payload=b"", *, page=None):
        self.requests.append((kind, payload, page))
        if kind == 1:
            data = bytearray(32)
            if page == 0:
                data[:12] = b"\x01" * 12
            else:
                data[21:24] = bytes((6, 1, 1))
            return bytes(data)
        if kind == 2:
            return struct.pack(
                "<I6H",
                self.features
                | (1 << 13)
                | (1 << 14)
                | (0 if self.fault == "capability" else 1 << 15),
                4,
                64,
                46,
                0,
                500,
                500,
            )
        if kind == 107:
            if self.fault == "clear_denied":
                return struct.pack("<HH", 7, 1)
            self.fault_bits, self.state = 0, 5
            return bytes(4)
        if kind in (120, 123):
            identifier = payload[0]
            saved_revision = self.revision + (self.fault == "saved_revision" and kind == 123)
            value = {0: self.revision, 1: self.speed_limit, 2: 3000}[identifier]
            return struct.pack(
                "<HHBBHIII",
                0,
                0,
                identifier,
                2 if identifier == 0 else 1,
                1,
                value,
                self.revision,
                saved_revision,
            )
        if kind == 126:
            data = bytearray(32)
            data[4] = 1
            struct.pack_into("<i", data, 24, 40000)
            return bytes(data)
        if kind == 121:
            identifier, value_type, size, zero, revision, value = struct.unpack("<4BIi", payload)
            if (identifier, value_type, size, zero, revision) != (1, 1, 4, 0, self.revision):
                raise ValueError("速度上限写入字段错误")
            if self.fault == "limit_denied":
                return struct.pack("<HH", 11, 4)
            self.speed_limit = value
            self.revision += 1
            if self.fault == "limit_ack":
                self.timeouts[121] = 1
                raise TimeoutError("limit ACK lost")
            return struct.pack("<HHBBHIII", 0, 0, 1, 1, 3, value, self.revision, 1)
        if kind == 160:
            self.cal_kind = struct.unpack("<HH", payload)[0]
            self.starts += 1
            self.state = 4
            if self.fault == "cal_ack":
                self.timeouts[160] = 1
                raise TimeoutError("calibration ACK lost")
            if self.fault == "stale" and self.starts == 1:
                return ops.JOB.pack(5, 5, 0, 0, 0, 0)
            return ops.JOB.pack(1, 0, 7, self.cal_kind, 1, 1)
        if kind == 162:
            self.cancelled = True
            return ops.JOB.pack(1, 0, 7, self.cal_kind, 3, 1)
        if kind == 161:
            if self.status_losses:
                self.status_losses -= 1
                self.timeouts[161] = self.timeouts.get(161, 0) + 1
                raise TimeoutError("job status delayed during Flash save")
            phase, effects = (4, 1) if self.cancelled else self.phases.pop(0)
            if phase == 2:
                self.revision, self.state = 2, 5
            return ops.JOB.pack(
                0,
                7 if phase == 5 else 0,
                8 if self.fault == "ticket" else 7,
                self.cal_kind,
                phase,
                effects,
            )
        raise ValueError(kind)


class MotorOperationsTests(unittest.TestCase):
    def setUp(self):
        self.endpoint = Endpoint()
        self.link = patch.object(ops, "Link", return_value=self.endpoint)
        self.link.start()
        self.addCleanup(self.link.stop)
        self.sleep = patch.object(ops.time, "sleep")
        self.sleep.start()
        self.addCleanup(self.sleep.stop)

    def test_calibration_requires_completed_saved_ticket_and_revision(self):
        report = ops.run_angle_calibration(None, 6)
        ops.Link.assert_called_once_with(None, 6, timeout_s=0.5)
        self.assertEqual(report["reply_timeout_ms"], 500)
        self.assertTrue(report["passed"] and report["stop_confirmed"])
        self.assertEqual((report["phase"], report["effects"]), (2, 2))
        self.assertTrue(any(kind == 123 for kind, *_ in self.endpoint.requests))

    def test_calibration_timeout_does_not_change_default_link_deadline(self):
        class SilentPort:
            def send(self, *_):
                pass

            def receive(self, timeout):
                self.timeout = timeout
                return None

        for timeout in (0.05, 0.5):
            port = SilentPort()
            link = hardware.Link(port, 6, **({"timeout_s": timeout} if timeout == 0.5 else {}))
            with self.assertRaises(TimeoutError):
                link.request(161, struct.pack("<I", 7))
            self.assertAlmostEqual(port.timeout, timeout, delta=0.01)
            self.assertEqual(link.timeouts, {161: 1})

    def test_unknown_start_is_not_replayed_and_stops(self):
        self.endpoint.fault = "cal_ack"
        report = ops.run_angle_calibration(None, 6)
        self.assertFalse(report["passed"])
        self.assertTrue(report["stop_confirmed"])
        self.assertEqual(self.endpoint.starts, 1)
        self.assertEqual(report["communication_timeouts"], {160: 1})

    def test_read_only_job_retry_keeps_timeout_and_does_not_restart(self):
        self.endpoint.status_losses = 1
        report = ops.run_angle_calibration(None, 6)
        self.assertTrue(report["passed"] and report["stop_confirmed"])
        self.assertEqual(self.endpoint.starts, 1)
        self.assertEqual(report["job_status_retries"], 1)
        self.assertEqual(report["communication_timeouts"], {161: 1})
        self.assertFalse(report["communication_reliability_passed"])

    def test_read_only_job_retry_is_bounded_and_stops(self):
        self.endpoint.status_losses = 3
        report = ops.run_angle_calibration(None, 6)
        self.assertFalse(report["passed"])
        self.assertTrue(report["stop_confirmed"])
        self.assertEqual(report["ticket"], 7)
        self.assertEqual(self.endpoint.starts, 1)
        self.assertEqual(report["communication_timeouts"], {161: 3})

    def test_electrical_zero_uses_its_own_ticket_kind(self):
        report = ops.run_angle_calibration(None, 6, kind=2)
        self.assertTrue(report["passed"])
        self.assertIn((160, b"\x02\0\0\0", None), self.endpoint.requests)
        self.assertEqual(report["kind"], 2)

    def test_explicit_stale_rejection_can_choose_a_new_sequence(self):
        self.endpoint.fault = "stale"
        report = ops.run_angle_calibration(None, 6)
        self.assertTrue(report["passed"])
        self.assertEqual(self.endpoint.starts, 2)

    def test_cancel_waits_for_real_terminal_and_stop(self):
        report = ops.run_angle_calibration(
            None, 6, stopped=lambda: any(k == 161 for k, *_ in self.endpoint.requests)
        )
        self.assertTrue(report["cancelled"] and report["stop_confirmed"])
        self.assertFalse(report["passed"])
        self.assertEqual(report["phase"], 4)

    def test_wrong_ticket_or_saved_revision_does_not_pass(self):
        for fault in ("ticket", "saved_revision"):
            with self.subTest(fault=fault):
                self.endpoint.__init__()
                self.endpoint.fault = fault
                report = ops.run_angle_calibration(None, 6)
                self.assertFalse(report["passed"])
                self.assertTrue(report["stop_confirmed"])

    def test_persisted_failure_marker_is_not_calibration_success(self):
        self.endpoint.phases = [(5, 2)]
        report = ops.run_angle_calibration(None, 6)
        self.assertFalse(report["passed"])
        self.assertEqual(report["effects"], 2)

    def test_missing_capability_does_not_start_or_stop_other_work(self):
        self.endpoint.fault = "capability"
        report = ops.run_angle_calibration(None, 6)
        self.assertFalse(report["passed"])
        self.assertEqual(self.endpoint.commands, [])

    def speed_run(self, targets=(), fault="", speed_limit=None, *, target=0, output_target=False):
        self.endpoint.fault = fault
        commands, samples = queue.Queue(), []
        for update in targets:
            commands.put(update)
        with tempfile.TemporaryDirectory() as directory:
            out = Path(directory)
            report = ops.run_speed_control(
                None,
                6,
                target,
                360,
                commands,
                out,
                counter_db=out / "counter.db",
                speed_limit=speed_limit,
                output_target=output_target,
                stopped=lambda: len(samples) >= 2,
                feedback=samples.append,
            )
        return report, samples

    def test_speed_limit_write_readback_then_enable(self):
        report, samples = self.speed_run((5, -5), speed_limit=30)
        self.assertTrue(report["passed"])
        self.assertEqual(report["speed_limit_rad_s"], 30)
        self.assertEqual([s["actual_rad_s"] for s in samples], [5, -5])
        self.assertEqual(sum(k == 121 for k, *_ in self.endpoint.requests), 1)
        self.assertTrue(report["enable_attempted"])

    def test_speed_limit_denied_or_ack_lost_never_enables_or_replays(self):
        for fault in ("limit_denied", "limit_ack"):
            with self.subTest(fault=fault):
                self.endpoint.__init__()
                report, _ = self.speed_run(fault=fault, speed_limit=30)
                self.assertFalse(report["passed"] or report["enable_attempted"])
                self.assertEqual(self.endpoint.commands, [])
                self.assertEqual(sum(k == 121 for k, *_ in self.endpoint.requests), 1)

    def test_speed_enable_updates_both_signs_renews_and_stops(self):
        report, samples = self.speed_run((1.2345, -1))
        self.assertTrue(report["passed"] and report["stop_confirmed"])
        self.assertEqual([r["actual_rad_s"] for r in samples], [1.235, -1])
        starts = [payload for kind, payload, _ in self.endpoint.commands if kind == 117]
        self.assertEqual(starts, [b"\x02\0\0\0"])
        leases = [
            struct.unpack("<I", payload)[0]
            for kind, payload, _ in self.endpoint.commands
            if kind == 118
        ]
        self.assertEqual(leases, [1, 2])

    def test_output_coordinates_convert_limits_without_rescaling_wire_targets(self):
        self.endpoint.features = 1 << 26
        report, samples = self.speed_run((0.05, -0.05))
        self.assertTrue(report["passed"])
        self.assertEqual(report["motion_coordinate"], "output_shaft")
        self.assertEqual(report["target_speed_limit_rad_s"], 0.1)
        self.assertEqual([row["actual_rad_s"] for row in samples], [0.05, -0.05])

    def test_manual_output_target_uses_board_limit_and_both_wire_coordinates(self):
        for features, wire_target in ((0, 209440), ((1 << 26) | (1 << 27), 5236)):
            with self.subTest(features=features):
                self.endpoint.__init__()
                self.endpoint.features = features
                self.endpoint.speed_limit = 251327
                report, samples = self.speed_run((-5.236,), target=5.236, output_target=True)
                self.assertTrue(report["passed"] and report["stop_confirmed"], report)
                self.assertAlmostEqual(report["output_speed_limit_rad_s"], 6.283175)
                targets = [
                    struct.unpack_from("<i", payload, 8)[0]
                    for kind, payload, _ in self.endpoint.commands
                    if kind == 116
                ]
                self.assertEqual(targets, [wire_target, -wire_target])
                self.assertEqual([row["actual_rad_s"] for row in samples], [-5.236, -5.236])
                self.assertFalse(any(kind == 121 for kind, *_ in self.endpoint.requests))

    def test_manual_output_target_over_board_limit_stops_before_enable(self):
        self.endpoint.features = 1 << 26
        self.endpoint.speed_limit = 251327
        report, _ = self.speed_run(target=7, output_target=True)
        self.assertFalse(report["enable_attempted"])
        self.assertIn("输出轴目标 7 rad/s 超出板端限值", report["fatal"])
        self.assertIn("±60.00 rpm", report["fatal"])
        self.assertEqual(self.endpoint.commands, [])

    def test_output_coordinates_travel_is_not_divided_by_ratio_again(self):
        self.endpoint.features = 1 << 26
        self.endpoint.position_after_target = 8000
        report, samples = self.speed_run()
        self.assertFalse(report["passed"])
        self.assertIn("输出行程", report["fatal"])
        self.assertTrue(report["stop_confirmed"])
        self.assertEqual(samples, [])

    def test_speed_start_or_lease_loss_never_reenables(self):
        for fault in ("start_ack", "lease_ack"):
            with self.subTest(fault=fault):
                self.endpoint.__init__()
                report, _ = self.speed_run(fault=fault)
                self.assertFalse(report["passed"])
                self.assertTrue(report["stop_confirmed"])
                self.assertEqual(sum(k == 117 for k, *_ in self.endpoint.commands), 1)
                self.assertFalse(report["communication_reliability_passed"])

    def test_speed_update_cannot_exceed_actual_board_limit(self):
        report, _ = self.speed_run((5,))
        self.assertFalse(report["passed"])
        self.assertIn("板端限值", report["fatal"])
        self.assertTrue(report["stop_confirmed"])

    def test_pending_stop_does_not_claim_outputs_off(self):
        with patch.object(ops.time, "monotonic", side_effect=itertools.count().__next__):
            report, _ = self.speed_run(fault="stop_pending")
        self.assertFalse(report["passed"] or report["stop_confirmed"])

    def test_counter_refills_are_independent_and_skip_zero_on_wrap(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "counter.db"
            counters = ops.MotionCounters(path, "uid", 0xFFFFFFFF, 65535)
            leases = [counters.lease() for _ in range(513)]
            self.assertEqual(leases, list(range(1, 514)))
            self.assertEqual([counters.target() for _ in range(17)], list(range(17)))
            fresh = ops.MotionCounters(path, "uid", 0, 0)
            self.assertGreater(fresh.lease(), leases[-1])
            self.assertGreater(fresh.target(), 16)

    def test_speed_panel_rejects_bad_fields_without_opening_a_port(self):
        fields = dict(speed="0", travel="360", lease="0", target="0")
        self.assertEqual(motor_speed_fields(fields)["speed"], 0)
        for key, value in (
            ("speed", "nan"),
            ("target", "65536"),
            ("lease", "-1"),
        ):
            with self.subTest(key=key), self.assertRaises(ValueError):
                motor_speed_fields({**fields, key: value})

    def test_speed_continues_across_many_turns_until_operator_stops(self):
        status = self.endpoint.status
        positions = iter((0, 0, 1000000000, -1000000000, 1000000000, 0))
        samples = []

        def moving_status():
            data = bytearray(status())
            struct.pack_into("<i", data, 12, next(positions, 0))
            return bytes(data)

        self.endpoint.status = moving_status
        with tempfile.TemporaryDirectory() as directory:
            report = ops.run_speed_control(
                None,
                6,
                3,
                None,
                queue.Queue(),
                Path(directory),
                counter_db=Path(directory) / "counter.db",
                stopped=lambda: len(samples) >= 3,
                feedback=samples.append,
            )
        self.assertTrue(report["passed"] and report["stop_confirmed"], report)
        self.assertEqual(len(samples), 3)
        self.assertGreater(abs(samples[0]["position_rad"]), 10000)
        self.assertIsNone(motor_speed_fields(dict(speed="3", lease="0", target="0"))["travel"])

    def test_output_axis_speed_test_converts_and_checks_geometry_before_enable(self):
        self.endpoint.speed_limit = 50000
        test = ops.OutputSpeedTest(0.1, 80)
        with tempfile.TemporaryDirectory() as directory:
            out = Path(directory)
            for uid, travel, reason in (("02" * 12, 50, "UID"), ("01" * 12, 30, "行程")):
                result = ops.run_speed_control(
                    None,
                    6,
                    1.0,
                    travel,
                    queue.Queue(),
                    out,
                    expected_uid=uid,
                    output_test=test,
                )
                self.assertIn(reason, result["fatal"])
                self.assertFalse(result["enable_attempted"])
            for features, wire in ((0, 40000), (1 << 26, 1000)):
                self.endpoint.features = features
                result = ops.run_speed_control(
                    None,
                    6,
                    1.0,
                    50,
                    queue.Queue(),
                    out,
                    expected_uid="01" * 12,
                    output_test=test,
                    counter_db=out / "counter.db",
                )
                self.assertTrue(result["passed"] and result["stop_confirmed"], result)
                target = [payload for kind, payload, _ in self.endpoint.commands if kind == 116][-1]
                self.assertEqual(struct.unpack_from("<i", target, 8)[0], wire)
                self.assertAlmostEqual(result["last_feedback"]["output_rpm"], 9.5493, places=4)

    def test_timed_speed_without_travel_checks_revision_and_stops_after_twelve_seconds(self):
        self.endpoint.speed_limit = 50000
        self.endpoint.position_after_target = 1000000000
        clock = [0.0]

        def sleep(seconds):
            clock[0] += seconds

        with tempfile.TemporaryDirectory() as directory:
            out = Path(directory)
            incompatible = ops.run_speed_control(
                None,
                6,
                1.0,
                None,
                queue.Queue(),
                out,
                expected_uid="01" * 12,
                output_test=ops.OutputSpeedTest(12, 80, self.endpoint.revision + 1),
            )
            self.assertIn("保存修订", incompatible["fatal"])
            self.assertFalse(incompatible["enable_attempted"])
            with (
                patch.object(ops.time, "monotonic", side_effect=lambda: clock[0]),
                patch.object(ops.time, "sleep", side_effect=sleep),
            ):
                result = ops.run_speed_control(
                    None,
                    6,
                    1.0,
                    None,
                    queue.Queue(),
                    out,
                    expected_uid="01" * 12,
                    counter_db=out / "counter.db",
                    output_test=ops.OutputSpeedTest(12, 80, self.endpoint.revision),
                )
        self.assertTrue(result["passed"] and result["stop_confirmed"], result)
        self.assertEqual(result["verified_saved_revision"], self.endpoint.revision)
        self.assertGreater(result["last_feedback"]["elapsed_s"], 11.9)
        self.assertLess(result["last_feedback"]["elapsed_s"], 12)
        self.assertGreaterEqual(clock[0], 12)
        self.assertEqual(self.endpoint.commands[-1][0], 110)

    def test_hall_fault_speed_recovery_requires_matching_fault_revision_and_motor_feedback(self):
        cases = (
            (0x4001, 0, 1, 1, "", "零故障", 0),
            (0x4000, 1 << 26, 1, 1, "", "需升级以使用电机轴测速", 0),
            (0x4000, 3 << 26, 1, 1, "motor_feedback", "编码器反馈无效", 0),
            (0x4000, 3 << 26, 1, 1, "", None, 1),
            (0x4000, 0, 2, 1, "", "保存修订", 0),
            (0x4000, 0, 1, 2, "", "输出轴限速", 0),
            (0x4000, 0, 1, 1, "clear_denied", "拒绝清除", 1),
            (0x4000, 0, 1, 1, "", None, 1),
        )
        with tempfile.TemporaryDirectory() as directory:
            out = Path(directory)
            for faults, features, revision, target, fault, reason, clears in cases:
                with self.subTest(
                    faults=faults, features=features, revision=revision, reason=reason
                ):
                    self.endpoint.__init__()
                    self.endpoint.fault_bits = faults
                    self.endpoint.state = 8
                    self.endpoint.features = features
                    self.endpoint.invalid_output = bool(features & (1 << 27))
                    self.endpoint.fault = fault
                    self.endpoint.speed_limit = 50000
                    samples = []
                    result = ops.run_speed_control(
                        None,
                        6,
                        target,
                        None,
                        queue.Queue(),
                        out,
                        expected_uid="01" * 12,
                        output_test=ops.OutputSpeedTest(12, 80, revision, hall_fault=0x4000),
                        counter_db=out / "counter.db",
                        stopped=lambda samples=samples: len(samples) >= 2,
                        feedback=samples.append,
                    )
                    self.assertEqual(
                        sum(kind == 107 for kind, *_ in self.endpoint.requests), clears
                    )
                    if reason:
                        self.assertIn(reason, result["fatal"])
                        self.assertFalse(result["enable_attempted"])
                    else:
                        self.assertTrue(result["passed"] and result["stop_confirmed"], result)
                        self.assertEqual(result["hall_fault_clear"]["before"]["faults"], 0x4000)
                        self.assertEqual(result["motion_coordinate"], "motor_shaft")
                        self.assertEqual(len(samples), 2)
                        payload = next(
                            value for kind, value, _ in self.endpoint.commands if kind == 116
                        )
                        self.assertEqual(payload[1], bool(features & (1 << 27)))
                        self.assertEqual(struct.unpack_from("<i", payload, 8)[0], 40000)
                        self.assertEqual(any(self.endpoint.status_coordinates), bool(payload[1]))

    def test_adaptive_window_waits_through_ramp_and_does_not_restart_after_lock(self):
        with tempfile.TemporaryDirectory() as directory:
            out = Path(directory)
            for reaches_target in (True, False):
                with self.subTest(reaches_target=reaches_target):
                    self.endpoint.__init__()
                    self.endpoint.speed_limit = 50000
                    clock = [0.0]
                    original = self.endpoint.status
                    samples = []

                    def status(clock=clock, original=original, reaches_target=reaches_target):
                        data = bytearray(original())
                        seconds = clock[0]
                        fraction = min(1, seconds / 2.5) if reaches_target else 0.8
                        if 2.55 <= seconds < 2.65 or 4.0 <= seconds < 4.05:
                            fraction = 0.9
                        struct.pack_into("<i", data, 20, round(40000 * fraction))
                        return bytes(data)

                    self.endpoint.status = status
                    with (
                        patch.object(
                            ops.time, "monotonic", side_effect=lambda clock=clock: clock[0]
                        ),
                        patch.object(
                            ops.time,
                            "sleep",
                            side_effect=lambda seconds, clock=clock: clock.__setitem__(
                                0, clock[0] + seconds
                            ),
                        ),
                    ):
                        result = ops.run_speed_control(
                            None,
                            6,
                            1.0,
                            None,
                            queue.Queue(),
                            out,
                            output_test=ops.OutputSpeedTest(
                                10, 80, settling=ops.SpeedSettling(0.5, 10, 0.05, 0.2)
                            ),
                            counter_db=out / "counter.db",
                            feedback=samples.append,
                        )
                    self.assertTrue(result["stop_confirmed"])
                    if reaches_target:
                        self.assertTrue(result["passed"], result)
                        start, end = result["steady_window_s"]
                        self.assertGreaterEqual(start, 3.15)
                        self.assertLess(start, 3.25)
                        self.assertAlmostEqual(end - start, 10)
                        self.assertTrue(
                            any(
                                start < row["elapsed_s"] < end and row["actual_rad_s"] == 36
                                for row in samples
                            )
                        )
                        self.assertLess(end - samples[-1]["elapsed_s"], 0.2)
                    else:
                        self.assertTrue(result["settling_timed_out"])
                        self.assertFalse(result["passed"])
                        self.assertNotIn("steady_window_s", result)
                        self.assertLess(clock[0], 10.1)

    def test_status_measurements_use_correct_fields_and_valid_bits(self):
        original = self.endpoint.status

        def status():
            data = bytearray(original())
            struct.pack_into("<H", data, 24, (1 << 4) | (1 << 5) | (1 << 9))
            struct.pack_into("<h", data, 26, 2812)
            struct.pack_into("<h", data, 28, -1234)
            struct.pack_into("<h", data, 34, 5000)
            struct.pack_into("<h", data, 36, 2356)
            return bytes(data)

        self.endpoint.status = status
        values = ops.read_device_status(None, 6, "01" * 12)
        self.assertEqual(values["bus_voltage_v"], 28.12)
        self.assertEqual(values["bus_current_a"], -1.234)
        self.assertEqual(values["motor_temperature_c"], 23.56)
        data = bytearray(status())
        struct.pack_into("<H", data, 24, 0)
        self.assertEqual(
            ops.status_measurements(data),
            dict(bus_voltage_v=None, bus_current_a=None, motor_temperature_c=None),
        )
        struct.pack_into("<H", data, 24, (1 << 4) | (1 << 5) | (1 << 9))
        struct.pack_into("<h", data, 26, -32768)
        struct.pack_into("<h", data, 28, -32768)
        struct.pack_into("<h", data, 36, -32768)
        self.assertEqual(
            ops.status_measurements(data),
            dict(bus_voltage_v=None, bus_current_a=None, motor_temperature_c=None),
        )

    def test_actual_device_summary_and_complete_csv_channels(self):
        device = ops.read_device_status(None, 6, "01" * 12)
        self.assertEqual((device["app_version"], device["loader_version"]), (0, 0))
        self.assertEqual((device["state"], device["mode"]), (5, 0))
        with self.assertRaisesRegex(ValueError, "UID"):
            ops.read_device_status(None, 6, "02" * 12)
        samples = []
        with tempfile.TemporaryDirectory() as directory:
            out = Path(directory)
            report = ops.run_speed_control(
                None,
                6,
                3,
                None,
                queue.Queue(),
                out,
                counter_db=out / "counter.db",
                stopped=lambda: len(samples) == 1,
                feedback=samples.append,
            )
            import csv

            with (out / "speed.csv").open(newline="", encoding="utf-8") as stream:
                rows = list(csv.DictReader(stream))
        self.assertTrue(report["stop_confirmed"])
        self.assertEqual(report["final_status"], dict(state=5, mode=0, faults=0))
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["reference_position_rad"], "")
        self.assertEqual(float(rows[0]["actual_rad_s"]), 3)
        self.assertIn("reference_iq_a", rows[0])
        self.assertIn("reference_speed_rad_s", rows[0])

    def test_speed_csv_batches_flush_and_preserves_all_rows(self):
        import csv

        samples, streams = [], []
        clock = [0.0]
        original_open = Path.open

        def open_trace(path, *args, **kwargs):
            stream = original_open(path, *args, **kwargs)
            if path.name == "speed.csv" and args and args[0] == "w":
                wrapped = MagicMock(wraps=stream)
                wrapped.__enter__.return_value = wrapped
                wrapped.__exit__.side_effect = stream.__exit__
                streams.append(wrapped)
                return wrapped
            return stream

        def sleep(seconds):
            clock[0] += seconds

        with tempfile.TemporaryDirectory() as directory:
            out = Path(directory)
            with (
                patch.object(Path, "open", open_trace),
                patch.object(ops.time, "monotonic", side_effect=lambda: clock[0]),
                patch.object(ops.time, "sleep", side_effect=sleep),
            ):
                report = ops.run_speed_control(
                    None,
                    6,
                    3,
                    None,
                    queue.Queue(),
                    out,
                    counter_db=out / "counter.db",
                    stopped=lambda: len(samples) == 12,
                    feedback=samples.append,
                )
            with (out / "speed.csv").open(newline="", encoding="utf-8") as stream:
                rows = list(csv.DictReader(stream))
        self.assertTrue(report["passed"] and report["stop_confirmed"])
        self.assertEqual(len(rows), 12)
        self.assertGreater(streams[0].flush.call_count, 0)
        self.assertLessEqual(streams[0].flush.call_count, 3)

    def test_worker_detects_lost_gui_parent(self):
        with (
            tempfile.TemporaryDirectory() as directory,
            patch.object(desktop, "controlcan_dll_path", return_value=Path("test.dll")),
            patch.object(desktop.os, "add_dll_directory", create=True),
            patch.object(desktop, "ControlCanFd"),
            patch.object(desktop, "check_current_motor"),
            patch.object(
                desktop.mp, "parent_process", return_value=SimpleNamespace(is_alive=lambda: False)
            ),
            patch.object(
                desktop,
                "run_speed_control",
                side_effect=lambda *args, **kwargs: {
                    "profile": "motor_speed",
                    "node": 6,
                    "cancelled": kwargs["stopped"](),
                },
            ) as run,
        ):
            desktop.acquire(
                dict(
                    profile="motor_speed",
                    bus={},
                    node=6,
                    expected_uid="01" * 12,
                    speed=0,
                    travel=360,
                    last_lease=0,
                    last_target=0,
                ),
                Path(directory),
                queue.Queue(),
                SimpleNamespace(is_set=lambda: False),
                queue.Queue(),
            )
            self.assertTrue(run.call_args.kwargs["stopped"]())
            self.assertTrue(run.call_args.kwargs["output_target"])
            self.assertEqual(run.call_args.kwargs["expected_uid"], "01" * 12)
            self.assertNotIn("speed_limit", run.call_args.kwargs)
