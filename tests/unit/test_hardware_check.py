"""使用真实 CAN 编解码的内存端口覆盖硬件检查的发现、驱动、关相和物理旋向。"""

import json
import struct
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools/bench"))
import hardware_check as hw
from upgrade_client import crc8, crc16, decode_reply, encode_frames

ROOT = Path(__file__).resolve().parents[2]


def response(node, kind, seq, data):
    header = struct.pack("<HBBBBHHIB", 0xA55A, 1, 0x10, node, 2, kind, seq, len(data), 0)
    body = header + bytes((crc8(header),)) + data
    body += struct.pack("<H", crc16(body))
    size = next(n for n in (20, 24, 32, 48, 64) if n >= len(body))
    return 0x0CEF0200 | node, body.ljust(size, bytes(1))


class Port:
    def __init__(self, fault="", measurements=False, version=0x01080C):
        self.version = version
        self.measurements = measurements
        self.fault = fault
        self.pending = []
        self.sent = []
        self.run = 7
        self.stage = 0
        self.start_seq = None
        self.running = False
        self.mode = 0
        self.outward = 0
        self.previous_lease = 0
        self.lost_lease_ack = False
        self.status_was_busy = False
        self.uid = bytes(range(12))

    def send(self, identifier, data):
        node, kind, seq, size = struct.unpack_from("<BHHI", data, 5)
        payload = data[16 : 16 + size]
        assert (identifier, data) == encode_frames(node, kind, payload, seq)[0]
        self.sent.append((kind, seq, payload))
        reply = bytes(4)
        if kind == 1:
            page = int.from_bytes(payload[8:10], "little")
            reply = bytearray(32)
            reply[:12] = self.uid
            if page:
                reply[20:23] = bytes((1, node, 2 if self.fault == "loader" else 1))
                if self.fault == "unprovisioned":
                    reply[20] = 0
            if not page:
                struct.pack_into("<I", reply, 20, self.version)
            if self.fault == "busy":
                reply = b"\x06\0\0\0"
        elif kind == 2:
            reply = struct.pack(
                "<I6H",
                0
                if self.fault == "legacy"
                else (0x2FFD00 | (int(self.measurements) << 20))
                & (~(1 << 21) if self.fault == "no_resistance" else 0xFFFFFFFF),
                0x64E,
                64,
                46,
                0,
                500,
                500,
            )
        elif kind == 127:
            if payload and payload[1] == 3:
                reply = struct.pack(
                    "<HH4BI4B7f",
                    0,
                    0,
                    2,
                    2 if self.fault == "resistance_missing" else 1,
                    2 if self.fault == "resistance_missing" else 0,
                    0,
                    self.run,
                    0,
                    3 if self.fault == "resistance_missing" else 0,
                    0,
                    63,
                    1.905,
                    1.905,
                    1.905,
                    1.905,
                    3.81,
                    3.81,
                    3.81,
                )
                self.pending.append(response(node, kind, seq, reply))
                return
            if payload and payload[1] == 2:
                _, _, item, _, expected = struct.unpack("<4BI", payload)
                assert not self.running and expected == self.run
                values = {
                    0: (2050.0, 4.0, 0.0, 0.0),
                    4: (26.99, 29.01, 0.0, 0.0),
                    8: (1.2, -1.1, 0.8, 0.9),
                    10: (500, 3500, 600, 3400),
                }.get(item, (0.0, 0.0, 0.0, 0.0))
                valid = 3 if item in (0, 4) else 15 if item in (8, 10) else 0
                reply = struct.pack("<HH4BII4f", 0, 0, 2, item, valid, 0, self.run, 2000, *values)
                self.pending.append(response(node, kind, seq, reply))
                return
            if payload:
                _, _, self.mode, _, previous = struct.unpack("<4BI", payload)
                if seq != self.start_seq:
                    assert previous == self.run
                    self.start_seq = seq
                    self.run += 1
                    self.stage = 1
                    if self.fault == "start127loss":
                        return
            elif self.stage == 1:
                self.stage = (6 if self.mode == 3 else 4) if self.fault != "hot" else 5
            elif self.stage == 6:
                self.stage = 5 if self.fault == "resistance_missing" else 2
            items = [1] * 8 + [0, 0] if self.stage in (2, 3, 4, 5, 6) else [0] * 10
            if self.fault == "hot" and self.stage == 5:
                items[5] = 0
            if self.mode == 3 and self.stage == 4:
                items[8:] = [1, 1]
                if self.fault == "reversed":
                    items[8] = 2
            if self.fault == "hallabort" and self.stage == 5:
                items[9] = 2
            reply = struct.pack(
                "<HHBBI10BB",
                int(bool(payload)),
                0,
                2,
                self.stage,
                self.run + (self.fault == "old" and not payload and self.stage != 0),
                *items,
                (0x96 if self.fault == "reversed" else 0x99) if items[8] else 0,
            )
        elif kind == 108:
            reply = bytearray(45)
            reply[42:] = bytes((node, 6 if self.running else 5, 10 if self.running else 0))
            if self.fault == "prefault":
                reply[43] = 8
                struct.pack_into("<I", reply, 4, 0x20)
            struct.pack_into("<H", reply, 24, 0x202)
            struct.pack_into("<h", reply, 36, 6500 if self.fault == "hot" else 2500)
            if self.fault == "travel" and self.running:
                struct.pack_into("<i", reply, 12, 100000)
            if self.fault == "statusbusy" and not self.status_was_busy:
                self.status_was_busy = True
                reply = b"\x06\0\0\0"
        elif kind == 120:
            reply = struct.pack("<HHBBBBIII", 0, 0, 5, 2, 1, 1, 14, 1, 0)
        elif kind == 126:
            reply = bytearray(32)
            reply[4] = 1
            struct.pack_into("<i", reply, 24, 40000)
        elif kind == 117:
            assert seq == self.start_seq and payload == b"\x0a\0\0\0" and self.stage == 2
            if self.fault == "denied":
                reply = b"\x05\0\0\0"
            else:
                self.running, self.stage = True, 3
                if self.fault == "startloss":
                    return
                reply = b"\x01\0\0\0"
        elif kind == 116:
            mode, pos, speed, iq, vq, freq, current, vd = struct.unpack("<B3x7i", payload)
            assert (
                self.running
                and mode == 10
                and current > 0
                and (pos, speed, iq, vq, vd) == (0, 0, 0, 0, 0)
            )
            self.outward += int(freq != 0)
        elif kind == 118:
            count = int.from_bytes(payload, "little")
            assert self.running and count > self.previous_lease
            self.previous_lease = count
            if self.fault in ("hallabort", "leaserejected"):
                if self.fault == "hallabort":
                    self.running, self.stage = False, 5
                reply = b"\x05\0\0\0"
            if self.fault == "linkloss":
                return
            if self.fault == "leaseackloss" and not self.lost_lease_ack:
                self.lost_lease_ack = True
                return
        elif kind == 110:
            if self.fault == "stoploss":
                return
            if self.fault == "pendingstop":
                reply = b"\x01\0\x01\0"
            self.running = False
            self.stage = 4 if self.outward == 2 else 5
        else:
            raise AssertionError(kind)
        frame = response(node, kind, seq, reply)
        if self.fault == "absent":
            return
        if self.fault == "noise":
            self.pending += [
                response(node + 1, kind, seq, reply),
                response(node, kind, (seq - 1) & 65535, reply),
                (frame[0], frame[1][:15] + bytes((frame[1][15] ^ 1,)) + frame[1][16:]),
            ]
            return
        self.pending.append(frame)

    def receive(self, seconds):
        return self.pending.pop(0) if self.pending else None


class HardwareHostTest(unittest.TestCase):
    def setUp(self):
        self.clock = 0.0
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.db = Path(self.tmp.name) / "counters.db"
        timepatch = patch.object(hw, "time", autospec=True)
        self.time = timepatch.start()
        self.addCleanup(timepatch.stop)
        self.time.monotonic.side_effect = lambda: self.clock
        self.time.sleep.side_effect = self.sleep

    def sleep(self, interval):
        self.clock += interval

    def check(self, fault="", drive=False, **kwargs):
        port = Port(fault)
        config = hw.Drive(0.25, 2, 700, 5) if drive else None
        result = hw.run_check(
            port, 6, drive=config, previous=0, previous_target=0, counter_db=self.db, **kwargs
        )
        return result, port

    def test_phase_events_follow_scope_and_keep_snapshots(self):
        events = []
        report, _ = self.check(phase_progress=events.append)
        self.assertEqual(events[0]["communication"], "进行中")
        self.assertEqual(events[0]["static"], "待执行")
        self.assertEqual(report["phase_states"]["static"], "已完成")
        self.assertEqual(report["phase_states"]["resistance"], "本轮不测")
        self.assertEqual(report["phase_states"]["dynamic"], "本轮不测")
        report, port = self.check(fault="no_resistance", drive=True)
        self.assertEqual(report["phase_states"]["communication"], "未完成")
        self.assertEqual(report["phase_states"]["static"], "未执行")
        self.assertFalse(any(kind in (116, 117, 118, 127) for kind, _, _ in port.sent))

    def test_cancel_keeps_phase_incomplete_and_confirms_stop(self):
        port = Port(measurements=True)
        events = []
        report = hw.run_check(
            port,
            6,
            resistance_only=True,
            stopped=lambda: port.stage == 6,
            phase_progress=events.append,
        )
        self.assertEqual(report["phase_states"]["resistance"], "未完成")
        self.assertEqual(report["phase_states"]["dynamic"], "本轮不测")
        self.assertTrue(report["stop_confirmed"])
        self.assertFalse(report["subset_passed"])

    def test_missing_resistance_is_explicit_and_never_starts_detection(self):
        port = Port(fault="no_resistance", version=0x010808)
        report = hw.run_check(port, 4, drive=hw.Drive(1, 2, 700, 5))
        self.assertTrue(report["upgrade_required"])
        self.assertIn("1.8.8", report["note"])
        self.assertIn("1.8.12", report["note"])
        self.assertEqual(hw.result_rows(report)[8][3], "不支持")
        self.assertNotIn("0.5A", hw.result_rows(report)[8][2])
        self.assertFalse(any(kind in (116, 117, 118, 127) for kind, _, _ in port.sent))

    def test_resistance_only_stops_without_dynamic_commands(self):
        for fault in ("", "resistance_missing"):
            with self.subTest(fault=fault):
                port = Port(fault=fault, measurements=True)
                report = hw.run_check(port, 6, resistance_only=True)
                self.assertTrue(report["stop_confirmed"])
                self.assertEqual(report["scope"], "static_resistance")
                self.assertEqual(report["subset_passed"], not bool(fault))
                self.assertFalse(any(kind in (116, 117, 118) for kind, _, _ in port.sent))
                self.assertTrue(any(kind == 110 for kind, _, _ in port.sent))
                self.assertEqual(hw.result_rows(report)[9][3], "不在本轮范围")

    def test_resistance_only_cancel_during_measurement_still_stops(self):
        port = Port(measurements=True)
        report = hw.run_check(port, 6, resistance_only=True, stopped=lambda: port.stage == 6)
        self.assertTrue(report["stop_confirmed"])
        self.assertFalse(report["subset_passed"])
        self.assertFalse(any(kind in (116, 117, 118) for kind, _, _ in port.sent))

    def test_current_labels_follow_device_firmware(self):
        for version, currents, label in (
            (0x01080B, [0.5, 1.0], "0.5A/1A"),
            (0x01080C, [1.0, 2.0], "1A/2A"),
        ):
            report = hw.run_check(Port(version=version), 6)
            self.assertEqual(report["test_currents_a"], currents)
            self.assertIn(label, hw.result_rows(report)[8][2])

    def test_measurement_pages_preserve_results_and_render_values(self):
        port = Port(measurements=True)
        report = hw.run_check(port, 6)
        self.assertEqual(report["items"]["Ia"], "正常")
        self.assertEqual(report["measurements"]["0"]["values"], [2050, 4, None, None])
        rows = hw.result_rows(report)
        self.assertIn("2050.00", rows[0][1])
        self.assertIn("4.00", rows[0][1])
        self.assertIn("26.99", rows[4][1])
        self.assertIn("29.01", rows[4][1])
        self.assertIn("27～29", rows[4][2])
        # 显示读取值不会重新推导或覆盖板端的判定。
        self.assertEqual(rows[4][3], "正常")
        self.assertEqual(len([p for k, _, p in port.sent if k == 127 and p and p[1] == 2]), 11)
        report["board_items"][7] = 3
        hw.direction_results(report)
        self.assertEqual(hw.result_rows(report)[7][1], "未装")
        report["board_items"][0] = 0
        hw.direction_results(report)
        self.assertIn("部分采样", hw.result_rows(report)[0][1])

    def test_old_firmware_has_no_fake_values_or_measurement_queries(self):
        report, port = self.check()
        self.assertEqual(report["measurements"], {})
        self.assertIn("未提供", hw.result_rows(report)[0][1])
        self.assertFalse(any(k == 127 and p and p[1] == 2 for k, _, p in port.sent))

    def test_measurement_decoder_rejects_stale_or_invalid_values(self):
        class Reply:
            def __init__(self, data):
                self.data = data

            def send(self, identifier, frame):
                self.seq = struct.unpack_from("<H", frame, 8)[0]

            def receive(self, seconds):
                return response(6, 127, self.seq, self.data)

        for run, valid, first in ((9, 1, 28.0), (8, 16, 28.0), (8, 1, float("nan"))):
            data = struct.pack("<HH4BII4f", 0, 0, 2, 4, valid, 0, run, 20, first, 0, 0, 0)
            with self.subTest(run=run, valid=valid), self.assertRaises(ValueError):
                hw.Link(Reply(data), 6).measurement(4, 8)
        data = struct.pack("<HH4BII4f", 0, 0, 2, 4, 0, 0, 8, 0, 0, 0, 0, 0)
        self.assertEqual(hw.Link(Reply(data), 6).measurement(4, 8)["values"], [None] * 4)

    def test_golden_frame(self):
        row = json.loads((ROOT / "tests/fixtures/protocol/hardware_check_v2.json").read_text())
        self.assertEqual(
            encode_frames(6, 127, bytes.fromhex(row["start_payload"]), 4660)[0][0], 0x0CEF0602
        )
        self.assertEqual(
            encode_frames(6, 127, bytes.fromhex(row["start_payload"]), 4660)[0][1].hex(),
            row["start_frame"],
        )
        self.assertEqual(
            decode_reply(0x0CEF0206, bytes.fromhex(row["report_frame"]), 6, 127, 4660).hex(),
            row["report_payload"],
        )

    def test_comm_and_compatibility(self):
        for fault, comm, probes in (
            ("absent", "异常", 3),
            ("noise", "异常", 3),
            ("busy", "正常", 1),
            ("legacy", "正常", 2),
            ("loader", "正常", 2),
        ):
            with self.subTest(fault=fault):
                r, bus = self.check(fault, True)
                self.assertEqual(r["communication"], comm)
                self.assertEqual(set(r["items"].values()), {"未测"})
                self.assertEqual(sum(k == 1 for k, _, _ in bus.sent), probes)
                self.assertFalse(any(k in (110, 116, 117, 118, 127) for k, _, _ in bus.sent))

    def test_static_and_replay(self):
        r, bus = self.check("start127loss")
        self.assertEqual((r["stage"], r["communication"], r["run_id"]), (4, "正常", 8))
        self.assertEqual(list(r["items"].values()), ["正常"] * 8 + ["未测"] * 2)
        starts = [(s, p) for k, s, p in bus.sent if k == 127 and p]
        self.assertEqual(starts, [starts[0]] * 2)
        self.assertFalse(any(k in (110, 116, 117, 118) for k, _, _ in bus.sent))
        stale, _ = self.check("old")
        self.assertEqual((stale["run_id"], stale["stage"]), (8, 1))

    def test_first_static_reply_can_take_more_than_50_ms(self):
        port = Port()
        receive = port.receive

        def delayed(seconds):
            self.assertGreater(seconds, 0.05)
            self.clock += 0.12
            return receive(seconds)

        port.receive = delayed
        report = hw.run_check(port, 6)
        self.assertEqual(report["stage"], 4, report)
        self.assertEqual(report["reply_timeout_ms"], 500)
        self.assertEqual(report["communication_timeouts"], {})

    def test_temperature(self):
        r, bus = self.check(drive=True, ambient=-1)
        self.assertFalse(any(k == 127 for k, _, _ in bus.sent))
        self.assertEqual(r["communication"], "正常")
        for ambient, expected in ((None, "未测"), (25, "异常")):
            r, bus = self.check("hot", True, ambient=ambient)
            self.assertEqual((r["items"][hw.ITEMS[5]], r["motor_temperature_c"]), (expected, 65))
            self.assertFalse(any(k == 117 for k, _, _ in bus.sent))

    def test_motion_and_observation(self):
        r, bus = self.check(drive=True)
        self.assertEqual((r["communication"], r["stage"], r["stop_confirmed"]), ("正常", 4, True))
        self.assertGreater(bus.previous_lease, 3)
        self.assertEqual(list(r["items"].values())[-2:], ["正常"] * 2)
        hw.direction_results(r, (1, 2), (1, 2))
        self.assertEqual(list(r["items"].values())[-2:], ["正常"] * 2)
        hw.direction_results(r, (2, 1), (1, 0))
        self.assertEqual(list(r["items"].values())[-2:], ["异常", "未测"])
        r["directions"] = 0x9F
        r["board_items"][8] = 0
        hw.direction_results(r, (1, 2), (1, 2))
        self.assertEqual(list(r["items"].values())[-2:], ["异常", "正常"])
        r["board_items"][8:] = [2, 3]
        hw.direction_results(r)
        self.assertEqual(list(r["items"].values())[-2:], ["异常", "未装"])

    def test_motion_without_production_identity(self):
        r, bus = self.check("unprovisioned", True)
        self.assertEqual((r["communication"], r["stage"], r["stop_confirmed"]), ("正常", 4, True))
        self.assertIn(117, [kind for kind, _, _ in bus.sent])
        self.assertGreater(bus.previous_lease, 3)

    def test_reversed_motion_exposes_both_command_directions(self):
        report, bus = self.check("reversed", True)
        self.assertEqual(report["items"]["电机角动态"], "异常")
        self.assertEqual(report["motor_direction"]["result"], "reversed")
        self.assertEqual(report["motor_direction"]["measured"], [2, 1])
        text = hw.summarize(report)
        self.assertIn("编码器方向反向", text)
        self.assertIn("去程：正向命令→编码器反转", text)
        self.assertIn("回程：反向命令→编码器正转", text)
        self.assertTrue(report["stop_confirmed"])
        self.assertEqual(sum(kind == 117 for kind, *_ in bus.sent), 1)

    def test_direction_diagnosis_does_not_guess_from_missing_motion(self):
        for bits, board_result, expected in (
            (0, 0, "incomplete"),
            (15, 2, "incomplete"),
            (9, 1, "consistent"),
            (9, 2, "dynamic_failure"),
            (10, 2, "mismatch"),
        ):
            with self.subTest(bits=bits, board_result=board_result):
                report = hw.empty_report(6)
                report.update(stage=4, directions=bits)
                report["board_items"][8] = board_result
                hw.direction_results(report)
                self.assertEqual(report["motor_direction"]["result"], expected)

    def test_existing_fault_blocks_drive_without_claiming_reverse(self):
        report, bus = self.check("prefault", True)
        self.assertEqual(report["drive_precheck"]["faults"], 32)
        self.assertIn("0x00000020", hw.summarize(report))
        self.assertEqual(report["motor_direction"]["result"], "incomplete")
        self.assertFalse(any(kind == 117 for kind, *_ in bus.sent))

    def test_direction_detail_survives_json_roundtrip(self):
        report, _ = self.check("reversed", True)
        saved = json.loads(json.dumps(report, ensure_ascii=False))
        self.assertEqual(saved["motor_direction"], report["motor_direction"])
        self.assertEqual(hw.summarize(saved), hw.summarize(report))

    def test_board_abort_precedes_lease_rejection(self):
        report, bus = self.check("hallabort", True)
        self.assertEqual(report["board_abort_items"], ["霍尔动态"])
        self.assertIn("板端已中止，异常项：霍尔动态", report["note"])
        self.assertIn("Type 118 拒绝", report["drive_command_error"])
        self.assertEqual(report["items"]["电机角动态"], "未测")
        self.assertTrue(report["stop_confirmed"])
        kinds = [kind for kind, _, _ in bus.sent]
        self.assertEqual(kinds.count(117), 1)
        self.assertEqual(kinds.count(118), 1)
        rejected = kinds.index(118)
        self.assertLess(kinds.index(127, rejected), kinds.index(110, rejected))

    def test_lease_rejection_while_driving_is_not_hardware_abort(self):
        report, bus = self.check("leaserejected", True)
        self.assertNotIn("board_abort_items", report)
        self.assertEqual(report["note"], "Type 118 拒绝，result=5 detail=0")
        self.assertTrue(report["stop_confirmed"])
        self.assertEqual(sum(kind == 117 for kind, *_ in bus.sent), 1)

    def test_lost_lease_ack_completes_checks_without_hiding_timeout(self):
        r, bus = self.check("leaseackloss", True)
        self.assertEqual((r["stage"], r["stop_confirmed"]), (4, True))
        self.assertFalse(r["communication_reliability_passed"])
        self.assertEqual(r["communication_timeouts"], {118: 1})
        self.assertEqual(sum(kind == 117 for kind, _, _ in bus.sent), 1)
        self.assertIn("驱动仍按50 ms", hw.summarize(r))

    def test_busy_status_is_retried_without_restarting_motion(self):
        r, bus = self.check("statusbusy", True)
        self.assertEqual((r["stage"], r["stop_confirmed"], r["status_busy_replies"]), (4, True, 1))
        self.assertTrue(r["communication_reliability_passed"])
        self.assertEqual(sum(kind == 117 for kind, _, _ in bus.sent), 1)

    def test_missing_phase_prevents_dynamic_start(self):
        report, bus = self.check("resistance_missing", True)
        self.assertTrue(report["stop_confirmed"])
        self.assertEqual(report["stage"], 5)
        self.assertNotIn(117, [kind for kind, _, _ in bus.sent])
        self.assertIn("B相回路缺相", hw.result_rows(report)[8][3])

    def test_resistance_values_and_fault_details(self):
        report, bus = self.check("", True)
        row = next(row for row in hw.result_rows(report) if row[0] == "相电阻")
        self.assertEqual(row[3], "正常")
        self.assertIn("1.9050", row[1])
        self.assertIn("0.9525", row[2])
        self.assertTrue(
            any(
                kind == 127 and payload[1:3] == bytes((1, 3))
                for kind, _, payload in bus.sent
                if payload
            )
        )
        report["resistance"].update(result=2, open_phases=2, open_lines=3)
        self.assertIn("B相回路缺相", hw.result_rows(report)[8][3])
        report["resistance"].update(result=3, open_phases=0, open_lines=0, low_phases=4)
        self.assertIn("C相电阻异常小", hw.result_rows(report)[8][3])
        report["resistance"].update(result=0, low_phases=0, high_phases=1, failure=10)
        self.assertIn("A相电阻偏高", hw.result_rows(report)[8][3])

    def test_stop_paths(self):
        for fault in ("startloss", "linkloss", "travel", "stoploss", "pendingstop", "denied"):
            with self.subTest(fault=fault):
                r, bus = self.check(fault, True)
                self.assertTrue(r["note"])
                self.assertIn(117, [k for k, _, _ in bus.sent])
                self.assertIn(110, [k for k, _, _ in bus.sent])
                self.assertEqual(r["stop_confirmed"], fault not in ("stoploss", "pendingstop"))
                if fault == "pendingstop":
                    self.assertEqual(r["communication"], "正常")
                if fault in ("linkloss", "startloss", "stoploss"):
                    self.assertEqual(r["communication"], "异常")

    def test_persisted_motion_counters(self):
        with self.assertRaises(hw.Rejected):
            hw.reserve_counters(self.db, "axis", None, None)
        self.assertEqual(hw.reserve_counters(self.db, "axis", 88, 65535), (88, 65535))
        self.assertEqual(hw.reserve_counters(self.db, "axis", None, None), (600, 15))
        for val in (float("nan"), float("inf"), 0):
            with self.assertRaises(ValueError):
                hw.Drive(val, 1, 120, 1)


if __name__ == "__main__":
    unittest.main()
