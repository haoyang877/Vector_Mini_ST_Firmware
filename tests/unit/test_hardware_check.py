"""使用真实 CAN 编解码的内存端口覆盖硬件检查的发现、驱动、关相和物理旋向。"""

import json
import struct
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools/bench"))
import hardware_check as hw
from upgrade_client import crc8, crc16, decode_reply, encode_frames


def response(node, kind, seq, data):
    header = struct.pack("<HBBBBHHIB", 0xA55A, 1, 0x10, node, 2, kind, seq, len(data), 0)
    body = header + bytes((crc8(header),)) + data
    body += struct.pack("<H", crc16(body))
    size = next(n for n in (20, 24, 32, 48, 64) if n >= len(body))
    return 0x0CEF0200 | node, body.ljust(size, bytes(1))


class Port:
    def __init__(self, fault=""):
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
            if self.fault == "busy":
                reply = b"\x06\0\0\0"
        elif kind == 2:
            reply = struct.pack(
                "<I6H", 0 if self.fault == "legacy" else 0xFFD00, 0x64E, 64, 46, 0, 500, 500
            )
        elif kind == 127:
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
                self.stage = (2 if self.mode == 2 else 4) if self.fault != "hot" else 5
            items = [1] * 8 + [0, 0] if self.stage in (2, 3, 4, 5) else [0] * 10
            if self.fault == "hot" and self.stage == 5:
                items[5] = 0
            if self.mode == 2 and self.stage == 4:
                items[8:] = [1, 1]
            reply = struct.pack(
                "<HHBBI10BB",
                int(bool(payload)),
                0,
                2,
                self.stage,
                self.run + (self.fault == "old" and not payload and self.stage != 0),
                *items,
                0x99 if items[8] else 0,
            )
        elif kind == 108:
            reply = bytearray(45)
            reply[42:] = bytes((node, 6 if self.running else 5, 10 if self.running else 0))
            struct.pack_into("<H", reply, 24, 0x202)
            struct.pack_into("<h", reply, 36, 6500 if self.fault == "hot" else 2500)
            if self.fault == "travel" and self.running:
                struct.pack_into("<i", reply, 12, 100000)
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
            if self.fault == "linkloss":
                return
        elif kind == 110:
            if self.fault == "stoploss":
                return
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
        config = hw.Drive(0.25, 2, 120, 5) if drive else None
        result = hw.run_check(
            port, 6, drive=config, previous=0, previous_target=0, counter_db=self.db, **kwargs
        )
        return result, port

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
        self.assertEqual(list(r["items"].values())[-2:], ["未测"] * 2)
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

    def test_stop_paths(self):
        for fault in ("startloss", "linkloss", "travel", "stoploss", "denied"):
            with self.subTest(fault=fault):
                r, bus = self.check(fault, True)
                self.assertTrue(r["note"])
                self.assertIn(117, [k for k, _, _ in bus.sent])
                if fault == "denied":
                    self.assertNotIn(110, [k for k, _, _ in bus.sent])
                else:
                    self.assertIn(110, [k for k, _, _ in bus.sent])
                    self.assertEqual(r["stop_confirmed"], fault != "stoploss")
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
