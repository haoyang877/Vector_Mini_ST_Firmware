"""故障位、单次显式清除与回读确认，不访问硬件。"""

import struct
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools/bench"))
import can_motor_operations as ops


class FaultLink:
    def __init__(self, states, reply=b"\0" * 4, uid="01" * 12):
        self.states = list(states)
        self.reply, self.uid = reply, uid
        self.requests, self.timeouts = [], {}

    def request(self, kind, *, page=None):
        self.requests.append((kind, page))
        if kind == 1:
            data = bytearray(32)
            if page == 0:
                data[:12] = bytes.fromhex(self.uid)
            else:
                data[21:23] = bytes((6, 1))
            return data
        if kind == 107:
            if isinstance(self.reply, Exception):
                self.timeouts[kind] = 1
                raise self.reply
            return self.reply
        raise AssertionError(f"Unexpected write {kind}")

    def status(self):
        value = self.states.pop(0) if len(self.states) > 1 else self.states[0]
        data = bytearray(45)
        struct.pack_into("<I", data, 4, value[0])
        data[42:] = bytes((6, value[1], value[2]))
        return data


class FaultClearTests(unittest.TestCase):
    def run_clear(self, link, **kwargs):
        clock = [0.0]

        def sleep(seconds):
            clock[0] += seconds

        with (
            patch.object(ops, "Link", return_value=link),
            patch.object(ops.time, "monotonic", side_effect=lambda: clock[0]),
            patch.object(ops.time, "sleep", side_effect=sleep),
        ):
            return ops.run_clear_fault(None, 6, "01" * 12, **kwargs)

    def test_combined_fault_names_and_unknown_bits(self):
        self.assertEqual(ops.describe_faults(0), "无故障")
        self.assertEqual(ops.describe_faults(0x20), "编码器故障")
        self.assertEqual(ops.describe_faults(0x200), "配置/标定错误")
        self.assertEqual(ops.describe_faults(0x600), "参数非法")
        self.assertEqual(ops.describe_faults(0x2200), "编码器未完成标定")
        self.assertEqual(ops.describe_faults(0x0010), "MCU过温")
        self.assertEqual(ops.describe_faults(0x10000), "电机绕组热保护")
        text = ops.describe_faults(0x80000021)
        self.assertIn("过流", text)
        self.assertIn("编码器故障", text)
        self.assertIn("0x80000000", text)
        self.assertIn("其他", ops.describe_faults(1 << 12))

    def test_single_clear_waits_for_actual_idle_and_zero(self):
        link = FaultLink([(32, 8, 0), (32, 8, 0), (0, 5, 0)], struct.pack("<HH", 1, 0))
        report = self.run_clear(link)
        self.assertTrue(report["passed"])
        self.assertEqual(report["before"]["faults"], 32)
        self.assertEqual(report["after"]["faults"], 0)
        self.assertEqual(link.requests.count((107, None)), 1)

    def test_ack_is_not_success_when_source_remains(self):
        report = self.run_clear(FaultLink([(32, 8, 0)]))
        self.assertFalse(report["passed"])
        self.assertFalse(report["cleared"])
        self.assertEqual(report["after"]["faults"], 32)

    def test_rejected_clear_preserves_result_and_state(self):
        report = self.run_clear(FaultLink([(32, 8, 0)], struct.pack("<HH", 5, 9)))
        self.assertFalse(report["passed"])
        self.assertEqual((report["result"], report["detail"]), (5, 9))
        self.assertEqual(report["after"]["faults"], 32)

    def test_lost_ack_does_not_resend_or_hide_timeout(self):
        link = FaultLink([(32, 8, 0), (0, 5, 0)], TimeoutError("ACK lost"))
        report = self.run_clear(link)
        self.assertTrue(report["cleared"])
        self.assertFalse(report["passed"])
        self.assertEqual(report["communication_timeouts"], {107: 1})
        self.assertEqual(link.requests.count((107, None)), 1)

    def test_uid_change_prevents_write(self):
        link = FaultLink([(32, 8, 0)], uid="02" * 12)
        report = self.run_clear(link)
        self.assertTrue(report["identity_changed"])
        self.assertFalse(report["clear_sent"])
        self.assertNotIn((107, None), link.requests)

    def test_active_state_prevents_write(self):
        for state, mode in ((6, 2), (4, 0), (5, 2)):
            with self.subTest(state=state, mode=mode):
                link = FaultLink([(32, state, mode)])
                self.assertFalse(self.run_clear(link)["clear_sent"])
                self.assertNotIn((107, None), link.requests)

    def test_no_fault_or_cancel_does_not_write(self):
        clean = FaultLink([(0, 5, 0)])
        self.assertTrue(self.run_clear(clean)["passed"])
        self.assertNotIn((107, None), clean.requests)
        cancelled = FaultLink([(32, 8, 0)])
        self.assertTrue(self.run_clear(cancelled, stopped=lambda: True)["cancelled"])
        self.assertNotIn((107, None), cancelled.requests)


if __name__ == "__main__":
    unittest.main()
