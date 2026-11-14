"""参数事务协议替身：覆盖身份、修订、部分失败及真实 Flash 回读语义。"""

import struct
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools/bench"))
import can_parameters as parameters


class Endpoint:
    def __init__(self):
        self.revision = self.saved_revision = 7
        self.timeouts = {}
        self.calls = []
        self.values = {i: 1000 for i in parameters.parameter_ids("current")}
        self.stored = self.values.copy()
        self.fault = ""

    def reply(self, identifier, raw=0, kind=120):
        return struct.pack(
            "<HHBBBBIII",
            0,
            0,
            identifier,
            0 if kind == 122 else parameters.CATALOG[identifier]["value_type"],
            0 if kind == 122 else 3,
            1,
            raw,
            self.revision,
            self.saved_revision,
        )

    def status(self):
        data = bytearray(45)
        data[43] = 6 if self.fault == "running" else 5
        return bytes(data)

    def request(self, kind, payload=b"", *, page=None):
        self.calls.append((kind, payload))
        if kind == 1:
            data = bytearray(32)
            if page == 0:
                data[:12] = b"\x02" * 12 if self.fault == "uid" else b"\x01" * 12
            else:
                data[21:23] = bytes((6, 1))
            return bytes(data)
        if kind == 2:
            return struct.pack("<I12x", 1 << 14)
        if kind in (120, 123):
            identifier = payload[0]
            value = (self.stored if kind == 123 else self.values)[identifier]
            if kind == 123 and self.fault == "flash_mismatch":
                value += 1
            return self.reply(identifier, value)
        if kind == 121:
            identifier, value_type, length, reserved, revision, raw = struct.unpack(
                "<4BII", payload
            )
            if self.fault == "partial" and identifier == 12:
                return struct.pack("<HH", 11, 4)
            if self.fault == "write_timeout":
                self.values[identifier] = raw
                raise TimeoutError("写入应答丢失")
            if (value_type, length, reserved, revision) != (1, 4, 0, self.revision):
                raise ValueError("不正确的写入请求")
            self.values[identifier] = raw
            self.revision += 1
            return self.reply(identifier, raw)
        if kind == 122:
            if payload != struct.pack("<I", self.revision):
                raise ValueError("保存版本错误")
            self.stored = self.values.copy()
            self.saved_revision = self.revision
            if self.fault == "save_timeout":
                raise TimeoutError("保存应答丢失")
            return self.reply(0, kind=122)
        raise ValueError(f"非参数命令 {kind}")


class ParameterTests(unittest.TestCase):
    def setUp(self):
        self.endpoint = Endpoint()

    def run_operation(self, **kwargs):
        with patch.object(parameters, "Link", return_value=self.endpoint):
            return parameters.run_parameters(None, 6, "01" * 12, "current", **kwargs)

    def snapshot(self):
        return self.run_operation()

    def test_units_types_readonly_and_nonfinite(self):
        self.assertEqual(parameters.encode_value(16, "0.012"), 12000)
        self.assertEqual(parameters.encode_value(11, "1.0005"), 1001)
        self.assertEqual(parameters.value_text(8, 23000), "0.000023")
        for identifier, value in ((5, "8"), (11, "NaN"), (11, "1001"), (104, "2"), (11, "1e1000")):
            with self.subTest(identifier=identifier, value=value), self.assertRaises(ValueError):
                parameters.encode_value(identifier, value)

    def test_save_checks_flash_and_uses_updated_revision(self):
        snapshot = self.snapshot()
        report = self.run_operation(changes={11: 1200, 12: 2500}, snapshot=snapshot)
        self.assertTrue(report["persisted"], report)
        self.assertEqual(report["revision"], 9)
        self.assertEqual(self.endpoint.stored[12], 2500)
        self.assertEqual(sum(kind == 122 for kind, _ in self.endpoint.calls), 1)
        self.assertTrue(all(kind in (1, 2, 120, 121, 122, 123) for kind, _ in self.endpoint.calls))

    def test_identity_running_and_revision_block_all_writes(self):
        for fault in ("uid", "running", "revision", "same_revision_changed"):
            with self.subTest(fault=fault):
                self.endpoint = Endpoint()
                snapshot = self.snapshot()
                self.endpoint.fault = fault
                if fault == "revision":
                    self.endpoint.revision += 1
                if fault == "same_revision_changed":
                    self.endpoint.values[11] += 1
                report = self.run_operation(changes={11: 1200}, snapshot=snapshot)
                self.assertFalse(report["passed"])
                self.assertFalse(any(kind in (121, 122) for kind, _ in self.endpoint.calls))

    def test_partial_failure_keeps_confirmed_ram_change_without_save(self):
        snapshot = self.snapshot()
        self.endpoint.fault = "partial"
        report = self.run_operation(changes={11: 1200, 12: 2500}, snapshot=snapshot)
        self.assertFalse(report["passed"])
        self.assertEqual(list(report["applied"]), [11])
        self.assertEqual(self.endpoint.values[11], 1200)
        self.assertFalse(report["save_sent"])

    def test_lost_ack_and_flash_mismatch_are_not_success_or_retried(self):
        for fault in ("write_timeout", "save_timeout", "flash_mismatch"):
            with self.subTest(fault=fault):
                self.endpoint = Endpoint()
                snapshot = self.snapshot()
                self.endpoint.fault = fault
                report = self.run_operation(changes={11: 1200}, snapshot=snapshot)
                self.assertFalse(report["passed"])
                self.assertFalse(report["persisted"])
                self.assertEqual(sum(kind == 121 for kind, _ in self.endpoint.calls), 1)
                self.assertLessEqual(sum(kind == 122 for kind, _ in self.endpoint.calls), 1)

    def test_cross_group_and_cancel_never_write(self):
        for kwargs in (dict(changes={1: 30000}), dict(changes={11: 1200}, stopped=lambda: True)):
            snapshot = self.snapshot()
            report = self.run_operation(snapshot=snapshot, **kwargs)
            self.assertFalse(report["passed"])
            self.assertFalse(any(kind in (121, 122) for kind, _ in self.endpoint.calls))

    def test_read_progress_and_cancel_between_parameters(self):
        progress = []
        report = self.run_operation(
            progress=lambda done, total: progress.append((done, total)),
            stopped=lambda: len(progress) == 2,
        )
        self.assertFalse(report["passed"])
        self.assertEqual(progress, [(1, 4), (2, 4)])
        self.assertEqual(sum(kind == 120 for kind, _ in self.endpoint.calls), 2)
        self.assertFalse(any(kind in (121, 122) for kind, _ in self.endpoint.calls))
        self.assertGreaterEqual(report["elapsed_s"], 0)

    def test_cancel_before_start_does_not_access_device(self):
        report = self.run_operation(stopped=lambda: True)
        self.assertFalse(report["passed"])
        self.assertEqual(self.endpoint.calls, [])

    def test_read_all_has_one_identity_check_and_one_revision(self):
        self.endpoint.values = {i: 1000 for i in parameters.parameter_ids("all")}
        progress = []
        with patch.object(parameters, "Link", return_value=self.endpoint):
            report = parameters.run_parameters(
                None,
                6,
                "01" * 12,
                "all",
                progress=lambda done, total: progress.append((done, total)),
            )
        self.assertTrue(report["passed"], report)
        self.assertEqual(len(report["rows"]), 30)
        self.assertEqual(progress[-1], (30, 30))
        self.assertEqual(sum(kind == 1 for kind, _ in self.endpoint.calls), 2)
        self.assertEqual(sum(kind == 2 for kind, _ in self.endpoint.calls), 1)
        self.assertEqual(sum(kind == 120 for kind, _ in self.endpoint.calls), 30)
        self.assertFalse(any(kind in (121, 122) for kind, _ in self.endpoint.calls))

    def test_all_read_never_accepts_write(self):
        with patch.object(parameters, "Link", return_value=self.endpoint):
            report = parameters.run_parameters(None, 6, "01" * 12, "all", changes={11: 100})
        self.assertFalse(report["passed"])
        self.assertEqual(self.endpoint.calls, [])

    def test_changed_revision_during_all_read_is_not_published(self):
        self.endpoint.values = {i: 1000 for i in parameters.parameter_ids("all")}

        def progress(done, total):
            if done == 5:
                self.endpoint.revision += 1

        with patch.object(parameters, "Link", return_value=self.endpoint):
            report = parameters.run_parameters(None, 6, "01" * 12, "all", progress=progress)
        self.assertFalse(report["passed"])
        self.assertNotIn("rows", report)
