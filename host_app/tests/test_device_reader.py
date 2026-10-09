"""只读辅助进程的假端口契约：默认测试绝不打开物理适配器。"""

from __future__ import annotations

import ctypes
import importlib
import io
import json
import struct
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
reader = importlib.import_module("device_reader")
adapter = importlib.import_module("ports.controlcanfd")
wire = importlib.import_module("upgrade_client")


def reply(node: int, kind: int, sequence: int, payload: bytes) -> tuple[int, bytes]:
    """生成严格合法的公司回复帧，仅供无设备测试。"""
    header = struct.pack("<HBBBBHHIB", 0xA55A, 1, 0x10, node, 2, kind, sequence, len(payload), 0)
    data = header + bytes([wire.crc8(header)]) + payload
    data += struct.pack("<H", wire.crc16(data))
    dlc = next(size for size in (20, 24, 32, 48, 64) if size >= len(data))
    return (3 << 26) | (0xEF << 16) | (2 << 8) | node, data.ljust(dlc, b"\x00")


class FakePort:
    """仅返回预置内存帧，记录所有发送请求。"""

    def __init__(self, frames: list[tuple[int, bytes]]) -> None:
        self.frames = frames
        self.sent: list[tuple[int, bytes]] = []

    def send(self, identifier: int, data: bytes) -> None:
        self.sent.append((identifier, data))

    def receive(self, timeout_s: float) -> tuple[int, bytes] | None:
        return self.frames.pop(0) if self.frames else None


class FakeFunction:
    """仅模拟 SDK 函数签名配置，不调用系统 DLL。"""

    def __init__(self, result: int = 0) -> None:
        self.result = result

    def __call__(self, *args: object) -> int:
        return self.result


class FakeDll:
    """SDK 打开返回失败，用于核对独占占用错误路径。"""

    def __init__(self) -> None:
        self.functions: dict[str, FakeFunction] = {}

    def __getattr__(self, name: str) -> FakeFunction:
        return self.functions.setdefault(name, FakeFunction())


class ReaderChecks(unittest.TestCase):
    def test_get_only_and_request_correlation(self) -> None:
        valid = reply(3, 108, 7, bytes(45))
        wrong = reply(4, 108, 7, bytes(45))
        port = FakePort([wrong, valid])
        self.assertEqual(reader.observe_once(port, 3, 108, 7), valid)
        self.assertEqual(len(port.sent), 1)
        self.assertEqual(port.sent[0][0], (3 << 26) | (0xEF << 16) | (3 << 8) | 2)
        for kind in (101, 110, 116, 117, 118, 121, 122, 160, 166):
            with self.assertRaises(ValueError):
                reader.observe_once(port, 3, kind, 8)
        self.assertEqual(len(port.sent), 1)
        with self.assertRaises(TimeoutError):
            reader.observe_once(FakePort([]), 3, 1, 9, 0)

    def test_line_protocol_rejects_write_without_sending(self) -> None:
        port = FakePort([reply(3, 2, 5, bytes(16))])
        output = io.StringIO()
        reader.serve(port, 3, io.StringIO('{"type":2,"sequence":5,"page":0}\n'), output)
        lines = [json.loads(line) for line in output.getvalue().splitlines()]
        self.assertEqual([line["event"] for line in lines], ["ready", "frame"])
        self.assertEqual(bytes.fromhex(lines[1]["data"]), reply(3, 2, 5, bytes(16))[1])
        with self.assertRaises(ValueError):
            reader.serve(
                port, 3, io.StringIO('{"type":116,"sequence":6,"page":0}\n'), io.StringIO()
            )
        self.assertEqual(len(port.sent), 1)

    def test_sdk_flags_and_exclusive_open_failure(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            fake_path = Path(directory) / "fake.dll"
            fake_path.write_bytes(b"fake")
            with patch.object(adapter.C, "WinDLL", return_value=FakeDll()):
                with self.assertRaisesRegex(OSError, "独占"):
                    with adapter.ControlCanFd(fake_path):
                        self.fail("fake SDK unexpectedly opened")
        port = adapter.ControlCanFd(Path("unused.dll"))
        port.channel = 1

        class ReceiveDll:
            """向既有端口返回单条假 SDK 帧。"""

            def __init__(
                self, *, eff: int = 1, brs: int = 1, rtr: int = 0, error: int = 0, esi: int = 0
            ) -> None:
                self.eff, self.brs, self.rtr = eff, brs, rtr
                self.error, self.esi = error, esi

            def ZCAN_ReceiveFD(self, channel: int, buffer: object, capacity: int, wait: int) -> int:
                item = ctypes.cast(buffer, ctypes.POINTER(adapter._RXFD))[0]
                item.frame.can_id = 0x08EF0203
                item.frame.length = 20
                item.frame.eff = self.eff
                item.frame.brs = self.brs
                item.frame.rtr = self.rtr
                item.frame.err = self.error
                item.frame.esi = self.esi
                return 1

        port.dll = ReceiveDll(eff=0)
        self.assertIsNone(port.receive(0))
        port.dll = ReceiveDll(brs=0)
        self.assertIsNone(port.receive(0))
        port.dll = ReceiveDll(rtr=1)
        self.assertIsNone(port.receive(0))
        port.dll = ReceiveDll(error=1)
        with self.assertRaisesRegex(OSError, "总线错误"):
            port.receive(0)
        port.dll = ReceiveDll(esi=1)
        with self.assertRaisesRegex(OSError, "error-passive"):
            port.receive(0)
        port.dll = ReceiveDll()
        self.assertEqual(port.receive(0), (0x08EF0203, bytes(20)))


if __name__ == "__main__":
    unittest.main()
