"""空板测试工装的事务回归，不连接探针。"""

import struct
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from test_device_maintenance import Probe

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools/bench"))
import install_axis_board as tool  # noqa: E402


class Board(Probe):
    def __init__(self):
        super().__init__()
        self.running = False
        self.cut_at = None

    def memory_write32(self, address, values):
        self.registers[address] = values[0]
        if address == 0xE000ED0C:
            self.registers[0xE000ED08] = tool.APP

    def flash_write8(self, address, data):
        if address == self.cut_at:
            self.cut_at = None
            super().flash_write8(address, data[:120])
            raise OSError("模拟写入中断")
        super().flash_write8(address, data)

    def halted(self):
        return not self.running

    def halt(self):
        self.running = False
        return super().halt()

    def reset(self, halt):
        self.running = not halt

    def restart(self):
        self.running = True


class InstallTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.board = Board()
        self.axis = tool.parse_axis("0x04")
        target = tool.maintenance.load_target("yg_esc_v1_at32f456")
        image = b"\xff" * 2048
        debug = bytearray(image)
        for delta, magic in tool.MARKERS:
            struct.pack_into(">II", debug, 512 + delta, magic, ~magic & 0xFFFFFFFF)
        self.bundle = tool.Bundle(target, b"L" * 256, image, bytes(debug), {}, "test")
        self.record = bytearray(2288)
        struct.pack_into("<f", self.record, 2152, 20)
        # 芯片的启动回读由单独实板验证；此处验证调用次序与 Flash 写入范围。
        self.boot = self.enterContext(patch.object(tool, "verify_boot"))
        self.enterContext(patch.object(tool.time, "sleep"))

    def process(self, verify=False):
        tool.process_board(self.board, self.bundle, self.axis, self.record, self.root, verify)

    def test_install_repeat_and_verify(self):
        self.process()
        writes = self.board.writes.copy()
        self.assertTrue(writes)
        self.assertEqual(self.board.flash[0x1C000 : 0x1C000 + len(self.record)], self.record)
        self.assertEqual(self.board.flash[0x1F800:], b"\xff" * (262144 - 0x1F800))
        self.process()
        self.process(verify=True)
        self.assertEqual(writes, self.board.writes)
        self.assertEqual(self.boot.call_count, 3)

    def test_mismatch_and_verify_only_do_not_write(self):
        with self.assertRaises(ValueError):
            self.process(verify=True)
        self.board.flash[-1] = 0x55
        with self.assertRaises(ValueError):
            self.process()
        self.assertFalse(self.board.writes)
        self.assertFalse(self.board.halts)

    def test_interrupted_record_resumes_only_matching_plan(self):
        self.board.cut_at = tool.PARAM
        with self.assertRaises(OSError):
            self.process()
        self.assertFalse(self.boot.called)
        self.record[0] = 7
        writes = self.board.writes.copy()
        with self.assertRaises(ValueError):
            self.process()
        self.assertEqual(writes, self.board.writes)
        self.record[0] = 0
        self.process()
        self.assertEqual(self.board.flash[0x1C000 : 0x1C000 + len(self.record)], self.record)
        self.boot.assert_called_once()
