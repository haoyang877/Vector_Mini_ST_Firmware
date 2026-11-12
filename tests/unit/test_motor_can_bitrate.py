"""主机工具按真实参数 revision 保存，使用协议黄金字节并检查 Flash 回读。"""

import contextlib
import io
import json
import struct
import unittest
from pathlib import Path
from unittest.mock import patch

from tools.bench import motor_can_bitrate as bitrate

ROOT = Path(__file__).resolve().parents[2]


class CanBitrateHostTest(unittest.TestCase):
    def reply(self, kind):
        value = (1000 << 16) | 3000
        if kind == 120:
            return bitrate.REPLY.pack(0, 0, 0x71, 2, 3, 1, (500 << 16) | 2000, 1, 1)
        if kind == 121:
            return bitrate.REPLY.pack(0, 0, 0x71, 2, 3, 3, value, 2, 1)
        if kind == 122:
            return bitrate.REPLY.pack(0, 0, 0, 0, 0, 1, 0, 2, 2)
        return bitrate.REPLY.pack(0, 0, 0x71, 2, 3, 1, value, 2, 2)

    def test_write_save_and_read_back_match_wire_vectors(self):
        fixture = ROOT / "tests/fixtures/protocol/at32f456_can_bitrate_v1.json"
        vectors = json.loads(fixture.read_text())["set_1M_3M_from_revision_1"]
        with patch.object(
            bitrate, "request", side_effect=lambda bus, node, k, p: self.reply(k)
        ) as send:
            with contextlib.redirect_stdout(io.StringIO()):
                bitrate.configure(None, 3, [1000, 3000])
        actual = [(c.args[2], c.args[3]) for c in send.call_args_list]
        self.assertEqual(actual, [(int(k), bytes.fromhex(p)) for k, p in vectors.items()])

    def test_denied_write_stops_before_save(self):
        with patch.object(
            bitrate, "request", side_effect=[self.reply(120), struct.pack("<HH", 2, 5)]
        ) as send:
            with self.assertRaisesRegex(RuntimeError, "detail=5"):
                bitrate.configure(None, 3, [1000, 3000])
            self.assertEqual(send.call_count, 2)

    def test_mismatched_flash_is_reported(self):
        saved = bitrate.REPLY.pack(0, 0, 0x71, 2, 3, 1, (500 << 16) | 2000, 2, 2)
        replies = [self.reply(k) for k in (120, 121, 122)] + [saved]
        with patch.object(bitrate, "request", side_effect=replies):
            with self.assertRaisesRegex(RuntimeError, "Flash 参数回读不匹配"):
                bitrate.configure(None, 3, [1000, 3000])
