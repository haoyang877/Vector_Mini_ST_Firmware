"""主机工具按真实参数 revision 保存，使用协议黄金字节并检查 Flash 回读。"""

import contextlib
import io
import json
import struct
import sys
import tempfile
import unittest
from pathlib import Path
from queue import SimpleQueue
from threading import Event
from unittest.mock import patch

from tools.bench import motor_can_bitrate as bitrate

ROOT = Path(__file__).resolve().parents[2]
VECTORS = json.loads((ROOT / "tests/fixtures/protocol/at32f456_can_bitrate_v1.json").read_text())


class CanBitrateHostTest(unittest.TestCase):
    def reply(self, kind):
        value = (1000 << 16) | 3000
        if kind == 120:
            default = int.from_bytes(bytes.fromhex(VECTORS["boot_default"]["value_le"]), "little")
            return bitrate.REPLY.pack(0, 0, 0x71, 2, 3, 1, default, 1, 1)
        if kind == 121:
            return bitrate.REPLY.pack(0, 0, 0x71, 2, 3, 3, value, 2, 1)
        if kind == 122:
            return bitrate.REPLY.pack(0, 0, 0, 0, 0, 1, 0, 2, 2)
        return bitrate.REPLY.pack(0, 0, 0x71, 2, 3, 1, value, 2, 2)

    def test_write_save_and_read_back_match_wire_vectors(self):
        vectors = VECTORS["set_1M_3M_from_revision_1"]
        with patch.object(
            bitrate, "request", side_effect=lambda bus, node, k, p: self.reply(k)
        ) as send:
            with contextlib.redirect_stdout(io.StringIO()):
                bitrate.configure(None, 3, [1000, 3000])
        actual = [(c.args[2], c.args[3]) for c in send.call_args_list]
        self.assertEqual(actual, [(int(k), bytes.fromhex(p)) for k, p in vectors.items()])

    def test_cli_passes_default_and_all_firmware_profiles_to_selected_channel(self):
        dll = Path("ControlCANFD.dll").resolve()
        for profile, nominal, data, *_ in VECTORS["registers"]:
            with self.subTest(profile=profile):
                args = ["tool", "--dll", str(dll), "--node", "4", "--set", str(nominal), str(data)]
                if profile == 0:
                    args.extend(["--current", "500", "2000", "--channel-index", "1"])
                with (
                    patch("sys.argv", args),
                    patch.object(bitrate.os, "add_dll_directory", create=True),
                    patch.object(bitrate, "ControlCanFd") as adapter,
                    patch.object(bitrate, "configure") as configure,
                ):
                    bitrate.main()
                adapter.assert_called_once_with(
                    dll,
                    0,
                    channel_index=int(profile == 0),
                    nominal_bps=500_000 if profile == 0 else 1_000_000,
                    data_bps=2_000_000 if profile == 0 else 5_000_000,
                )
                configure.assert_called_once_with(
                    adapter.return_value.__enter__(), 4, [nominal, data]
                )

    def test_desktop_binds_uid_before_read_or_save(self):
        try:
            import tkinter  # noqa: F401
        except ImportError:
            self.skipTest("Tk 未安装")
        with patch.object(sys, "path", [str(ROOT / "tools/bench"), *sys.path]):
            import canfd_desktop as desktop

        uid = bytes(range(12))
        targets = [{"node": n, "uid": uid.hex()} for n in (6, 7)]
        for setting in [None, *[(row[1], row[2]) for row in VECTORS["registers"]]]:
            with (
                self.subTest(setting=setting),
                tempfile.TemporaryDirectory() as folder,
                patch.object(desktop, "controlcan_dll_path", return_value=Path("adapter.dll")),
                patch.object(desktop.os, "add_dll_directory", create=True),
                patch.object(desktop, "ControlCanFd") as adapter,
                patch.object(desktop, "read_node_uids", side_effect=[{uid}, {uid, bytes(12)}]),
                patch.object(
                    desktop, "configure_bitrate", return_value={"ram": 1, "flash": 1}
                ) as cfg,
            ):
                events = SimpleQueue()
                settings = {"profile": "can_bitrate", "bus": {}, "set": setting, "targets": targets}
                desktop.acquire(settings, Path(folder), events, Event())
                tag, result = events.get_nowait()
                self.assertEqual(tag, "done")
                self.assertFalse(result["pass"])
                self.assertIn("节点 7 身份", result["fatal"])
                self.assertEqual([row["node"] for row in result["results"]], [6])
                self.assertEqual(cfg.call_count, 1)
                self.assertEqual(cfg.call_args.args, (adapter.return_value.__enter__(), 6, setting))

    def test_read_legacy_record_even_with_zero_saved_revision(self):
        # 旧输出轴扩展可使整轴 saved_revision 为零，123 仍可校验并读取基础记录。
        stored = int.from_bytes(bytes.fromhex(VECTORS["saved_500K_2M"]["value_le"]), "little")
        reply = bitrate.REPLY.pack(0, 0, 0x71, 2, 3, 2, stored, 1, 0)
        with patch.object(bitrate, "request", return_value=reply) as send:
            with contextlib.redirect_stdout(io.StringIO()) as log:
                bitrate.configure(None, 4)
        self.assertEqual([call.args[2] for call in send.call_args_list], [120, 123])
        self.assertIn("Flash 500/2000", log.getvalue())

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
