"""通过 GUI 使用的工作进程入口验证真实烧录逻辑，所有传输由内存探针完成。"""

import hashlib
import io
import json
import shutil
import struct
import sys
import tempfile
import unittest
import zlib
from pathlib import Path
from queue import SimpleQueue
from unittest.mock import MagicMock, Mock, patch

from intelhex import IntelHex

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools/bench"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import five_axis_can_stability as can  # noqa: E402
import five_axis_jlink as swd  # noqa: E402
from test_device_maintenance import Probe  # noqa: E402
from upgrade_client import crc8, crc16  # noqa: E402


class FlashGuiTests(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.directory = Path(temp.name)
        self.target = "yg_esc_v1_at32f456.json"
        shutil.copyfile(ROOT / "shared/targets" / self.target, self.directory / self.target)
        self.probe = Probe()
        self.probe.registers[0xE0042000] = 0x70053253
        self.probe.registers[0x40023874] = 3
        self.serial = 123
        self.node = 3
        self.probe.connected_emulators = Mock(return_value=[Mock(SerialNumber=self.serial)])
        for method in ("open", "opened", "close", "set_tif", "connect"):
            setattr(self.probe, method, Mock(return_value=True))
        self.probe.exec_command = Mock(return_value=0)
        self.probe.speed = 1000
        self.probe.set_speed = Mock(side_effect=lambda khz: setattr(self.probe, "speed", khz))
        self.probe.reset = Mock()
        self.probe.memory_write32 = self.write_register
        self.probe.restart = Mock()
        self.probe.halted = Mock(return_value=False)
        desc = struct.pack(">4I", 0x59494431, 3, 0x53593549, 7)
        self.probe.flash[0x4200:0x4210] = desc
        raw = bytearray([0x52] * 16384)
        struct.pack_into("<f", raw, 0, float(self.node))
        struct.pack_into("<II", raw, 2160, 14, 0x454E4332)
        struct.pack_into("<I", raw, 2140, zlib.crc32(raw[2144:2288], zlib.crc32(raw[:2140])))
        self.probe.flash[0x1C000:0x20000] = raw
        self.before = bytes(self.probe.flash)
        self.image = IntelHex()
        self.image.frombytes(b"loader", offset=0x08000000)
        self.image.frombytes(b"x" * 512 + desc, offset=0x08004000)
        self.hex_file = self.directory / "paired.hex"
        self.batch = {"motors": [{"uid": self.probe.uid.hex(), "node": self.node}]}
        self.freeze_image()
        self.probe.flash_file = Mock(side_effect=self.write_hex)
        self.pylink = Mock()
        self.pylink.JLink.return_value = self.probe
        self.identity = {"uid": self.probe.uid.hex(), "serial": self.serial, "node": self.node}
        self.number = 0
        self.bus = MagicMock()
        self.port = self.bus.__enter__.return_value
        self.port.receive.side_effect = self.can_reply
        (self.directory / "ControlCANFD.dll").touch()

    def write_register(self, address, values):
        for port in (0x40020000, 0x40020400):
            if address == port + 0x28:
                self.probe.registers[port + 0x14] = (
                    self.probe.registers.get(port + 0x14, 0) & ~values[0]
                )
                return
            if address in (port, port + 4, port + 12):
                self.probe.registers[address] = values[0]
                return
        if address in (0x40023830, 0x40023844):
            self.probe.registers[address] = values[0]
            return
        Probe.memory_write32(self.probe, address, values)

    def can_reply(self, timeout):
        self.assertGreater(timeout, 0)
        _, request = self.port.send.call_args.args
        kind, seq = struct.unpack_from("<HH", request, 6)
        payload = bytearray(32 if kind == 1 else 45)
        if kind == 1:
            payload[:12] = self.probe.uid
            struct.pack_into("<II", payload, 20, 0x602, 0x20200)
        else:
            payload[42] = self.node
        header = struct.pack(
            "<HBBBBHHIB", 0xA55A, 1, 0x10, self.node, 2, kind, seq, len(payload), 0
        )
        frame = header + bytes([crc8(header)]) + payload
        return 0x0CEF0200 | self.node, (frame + struct.pack("<H", crc16(frame))).ljust(64, b"\0")

    def freeze_image(self):
        stream = io.StringIO()
        self.image.write_hex_file(stream)
        content = stream.getvalue().encode()
        self.hex_file.write_bytes(content)
        self.batch["hex_sha256"] = hashlib.sha256(content).hexdigest()
        (self.directory / "five_axis_can_stability.json").write_text(
            json.dumps(self.batch), encoding="utf-8"
        )

    def write_hex(self, path, offset):
        self.assertEqual(offset, 0)
        image = IntelHex(path)
        for start, end in image.segments():
            self.probe.flash_write8(start, image.tobinstr(start=start, end=end - 1))

    def job(self, mode="jlink_flash", identity=None, restart=False, first_fit_node=None):
        self.number += 1
        directory = self.directory / str(self.number)
        directory.mkdir()
        settings = {
            "profile": mode,
            "identity": identity or self.identity,
            "hex": str(self.hex_file),
            "restart_after_flash": restart,
            "first_fit_node": first_fit_node,
            "bus": {"channel_index": 0, "nominal_bps": 1000000, "data_bps": 5000000},
        }
        events = SimpleQueue()
        with (
            patch.dict(sys.modules, {"pylink": self.pylink}),
            patch.object(can, "ControlCanFd", return_value=self.bus),
            patch.object(can.os, "add_dll_directory", return_value=MagicMock(), create=True),
            patch.object(
                swd, "controlcan_dll_path", return_value=self.directory / "ControlCANFD.dll"
            ),
            patch.object(sys, "frozen", True, create=True),
            patch.object(sys, "_MEIPASS", str(self.directory), create=True),
        ):
            swd.run_jlink(settings, directory, events, None)
        report = json.loads((directory / "summary.json").read_text(encoding="utf-8"))
        while not events.empty():
            last = events.get()
        self.assertEqual(last, ("done", report))
        return report

    def test_identify_program_and_repeat(self):
        first = self.job("jlink_identify")
        self.assertEqual(first["identity"], self.identity)
        self.assertEqual((self.probe.halts, self.probe.writes), (0, []))
        report = self.job(identity=first["identity"])
        self.assertTrue(report["success"], report)
        self.assertEqual(report["swd_khz"], 8000)
        self.assertTrue(report["cpu_halted"] and report["protected_regions_verified"])
        self.assertEqual((self.directory / "2/before/flash.bin").read_bytes(), self.before)
        self.assertEqual(self.probe.flash[0x1C000:], self.before[0x1C000:])
        self.assertTrue(self.job()["already_installed"])
        self.probe.reset.assert_not_called()
        self.probe.flash_file.assert_called_once()

    def allow_core_reset(self):
        original_write = self.probe.memory_write32
        self.probe.registers[0xE000EDFC] = 0x01000001
        self.probe.registers[0xE000ED0C] = 0xFA050300
        self.probe.registers[0xE000EDF0] = 0x3000B
        self.probe.registers[0xE000ED08] = 0x08004000

        def write(address, values):
            if address == 0xE000ED0C:
                self.assertEqual(values, [0x05FA0304])
                self.assertFalse(self.probe.registers[0xE000EDF0] & 8)
                self.probe.reset(halt=False)
            elif address == 0xE000EDF0:
                self.assertEqual(values, [0xA05F0003])
                self.probe.registers[address] = 0x30003
            elif address == 0xE000EDFC:
                self.probe.registers[address] = values[0]
            else:
                original_write(address, values)

        self.probe.memory_write32 = write

    def test_boot_after_write_and_after_skipping_identical_flash(self):
        self.allow_core_reset()
        self.probe.halted.side_effect = [True, False, False, False]
        for installed in (False, True):
            report = self.job(restart=True)
            self.assertTrue(report["success"], report)
            self.assertTrue(report["app_started"] and report["parameters_verified_after_boot"])
            self.assertFalse(report["cpu_halted"])
            self.assertEqual(report["can_test"]["passed"], 11)
            self.assertEqual(bool(report.get("already_installed")), installed)
            self.probe.reset.assert_called_with(halt=False)
        self.assertEqual(self.probe.reset.call_count, 2)
        self.probe.restart.assert_called_once()
        self.assertEqual(self.probe.registers[0xE000EDFC], 0x01000000)
        self.probe.flash_file.assert_called_once()

    def test_loader_still_running_after_reset_is_reported_separately(self):
        self.allow_core_reset()
        self.probe.registers[0xE000ED08] = 0x08000000
        with patch.object(swd.time, "monotonic", side_effect=range(0, 200, 5)):
            report = self.job(restart=True)
        self.assertFalse(report["success"])
        self.assertTrue(report["protected_regions_verified"])
        self.assertIn("未检测到 APP", report["fatal"])
        self.assertNotIn("app_started", report)

    def test_can_timeout_after_boot_keeps_flash_and_boot_results(self):
        self.allow_core_reset()
        self.port.receive.side_effect = [None]
        report = self.job(restart=True)
        self.assertTrue(report["protected_regions_verified"] and report["app_started"])
        self.assertFalse(report["success"])
        self.assertEqual(report["can_test"]["passed"], 0)
        self.assertIn("50 ms", report["fatal"])
        self.port.send.assert_called_once()

    def test_identify_accepts_observed_segger_command_results(self):
        self.probe.exec_command.side_effect = [1, 1, 0, 1, 1]
        report = self.job("jlink_identify")
        self.assertTrue(report["success"], report)
        self.assertEqual(report["identity"], self.identity)
        self.probe.flash_file.assert_not_called()

    def test_unstable_high_speed_falls_back_before_writing(self):
        original = self.probe.memory_read8

        def noisy_read(address, size):
            if address == 0x08004000 and size == 4096:
                if self.probe.speed == 8000:
                    raise OSError("SWD error")
                if self.probe.speed == 4000:
                    return bytes(size)
            return original(address, size)

        self.probe.memory_read8 = noisy_read
        report = self.job()
        self.assertTrue(report["success"], report)
        self.assertEqual(report["swd_khz"], 1000)
        self.assertEqual(
            [c.args[0] for c in self.probe.set_speed.call_args_list], [8000, 4000, 1000]
        )

    def test_unstable_low_speed_aborts_before_writing(self):
        original = self.probe.memory_read8

        def noisy_read(address, size):
            if self.probe.set_speed.called and address == 0x08004000 and size == 4096:
                return bytes(size)
            return original(address, size)

        self.probe.memory_read8 = noisy_read
        report = self.job()
        self.assertFalse(report["success"] or report["write_started"])
        self.probe.flash_file.assert_not_called()

    def test_command_error_stops_before_swd(self):
        self.probe.exec_command.side_effect = RuntimeError("J-Link rejected the command")
        report = self.job("jlink_identify")
        self.assertFalse(report["success"] or report["write_started"])
        self.assertIn("rejected", report["fatal"])
        self.probe.connect.assert_not_called()
        self.probe.flash_file.assert_not_called()
        self.probe.close.assert_called_once()

    def test_mismatch_refuses_before_write(self):
        for key, value in (("uid", "00" * 12), ("serial", 321)):
            with self.subTest(key=key):
                result = self.job(identity={**self.identity, key: value})
                self.assertFalse(result["success"] or result["write_started"])
        self.probe.uid = bytes([255]) * 12
        self.assertIn("UID 无效", self.job()["fatal"])
        self.assertEqual(self.probe.halts, 0)
        self.probe.flash_file.assert_not_called()

    def blank_board(self, node):
        self.node = node
        self.probe.uid = bytes(range(node, node + 12))
        self.probe.flash[:] = b"\xff" * len(self.probe.flash)
        self.probe.flash[0x1F800:0x1F804] = b"SN42"
        folder = self.directory / "first-fit"
        folder.mkdir(exist_ok=True)
        record = bytearray(2288)
        struct.pack_into("<f", record, 0, float(node))
        struct.pack_into("<I", record, 2136, 1)
        struct.pack_into("<II", record, 2160, 14, 0x454E4332)
        struct.pack_into("<4I", record, 2208, 0x32535841, 2, swd.FIRST_FIT_AXES[node][2], 2)
        struct.pack_into("<I", record, 2236, zlib.crc32(record[2208:2236]))
        struct.pack_into("<I", record, 2140, zlib.crc32(record[2144:], zlib.crc32(record[:2140])))
        (folder / f"{node}.bin").write_bytes(record)
        report = self.job("jlink_identify")
        self.assertTrue(report["success"], report)
        self.assertIsNone(report["identity"]["node"])
        return report["identity"], record

    def test_first_fit_all_five_roles_and_normal_update(self):
        self.allow_core_reset()
        for node in swd.FIRST_FIT_AXES:
            with self.subTest(node=node):
                identity, record = self.blank_board(node)
                self.assertFalse(self.job(identity=identity)["write_started"])
                result = self.job(identity=identity, first_fit_node=node, restart=True)
                self.assertTrue(result["success"], result)
                self.assertTrue(result["parameters_verified_after_boot"])
                self.assertEqual(result["can_test"]["passed"], 11)
                self.assertEqual(self.probe.flash[0x1C000 : 0x1C000 + len(record)], record)
                self.assertEqual(self.probe.flash[0x1F800:0x1F804], b"SN42")
                binding = self.directory / f"first-fit-{identity['uid']}.json"
                self.assertEqual(json.loads(binding.read_text())["node"], node)
                identified = self.job("jlink_identify")["identity"]
                self.assertEqual(identified, {**identity, "node": node})
                self.assertTrue(self.job(identity=identified)["already_installed"])
                self.assertFalse(
                    self.job(identity=identified, first_fit_node=node)["write_started"]
                )

    def test_first_fit_with_gated_gpio_clocks(self):
        identity, _ = self.blank_board(3)
        read = self.probe.memory_read32

        def gated_read(address, size):
            if 0x40020000 <= address < 0x40020800 and not read(0x40023830, 1)[0] & 3:
                return [0xEBFFFFFF if address < 0x40020400 else 0xFFFFFF8F]
            return read(address, size)

        self.probe.memory_read32 = gated_read
        self.probe.registers.update({0x40020000: 0xEBFFFFFF, 0x40020400: 0xFFFFFF8F})
        self.allow_core_reset()
        result = self.job(identity=identity, first_fit_node=3, restart=True)
        self.assertTrue(result["success"], result)
        self.assertEqual(result["can_test"]["passed"], 11)
        self.assertEqual(self.probe.registers[0x40020000], (0xEBFFFFFF & ~0xFC0000) | 0x540000)
        self.assertEqual(self.probe.registers[0x40020400], (0xFFFFFF8F & ~0x30F) | 0x105)

    def test_real_high_pin_blocks_first_fit(self):
        identity, _ = self.blank_board(3)
        self.probe.registers[0x40020410] = 0x10
        result = self.job(identity=identity, first_fit_node=3)
        self.assertIn("B 端口高电平=0x0010", result["fatal"])
        self.assertFalse(result["write_started"])
        self.probe.flash_file.assert_not_called()

    def test_existing_program_with_empty_parameters_keeps_output_interlock(self):
        identity, _ = self.blank_board(3)
        self.probe.flash[0x4000] = 0
        self.probe.registers[0x40010044] = 0x8000
        result = self.job(identity=identity, first_fit_node=3)
        self.assertIn("主输出=0x8000", result["fatal"])
        self.assertEqual(self.probe.halts, 0)
        self.assertNotIn(0x40023830, self.probe.registers)
        self.probe.flash_file.assert_not_called()

    def test_blank_board_starts_watchdog_clock_before_flash(self):
        identity, _ = self.blank_board(3)
        self.probe.registers[0x40023874] = 0
        original = self.probe.memory_write32

        def clock_write(address, values):
            if address == 0x40023874:
                self.assertEqual(values, [1])
                self.probe.registers[address] = 3
            else:
                original(address, values)

        self.probe.memory_write32 = clock_write
        result = self.job(identity=identity, first_fit_node=3)
        self.assertTrue(result["success"], result)
        self.assertEqual(self.probe.registers[0x40023874], 3)

    def test_wrong_chip_or_changed_probe_target_cannot_be_provisioned(self):
        identity, _ = self.blank_board(4)
        self.probe.registers[0xE0042000] ^= 1
        self.assertFalse(self.job(identity=identity, first_fit_node=4)["success"])
        self.probe.registers[0xE0042000] = 0x70053253
        self.probe.uid = bytes(range(8, 20))
        self.assertFalse(self.job(identity=identity, first_fit_node=4)["success"])
        self.probe.flash_file.assert_not_called()

    def test_corrupt_or_old_parameters_do_not_count_as_blank(self):
        identity, _ = self.blank_board(7)
        self.probe.flash[0x3C000] = 0
        result = self.job(identity=identity, first_fit_node=7)
        self.assertFalse(result["write_started"])
        self.assertIn("并非空白板", result["fatal"])
        self.probe.flash[0x1C000] = 0
        self.assertFalse(self.job("jlink_identify")["success"])
        self.probe.flash_file.assert_not_called()

    def test_first_fit_resumes_same_role_after_partial_parameter_write(self):
        identity, record = self.blank_board(6)

        def interrupt(path, address):
            self.probe.flash[0x1C000:0x1C008] = record[:8]
            raise OSError("probe removed")

        self.probe.flash_file.side_effect = interrupt
        self.assertFalse(self.job(identity=identity, first_fit_node=6)["success"])
        retry = self.job("jlink_identify")["identity"]
        self.assertEqual(retry["resume_node"], 6)
        self.assertFalse(self.job(identity=retry, first_fit_node=7)["write_started"])
        self.probe.flash_file.side_effect = self.write_hex
        result = self.job(identity=retry, first_fit_node=6)
        self.assertTrue(result["success"], result)
        self.assertIn("recovery_backup", result)

    def test_wrong_file_or_range_rejected_before_connect(self):
        self.hex_file.write_text("wrong", encoding="ascii")
        self.assertIn("SHA-256", self.job()["fatal"])
        self.image.frombytes(b"x", offset=0x0801C000)
        self.freeze_image()
        self.assertFalse(self.job()["success"])
        self.probe.open.assert_not_called()

    def test_corrupt_readback_fails_and_preserves_original_snapshot(self):
        def corrupt(path, address):
            self.write_hex(path, address)
            self.probe.flash[0x1F800] ^= 1

        self.probe.flash_file.side_effect = corrupt
        result = self.job(restart=True)
        self.probe.reset.assert_not_called()
        self.assertTrue(result["write_started"])
        self.assertFalse(result["success"])
        self.assertIn("备份不一致", result["fatal"])
        self.assertEqual((self.directory / "1/before/flash.bin").read_bytes(), self.before)

        retry = self.job()
        self.assertFalse(retry["success"] or retry["write_started"])
        self.assertIn("上次烧录前的备份不一致", retry["fatal"])
        self.probe.flash_file.assert_called_once()

    def test_interrupted_write_resumes_using_matching_original_backup(self):
        def interrupt(path, address):
            self.probe.flash[0x4200:0x4210] = bytes(16)
            raise OSError("SWD interrupted")

        self.probe.flash_file.side_effect = interrupt
        first = self.job()
        self.assertTrue(first["write_started"])
        self.assertFalse(first["success"])
        self.probe.flash_file.side_effect = self.write_hex
        result = self.job()
        self.assertTrue(result["success"], result)
        self.assertEqual(result["recovery_backup"], "1")
