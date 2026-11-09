"""双板维护的真实核心测试：全保护域、原样迁移及失败前零写入。"""

import struct
import sys
import tempfile
import unittest
import zlib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "loader/tools"))
import device_maintenance as maintenance  # noqa: E402


class Probe:
    """带完整Flash及操作记录的探针替身。"""

    def __init__(self, size=262144):
        self.flash = bytearray(b"\xff" * size)
        self.uid = bytes(range(12))
        self.writes = []
        self.halts = 0
        self.registers = {}

    def memory_read8(self, address, size):
        if address in (0x1FFFF7E8, 0x1FFF7590):
            return self.uid[:size]
        return self.flash[address - 0x08000000 : address - 0x08000000 + size]

    def memory_read16(self, address, size):
        return [len(self.flash) // 1024]

    def memory_read32(self, address, size):
        return [self.registers.get(address, 0)]

    def memory_write32(self, address, values):
        if address not in (0xE0042008, 0x40003000, 0x40003004, 0x40003008, 0x40003010):
            raise RuntimeError("Unexpected register write")
        self.registers[address] = values[0]

    def flash_write8(self, address, data):
        self.writes.append((address, len(data)))
        self.flash[address - 0x08000000 : address - 0x08000000 + len(data)] = bytes(data)

    def halt(self):
        self.halts += 1
        return True


class MaintenanceTests(unittest.TestCase):
    """只使用内存探针，不加载DLL或执行硬件操作。"""

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(dir=ROOT / "outputs")
        self.addCleanup(self.temp.cleanup)
        self.directory = Path(self.temp.name)
        self.target = maintenance.load_target("yg_esc_v1_at32f456")
        self.probe = Probe()

    def parameter_image(self):
        raw = bytearray(b"\x52" * 16384)
        struct.pack_into("<f", raw, 0, 3.0)
        struct.pack_into("<II", raw, 2160, 14, 0x454E4332)
        crc = zlib.crc32(raw[2144:2288], zlib.crc32(raw[:2140]))
        struct.pack_into("<I", raw, 2140, crc)
        return bytes(raw)

    def test_snapshot_is_read_only(self):
        report = maintenance.capture(
            self.probe, self.target, self.probe.uid, self.directory / "snapshot"
        )
        self.assertEqual(report["flash"]["bytes"], 262144)
        self.assertEqual((self.probe.writes, self.probe.halts), ([], 0))

    def test_large_snapshot_uses_bounded_transactions_for_two_complete_passes(self):
        calls = []

        def read(address, size):
            if size > 4096:
                raise RuntimeError("Probe rejects oversized transfer")
            calls.append((address, size))
            return self.probe.flash[address - 0x08000000 : address - 0x08000000 + size]

        self.probe.memory_read8 = read
        self.assertEqual(
            maintenance.read_twice(self.probe, 0x08000000, 262144), bytes(self.probe.flash)
        )
        self.assertEqual(calls[:64], calls[64:])
        self.assertEqual(len(calls), 128)

    def test_late_chunk_mismatch_is_rejected_without_retry(self):
        calls = []

        def read(address, size):
            calls.append(address)
            return bytes([0 if len(calls) == 4 else 255]) * size

        self.probe.memory_read8 = read
        with self.assertRaisesRegex(RuntimeError, "不一致"):
            maintenance.read_twice(self.probe, 0x08000000, 8192)
        self.assertEqual(len(calls), 4)

    def test_short_swd_read_fails_immediately(self):
        self.probe.memory_read8 = lambda address, size: bytes(size - 1)
        with self.assertRaisesRegex(RuntimeError, "短读"):
            maintenance.read_twice(self.probe, 0x08000000, 8192)

    def test_entire_protected_region_and_loader_are_checked(self):
        before = bytes(self.probe.flash)
        for offset in (0, 0x3FFF, 0x1C000, 0x1FFFF, 0x3FFFF):
            after = bytearray(before)
            after[offset] = 0
            with self.subTest(offset=offset), self.assertRaises(RuntimeError):
                maintenance.verify_protected(before, bytes(after), self.target)
        after = bytearray(before)
        after[0x4000] = 0
        self.assertTrue(
            maintenance.verify_protected(before, bytes(after), self.target)[
                "protected_regions_verified"
            ]
        )

    def test_wrong_uid_has_no_write_or_halt(self):
        with self.assertRaises(RuntimeError):
            maintenance.capture(self.probe, self.target, bytes(12), self.directory / "snapshot")
        self.assertEqual((self.probe.writes, self.probe.halts), ([], 0))

    def test_migration_preserves_all_bytes_and_is_bound_to_snapshot(self):
        parameters = self.parameter_image()
        self.probe.flash[0x3C000:] = parameters
        snapshot = self.directory / "snapshot"
        report = maintenance.capture(self.probe, self.target, self.probe.uid, snapshot)
        output = self.directory / "migration"
        result = maintenance.prepare_migration(snapshot, output, report["flash"]["sha256"])
        self.assertEqual((output / "parameters.bin").read_bytes(), parameters)
        self.assertEqual(result["destination_address"], 0x0801C000)
        with self.assertRaises(ValueError):
            maintenance.prepare_migration(snapshot, self.directory / "bad", "0" * 64)
        self.assertFalse((self.directory / "bad").exists())

    def test_invalid_parameter_crc_is_not_rebuilt(self):
        raw = bytearray(self.parameter_image())
        raw[100] ^= 1
        with self.assertRaises(ValueError):
            maintenance.validate_parameters(bytes(raw))

    def test_install_preserves_parameter_tail_and_leaves_trial_unconfirmed(self):
        self.probe.flash[0x1C000:0x20000] = self.parameter_image()
        original = bytes(self.probe.flash)
        image = bytearray(1024)
        image[0x240:0x258] = b"\xff" * 24
        report = maintenance.install(
            self.probe,
            self.target,
            self.probe.uid,
            bytes(256),
            bytes(image),
            self.directory / "install",
        )
        self.assertTrue(report["install_verified"])
        self.assertFalse(report["app_confirmed"])
        self.assertEqual(bytes(self.probe.flash[0x1C000:]), original[0x1C000:])
        self.assertEqual(self.probe.flash[0x4248:0x4258], b"\xff" * 16)
        self.assertEqual(self.probe.writes[-1], (0x08004000, 2048))

    def test_migration_refuses_already_installed_target(self):
        self.probe.flash[0x3C000:] = self.parameter_image()
        struct.pack_into(">4I", self.probe.flash, 0x4200, 0x59494431, 3, 0x53593549, 7)
        snapshot = self.directory / "snapshot"
        report = maintenance.capture(self.probe, self.target, self.probe.uid, snapshot)
        with self.assertRaises(ValueError):
            maintenance.prepare_migration(
                snapshot, self.directory / "migration", report["flash"]["sha256"]
            )

    def test_old_nonempty_parameters_require_explicit_migration_before_writes(self):
        self.probe.flash[0x3C000:] = self.parameter_image()
        image = bytearray(1024)
        image[0x240:0x258] = b"\xff" * 24
        with self.assertRaisesRegex(ValueError, "迁移包"):
            maintenance.install(
                self.probe,
                self.target,
                self.probe.uid,
                bytes(256),
                bytes(image),
                self.directory / "install",
            )
        self.assertEqual((self.probe.writes, self.probe.halts), ([], 0))

    def test_failed_halt_never_writes_flash(self):
        self.probe.halt = lambda: False
        image = bytearray(1024)
        image[0x240:0x258] = b"\xff" * 24
        with self.assertRaisesRegex(RuntimeError, "CPU暂停"):
            maintenance.install(
                self.probe,
                self.target,
                self.probe.uid,
                bytes(256),
                bytes(image),
                self.directory / "install",
            )
        self.assertEqual(self.probe.writes, [])

    def test_watchdog_update_timeout_never_writes_flash(self):
        self.probe.registers[0x4000300C] = 1
        image = bytearray(1024)
        image[0x240:0x258] = b"\xff" * 24
        with self.assertRaisesRegex(RuntimeError, "WDT配置更新超时"):
            maintenance.install(
                self.probe,
                self.target,
                self.probe.uid,
                bytes(256),
                bytes(image),
                self.directory / "install",
            )
        self.assertEqual(self.probe.writes, [])


if __name__ == "__main__":
    unittest.main()
