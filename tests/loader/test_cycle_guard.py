"""耐久测试的离线探针替身；不加载DLL、不连接硬件。"""

from pathlib import Path
import struct
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "loader/tools"))
from loader_cycle_test import validate_power_samples, verify_outputs  # noqa: E402

UID = bytes(range(12))


class Probe:
    """记录允许的关相写入，模拟打开、身份和清理失败。"""

    def __init__(self):
        self.uid = UID
        self.is_halted = False
        self.fail_open = False
        self.fail_connect = False
        self.fail_write = False
        self.fail_close = False
        self.commands = []
        self.writes = []
        self.closed = False
        self.halt_count = 0
        self.registers = {
            0x40012C20: 0x1000,
            0x40012C44: 0x8000,
            0x48000000: 0x6BFFFFEF,
            0x48000400: 0xA9FAAABF,
        }

    def open(self, serial_no):
        if serial_no != 12345 or self.fail_open:
            raise RuntimeError("open failed")

    def exec_command(self, command):
        self.commands.append(command)

    def set_tif(self, interface):
        if interface != 1:
            raise RuntimeError("interface changed")

    def connect(self, device, speed):
        if self.fail_connect:
            raise RuntimeError("connect failed")

    def halted(self):
        return self.is_halted

    def memory_read32(self, address, count):
        if address == 0x1FFF7590 and count == 3:
            return struct.unpack("<3I", self.uid)
        return [self.registers[address]]

    def memory_write32(self, address, values):
        self.writes.append(address)
        if self.fail_write:
            raise RuntimeError("write failed")
        self.registers[address] = values[0]

    def halt(self):
        self.halt_count += 1
        self.is_halted = True

    def close(self):
        self.closed = True
        if self.fail_close:
            raise RuntimeError("close failed")


class GuardTests(unittest.TestCase):
    """验证功率掩码、错误板保护和异常后探针清理。"""

    def invoke(self, probe):
        module = SimpleNamespace(
            JLink=lambda lib: probe,
            library=SimpleNamespace(Library=lambda dllpath: None),
            enums=SimpleNamespace(JLinkInterfaces=SimpleNamespace(SWD=1)),
        )
        args = SimpleNamespace(jlink_dll=Path("unused.dll"), jlink_serial=12345)
        with patch.dict(sys.modules, {"pylink": module}):
            verify_outputs(args, UID)

    def test_ignores_other_pins_but_rejects_each_power_bit(self):
        safe = (0x1000, 0x3FC000, 15)
        validate_power_samples(safe, (0x1000, 0x3FC003, 31))
        for index, mask in enumerate((0x555, 0x3FC000, 15)):
            for bit in range(32):
                if mask & (1 << bit):
                    unsafe = list(safe)
                    unsafe[index] ^= 1 << bit
                    with self.subTest(index=index, bit=bit):
                        with self.assertRaisesRegex(RuntimeError, "not verified"):
                            validate_power_samples(safe, unsafe)

    def test_success_never_halts_or_writes(self):
        probe = Probe()
        self.invoke(probe)
        self.assertTrue(probe.closed)
        self.assertEqual(probe.halt_count, 0)
        self.assertEqual(probe.writes, [])
        self.assertEqual(probe.commands[-1], "SetRestartOnClose = 1")

    def test_wrong_uid_preserves_running_and_halted_board(self):
        for halted in (False, True):
            probe = Probe()
            probe.uid = UID[::-1]
            probe.is_halted = halted
            with self.assertRaisesRegex(RuntimeError, "UID changed"):
                self.invoke(probe)
            self.assertEqual(probe.writes, [])
            self.assertEqual(probe.halt_count, 0)
            self.assertTrue(probe.closed)
            self.assertEqual(
                probe.commands[-1], f"SetRestartOnClose = {0 if halted else 1}"
            )

    def test_open_or_connect_failure_never_resumes(self):
        probe = Probe()
        probe.fail_open = True
        with self.assertRaisesRegex(RuntimeError, "open failed"):
            self.invoke(probe)
        self.assertEqual(probe.commands, [])
        probe = Probe()
        probe.fail_connect = True
        with self.assertRaisesRegex(RuntimeError, "connect failed"):
            self.invoke(probe)
        self.assertTrue(probe.closed)
        self.assertEqual(probe.commands[-1], "SetRestartOnClose = 0")

    def test_unsafe_target_disables_only_power_registers(self):
        probe = Probe()
        probe.registers[0x40012C20] |= 1
        with self.assertRaisesRegex(RuntimeError, "not verified"):
            self.invoke(probe)
        self.assertEqual(probe.halt_count, 1)
        self.assertEqual(probe.writes, [0x40012C20, 0x40012C44, 0x48000000, 0x48000400])
        self.assertTrue(probe.closed)
        self.assertEqual(probe.commands[-1], "SetRestartOnClose = 0")

    def test_emergency_failure_still_closes_and_preserves_error(self):
        probe = Probe()
        probe.registers[0x40012C20] |= 1
        probe.fail_write = True
        with self.assertRaisesRegex(RuntimeError, "not verified.*write failed"):
            self.invoke(probe)
        self.assertTrue(probe.closed)
        self.assertEqual(probe.commands[-1], "SetRestartOnClose = 0")

    def test_close_failure_is_not_a_pass(self):
        probe = Probe()
        probe.fail_close = True
        with self.assertRaisesRegex(RuntimeError, "probe close: close failed"):
            self.invoke(probe)


if __name__ == "__main__":
    unittest.main()
