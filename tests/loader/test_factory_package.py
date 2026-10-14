"""初装包与选择性烧录的离线替身；真实C解码记录，不加载任何探针DLL。"""

import argparse
import json
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
from types import SimpleNamespace
from unittest.mock import patch
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "loader/tools"))
import loader_factory as factory  # noqa: E402

UID = bytes(range(12))
APP = struct.pack("<II", 0x20008000, factory.APP_BASE + 9) + bytes(range(56))
LOADER = struct.pack("<II", 0x20008000, factory.FLASH_BASE + 9) + b"\0" * 56
MANIFEST = {
    "app_base": factory.APP_BASE,
    "layout_id": factory.LAYOUT_ID,
    "hardware_id": factory.HARDWARE_ID,
    "param_abi": factory.PARAM_ABI,
    "protocol_version": 2,
    "app_version": 0x00050000,
    "image_bytes": len(APP),
    "image_crc32": factory.crc32(APP),
    "image_sha256": factory.sha256(APP),
}
PARTS = factory.segments(LOADER, APP, factory.boot_record(APP, MANIFEST["app_version"]))


class Probe:
    """Flash替身限制每次写入分区，记录复位与数据损坏。"""

    def __init__(self):
        self.data = bytearray(b"\xa5" * factory.FLASH_BYTES)
        self.uid = UID
        self.writes = []
        self.resets = []
        self.stopped = False
        self.corrupt_parameters = False
        self.fail_at_app = False
        self.flash_kib = 128
        self.registers = {
            a: 0
            for a in (
                0x40012C20,
                0x40012C44,
                0x48000000,
                0x48000400,
                0x48000014,
                0x48000414,
                0x48000010,
                0x48000410,
            )
        }

    def memory_read32(self, address, count):
        if address == 0x1FFF7590:
            return struct.unpack("<3I", self.uid)
        if address == 0xE0042000:
            return [0x468]
        return [self.registers[address]]

    def memory_read16(self, address, count):
        if address != 0x1FFF75E0 or count != 1:
            raise RuntimeError("Unexpected flash-size read")
        return [self.flash_kib]

    def memory_write32(self, address, values):
        self.registers[address] = values[0]

    def memory_read8(self, address, count):
        offset = address - factory.FLASH_BASE
        return self.data[offset : offset + count]

    def flash_write8(self, address, data):
        if not any(
            base <= address and address + len(data) <= base + len(part)
            for base, part in PARTS
        ):
            raise RuntimeError("Attempted protected write")
        self.writes.append(address)
        if address == factory.APP_BASE and self.fail_at_app:
            raise RuntimeError("Simulated APP programming failure")
        offset = address - factory.FLASH_BASE
        self.data[offset : offset + len(data)] = bytes(data)
        if address == factory.APP_BASE and self.corrupt_parameters:
            self.data[factory.PARAM_BASE - factory.FLASH_BASE] ^= 1

    def halt(self):
        self.stopped = True

    def halted(self):
        return self.stopped

    def reset(self, halt):
        self.resets.append(halt)
        self.stopped = halt


class FactoryTests(unittest.TestCase):
    """检查HEX边界、篡改拒绝、写入顺序和失败后不启动。"""

    def test_package_hex_and_tamper(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "app"
            source.mkdir()
            (source / "app.bin").write_bytes(APP)
            (source / "manifest.json").write_text(
                json.dumps(MANIFEST), encoding="utf-8"
            )
            loader = root / "loader.bin"
            loader.write_bytes(LOADER)
            package = root / "package"
            factory.build_package(loader, source, package)
            parts, _ = factory.load_package(package)
            self.assertEqual(parts, PARTS)
            mapped = {}
            upper = 0
            for line in (package / "combined.hex").read_text().splitlines():
                raw = bytes.fromhex(line[1:])
                self.assertEqual(sum(raw) & 255, 0)
                self.assertEqual(len(raw), raw[0] + 5)
                if raw[3] == 4:
                    upper = int.from_bytes(raw[4:6], "big") << 16
                elif raw[3] == 0:
                    start = upper + int.from_bytes(raw[1:3], "big")
                    mapped.update(
                        {start + i: value for i, value in enumerate(raw[4:-1])}
                    )
            expected = {
                base + i: value for base, data in PARTS for i, value in enumerate(data)
            }
            self.assertEqual(mapped, expected)
            self.assertNotIn(factory.RECORD_BASE + factory.PAGE_BYTES, mapped)
            self.assertNotIn(factory.PARAM_BASE, mapped)
            (package / "app.bin").write_bytes(APP[:-1] + b"\xff")
            with self.assertRaisesRegex(ValueError, "hash/length"):
                factory.load_package(package)

    def test_old_app_and_oversize_loader_rejected(self):
        old_app = struct.pack("<II", 0x20008000, factory.FLASH_BASE + 9) + APP[8:]
        for loader, app in (
            (LOADER, old_app),
            (b"\0" * (factory.LOADER_BYTES + 1), APP),
        ):
            with self.assertRaises(ValueError):
                factory.validate_components(loader, app, MANIFEST)

    def test_raw_bin_input_and_wrong_flash_size(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "app.bin").write_bytes(APP)
            (root / "loader.bin").write_bytes(LOADER)
            factory.build_package(
                root / "loader.bin",
                None,
                root / "package",
                root / "app.bin",
                MANIFEST["app_version"],
            )
            parts, _ = factory.load_package(root / "package")
            self.assertEqual(parts, PARTS)
            probe = Probe()
            probe.flash_kib = 256
            with self.assertRaisesRegex(RuntimeError, "128 KiB"):
                factory.program_probe(probe, PARTS, UID, root)
            self.assertEqual(probe.writes, [])
            self.assertFalse(probe.halted())

    def test_program_preserves_all_unowned_bytes_and_commits_last(self):
        probe = Probe()
        before = bytes(probe.data)
        with tempfile.TemporaryDirectory() as directory:
            report = factory.program_probe(probe, PARTS, UID, Path(directory))
        self.assertEqual(report["status"], "PASS")
        self.assertEqual(
            probe.writes,
            [
                factory.RECORD_BASE,
                factory.FLASH_BASE,
                factory.APP_BASE,
                factory.RECORD_BASE,
            ],
        )
        self.assertEqual(probe.data[0x3800:0x4000], before[0x3800:0x4000])
        self.assertEqual(probe.data[0x1C000:], before[0x1C000:])
        self.assertEqual(probe.resets, [])
        self.assertTrue(probe.halted())

    def test_wrong_uid_and_enabled_pwm_do_not_write_or_halt(self):
        for wrong_uid in (True, False):
            probe = Probe()
            if wrong_uid:
                probe.uid = UID[::-1]
            else:
                probe.registers[0x40012C20] = 1
            with (
                tempfile.TemporaryDirectory() as directory,
                self.assertRaises(RuntimeError),
            ):
                factory.program_probe(probe, PARTS, UID, Path(directory))
            self.assertFalse(probe.stopped)
            self.assertEqual(probe.writes, [])

    def test_program_failure_or_parameter_damage_never_commits_or_resets(self):
        for attribute in ("corrupt_parameters", "fail_at_app"):
            probe = Probe()
            setattr(probe, attribute, True)
            with (
                tempfile.TemporaryDirectory() as directory,
                self.assertRaises(RuntimeError),
            ):
                factory.program_probe(probe, PARTS, UID, Path(directory), run=True)
            self.assertEqual(probe.resets, [])
            self.assertEqual(probe.data[0x3000:0x3800], b"\xff" * 0x800)

    def test_run_after_verification(self):
        probe = Probe()
        with (
            tempfile.TemporaryDirectory() as directory,
            patch.object(factory.time, "sleep"),
        ):
            report = factory.program_probe(probe, PARTS, UID, Path(directory), run=True)
        self.assertEqual(probe.resets, [False])
        self.assertTrue(report["cpu_running"])

    def test_startup_power_failure_disables_and_halts(self):
        probe = Probe()
        original_reset = probe.reset

        def unsafe_reset(halt):
            original_reset(halt)
            probe.registers[0x40012C20] = 1

        probe.reset = unsafe_reset
        with (
            tempfile.TemporaryDirectory() as directory,
            patch.object(factory.time, "sleep"),
        ):
            with self.assertRaisesRegex(RuntimeError, "power disable"):
                factory.program_probe(probe, PARTS, UID, Path(directory), run=True)
        self.assertTrue(probe.halted())
        self.assertEqual(probe.registers[0x40012C20] & 0x555, 0)
        self.assertEqual(probe.resets, [False])

    def test_close_failure_is_saved_and_never_returns_pass(self):
        probe = Probe()
        probe.open = lambda serial_no: None
        probe.exec_command = lambda command: None
        probe.set_tif = lambda interface: None
        probe.connect = lambda device, speed: None

        def failed_close():
            raise RuntimeError("close failed")

        probe.close = failed_close
        module = SimpleNamespace(
            JLink=lambda lib: probe,
            enums=SimpleNamespace(JLinkInterfaces=SimpleNamespace(SWD=1)),
        )
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "app.bin").write_bytes(APP)
            (root / "loader.bin").write_bytes(LOADER)
            factory.build_package(
                root / "loader.bin",
                None,
                root / "package",
                root / "app.bin",
                MANIFEST["app_version"],
            )
            args = SimpleNamespace(
                package=root / "package",
                uid=UID.hex(),
                jlink_serial=123,
                speed_khz=100,
                out=root / "flash",
                pylink_dir=None,
                jlink_dll=None,
                run=False,
            )
            with (
                patch.dict(sys.modules, {"pylink": module}),
                self.assertRaisesRegex(RuntimeError, "close failed"),
            ):
                factory.flash_package(args)
            result = json.loads((args.out / "summary.json").read_text())
            self.assertEqual(result["status"], "FAIL")
            self.assertEqual(result["close_error"], "close failed")
            self.assertEqual(probe.resets, [])

    def test_close_preserves_running_state_only_after_verified_run(self):
        for run, expected_policy in ((False, 0), (True, 1)):
            probe = Probe()
            policies = []
            probe.open = lambda serial_no: None
            probe.exec_command = lambda command: policies.append(int(command[-1]))
            probe.set_tif = lambda interface: None
            probe.connect = lambda device, speed: None

            def close():
                # 模拟实机DLL关闭时按策略恢复或暂停目标。
                probe.stopped = not policies[-1]

            probe.close = close
            module = SimpleNamespace(
                JLink=lambda lib: probe,
                enums=SimpleNamespace(JLinkInterfaces=SimpleNamespace(SWD=1)),
            )
            with tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                (root / "app.bin").write_bytes(APP)
                (root / "loader.bin").write_bytes(LOADER)
                factory.build_package(
                    root / "loader.bin",
                    None,
                    root / "package",
                    root / "app.bin",
                    MANIFEST["app_version"],
                )
                args = SimpleNamespace(
                    package=root / "package",
                    uid=UID.hex(),
                    jlink_serial=123,
                    speed_khz=100,
                    out=root / "flash",
                    pylink_dir=None,
                    jlink_dll=None,
                    run=run,
                )
                with (
                    patch.dict(sys.modules, {"pylink": module}),
                    patch.object(factory.time, "sleep"),
                ):
                    report = factory.flash_package(args)
                self.assertEqual(report["status"], "PASS")
                self.assertEqual(policies[-1], expected_policy)
                self.assertEqual(probe.halted(), not run)


def native_record_check(cc, out):
    """让真实共享C契约解码Python生成的记录与APP向量。"""
    out.mkdir(parents=True, exist_ok=True)
    source = out / "factory_record.c"
    source.write_text(
        '#include "boot_contract.h"\n#include <stdio.h>\n'
        "int main(int argc, char **argv) { unsigned char r[48]; static unsigned char v[BOOT_APP_CAPACITY]; "
        "BootImage image; FILE *f; size_t size; "
        'if(argc!=3) return 1; f=fopen(argv[1],"rb"); if(!f) return 2; '
        'if(fread(r,1,48,f)!=48) return 3; fclose(f); f=fopen(argv[2],"rb"); '
        "if(!f) return 4; size=fread(v,1,sizeof(v),f); if(fgetc(f)!=EOF) return 4; fclose(f); "
        "if(!Boot_RecordDecode(r,&image) || !Boot_VectorsValid(v,image.size)) return 5; "
        "if(image.size!=size || image.crc!=Boot_Crc32(v,size) || image.version!=0x00050000UL) return 6; return 0; }\n",
        encoding="ascii",
    )
    record = out / "record.bin"
    app = out / "app.bin"
    record.write_bytes(factory.boot_record(APP, MANIFEST["app_version"]))
    app.write_bytes(APP)
    exe = out / "factory_record.exe"
    compiler = [cc] + (["cc"] if Path(cc).stem == "zig" else [])
    subprocess.run(
        compiler
        + [
            "-std=c99",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-I",
            str(ROOT / "shared"),
            str(source),
            str(ROOT / "shared/boot_contract.c"),
            "-o",
            str(exe),
        ],
        check=True,
    )
    subprocess.run([str(exe), str(record), str(app)], check=True)
    print("PASS factory record accepted by real Boot_RecordDecode/Boot_VectorsValid")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc")
    parser.add_argument(
        "--out", type=Path, default=ROOT / "outputs/tests/factory_package"
    )
    args = parser.parse_args()
    result = unittest.TextTestRunner().run(
        unittest.defaultTestLoader.loadTestsFromTestCase(FactoryTests)
    )
    if not result.wasSuccessful():
        raise SystemExit(1)
    if args.cc:
        native_record_check(args.cc, args.out)
