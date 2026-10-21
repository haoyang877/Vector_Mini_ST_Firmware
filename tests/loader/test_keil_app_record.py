"""验证 Keil 后处理的真实 ELF 边界、记录同步及保留区域；全部离线。"""

from pathlib import Path
import argparse
import re
import struct
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
import zlib

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools/build"))
import keil_app_record as record_tool  # noqa: E402


def fixture():
    """构造含两个加载段和实际版本符号的 ARM ELF32 镜像。"""
    app = bytearray(b"\xff" * record_tool.CAPACITY)
    struct.pack_into("<II", app, 0, 0x20008000, record_tool.APP_BASE + 0x101)
    struct.pack_into("<I", app, 0x80, 0x00060000)
    raw = bytearray(0x200 + len(app) + 0x100)
    shoff = 0x200 + len(app)
    struct.pack_into(
        "<16sHHIIIIIHHHHHH",
        raw,
        0,
        b"\x7fELF\x01\x01\x01" + bytes(9),
        2,
        40,
        1,
        record_tool.APP_BASE + 0x101,
        52,
        shoff,
        0,
        52,
        32,
        2,
        40,
        3,
        0,
    )
    for index, offset, address, size in (
        (0, 0x100, record_tool.RECORD_BASE, 48),
        (1, 0x200, record_tool.APP_BASE, len(app)),
    ):
        struct.pack_into(
            "<8I", raw, 52 + index * 32, 1, offset, address, address, size, size, 4, 8
        )
    raw[0x200 : 0x200 + len(app)] = app
    strings = b"\0g_firmware_software_version\0"
    raw[0x140 : 0x140 + len(strings)] = strings
    struct.pack_into(
        "<10I", raw, shoff + 40, 0, 3, 0, 0, 0x140, len(strings), 0, 0, 1, 0
    )
    struct.pack_into("<10I", raw, shoff + 80, 0, 2, 0, 0, 0x180, 16, 1, 0, 4, 16)
    struct.pack_into("<IIIBBH", raw, 0x180, 1, record_tool.APP_BASE + 0x80, 4, 0, 0, 1)
    return raw


class KeilRecordTests(unittest.TestCase):
    """坏输入必须在覆盖任何输出之前拒绝；有效记录跟随实际镜像版本和字节。"""

    def test_both_keil_targets_require_record_sync_before_download(self):
        directory = ROOT / "firmware/platform/stm32g4/cubemx/MDK-ARM"
        for name in ("Vector_Mini_ST", "Vector_Mini_ST_HIL"):
            project = ET.parse(directory / (name + ".uvprojx"))
            hook = project.find(".//AfterMake")
            self.assertEqual(hook.findtext("RunUserProg1"), "1")
            self.assertEqual(hook.findtext("nStopA1X"), "2")
            command = hook.findtext("UserProg1Name")
            self.assertIn('keil_app_record.py" "#L"', command)
            self.assertIn('.venv\\Scripts\\python.exe"', command)
            self.assertEqual(
                project.findtext(".//LDads/ScatterFile"), "Vector_Mini_ST.sct"
            )
            self.assertEqual(project.findtext(".//LDads/umfTarg"), "0")
            options = project.findtext(".//LDads/Misc")
            self.assertIn("--pad=0xFF", options)
            self.assertIn("--keep=keil_boot_record.o(BOOT_RECORD)", options)
            sources = project.findall(".//File[FileName='keil_boot_record.c']")
            self.assertEqual(len(sources), 1)
            source = directory / sources[0].findtext("FilePath").replace("\\", "/")
            self.assertTrue(source.is_file())

    def test_ccm_code_reservation_excludes_its_physical_ram_alias(self):
        """共用 scatter 必须排除 CCM 别名；保留原栈堆和启动复制入口。"""
        directory = ROOT / "firmware/platform/stm32g4/cubemx/MDK-ARM"
        scatter = (directory / "Vector_Mini_ST.sct").read_text(encoding="utf-8")
        scatter = re.sub(r";[^\r\n]*", "", scatter)
        regions = {}
        for name in ("RW_IRAM1", "ER_CCM_HOT"):
            matches = re.findall(
                rf"\b{name}\s+(0x[0-9a-fA-F]+)\s+(0x[0-9a-fA-F]+)\s*\{{([^}}]*)\}}",
                scatter,
            )
            self.assertEqual(len(matches), 1, name)
            start, size, body = matches[0]
            regions[name] = (int(start, 16), int(size, 16), body)
        rw_start, rw_size, _ = regions["RW_IRAM1"]
        ccm_start, ccm_size, ccm_body = regions["ER_CCM_HOT"]
        self.assertEqual(rw_start, 0x20000020)  # 前 32 B 属于复位邮箱。
        self.assertEqual((ccm_start, ccm_size), (0x10002000, 0x800))
        alias_start = 0x20005800 + ccm_start - 0x10000000
        self.assertLessEqual(rw_start + rw_size, alias_start)
        self.assertEqual(alias_start + ccm_size, 0x20008000)
        compact = re.sub(r"\s+", "", scatter)
        for name, limit in (
            ("RW_IRAM1", alias_start),
            ("ER_CCM_HOT", ccm_start + ccm_size),
        ):
            self.assertIn(f"ScatterAssert(ImageLimit({name})<=0x{limit:08X})", compact)
        self.assertEqual(
            re.findall(r"(\w+\.o)\s*\((i\.\w+)\)", ccm_body),
            [
                ("foc_algorithm.o", "i.FOC_Current"),
                ("foc_algorithm.o", "i.FOC_SetVoltageModulation"),
                *[
                    ("foc_pid.o", "i.PI_Controller_" + name)
                    for name in ("Configure", "Reset", "Run", "TrackOutput")
                ],
                ("motor_thermal.o", "i.MotorThermal_Accumulate"),
                ("motor_thermal.o", "i.MotorThermal_Update"),
            ],
        )
        startup = (directory / "startup_stm32g431xx.s").read_text(encoding="utf-8")
        for name, expected in (("Stack_Size", 3072), ("Heap_Size", 512)):
            size = re.search(rf"\b{name}\s+EQU\s+(0x[0-9a-fA-F]+)", startup)
            self.assertIsNotNone(size)
            self.assertEqual(int(size.group(1), 16), expected)
        self.assertRegex(startup, r"LDR\s+R0,\s*=__main\s+BX\s+R0")

    def test_sync_is_idempotent_and_preserves_every_other_axf_byte(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "motor.axf"
            original = fixture()
            path.write_bytes(original)
            report = record_tool.synchronize(path)
            patched = path.read_bytes()
            self.assertEqual(original[:0x100], patched[:0x100])
            self.assertEqual(original[0x130:], patched[0x130:])
            app = path.with_suffix(".bin").read_bytes()
            self.assertEqual(
                patched[0x100:0x130], record_tool.boot_record(app, 0x00060000)
            )
            self.assertEqual(record_tool.synchronize(path), report)
            self.assertEqual(path.read_bytes(), patched)
            # 将稀疏 HEX 套用到随机旧 Flash，保护分区必须逐字节保持。
            flash = bytearray(b"\xa5" * 0x20000)
            upper = 0
            for line in path.with_suffix(".hex").read_text().splitlines():
                raw = bytes.fromhex(line[1:])
                self.assertEqual(sum(raw) & 255, 0)
                if raw[3] == 4:
                    upper = int.from_bytes(raw[4:6], "big") << 16
                elif raw[3] == 0:
                    address = upper + int.from_bytes(raw[1:3], "big") - 0x08000000
                    flash[address : address + raw[0]] = raw[4:-1]
            self.assertEqual(flash[:0x3000], b"\xa5" * 0x3000)
            self.assertEqual(flash[0x3030:0x4000], b"\xa5" * (0x4000 - 0x3030))
            self.assertEqual(flash[0x1C000:], b"\xa5" * 0x4000)

    def test_version_and_crc_follow_image_instead_of_source_header(self):
        raw = fixture()
        struct.pack_into("<I", raw, 0x280, 0x01020304)
        raw[0x400] = 0x37
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "motor.axf"
            path.write_bytes(raw)
            result = record_tool.synchronize(path)
            self.assertEqual(result["app_version"], 0x01020304)
            self.assertEqual(
                result["image_crc32"], zlib.crc32(path.with_suffix(".bin").read_bytes())
            )
            self.assertEqual(
                path.read_bytes()[0x100:0x130],
                record_tool.boot_record(
                    bytes(raw[0x200 : 0x200 + record_tool.CAPACITY]), 0x01020304
                ),
            )

    def test_invalid_or_protected_layout_never_changes_files(self):
        mutations = []
        for position, value in (
            (0, 0),
            (52 + 12, 0x0801C000),
            (84 + 16, 64),
            (0x204, 0x08000001),
            (0x180 + 4, 0x20000000),
        ):
            raw = fixture()
            if position == 0:
                raw[0] = value
            else:
                struct.pack_into("<I", raw, position, value)
            mutations.append(raw)
        mutations.append(fixture()[:100])
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "motor.axf"
            for raw in mutations:
                path.write_bytes(raw)
                with self.assertRaises((ValueError, struct.error)):
                    record_tool.synchronize(path)
                self.assertEqual(path.read_bytes(), raw)
                self.assertFalse(path.with_suffix(".hex").exists())


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc")
    parser.add_argument(
        "--out", type=Path, default=ROOT / "outputs/tests/keil_app_record"
    )
    args, remaining = parser.parse_known_args()
    result = unittest.main(argv=[sys.argv[0], *remaining], exit=False)
    if not result.result.wasSuccessful():
        raise SystemExit(1)
    if args.cc:
        from test_factory_package import native_record_check

        native_record_check(args.cc, args.out)
        raw = fixture()
        struct.pack_into("<I", raw, 0x280, 0x00050000)
        axf = args.out / "keil_fixture.axf"
        axf.write_bytes(raw)
        record_tool.synchronize(axf)
        record = args.out / "keil_record.bin"
        record.write_bytes(axf.read_bytes()[0x100:0x130])
        subprocess.run(
            [
                str((args.out / "factory_record.exe").resolve()),
                str(record.resolve()),
                str(axf.with_suffix(".bin").resolve()),
            ],
            check=True,
        )
        print("PASS patched Keil image accepted by shared C record/vector/CRC decoder")
