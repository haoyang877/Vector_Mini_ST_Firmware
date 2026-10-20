"""验证 Keil LoaderApp 后处理的同源调试镜像与 CAN 包，不访问硬件。"""

from __future__ import annotations

import json
import struct
import sys
import tempfile
import unittest
import zlib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools/build"))
from keil_loader_app import image_segment, synchronize  # noqa: E402


class KeilLoaderAppTests(unittest.TestCase):
    """用真实 v3 黄金镜像模拟一个 ARMCC 单加载段 AXF。"""

    def setUp(self) -> None:
        (ROOT / "outputs").mkdir(exist_ok=True)
        directory = tempfile.TemporaryDirectory(dir=ROOT / "outputs")
        self.addCleanup(directory.cleanup)
        self.axf = Path(directory.name) / "Vector_Mini_ST_LoaderApp.axf"
        vector = json.loads(
            (ROOT / "tests/fixtures/loader_image_v3.json").read_text(encoding="utf-8")
        )
        image = bytearray.fromhex(vector["source_hex"])
        constants = json.loads(
            (ROOT / "shared/targets/vector_mini_st_g431.json").read_text(encoding="utf-8")
        )["constants"]
        self.app_base = int(constants["TARGET_APP_BASE"].rstrip("U"), 0)
        self.app_bytes = int(constants["TARGET_APP_BYTES"].rstrip("U"), 0)
        descriptor = int(constants["TARGET_DESCRIPTOR_OFFSET"].rstrip("U"), 0)
        entry = self.app_base + descriptor + 96 + 1
        struct.pack_into("<I", image, 4, entry)
        for offset, key in (
            (12, "TARGET_LAYOUT"),
            (16, "TARGET_MCU"),
            (20, "TARGET_PRODUCT"),
            (24, "TARGET_HARDWARE"),
            (28, "TARGET_APP_VERSION"),
            (36, "TARGET_PARAM_ABI"),
        ):
            struct.pack_into(">I", image, 512 + offset, int(constants[key].rstrip("U"), 0))
        schema = int(constants["TARGET_SCHEMA"].rstrip("U"), 0)
        struct.pack_into(">HH", image, 512 + 40, schema, schema)
        phoff = 52 + len(image)
        elf = bytearray(phoff + 32)
        ident = b"\x7fELF\x01\x01\x01" + bytes(9)
        struct.pack_into(
            "<16sHHIIIIIHHHHHH",
            elf,
            0,
            ident,
            2,
            40,
            1,
            entry,
            phoff,
            phoff + 32,
            0,
            52,
            32,
            1,
            40,
            0,
            0,
        )
        elf[52:phoff] = image
        struct.pack_into(
            "<8I", elf, phoff, 1, 52, self.app_base, self.app_base, len(image), len(image), 7, 8
        )
        self.axf.write_bytes(elf)

    def test_debug_axf_hex_bin_and_can_package_share_payload(self) -> None:
        """直接下载镜像的三标记有效，CAN 包的三标记仍为擦除态。"""
        report = synchronize(self.axf, 7)
        _, debug = image_segment(self.axf.read_bytes(), self.app_base, self.app_bytes)
        self.assertEqual(debug, self.axf.with_suffix(".bin").read_bytes())
        package = (self.axf.parent / "app-package/image.bin").read_bytes()
        self.assertEqual(len(debug), len(package))
        self.assertEqual(debug[:576], package[:576])
        self.assertEqual(package[576:600], b"\xff" * 24)
        for offset, magic in ((64, 0x41435433), (72, 0x54525933), (80, 0x4F4B4133)):
            self.assertEqual(
                struct.unpack_from(">II", debug, 512 + offset), (magic, (~magic) & 0xFFFFFFFF)
            )
        normalized = bytearray(debug)
        normalized[560:564] = bytes(4)
        normalized[576:600] = b"\xff" * 24
        self.assertEqual(zlib.crc32(normalized), int(report["image_crc32"], 16))
        decoded = bytearray()
        upper = 0
        for line in self.axf.with_suffix(".hex").read_text(encoding="ascii").splitlines():
            record = bytes.fromhex(line[1:])
            self.assertEqual(sum(record) & 255, 0)
            if record[3] == 4:
                upper = int.from_bytes(record[4:6], "big") << 16
            elif record[3] == 0:
                self.assertEqual(
                    upper + int.from_bytes(record[1:3], "big"), self.app_base + len(decoded)
                )
                decoded.extend(record[4 : 4 + record[0]])
        self.assertEqual(bytes(decoded), debug)
        self.assertEqual(report["image_token"], 7)

    def test_invalid_axf_leaves_existing_file_untouched(self) -> None:
        """错误目标或占位损坏必须使后处理失败，不生成可下载文件。"""
        original = self.axf.read_bytes()
        damaged = bytearray(original)
        damaged[52 + 512 + 15] = 0
        self.axf.write_bytes(damaged)
        with self.assertRaises(ValueError):
            synchronize(self.axf, 7)
        self.assertEqual(self.axf.read_bytes(), damaged)
        self.assertFalse(self.axf.with_suffix(".hex").exists())

    def test_repeated_after_build_reuses_verified_outputs(self) -> None:
        """无重新链接的第二次 After Build 保留事务和全部工件。"""
        first = synchronize(self.axf, None)
        original = self.axf.read_bytes()
        second = synchronize(self.axf, None)
        self.assertEqual(second, first)
        self.assertEqual(self.axf.read_bytes(), original)
        self.assertEqual(second["image_token"], first["image_token"])

    def test_corrupt_previous_report_invalidates_download_outputs(self) -> None:
        """已补过 AXF 但旧清单损坏时不能把派生产物继续当成功包。"""
        synchronize(self.axf, 7)
        self.axf.with_suffix(".debug.json").write_text("{broken", encoding="utf-8")
        with self.assertRaises(ValueError):
            synchronize(self.axf, 7)
        self.assertTrue(self.axf.is_file())
        self.assertFalse(self.axf.with_suffix(".debug.json").exists())
        self.assertFalse(self.axf.with_suffix(".hex").exists())
        self.assertFalse(self.axf.with_suffix(".bin").exists())
        self.assertFalse((self.axf.parent / "app-package/image.bin").exists())

    def test_bad_new_axf_clears_previous_success_outputs(self) -> None:
        """成功构建后 AXF 损坏，旧清单与旧 CAN 包不能被复用。"""
        synchronize(self.axf, 7)
        damaged = bytearray(self.axf.read_bytes())
        damaged[52 + 512 + 15] = 0
        self.axf.write_bytes(damaged)
        with self.assertRaises(ValueError):
            synchronize(self.axf, 7)
        self.assertEqual(self.axf.read_bytes(), damaged)
        self.assertFalse(self.axf.with_suffix(".debug.json").exists())
        self.assertFalse(self.axf.with_suffix(".hex").exists())
        self.assertFalse(self.axf.with_suffix(".bin").exists())
        self.assertFalse((self.axf.parent / "app-package/image.bin").exists())


if __name__ == "__main__":
    unittest.main()
