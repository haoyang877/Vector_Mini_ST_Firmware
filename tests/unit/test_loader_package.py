"""验证CRC32镜像打包和版本拒绝；仅处理合成文件，不访问硬件。"""

from __future__ import annotations

import json
import struct
import subprocess
import sys
import tempfile
import unittest
import zlib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
GOLDEN = ROOT / "tests/fixtures/loader_image_v3.json"
sys.path.insert(0, str(ROOT / "loader/tools"))
from package_image import build_package  # noqa: E402


class LoaderPackageTests(unittest.TestCase):
    """覆盖发布字节、旧profile与维护角色的实际打包入口。"""

    def current_source(self) -> bytes:
        """把冻结的 v3 示例映射到当前目标身份，供真实 CLI 拒绝路径使用。"""
        vector = json.loads(GOLDEN.read_text(encoding="utf-8"))
        image = bytearray.fromhex(vector["source_hex"])
        constants = json.loads(
            (ROOT / "shared/targets/vector_mini_st_g431.json").read_text(encoding="utf-8")
        )["constants"]
        app_base = int(constants["TARGET_APP_BASE"].rstrip("U"), 0)
        descriptor = int(constants["TARGET_DESCRIPTOR_OFFSET"].rstrip("U"), 0)
        struct.pack_into("<I", image, 4, app_base + descriptor + 96 + 1)
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
        return bytes(image)

    def package(self, image: bytes) -> tuple[subprocess.CompletedProcess, Path]:
        """在outputs临时目录调用真实CLI并保留到本用例结束。"""
        (ROOT / "outputs").mkdir(parents=True, exist_ok=True)
        directory = tempfile.TemporaryDirectory(dir=ROOT / "outputs")
        self.addCleanup(directory.cleanup)
        path = Path(directory.name)
        source = path / "app.bin"
        source.write_bytes(image)
        output = path / "package"
        result = subprocess.run(
            [
                sys.executable,
                str(ROOT / "loader/tools/package_image.py"),
                "--app-bin",
                str(source),
                "--token",
                "7",
                "--output",
                str(output),
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        return result, output

    def test_v3_golden_package(self) -> None:
        """冻结 schema12 向量的字节结果，不受当前发布目标升级影响。"""
        vector = json.loads(GOLDEN.read_text(encoding="utf-8"))
        source = bytes.fromhex(vector["source_hex"])
        target = json.loads(
            (ROOT / "shared/targets/vector_mini_st_g431.json").read_text(encoding="utf-8")
        )
        target["constants"].update(
            TARGET_APP_BASE="0x08005000U",
            TARGET_APP_BYTES="0x17000U",
            TARGET_LAYOUT="5U",
            TARGET_APP_VERSION="0x00010300U",
            TARGET_PARAM_ABI="0x000C10D0U",
            TARGET_SCHEMA="12U",
        )
        image, manifest, metadata = build_package(source, 7, target)
        self.assertEqual(image.hex(), vector["image_hex"])
        self.assertEqual(manifest.hex(), vector["manifest_hex"])
        self.assertEqual(metadata["format"], "loader-image-v3-crc32")
        normalized = bytearray(image)
        normalized[560:564] = bytes(4)
        self.assertEqual(zlib.crc32(normalized), struct.unpack_from(">I", image, 560)[0])

    def test_current_target_cli_package(self) -> None:
        """当前版本的命令行包仍能生成，三标记保持擦除态。"""
        result, output = self.package(self.current_source())
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual((output / "image.bin").read_bytes()[576:600], b"\xff" * 24)

    def test_layout6_golden_and_layout5_rejection(self) -> None:
        """迁移后清单和几何固定；旧布局不能伪装成当前升级包。"""
        target = json.loads(
            (ROOT / "shared/targets/vector_mini_st_g431.json").read_text(encoding="utf-8")
        )
        golden = json.loads(
            (ROOT / "tests/fixtures/loader_image_layout6.json").read_text(encoding="utf-8")
        )
        # 黄金值固定其发布版本，当前版本另由CLI用例覆盖。
        source = bytearray(self.current_source())
        struct.pack_into(">I", source, 512 + 28, golden["app_version"])
        target["constants"]["TARGET_APP_VERSION"] = f"0x{golden['app_version']:08X}U"
        image, manifest, metadata = build_package(source, 7, target)
        self.assertEqual(manifest.hex(), golden["manifest_hex"])
        self.assertEqual(metadata["image_crc32"], golden["crc32"])
        self.assertEqual(len(image), golden["image_bytes"])
        self.assertEqual(metadata["layout"], golden["layout"])
        self.assertEqual(
            int(target["constants"]["TARGET_APP_BASE"].rstrip("U"), 0), golden["app_base"]
        )
        old_layout = bytearray(source)
        struct.pack_into(">I", old_layout, 512 + 12, 5)
        with self.assertRaises(ValueError):
            build_package(old_layout, 7, target)

    def test_v3_wire_vectors_match_package(self) -> None:
        """跨端黄金向量必须引用同一清单，nonce以载荷小端传输。"""
        image = json.loads(GOLDEN.read_text(encoding="utf-8"))
        wire = json.loads(
            (ROOT / "docs/protocols/yg_protocol_upgrade_wire_v3_vectors.json").read_text(
                encoding="utf-8"
            )
        )
        nonce = int(wire["nonce_hex"], 16)
        self.assertNotEqual(nonce, 0)
        self.assertEqual(bytes.fromhex(wire["enter_request_hex"]), struct.pack("<Q", nonce))
        self.assertEqual(
            bytes.fromhex(wire["begin_request_hex"]),
            struct.pack("<Q", nonce) + bytes.fromhex(image["manifest_hex"]),
        )
        reply = bytes.fromhex(wire["idle_status_reply_hex"])
        self.assertEqual(len(reply), 40)
        self.assertEqual(reply[17], 0x06)
        self.assertEqual(struct.unpack_from("<Q", reply, 4)[0], 0)
        self.assertEqual(struct.unpack_from("<H", reply, 18)[0], 128)
        self.assertEqual(wire["manifest_fixture"], "tests/fixtures/loader_image_v3.json")
        self.assertEqual(
            bytes.fromhex(wire["confirm_request_hex"]),
            struct.pack("<II", 0x10300, int(image["crc32"], 16)),
        )
        self.assertEqual(bytes.fromhex(wire["confirm_ok_reply_hex"]), bytes(4))

    def test_old_profile_and_maintenance_are_rejected(self) -> None:
        """旧描述符及尚无安全角色交接的维护镜像不能生成部署包。"""
        for offset, value in (
            (516, 1),
            (516, 2),
            (520, 0x53593349),
            (520, 0x53593449),
            (524, 3),
            (524, 4),
            (544, 2),
            (540, 0x10000),
            (540, 0x10001),
            (576, 0),
        ):
            with self.subTest(offset=offset):
                source = bytearray(self.current_source())
                struct.pack_into(">I", source, offset, value)
                result, output = self.package(bytes(source))
                self.assertNotEqual(result.returncode, 0)
                self.assertFalse(output.exists())

    def test_invalid_vector_and_nonzero_placeholders_are_rejected(self) -> None:
        """拒绝越界向量和重复回填已有校验值的输入。"""
        for offset, value, endian in ((0, 0x20007FE8, "<"), (4, 0x0801C001, "<"), (556, 640, ">")):
            with self.subTest(offset=offset):
                source = bytearray(self.current_source())
                struct.pack_into(endian + "I", source, offset, value)
                result, output = self.package(bytes(source))
                self.assertNotEqual(result.returncode, 0)
                self.assertFalse(output.exists())


if __name__ == "__main__":
    unittest.main()
