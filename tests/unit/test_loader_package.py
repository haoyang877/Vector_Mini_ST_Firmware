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
GOLDEN = ROOT / "tests/fixtures/loader_image_v2.json"


class LoaderPackageTests(unittest.TestCase):
    """覆盖发布字节、旧profile与维护角色的实际打包入口。"""

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

    def test_v2_golden_package(self) -> None:
        """比较合成镜像的真实打包结果及独立CRC计算。"""
        vector = json.loads(GOLDEN.read_text(encoding="utf-8"))
        source = bytes.fromhex(vector["source_hex"])
        result, output = self.package(source)
        self.assertEqual(result.returncode, 0, result.stderr)
        image = (output / "image.bin").read_bytes()
        self.assertEqual(image.hex(), vector["image_hex"])
        self.assertEqual((output / "manifest.bin").read_bytes().hex(), vector["manifest_hex"])
        metadata = json.loads((output / "package.json").read_text(encoding="utf-8"))
        self.assertEqual(metadata["format"], "loader-image-v2-crc32")
        normalized = bytearray(image)
        normalized[560:564] = bytes(4)
        self.assertEqual(zlib.crc32(normalized), struct.unpack_from(">I", image, 560)[0])

    def test_v2_wire_vectors_match_package(self) -> None:
        """跨端黄金向量必须引用同一清单，nonce以载荷小端传输。"""
        image = json.loads(GOLDEN.read_text(encoding="utf-8"))
        wire = json.loads(
            (ROOT / "docs/protocols/yg_protocol_upgrade_wire_v2_vectors.json").read_text(
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

    def test_old_profile_and_maintenance_are_rejected(self) -> None:
        """旧描述符及尚无安全角色交接的维护镜像不能生成部署包。"""
        vector = json.loads(GOLDEN.read_text(encoding="utf-8"))
        for offset, value in ((516, 1), (520, 0x53593349), (524, 3), (544, 2)):
            with self.subTest(offset=offset):
                source = bytearray.fromhex(vector["source_hex"])
                struct.pack_into(">I", source, offset, value)
                result, output = self.package(bytes(source))
                self.assertNotEqual(result.returncode, 0)
                self.assertFalse(output.exists())

    def test_invalid_vector_and_nonzero_placeholders_are_rejected(self) -> None:
        """拒绝越界向量和重复回填已有校验值的输入。"""
        vector = json.loads(GOLDEN.read_text(encoding="utf-8"))
        for offset, value, endian in ((0, 0x20007FE8, "<"), (4, 0x08008841, "<"), (556, 640, ">")):
            with self.subTest(offset=offset):
                source = bytearray.fromhex(vector["source_hex"])
                struct.pack_into(endian + "I", source, offset, value)
                result, output = self.package(bytes(source))
                self.assertNotEqual(result.returncode, 0)
                self.assertFalse(output.exists())


if __name__ == "__main__":
    unittest.main()
