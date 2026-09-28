"""离线校验已链接APP并按描述符声明长度/CRC32生成96B升级清单；不改描述符身份、不连接硬件。"""

from __future__ import annotations

import argparse
import hashlib
import json
import struct
import sys
import zlib
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from project_paths import ROOT


def main() -> None:
    """读取原始APP BIN，补齐8B尾部，回填长度与整包CRC32并生成清单。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--app-bin", type=Path, required=True)
    parser.add_argument("--token", type=lambda value: int(value, 0), required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not 1 <= args.token <= 0xFFFFFFFF:
        parser.error("token必须为非零u32，不能复用当前镜像事务的token")
    target = json.loads(
        (ROOT / "shared/targets/vector_mini_st_g431.json").read_text(encoding="utf-8")
    )
    constants = target["constants"]

    def value(name: str) -> int:
        """读取目标中的无符号数值常量。"""
        return int(constants[name].rstrip("U"), 0)

    image = bytearray(args.app_bin.read_bytes())
    source_sha256 = hashlib.sha256(bytes(image)).hexdigest()
    offset = value("TARGET_DESCRIPTOR_OFFSET")
    if len(image) < offset + 64 or len(image) > value("TARGET_APP_BYTES"):
        parser.error("APP长度不在当前槽范围内；不能截断固件适配容量")
    words = struct.unpack(">10I2HI16s", bytes(image[offset : offset + 64]))
    expected = (
        0x59494431,
        2,
        0x53593449,
        value("TARGET_LAYOUT"),
        value("TARGET_MCU"),
        value("TARGET_PRODUCT"),
        value("TARGET_HARDWARE"),
    )
    if words[:7] != expected or words[9] != value("TARGET_PARAM_ABI"):
        parser.error("内嵌描述符目标不匹配；重新用Loader布局工程构建，不在打包时修补")
    role = words[8]
    # 44..47长度与48..63（含CRC占位）在链接后必须保持零，由本工具回填。
    if role != 1 or words[12] != 0 or words[13] != bytes(16):
        parser.error("描述符长度/CRC占位非零或角色不合法；v2仅支持普通APP")
    if not words[10] <= value("TARGET_SCHEMA") <= words[11]:
        parser.error("描述符不接受当前参数schema")
    while len(image) % 8 != 0:
        image.append(0xFF)
    image_bytes = len(image)
    if image_bytes > value("TARGET_APP_BYTES"):
        parser.error("补齐后APP超过当前槽；不能截断固件适配容量")
    struct.pack_into(">I", image, offset + 44, image_bytes)
    image[offset + 48 : offset + 52] = b"\x00\x00\x00\x00"
    crc = zlib.crc32(bytes(image)) & 0xFFFFFFFF
    struct.pack_into(">I", image, offset + 48, crc)
    sp, entry = struct.unpack_from("<II", image)
    if not (
        value("TARGET_RAM_BASE") < sp <= value("TARGET_RAM_END")
        and sp % 8 == 0
        and entry & 1
        and value("TARGET_APP_BASE") + offset + 64
        <= (entry & ~1)
        < value("TARGET_APP_BASE") + image_bytes
    ):
        parser.error("向量表不属于当前APP/RAM槽，或入口落入描述符/补齐区")
    digest = struct.pack(">I", crc) + bytes(28)
    manifest = struct.pack(
        ">11I2H32sI12s",
        0x494D3331,
        0x53593449,
        value("TARGET_LAYOUT"),
        value("TARGET_MCU"),
        value("TARGET_PRODUCT"),
        value("TARGET_HARDWARE"),
        words[7],
        args.token,
        image_bytes,
        role,
        value("TARGET_PARAM_ABI"),
        words[10],
        words[11],
        digest,
        0,
        bytes(12),
    )
    output = args.output.resolve()
    if not output.is_relative_to((ROOT / "outputs").resolve()):
        parser.error("生成包必须放在本工作树outputs目录")
    output.mkdir(parents=True, exist_ok=False)
    (output / "image.bin").write_bytes(bytes(image))
    (output / "manifest.bin").write_bytes(manifest)
    metadata = {
        "format": "loader-image-v2-crc32",
        "target": target["target"],
        "image_bytes": image_bytes,
        "image_crc32": f"{crc:08x}",
        "source_sha256": source_sha256,
        "manifest_sha256": hashlib.sha256(manifest).hexdigest(),
        "image_version": words[7],
        "image_token": args.token,
        "image_role": role,
        "authentication": "integrity_only",
        "validation": "not_run",
        "release_approved": False,
    }
    (output / "package.json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
