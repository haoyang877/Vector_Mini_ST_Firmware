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

sys.path.insert(0, str(ROOT / "shared/targets"))
from target_contract import load_target  # noqa: E402


def build_package(source: bytes, token: int, target: dict) -> tuple[bytes, bytes, dict]:
    """按共同目标生成未激活的 CAN 镜像和清单；不访问磁盘或硬件。"""
    if not 1 <= token <= 0xFFFFFFFF:
        raise ValueError("token必须为非零u32，不能复用当前镜像事务的token")
    constants = target["constants"]

    def value(name: str) -> int:
        """读取目标中的无符号数值常量。"""
        return int(constants[name].rstrip("U"), 0)

    image = bytearray(source)
    source_sha256 = hashlib.sha256(bytes(image)).hexdigest()
    offset = value("TARGET_DESCRIPTOR_OFFSET")
    if len(image) < offset + 96 or len(image) > value("TARGET_APP_BYTES"):
        raise ValueError("APP长度不在当前槽范围内；不能截断固件适配容量")
    words = struct.unpack(">10I2HI16s", bytes(image[offset : offset + 64]))
    expected = (
        0x59494431,
        3,
        0x53593549,
        value("TARGET_LAYOUT"),
        value("TARGET_MCU"),
        value("TARGET_PRODUCT"),
        value("TARGET_HARDWARE"),
    )
    if words[:7] != expected or words[9] != value("TARGET_PARAM_ABI"):
        raise ValueError("内嵌描述符目标不匹配；重新用Loader布局工程构建，不在打包时修补")
    role = words[8]
    # 44..47长度与48..63（含CRC占位）在链接后必须保持零，由本工具回填。
    if role != 1 or words[12] != 0 or words[13] != bytes(16):
        raise ValueError("描述符长度/CRC占位非零或角色不合法；v3仅支持普通APP")
    if image[offset + 64 : offset + 96] != b"\xff" * 24 + bytes(8):
        raise ValueError("持久标记不是原始擦除态；禁止重新打包已激活固件")
    if words[7] != value("TARGET_APP_VERSION"):
        raise ValueError("APP版本与冻结目标不一致；重新构建配套APP")
    if not words[10] <= value("TARGET_SCHEMA") <= words[11]:
        raise ValueError("描述符不接受当前参数schema")
    while len(image) % 8 != 0:
        image.append(0xFF)
    image_bytes = len(image)
    if image_bytes > value("TARGET_APP_BYTES"):
        raise ValueError("补齐后APP超过当前槽；不能截断固件适配容量")
    struct.pack_into(">I", image, offset + 44, image_bytes)
    image[offset + 48 : offset + 52] = b"\x00\x00\x00\x00"
    crc = zlib.crc32(bytes(image)) & 0xFFFFFFFF
    struct.pack_into(">I", image, offset + 48, crc)
    sp, entry = struct.unpack_from("<II", image)
    if not (
        value("TARGET_RAM_BASE") < sp <= value("TARGET_RAM_END")
        and sp % 8 == 0
        and entry & 1
        and value("TARGET_APP_BASE") + offset + 96
        <= (entry & ~1)
        < value("TARGET_APP_BASE") + image_bytes
    ):
        raise ValueError("向量表不属于当前APP/RAM槽，或入口落入描述符/补齐区")
    digest = struct.pack(">I", crc) + bytes(28)
    manifest = struct.pack(
        ">11I2H32sI12s",
        0x494D3331,
        0x53593549,
        value("TARGET_LAYOUT"),
        value("TARGET_MCU"),
        value("TARGET_PRODUCT"),
        value("TARGET_HARDWARE"),
        words[7],
        token,
        image_bytes,
        role,
        value("TARGET_PARAM_ABI"),
        words[10],
        words[11],
        digest,
        0,
        bytes(12),
    )
    metadata = {
        "format": "loader-image-v3-crc32",
        "target": target["target"],
        "image_bytes": image_bytes,
        "image_crc32": f"{crc:08x}",
        "source_sha256": source_sha256,
        "manifest_sha256": hashlib.sha256(manifest).hexdigest(),
        "image_sha256": hashlib.sha256(image).hexdigest(),
        "loader_version": value("TARGET_LOADER_VERSION"),
        "layout": value("TARGET_LAYOUT"),
        "parameter_schema": value("TARGET_SCHEMA"),
        "image_version": words[7],
        "image_token": token,
        "image_role": role,
        "authentication": "integrity_only",
        "validation": "not_run",
        "release_approved": False,
    }
    return bytes(image), manifest, metadata


def main() -> None:
    """读取原始APP BIN，生成未激活的 CAN 升级包。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target", help="目标名称或目标 JSON；省略保持 G431")
    parser.add_argument("--app-bin", type=Path, required=True)
    parser.add_argument("--token", type=lambda value: int(value, 0), required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    try:
        target = load_target(args.target)
        image, manifest, metadata = build_package(args.app_bin.read_bytes(), args.token, target)
    except ValueError as exc:
        parser.error(str(exc))
    output = args.output.resolve()
    if not output.is_relative_to((ROOT / "outputs").resolve()):
        parser.error("生成包必须放在本工作树outputs目录")
    output.mkdir(parents=True, exist_ok=False)
    (output / "image.bin").write_bytes(image)
    (output / "manifest.bin").write_bytes(manifest)
    (output / "package.json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
