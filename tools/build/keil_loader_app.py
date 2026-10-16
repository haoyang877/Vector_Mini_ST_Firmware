"""同步 LoaderApp 的 Keil 调试镜像与未激活 CAN 包；只处理本地构建文件。"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import secrets
import struct
import sys
import zlib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "loader/tools"))
from package_image import build_package  # noqa: E402

MARKERS = ((64, 0x41435433), (72, 0x54525933), (80, 0x4F4B4133))


def image_segment(raw: bytes, app_base: int, app_bytes: int) -> tuple[int, bytes]:
    """只接受 APP 槽的单一 ARM ELF32 Flash 加载段，返回文件偏移和原始镜像。"""
    if len(raw) < 52 or len(raw) > 64 * 1024 * 1024:
        raise ValueError("AXF 长度无效")
    header = struct.unpack_from("<16sHHIIIIIHHHHHH", raw)
    if raw[:7] != b"\x7fELF\x01\x01\x01" or header[1:4] != (2, 40, 1):
        raise ValueError("仅支持 ARM ELF32 little-endian 可执行文件")
    phoff, shoff = header[5:7]
    if (
        header[9] != 32
        or header[11] != 40
        or not 1 <= header[10] <= 16
        or phoff + header[10] * 32 > len(raw)
        or shoff + header[12] * 40 > len(raw)
    ):
        raise ValueError("AXF 表结构无效")
    loads = []
    for index in range(header[10]):
        kind, offset, virtual, address, size, memory_size, _, _ = struct.unpack_from(
            "<8I", raw, phoff + index * 32
        )
        if kind != 1 or size == 0:
            continue
        if (
            virtual != app_base
            or address != app_base
            or not 1 <= size <= app_bytes
            or memory_size < size
            or offset < 52
            or offset + size > len(raw)
        ):
            raise ValueError("AXF 加载段不属于当前 APP 槽")
        for start, length in ((phoff, header[10] * 32), (shoff, header[12] * 40)):
            if max(offset, start) < min(offset + size, start + length):
                raise ValueError("AXF 加载段与 ELF 表重叠")
        loads.append((offset, raw[offset : offset + size]))
    if len(loads) != 1:
        raise ValueError("AXF 必须只有一个 APP Flash 加载段")
    return loads[0]


def intel_hex(address: int, data: bytes) -> bytes:
    """把已校验加载段编码为单一地址范围的 Intel HEX。"""
    lines = []
    upper = -1

    def record(offset: int, kind: int, payload: bytes) -> str:
        body = bytes((len(payload), offset >> 8, offset & 255, kind)) + payload
        return ":" + (body + bytes((-sum(body) & 255,))).hex().upper()

    for offset in range(0, len(data), 16):
        current = address + offset
        if current >> 16 != upper:
            upper = current >> 16
            lines.append(record(0, 4, struct.pack(">H", upper)))
        lines.append(record(current & 0xFFFF, 0, data[offset : offset + 16]))
    lines.append(":00000001FF")
    return ("\n".join(lines) + "\n").encode("ascii")


def invalidate_outputs(axf: Path) -> None:
    """失败时撤销旧成功清单和派生产物；保留 AXF 供诊断或重新链接。"""
    for path in (
        axf.with_suffix(".debug.json"),
        axf.with_suffix(".hex"),
        axf.with_suffix(".bin"),
        axf.parent / "app-package/image.bin",
        axf.parent / "app-package/manifest.bin",
        axf.parent / "app-package/package.json",
    ):
        path.unlink(missing_ok=True)


def reuse_existing(
    axf: Path,
    raw: bytes,
    target: dict,
    token: int | None,
    app_base: int,
    app_bytes: int,
    descriptor: int,
) -> dict:
    """仅复用同目标、同字节且全部派生产物完好的既有调试镜像。"""
    report_path = axf.with_suffix(".debug.json")
    if not report_path.is_file():
        raise ValueError("AXF 已处理或占位无效，但缺少可复用清单")
    report = json.loads(report_path.read_text(encoding="utf-8"))
    expected = {
        axf.name,
        axf.with_suffix(".hex").name,
        axf.with_suffix(".bin").name,
        "app-package/image.bin",
        "app-package/manifest.bin",
        "app-package/package.json",
    }
    if (
        report.get("target_sha256")
        != hashlib.sha256(json.dumps(target, sort_keys=True).encode("utf-8")).hexdigest()
        or report.get("app_base") != app_base
        or report.get("debug_markers") != "activated_attempted_confirmed"
        or report.get("can_markers") != "erased"
        or set(report.get("artifacts", {})) != expected
        or (token is not None and report.get("image_token") != token)
    ):
        raise ValueError("现有调试清单与当前目标或事务不匹配")
    for relative, metadata in report["artifacts"].items():
        path = axf.parent / relative
        if (
            not path.is_file()
            or path.stat().st_size != metadata.get("bytes")
            or hashlib.sha256(path.read_bytes()).hexdigest() != metadata.get("sha256")
        ):
            raise ValueError(f"现有调试产物损坏：{relative}")
    if hashlib.sha256(raw).hexdigest() != report["artifacts"][axf.name]["sha256"]:
        raise ValueError("AXF 与现有调试清单不一致")
    _, debug = image_segment(raw, app_base, app_bytes)
    image = (axf.parent / "app-package/image.bin").read_bytes()
    normalized = bytearray(debug)
    for offset, magic in MARKERS:
        if struct.unpack_from(">II", debug, descriptor + offset) != (magic, (~magic) & 0xFFFFFFFF):
            raise ValueError("现有 AXF 启动标记无效")
        normalized[descriptor + offset : descriptor + offset + 8] = b"\xff" * 8
    if bytes(normalized) != image or debug != axf.with_suffix(".bin").read_bytes():
        raise ValueError("现有调试镜像与 CAN 包不一致")
    if report.get("image_bytes") != len(image):
        raise ValueError("现有调试镜像长度与清单不一致")
    return report


def build_outputs(axf: Path, token: int | None) -> dict:
    """从同一 AXF 派生调试下载镜像和未激活 CAN 包。"""
    axf = axf.resolve()
    if token is not None and not 1 <= token <= 0xFFFFFFFF:
        raise ValueError("token 必须是非零 u32")
    target = json.loads(
        (ROOT / "shared/targets/vector_mini_st_g431.json").read_text(encoding="utf-8")
    )
    constants = target["constants"]
    app_base = int(constants["TARGET_APP_BASE"].rstrip("U"), 0)
    app_bytes = int(constants["TARGET_APP_BYTES"].rstrip("U"), 0)
    descriptor = int(constants["TARGET_DESCRIPTOR_OFFSET"].rstrip("U"), 0)
    raw = bytearray(axf.read_bytes())
    file_offset, source = image_segment(raw, app_base, app_bytes)
    try:
        image, manifest, package = build_package(
            source, token if token is not None else secrets.randbelow(0xFFFFFFFF) + 1, target
        )
    except ValueError:
        return reuse_existing(axf, raw, target, token, app_base, app_bytes, descriptor)
    if len(image) != len(source):
        raise ValueError("AXF 加载段未按 8 B 对齐；需调整链接输入，不能只补外部 BIN")
    debug = bytearray(image)
    for offset, magic in MARKERS:
        struct.pack_into(">II", debug, descriptor + offset, magic, (~magic) & 0xFFFFFFFF)
    normalized = bytearray(debug)
    normalized[descriptor + 48 : descriptor + 52] = bytes(4)
    for offset, _ in MARKERS:
        normalized[descriptor + offset : descriptor + offset + 8] = b"\xff" * 8
    if zlib.crc32(normalized) != int(package["image_crc32"], 16):
        raise ValueError("调试镜像归一化 CRC 与 CAN 包不一致")
    raw[file_offset : file_offset + len(debug)] = debug
    hex_data = intel_hex(app_base, debug)
    output = {
        axf: bytes(raw),
        axf.with_suffix(".hex"): hex_data,
        axf.with_suffix(".bin"): bytes(debug),
        axf.parent / "app-package/image.bin": image,
        axf.parent / "app-package/manifest.bin": manifest,
        axf.parent / "app-package/package.json": (json.dumps(package, indent=2) + "\n").encode(
            "utf-8"
        ),
    }
    report = {
        "target_sha256": hashlib.sha256(
            json.dumps(target, sort_keys=True).encode("utf-8")
        ).hexdigest(),
        "app_base": app_base,
        "image_bytes": len(image),
        "image_crc32": package["image_crc32"],
        "image_version": package["image_version"],
        "image_token": package["image_token"],
        "layout": package["layout"],
        "parameter_schema": package["parameter_schema"],
        "debug_markers": "activated_attempted_confirmed",
        "can_markers": "erased",
        "release_approved": False,
        "hardware_validation": "not_run",
        "artifacts": {
            path.relative_to(axf.parent).as_posix(): {
                "bytes": len(data),
                "sha256": hashlib.sha256(data).hexdigest(),
            }
            for path, data in output.items()
        },
    }
    # 所有输入通过后才写临时文件；Keil 的非零返回阻止下载本次失败构建。
    for path, data in output.items():
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_name(path.name + ".sync.tmp")
        temporary.write_bytes(data)
    for path in output:
        path.with_name(path.name + ".sync.tmp").replace(path)
    report_path = axf.with_suffix(".debug.json")
    report_tmp = report_path.with_name(report_path.name + ".sync.tmp")
    report_tmp.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    report_tmp.replace(report_path)
    return report


def synchronize(axf: Path, token: int | None) -> dict:
    """处理或严格复用 APP 镜像；任何失败撤销旧成功清单。"""
    axf = axf.resolve()
    if not axf.is_relative_to((ROOT / "outputs").resolve()) or not axf.is_file():
        raise ValueError("AXF 必须是当前工作树 outputs 内的构建文件")
    try:
        return build_outputs(axf, token)
    except (OSError, ValueError, struct.error, KeyError, TypeError):
        invalidate_outputs(axf)
        raise


def main() -> int:
    """Keil After Build 入口；任何无效镜像返回非零。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("axf", type=Path)
    parser.add_argument("--token", type=lambda value: int(value, 0))
    args = parser.parse_args()
    try:
        configured = os.environ.get("YG_LOADER_IMAGE_TOKEN")
        token = args.token if args.token is not None else int(configured, 0) if configured else None
        report = synchronize(args.axf, token)
    except (OSError, ValueError, struct.error, KeyError) as exc:
        print(f"LoaderApp sync failed: {exc}", file=sys.stderr)
        return 1
    print(
        f"LoaderApp synchronized: {report['image_bytes']} B, "
        f"CRC={report['image_crc32']}, token={report['image_token']:08X}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
