"""离线填充 Keil AXF 的 APP 有效记录；只接受固定布局，不打开任何硬件。"""

import argparse
import hashlib
import json
from pathlib import Path
import struct
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "loader/tools"))
from loader_factory import APP_BASE, CAPACITY, RECORD_BASE, boot_record, combined_hex  # noqa: E402
from loader_updater import validate_image  # noqa: E402


def image_parts(raw):
    """解析 ARM ELF32 little-endian，返回记录文件偏移、完整 APP 与镜像内版本。"""
    if len(raw) < 52 or len(raw) > 16 * 1024 * 1024:
        raise ValueError("AXF 长度无效")
    header = struct.unpack_from("<16sHHIIIIIHHHHHH", raw)
    if raw[:7] != b"\x7fELF\x01\x01\x01" or header[1:4] != (2, 40, 1):
        raise ValueError("仅支持 ARM ELF32 little-endian 可执行文件")
    phoff, shoff = header[5:7]
    if header[9] != 32 or header[11] != 40 or not 0 < header[10] <= 16:
        raise ValueError("AXF 表结构无效")
    if phoff + header[10] * 32 > len(raw) or shoff + header[12] * 40 > len(raw):
        raise ValueError("AXF 表越界")
    loads = {}
    for index in range(header[10]):
        kind, offset, _, address, size, _, _, _ = struct.unpack_from(
            "<8I", raw, phoff + index * 32
        )
        if kind != 1 or size == 0:
            continue
        expected = {RECORD_BASE: 48, APP_BASE: CAPACITY}
        if address not in expected or size != expected[address] or address in loads:
            raise ValueError("加载段必须仅覆盖 48 B 记录和完整 APP 分区")
        if offset < 52 or offset + size > len(raw):
            raise ValueError("加载段文件范围无效")
        for table_start, table_size in (
            (phoff, header[10] * 32),
            (shoff, header[12] * 40),
        ):
            if max(offset, table_start) < min(offset + size, table_start + table_size):
                raise ValueError("加载段与 ELF 表重叠")
        loads[address] = (offset, size)
    if set(loads) != {RECORD_BASE, APP_BASE}:
        raise ValueError("缺少 APP 或记录加载段")
    record_offset, _ = loads[RECORD_BASE]
    app_offset, size = loads[APP_BASE]
    if max(record_offset, app_offset) < min(record_offset + 48, app_offset + size):
        raise ValueError("记录与 APP 文件范围重叠")
    app = bytes(raw[app_offset : app_offset + size])
    validate_image(app)
    sections = [
        struct.unpack_from("<10I", raw, shoff + index * 40)
        for index in range(header[12])
    ]
    versions = []
    for section in sections:
        _, kind, _, _, offset, size, link, _, _, entry_size = section
        if kind != 2:
            continue
        if (
            entry_size != 16
            or size % 16
            or link >= len(sections)
            or offset + size > len(raw)
        ):
            raise ValueError("符号表无效")
        strings = sections[link]
        if strings[1] != 3 or strings[4] + strings[5] > len(raw):
            raise ValueError("符号字符串表无效")
        names = raw[strings[4] : strings[4] + strings[5]]
        for position in range(offset, offset + size, 16):
            name, address, symbol_size, _, _, section_id = struct.unpack_from(
                "<IIIBBH", raw, position
            )
            if name >= len(names):
                raise ValueError("符号名称越界")
            if names[name:].split(b"\0", 1)[0] != b"g_firmware_software_version":
                continue
            if not 0 < section_id < len(sections) or symbol_size != 4:
                raise ValueError("版本符号无效")
            if not APP_BASE <= address <= APP_BASE + CAPACITY - 4:
                raise ValueError("版本符号不在 APP 内")
            versions.append(struct.unpack_from("<I", app, address - APP_BASE)[0])
    if len(versions) != 1:
        raise ValueError("必须有唯一的 APP 版本符号")
    return record_offset, app, versions[0]


def synchronize(axf):
    """只替换 AXF 的 48 B 记录，输出同名维护 HEX、纯 APP BIN 和哈希清单。"""
    raw = bytearray(axf.read_bytes())
    offset, app, version = image_parts(raw)
    record = boot_record(app, version)
    raw[offset : offset + 48] = record
    hex_data = combined_hex(((RECORD_BASE, record), (APP_BASE, app))).encode("ascii")
    outputs = {
        axf: bytes(raw),
        axf.with_suffix(".hex"): hex_data,
        axf.with_suffix(".bin"): app,
    }
    manifest = {
        "app_base": APP_BASE,
        "image_bytes": len(app),
        "image_crc32": struct.unpack_from("<I", record, 16)[0],
        "app_version": version,
        "record_base": RECORD_BASE,
        "record_hex": record.hex(),
        "artifacts": {
            path.name: {"bytes": len(data), "sha256": hashlib.sha256(data).hexdigest()}
            for path, data in outputs.items()
        },
    }
    # 先验证全部输入；临时文件与目标同目录，替换时不改变 ELF 任何地址或调试信息。
    for path, data in outputs.items():
        temporary = path.with_suffix(path.suffix + ".record.tmp")
        temporary.write_bytes(data)
        temporary.replace(path)
    axf.with_suffix(".boot.json").write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8"
    )
    return manifest


def main():
    """Keil After Build 入口；失败返回非零，使构建停止。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("axf", type=Path)
    args = parser.parse_args()
    try:
        result = synchronize(args.axf.resolve())
    except (OSError, ValueError, struct.error) as exc:
        print(f"APP record sync failed: {exc}", file=sys.stderr)
        return 1
    print(
        f"APP record synchronized: {result['image_bytes']} B, "
        f"CRC={result['image_crc32']:08X}, version={result['app_version']:08X}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
