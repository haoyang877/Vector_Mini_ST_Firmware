"""离线生成首次供应的boot双页；显式UID/floor/备份哈希，拒绝覆盖非空启动历史。"""

from __future__ import annotations

import argparse
import hashlib
import json
import struct
import zlib
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from project_paths import ROOT


def main() -> None:
    """只生成待审核的数据文件，不擦写设备、不从旧备份推断高水位。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--flash-backup", type=Path, required=True)
    parser.add_argument("--expected-sha256", required=True)
    parser.add_argument("--uid", required=True)
    parser.add_argument("--trusted-floor", type=lambda v: int(v, 0), required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    config = json.loads(
        (ROOT / "shared/targets/vector_mini_st_g431.json").read_text(encoding="utf-8")
    )
    c = config["constants"]
    flash = args.flash_backup.read_bytes()
    source_sha = hashlib.sha256(flash).hexdigest()
    if (
        len(flash) != int(c["TARGET_FLASH_BYTES"].rstrip("U"), 0)
        or source_sha != args.expected_sha256.lower()
    ):
        parser.error("完整Flash备份长度/指定SHA不一致")
    try:
        uid = bytes.fromhex(args.uid)
    except ValueError:
        parser.error("UID必须为24位十六进制")
    if len(uid) != 12 or uid in (bytes(12), bytes([255]) * 12):
        parser.error("UID必须是明确设备的96位非空值，不是电机SN")
    if not 0 <= args.trusted_floor < 0xFFFFFFFFFFFFFFFF:
        parser.error("可信floor需为未耗尽u64，不允许猜测或回退历史")
    base = int(c["TARGET_BOOT_BASE"].rstrip("U"), 0)
    offset = base - int(c["TARGET_FLASH_BASE"].rstrip("U"), 0)
    page_bytes = int(c["TARGET_ERASE_BYTES"].rstrip("U"), 0)
    if flash[offset : offset + page_bytes * 2] != bytes([255]) * (page_bytes * 2):
        parser.error("boot双页非全FF：禁止覆盖；首次旧独立APP转换需单独审核部署流程")
    header = bytearray(64)
    struct.pack_into(
        ">IHH12s4xQQ",
        header,
        0,
        int(c["TARGET_BOOT_MAGIC"].rstrip("U"), 0),
        int(c["TARGET_BOOT_VERSION"].rstrip("U"), 0),
        64,
        uid,
        1,
        args.trusted_floor,
    )
    struct.pack_into(">I", header, 40, zlib.crc32(header[:40] + header[44:56]))
    commit = int(c["TARGET_BOOT_HEADER_COMMIT"].removeprefix("UINT64_C(").removesuffix(")"), 0)
    struct.pack_into(">Q", header, 56, commit)
    blob = (header + bytes([255]) * (page_bytes - 64)) * 2
    output = args.output.resolve()
    if not output.is_relative_to((ROOT / "outputs").resolve()):
        parser.error("生成供应数据必须放在本工作树outputs目录")
    output.mkdir(parents=True, exist_ok=False)
    (output / "boot-seed.bin").write_bytes(blob)
    metadata = {
        "format": "boot-seed-v1",
        "address": hex(base),
        "uid": uid.hex(),
        "trusted_floor": args.trusted_floor,
        "source_sha256": source_sha,
        "seed_sha256": hashlib.sha256(blob).hexdigest(),
        "hardware_written": False,
        "validation": "not_run",
    }
    (output / "boot-seed.json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
