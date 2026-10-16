"""离线生成保留设备配置、清除角度标定的schema13参数页；不访问硬件。"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import struct
import zlib
from pathlib import Path

PARAM_BYTES = 0x4000
RECORD_BYTES = 4352
IDENTITY_OFFSET = 0x3800
THERMAL_DEFAULT = (3.0, 7.0, 65.98486785, 0.8, 0.7, 1.0, 0.5)
FEEDFORWARD_DEFAULT = (0, 0.028, 20.0, 200.0, 2.0)


def float32(value: float) -> float:
    """边界与C的float常量使用同一精度，避免拒绝合法的0.2f等端点。"""
    return struct.unpack("<f", struct.pack("<f", value))[0]


def record_crc(record: bytes, size: int) -> int:
    """按已发布规则跳过CRC槽，拒绝不完整输入。"""
    if len(record) < size or size not in (4304, RECORD_BYTES):
        raise ValueError("参数记录长度无效")
    return zlib.crc32(record[2144:size], zlib.crc32(record[:2140]))


def legacy_extension(raw: bytes, offset: int, magic: bytes, count: int) -> bytes | None:
    """仅接受带版本及CRC的旧交付扩展；缺省槽允许默认，损坏槽拒绝。"""
    block = raw[offset : offset + count]
    if block in (bytes(count), b"\xff" * count):
        return None
    if (
        block[:4] != magic
        or struct.unpack_from("<I", block, 4)[0] != 1
        or zlib.crc32(block[:-4]) != struct.unpack_from("<I", block, count - 4)[0]
    ):
        raise ValueError("旧交付扩展版本或CRC无效，不能静默丢弃配置")
    return block[8:-4]


def prepare(raw: bytes, source_sha256: str, node: int | None = None) -> tuple[bytes, dict]:
    """只变更原始参数记录；返回完整16KiB页镜像和明确变更清单。"""
    if len(raw) != PARAM_BYTES or hashlib.sha256(raw).hexdigest() != source_sha256.lower():
        raise ValueError("必须提供匹配SHA256的完整16KiB参数快照")
    schema, magic = struct.unpack_from("<II", raw, 2160)
    if schema not in (10, 12, 13) or magic != 0x454E4332:
        raise ValueError("仅支持已盘点的交付schema10及当前schema12/13")
    if schema in (12, 13):
        size = 4304 if schema == 12 else RECORD_BYTES
        if record_crc(raw, size) != struct.unpack_from("<I", raw, 2140)[0]:
            raise ValueError("源参数整记录CRC无效")
    record = bytearray(RECORD_BYTES)
    if schema == 10:
        # 只取双方语义一致的字段；旧位置/摩擦扩展不覆盖退役槽或新CRC。
        for start, end in ((0, 2136), (2144, 2156), (2172, 2176), (2180, 2188), (2208, 2240)):
            record[start:end] = raw[start:end]
    else:
        record[:4304] = raw[:4304]
        record[2136:2140] = bytes(4)
    thermal = struct.pack("<7f", *THERMAL_DEFAULT)
    feedforward = struct.pack("<I4f", *FEEDFORWARD_DEFAULT)
    if schema == 10:
        thermal = legacy_extension(raw, 2344, b"MTH1", 40) or thermal
        feedforward = legacy_extension(raw, 2312, b"SFF1", 32) or feedforward
    elif schema == 13:
        thermal, feedforward = raw[4304:4332], raw[4332:4352]
    continuous, peak, tau, enter, leave, trip, clear = struct.unpack("<7f", thermal)
    enabled, gain, accel, jerk, ff_limit = struct.unpack("<I4f", feedforward)
    if not (
        all(math.isfinite(x) for x in (continuous, peak, tau, enter, leave, trip, clear))
        and float32(0.01) <= continuous <= peak <= 100.0
        and float32(0.1) <= tau <= 86400.0
        and 0 <= clear < leave < enter < trip <= 1
        and enabled in (0, 1)
        and 0 <= gain <= float32(0.2)
        and float32(0.1) <= accel <= 100
        and 1 <= jerk <= 2000
        and 0 <= ff_limit <= 10
    ):
        raise ValueError("热配置或速度前馈配置不合法")
    actual_node = struct.unpack_from("<f", record)[0]
    if node is not None:
        actual_node = node
        struct.pack_into("<f", record, 0, node)
    if (
        not math.isfinite(actual_node)
        or not 1 <= actual_node <= 254
        or int(actual_node) != actual_node
    ):
        raise ValueError("旧节点不兼容；必须明确指定1..254的新节点")
    # 零位、资格及LUT全部清除；电流零点由APP每次上电重新采样，不恢复旧资格。
    record[36:41] = bytes(5)
    record[44:2092] = bytes(2048)
    record[2136:2144] = bytes(8)  # 清配置确认，测试前显式核对并确认。
    struct.pack_into("<II", record, 2160, 13, 0x454E4332)
    record[4304:4332], record[4332:4352] = thermal, feedforward
    struct.pack_into("<I", record, 2140, record_crc(record, RECORD_BYTES))
    result = bytes(record) + raw[RECORD_BYTES:]
    if result[IDENTITY_OFFSET:] != raw[IDENTITY_OFFSET:]:
        raise ValueError("身份页发生变更")
    report = {
        "source_schema": schema,
        "source_sha256": source_sha256.lower(),
        "output_schema": 13,
        "output_sha256": hashlib.sha256(result).hexdigest(),
        "record_bytes": RECORD_BYTES,
        "record_crc32": f"0x{record_crc(record, RECORD_BYTES):08x}",
        "node": int(actual_node),
        "configuration_confirmed": False,
        "cleared": ["encoder_zero", "encoder_lut", "encoder_calibration_flags"],
        "identity_page_preserved": True,
        "source_base_has_crc": schema != 10,
        "hardware_actions": False,
    }
    return result, report


def main() -> None:
    """只在显式输入与新输出路径下生成；不连接探针或CAN。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--source-sha256", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--node", type=int)
    args = parser.parse_args()
    if args.output.exists() or args.output.with_suffix(".json").exists():
        parser.error("输出必须是新文件，禁止覆盖原始快照")
    output, report = prepare(args.input.read_bytes(), args.source_sha256, args.node)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_bytes(output)
    args.output.with_suffix(".json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, ensure_ascii=True))


if __name__ == "__main__":
    main()
