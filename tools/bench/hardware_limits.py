"""外置检测阈值：严格校验、固定点编码和每轮可追溯快照。"""

import hashlib
import json
import math
import struct
import sys
from pathlib import Path

CONTRACT_PATH = (
    Path(sys._MEIPASS) / "hardware_check_limits_v1.json"
    if getattr(sys, "frozen", False)
    else Path(__file__).resolve().parents[2] / "shared/protocol/hardware_check_limits_v1.json"
)
FIELDS = json.loads(CONTRACT_PATH.read_text(encoding="utf-8"))["fields"]
CONVERSIONS = {
    "bus_min_mv": ("bus_min_v", 1000, 0),
    "bus_max_mv": ("bus_max_v", 1000, 0),
    "temperature_min_offset_deci_c": ("temperature_min_c", 10, 1000),
    "temperature_max_offset_deci_c": ("temperature_max_c", 10, 1000),
    "rotor_static_max_counts": ("rotor_static_max_deg", 2097152 / 360, 0),
    "motor_trend_per_3000": ("motor_trend_min_ratio", 3000, 0),
    "hall_trend_permille": ("hall_trend_min_ratio", 1000, 0),
    "resistance_min_permille": ("resistance_min_ratio", 1000, 0),
    "resistance_max_permille": ("resistance_max_ratio", 1000, 0),
}


def default_path() -> Path:
    """独立程序读取EXE旁可编辑文件，源码运行读取版本库默认文件。"""
    return (
        Path(sys.executable).parent / "hardware_check_limits.json"
        if getattr(sys, "frozen", False)
        else Path(__file__).with_name("hardware_check_limits.json")
    )


def read_limits(path: Path | None = None) -> dict:
    """拒绝缺项、未知项和越界，生成与实际线路量化一致的记录。"""
    path = path or default_path()
    try:
        raw = path.read_bytes()
    except OSError as exc:
        raise ValueError(f"无法读取检测阈值配置 {path.name}：{exc}") from exc
    config = json.loads(raw.decode("utf-8-sig"))
    if (
        not isinstance(config, dict)
        or set(config) != {"schema_version", "revision", "limits"}
        or type(config["schema_version"]) is not int
        or config["schema_version"] != 1
    ):
        raise ValueError("检测配置格式错误或schema_version不支持")
    revision = config["revision"]
    if type(revision) is not int or not 1 <= revision <= 0xFFFFFFFF:
        raise ValueError("检测配置revision须为非零u32整数")
    values = config["limits"]
    expected = {CONVERSIONS.get(name, (name, 1, 0))[0] for name, *_ in FIELDS}
    if not isinstance(values, dict) or set(values) != expected:
        raise ValueError("检测配置limits字段缺失或含未知项")
    wire, effective = [], {}
    for name, _, minimum, maximum in FIELDS:
        key, scale, offset = CONVERSIONS.get(name, (name, 1, 0))
        value = values[key]
        if type(value) not in (int, float) or not math.isfinite(value):
            raise ValueError(f"检测阈值{key}须为有限数值")
        if not (minimum - offset) / scale <= value <= (maximum - offset) / scale:
            raise ValueError(f"检测阈值{key}超出允许范围")
        encoded = round(value * scale + offset)
        if not minimum <= encoded <= maximum:
            raise ValueError(f"检测阈值{key}超出允许范围")
        if key not in {entry[0] for entry in CONVERSIONS.values()} and value != encoded:
            raise ValueError(f"检测阈值{key}须为整数")
        wire.append(encoded)
        effective[key] = (encoded - offset) / scale
    for low, high in (
        ("bus_min_v", "bus_max_v"),
        ("temperature_min_c", "temperature_max_c"),
        ("hall_raw_min", "hall_raw_max"),
        ("resistance_min_ratio", "resistance_max_ratio"),
    ):
        if effective[low] >= effective[high]:
            raise ValueError(f"检测阈值{low}须小于{high}")
    return dict(
        schema_version=1,
        revision=revision,
        limits=effective,
        wire_values=wire,
        sha256=hashlib.sha256(raw).hexdigest(),
        filename=path.name,
        requested_limits=values,
    )


def start_payload(config: dict, mode: int, previous_run: int) -> bytes:
    """整份阈值与检测启动原子提交，44字节可装入单帧。"""
    return struct.pack(
        "<4BII16H", 2, 1, mode, 1, previous_run, config["revision"], *config["wire_values"]
    )


def confirm_start(data: bytes, config: dict, previous_run: int) -> int:
    """核对MCU实际受理版本、全部阈值及轮号。"""
    expected = struct.pack("<I16H", config["revision"], *config["wire_values"])
    if len(data) != 44 or data[:4] != b"\x01\0\0\0" or data[4:40] != expected:
        raise ValueError("MCU未确认本轮完整检测配置")
    run = struct.unpack_from("<I", data, 40)[0]
    if run != previous_run + 1:
        raise ValueError("MCU检测配置轮号不匹配")
    return run
