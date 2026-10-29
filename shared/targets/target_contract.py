"""Loader 构建与主机共用的目标契约读取；不探测设备、不隐式切换目标。"""

from __future__ import annotations

import json
import re
from pathlib import Path

DEFAULT_TARGET = "vector_mini_st_g431"


def target_value(target: dict, name: str) -> int:
    """解析无符号常量或单一常量引用，不执行表达式。"""
    visited = set()
    while name not in visited:
        visited.add(name)
        value = target["constants"][name]
        if isinstance(value, str) and re.fullmatch(r"TARGET_[A-Z0-9_]+", value):
            name = value
            continue
        if not isinstance(value, str) or not re.fullmatch(r"(?:0x[0-9A-Fa-f]+|[0-9]+)U?", value):
            raise ValueError(f"目标常量格式非法：{name}")
        number = int(value.removesuffix("U"), 0)
        if not 0 <= number <= 0xFFFFFFFF:
            raise ValueError(f"目标常量超出 u32：{name}")
        return number
    raise ValueError("目标常量存在循环引用")


def load_target(selection: str | Path | None = None, *, directory: Path | None = None) -> dict:
    """按名称或明确文件加载目标；缺失和不兼容均报错，不回退到默认目标。"""
    base = directory if directory is not None else Path(__file__).resolve().parent
    selected = str(selection) if selection is not None else DEFAULT_TARGET
    if re.fullmatch(r"[a-z][a-z0-9_]*", selected):
        path = base / f"{selected}.json"
        expected_name = selected
    else:
        path = Path(selected)
        expected_name = None
    target = json.loads(path.read_text(encoding="utf-8"))
    if (
        not isinstance(target, dict)
        or target.get("schema") != 1
        or not isinstance(target.get("constants"), dict)
        or not isinstance(target.get("target"), str)
        or not re.fullmatch(r"[a-z][a-z0-9_]*", target["target"])
        or (expected_name is not None and target["target"] != expected_name)
    ):
        raise ValueError("目标名称或契约版本不匹配")
    # 本轮保留 v3 的 Cortex-M/8B 写入契约；声明不支持的能力必须在操作前拒绝。
    if target.get("image_architecture", "cortex-m") != "cortex-m":
        raise ValueError("尚不支持该镜像启动架构")
    values = {key: target_value(target, key) for key in target["constants"]}
    if values["TARGET_PROGRAM_BYTES"] != 8 or not 0 < values["TARGET_APP_BYTES"] <= 131072:
        raise ValueError("目标不兼容现有 v3 写入单位或 APP 容量上限")
    for name in (
        "TARGET_LAYOUT",
        "TARGET_MCU",
        "TARGET_PRODUCT",
        "TARGET_HARDWARE",
        "TARGET_PARAM_ABI",
    ):
        if not values[name]:
            raise ValueError(f"目标身份不能为零：{name}")
    flash = values["TARGET_FLASH_BASE"]
    end = flash + values["TARGET_FLASH_BYTES"]
    app = values["TARGET_APP_BASE"]
    param = values["TARGET_PARAM_BASE"]
    identity = values["TARGET_IDENTITY_BASE"]
    erase = values["TARGET_ERASE_BYTES"]
    if (
        end > 0x100000000
        or not erase
        or erase % 8
        or any(
            values[k] % erase
            for k in (
                "TARGET_FLASH_BASE",
                "TARGET_LOADER_BYTES",
                "TARGET_APP_BASE",
                "TARGET_APP_BYTES",
                "TARGET_PARAM_BASE",
                "TARGET_PARAM_BYTES",
                "TARGET_IDENTITY_BASE",
                "TARGET_IDENTITY_BYTES",
            )
        )
        or not flash + values["TARGET_LOADER_BYTES"]
        <= app
        < app + values["TARGET_APP_BYTES"]
        <= param
        or not param
        < identity
        < identity + values["TARGET_IDENTITY_BYTES"]
        <= param + values["TARGET_PARAM_BYTES"]
        <= end
        or not values["TARGET_RAM_BASE"] < values["TARGET_RAM_END"] <= 0xFFFFFFFF - 31
        or values["TARGET_MAILBOX_ADDRESS"] != values["TARGET_RAM_END"]
        or values["TARGET_DESCRIPTOR_OFFSET"] % 8
        or not 8 <= values["TARGET_DESCRIPTOR_OFFSET"] <= values["TARGET_APP_BYTES"] - 96
    ):
        raise ValueError("目标分区、擦除对齐、描述符或邮箱配置无效")
    if "build" in target:
        build = target["build"]
        for key in ("artifact_mcu", "app_name"):
            # Keil工程名允许连字符；发布用MCU字段仍保持原有约束。
            pattern = r"[A-Za-z0-9_][A-Za-z0-9_-]*" if key == "app_name" else r"[A-Za-z0-9_]+"
            if not isinstance(build.get(key), str) or not re.fullmatch(pattern, build[key]):
                raise ValueError(f"目标构建名称非法：{key}")
    return target


def repository_path(root: Path, relative: str) -> Path:
    """解析目标中的仓库相对路径，拒绝越界和绝对路径。"""
    source = Path(relative)
    path = (root / source).resolve()
    if source.is_absolute() or not path.is_relative_to(root.resolve()):
        raise ValueError("目标路径必须在仓库内")
    return path
