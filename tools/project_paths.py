"""直接 CLI 工具与离线测试共享的仓库路径；本模块不访问硬件。"""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
FIRMWARE = ROOT / "firmware"
CUBEMX = FIRMWARE / "platform/bsp/mcu/stm32g4/generated"
KEIL = ROOT / "projects/stm32g4/MDK-ARM"
BUILD = ROOT / "outputs/build/keil"

# Existing tools remain directly executable without an editable installation.
for relative in (
    "tools/bench",
    "tools/analysis",
    "tools/build",
    "tests/unit/native",
):
    directory = str(ROOT / relative)
    if directory not in sys.path:
        sys.path.append(directory)

NATIVE_INCLUDE_DIRS = (
    "shared/checksum",
    "shared/protocol",
    "shared/boot",
    "shared/image",
    "shared/targets",
    "shared/targets/vector_mini_st_g431",
    "loader/product",
    "loader/ports/include",
    "firmware/app/composition",
    "firmware/app/runtime",
    "firmware/app/motor_state_machine",
    "firmware/motor/math",
    "firmware/motor",
    "firmware/motor/foc",
    "firmware/motor/position",
    "firmware/motor/calibration",
    "firmware/motor/feedback",
    "firmware/motor/protection",
    "firmware/app/interfaces",
    "firmware/parameters",
    "firmware/communication/can",
    "firmware/communication/commands",
    "firmware/platform/bsp/include",
    "firmware/platform/devices",
    "firmware/platform/config",
    "firmware/platform/bsp",
    "firmware/app/config",
    "firmware/app/config/targets",
    "firmware/communication/telemetry",
    "firmware/platform/bsp",
    "firmware/app/config",
    "firmware/app/config/targets",
    "firmware/platform/bsp/mcu/stm32g4",
)


def resolve_source_path(tree: Path, current: str, *legacy: str) -> Path:
    """从指定源码树选择真实存在的当前或旧路径；不回退到工作区的另一实现。"""
    for relative in (current, *legacy):
        candidate = tree / relative
        if candidate.is_file():
            return candidate
    raise FileNotFoundError(f"源码树缺少当前及历史路径：{tree}: {(current, *legacy)}")


def native_include_flags(tree: Path = ROOT) -> list[str]:
    """为实际源码树提供头搜索目录；旧服务目录仅用于冻结基线自己的头文件。"""
    legacy = (
        "firmware/services/lifecycle",
        "firmware/services/parameters",
        "firmware/services/telemetry",
    )
    return [
        item
        for relative in (*NATIVE_INCLUDE_DIRS, *legacy)
        if (tree / relative).is_dir()
        for item in ("-I", str(tree / relative))
    ]


NATIVE_INCLUDE_FLAGS = native_include_flags()
