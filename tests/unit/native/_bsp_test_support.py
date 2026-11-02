"""BSP离线夹具共用编译入口；函数体逐字取自生产文件，硬件状态由测试注入。"""

import argparse
import subprocess
from collections.abc import Sequence
from pathlib import Path

from run_position_servo_tests import NATIVE_INCLUDE_FLAGS, ROOT, function_source


def arguments(description: str) -> argparse.Namespace:
    """解析原生测试统一的编译器与证据目录参数。"""
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument("--cc", required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    args.out = args.out.resolve()
    args.out.mkdir(parents=True, exist_ok=True)
    return args


def production(board: int, module: str, *names: str) -> str:
    """提取指定MCU的真实函数，不替换其中的判断、计算或硬件访问。"""
    mcu = "stm32g4" if board == 1 else "at32f456"
    source = (ROOT / f"firmware/platform/bsp/mcu/{mcu}/{module}.c").read_text(encoding="utf-8")
    return "\n".join(function_source(source, name) for name in names)


def resources(board: int) -> str:
    """引入真实资源映射，测试期望值独立使用板级接线数值。"""
    mcu = "stm32g4" if board == 1 else "at32f456"
    path = ROOT / f"firmware/platform/bsp/mcu/{mcu}/mcu_resources.h"
    return f'#include "{path.as_posix()}"\n'


def run_c(
    args: argparse.Namespace, name: str, source: str, board: int, extra: Sequence[str | Path] = ()
) -> None:
    """保存夹具和日志，严格编译后执行；失败保持原始诊断。"""
    fixture = args.out / f"{name}.c"
    fixture.write_text(source, encoding="utf-8")
    executable = args.out / f"{name}.exe"
    compiler = [args.cc] + (["cc"] if Path(args.cc).stem.lower() == "zig" else [])
    command = (
        compiler
        + NATIVE_INCLUDE_FLAGS
        + [
            "-std=c11",
            "-O2",
            "-UNDEBUG",
            "-Wall",
            "-Wextra",
            "-Werror",
            f"-DBSP_BOARD={board}",
            str(fixture),
            *map(str, extra),
            "-lm",
            "-o",
            str(executable),
        ]
    )
    logs = []
    for invocation in (command, [str(executable)]):
        result = subprocess.run(invocation, cwd=ROOT, capture_output=True, text=True)
        logs.append(result.stdout + result.stderr)
        (args.out / f"{name}.log").write_text("\n".join(logs), encoding="utf-8")
        print(logs[-1], end="")
        result.check_returncode()
