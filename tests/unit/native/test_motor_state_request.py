"""编译真实参数事务模块与 Axis 转发，核对请求/保存合同；不访问硬件。"""

import argparse
import subprocess
from pathlib import Path

from run_position_servo_tests import NATIVE_INCLUDE_FLAGS, ROOT, function_source

# 逐函数从生产源码切片，夹具编译的正是真实实现；旧实现没有新辅助函数时跳过。
REQUEST_FUNCTIONS = (
    "advance_config_revision",
    "same_write",
    "stale_sequence",
    "matches_cached_request",
    "refresh_save_result",
    "MotorAxis_ReadParameter",
    "MotorAxis_WriteParameter",
    "MotorAxis_RequestSave",
    "MotorAxis_PollSave",
    "MotorAxis_SaveIdle",
    "MotorAxis_ConfigurationRevisionAvailable",
    "MotorAxis_ServiceBackground",
)


def run_variant(source_path: Path, output: Path, compiler_path: str) -> str:
    """编译实际生产实现，以桩提供边界，返回公共参数/保存路径的调用摘要。"""
    output.mkdir(parents=True, exist_ok=True)
    source = source_path.read_text(encoding="utf-8")
    fixture = (ROOT / "tests/unit/native/motor_state_request_test.c").read_text(encoding="utf-8")
    if '#include "motor_parameter_transaction.h"' in source:
        fixture = "#define MOTOR_PARAMETER_TRANSACTION_PRESENT 1\n" + fixture
    for name in REQUEST_FUNCTIONS:
        try:
            fixture += "\n" + function_source(source, name)
        except ValueError:
            continue
    combined = output / "motor_state_request.c"
    combined.write_text(fixture, encoding="utf-8")

    compiler = [compiler_path] + (["cc"] if Path(compiler_path).stem == "zig" else [])
    executable = output / "motor_state_request.exe"
    command = (
        compiler
        + NATIVE_INCLUDE_FLAGS
        + [
            "-I",
            str(ROOT / "firmware/services/lifecycle"),
            "-std=c99",
            "-O1",
            "-UNDEBUG",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-DCURRENT_SENSE_SHUNT_MILLIOHM=6U",
            str(combined),
            "-o",
            str(executable),
        ]
    )
    logs = []
    for invocation in (command, [str(executable)]):
        result = subprocess.run(invocation, capture_output=True, text=True)
        logs.append(result.stdout + result.stderr)
        (output / "test.log").write_text("\n".join(logs), encoding="utf-8")
        print(logs[-1], end="")
        result.check_returncode()
    return next(line for line in logs[-1].splitlines() if line.startswith("EQUIVALENCE "))


def main() -> None:
    """执行现实现，给定基点时比较相同输入下参数/保存结果与边界调用次序。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", required=True)
    parser.add_argument("--out", type=Path, default=ROOT / "outputs/motor_state_request_tests")
    parser.add_argument("--reference-source", type=Path)
    args = parser.parse_args()
    output = args.out.resolve()
    current = run_variant(ROOT / "firmware/app/runtime/motor_axis.c", output / "current", args.cc)
    if args.reference_source:
        reference = run_variant(args.reference_source, output / "reference", args.cc)
        if current != reference:
            raise RuntimeError(f"参数/保存行为与基点不同：{current} != {reference}")
        print("PASS parameter/background baseline equivalence")


if __name__ == "__main__":
    main()
