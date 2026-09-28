"""使用真实 motor_state.c 验证参数读写/保存请求的身份冲突合同；不访问硬件。"""

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
    "MotorControl_ReadParameter",
    "MotorControl_WriteParameter",
    "MotorControl_RequestSave",
)


def main() -> None:
    """编译真实参数协调函数，以桩替换运行所有者、临界区与参数值服务。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", required=True)
    parser.add_argument("--out", type=Path, default=ROOT / "outputs/motor_state_request_tests")
    args = parser.parse_args()
    output = args.out.resolve()
    output.mkdir(parents=True, exist_ok=True)

    source = (ROOT / "firmware/app/motor_state.c").read_text(encoding="utf-8")
    fixture = (ROOT / "tests/unit/native/motor_state_request_test.c").read_text(encoding="utf-8")
    for name in REQUEST_FUNCTIONS:
        try:
            fixture += "\n" + function_source(source, name)
        except ValueError:
            continue
    combined = output / "motor_state_request.c"
    combined.write_text(fixture, encoding="utf-8")

    compiler = [args.cc] + (["cc"] if Path(args.cc).stem == "zig" else [])
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


if __name__ == "__main__":
    main()
