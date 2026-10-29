"""验证真实状态快照在快慢环抢占下保持一致，并保留有界兜底。"""

import argparse
import subprocess
from pathlib import Path

from run_position_servo_tests import NATIVE_INCLUDE_FLAGS, ROOT, function_source


def main() -> None:
    """编译真实读取函数，注入版本更新并验证完整结果。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", required=True, type=Path)
    parser.add_argument("--out", type=Path, default=ROOT / "outputs/motor_status_snapshot")
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    source = (ROOT / "firmware/app/runtime/motor_axis.c").read_text(encoding="utf-8")
    functions = "\n".join(
        function_source(source, name) for name in ("read_status_snapshot", "MotorAxis_ReadStatus")
    )
    fixture = (ROOT / "tests/unit/native/motor_status_snapshot_test.c").read_text(encoding="utf-8")
    generated = args.out / "snapshot.c"
    generated.write_text(fixture.replace("/* REAL_FUNCTIONS */", functions), encoding="utf-8")
    exe = args.out / "snapshot.exe"
    compiler = [str(args.cc)] + (["cc"] if args.cc.stem.lower() == "zig" else [])
    command = (
        compiler
        + NATIVE_INCLUDE_FLAGS
        + [
            "-std=c99",
            "-O1",
            "-UNDEBUG",
            "-Wall",
            "-Wextra",
            "-Werror",
            str(generated),
            "-o",
            str(exe),
        ]
    )
    log = []
    for invocation in (command, [str(exe)]):
        result = subprocess.run(invocation, capture_output=True, text=True, check=False)
        log.append(result.stdout + result.stderr)
        print(log[-1], end="")
        (args.out / "test.log").write_text("".join(log), encoding="utf-8")
        result.check_returncode()


if __name__ == "__main__":
    main()
