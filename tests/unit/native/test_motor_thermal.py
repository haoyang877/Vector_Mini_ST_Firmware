"""仅编译真实绕组热纯核、APP组合与FOC入口；不访问硬件。"""

import argparse
import subprocess
from pathlib import Path

from run_position_servo_tests import NATIVE_INCLUDE_FLAGS, ROOT


def main() -> None:
    """编译一次，在各自冷启动进程中执行四个必要边界场景。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", required=True, type=Path)
    parser.add_argument("--out", type=Path, default=ROOT / "outputs/motor_thermal")
    args = parser.parse_args()
    output = args.out.resolve()
    output.mkdir(parents=True, exist_ok=True)
    executable = output / "motor_thermal.exe"
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
            str(ROOT / "tests/unit/motor_thermal_test.c"),
            str(ROOT / "firmware/motor/protection/motor_thermal.c"),
            str(ROOT / "firmware/motor/protection/motor_thermal_config.c"),
            str(ROOT / "firmware/app/runtime/motor_thermal_control.c"),
            str(ROOT / "firmware/motor/foc/foc_algorithm.c"),
            str(ROOT / "firmware/motor/foc/foc_pid.c"),
            str(ROOT / "firmware/motor/math/control_math.c"),
            "-lm",
            "-o",
            str(executable),
        ]
    )
    logs = []
    for invocation in [command] + [
        [str(executable), name] for name in ("core", "runtime", "watchdog", "foc")
    ]:
        result = subprocess.run(invocation, capture_output=True, text=True, check=False)
        logs.append(result.stdout + result.stderr)
        (output / "test.log").write_text("\n".join(logs), encoding="utf-8")
        print(logs[-1], end="")
        result.check_returncode()


if __name__ == "__main__":
    main()
