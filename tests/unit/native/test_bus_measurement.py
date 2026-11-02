"""用真实测量类型与滤波实现验证母线电压、电流的数值语义。"""

import argparse
import subprocess
from pathlib import Path

from run_position_servo_tests import NATIVE_INCLUDE_FLAGS, ROOT, function_source


def main() -> None:
    """执行纯计算回归，无BSP采样或硬件访问。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", required=True, type=Path)
    parser.add_argument("--out", required=True, type=Path)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    production = []
    for file_name, function_name in (
        ("motor_voltage.c", "MotorVoltage_Update"),
        ("motor_current.c", "MotorCurrent_Update"),
    ):
        source = (ROOT / "firmware/motor/sensing" / file_name).read_text(encoding="utf-8")
        production.append(function_source(source, function_name))
    fixture = (ROOT / "tests/unit/native/bus_measurement_test.c").read_text(encoding="utf-8")
    generated = args.out.resolve() / "bus_measurement.c"
    generated.write_text(
        fixture.replace("/* BUS_MEASUREMENT_PRODUCTION */", "\n".join(production)),
        encoding="utf-8",
    )
    executable = args.out.resolve() / "bus_measurement.exe"
    compiler = [str(args.cc)] + (["cc"] if args.cc.stem.lower() == "zig" else [])
    subprocess.run(
        compiler
        + NATIVE_INCLUDE_FLAGS
        + [
            "-std=c99",
            "-O2",
            "-UNDEBUG",
            "-Wall",
            "-Wextra",
            "-Werror",
            str(generated),
            "-lm",
            "-o",
            str(executable),
        ],
        check=True,
    )
    subprocess.run([str(executable)], check=True)


if __name__ == "__main__":
    main()
