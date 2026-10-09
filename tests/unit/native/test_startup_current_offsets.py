"""离线验证真实上电初始化/电流校正生产体；无硬件和持久化入口。"""

import argparse
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
from project_paths import NATIVE_INCLUDE_FLAGS, ROOT
from run_position_servo_tests import function_source


def main() -> None:
    """仅替换存储/启动边界并在2/6mΩ下编译执行生产初始化及校正逻辑。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    output = args.out.resolve()
    output.mkdir(parents=True, exist_ok=True)
    axis = (ROOT / "firmware/app/runtime/motor_axis.c").read_text(encoding="utf-8")
    sensing = (ROOT / "firmware/motor/foc/foc_sensing.c").read_text(encoding="utf-8")
    algorithm = (ROOT / "firmware/motor/foc/foc_algorithm.c").read_text(encoding="utf-8")
    context_end = axis.index("CurrentOffsetCalibration s_current_offset_calibration;") + len(
        "CurrentOffsetCalibration s_current_offset_calibration;"
    )
    context = axis[axis.index("#define CURRENT_OFFSET_SAMPLE_COUNT") : context_end]
    production = context + "\n#define FOC_MAX_MODULATION (0.95f * 0.866025403f)\n"
    production += "#define CURRENT_OFFSET_MIN_COUNTS 1948.0f\n"
    production += (
        "#define CURRENT_OFFSET_MAX_COUNTS 2148.0f\n#define OVERCURRENT_CONFIRM_CYCLES 5U\n"
    )
    production += function_source(sensing, "MotorCurrentOffsets_Valid")
    production += function_source(sensing, "Current_Cal")
    for name in ("FOC_MaximumVoltage", "FOC_CurrentController_Reset"):
        production += function_source(algorithm, name)
    transaction = (ROOT / "firmware/app/runtime/motor_parameter_transaction.c").read_text(
        encoding="utf-8"
    )
    for name in (
        "MotorParameterTransaction_AdvanceRevision",
        "MotorParameterTransaction_AdvanceMaintenanceRevision",
        "MotorParameterTransaction_Init",
    ):
        production += function_source(transaction, name)
    for name in (
        "configure_controllers",
        "update_current_controller_limits",
        "initialize_current_offsets",
        "fail_current_offset_calibration",
        "sample_phase_currents",
        "MotorAxis_Init",
        "run_fast_task",
        "MotorAxis_FastTick",
    ):
        production += function_source(axis, name)
    fixture = (ROOT / "tests/unit/startup_current_offsets_test.c").read_text(encoding="utf-8")
    fixture = fixture.replace(
        "/* STARTUP_CURRENT_OFFSET_PRODUCTION：脚本插入真实生产体。 */", production
    )
    source = output / "startup_current_offsets.c"
    source.write_text(fixture, encoding="utf-8")
    compiler = [args.cc] + (["cc"] if Path(args.cc).stem.lower() == "zig" else [])
    for shunt in (2, 6):
        executable = output / f"startup_current_offsets_{shunt}.exe"
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
                "-Wno-unused-parameter",
                f"-DCURRENT_SENSE_SHUNT_MILLIOHM={shunt}U",
                str(source),
                str(ROOT / "firmware/motor/foc/foc_pid.c"),
                str(ROOT / "firmware/motor/math/control_math.c"),
                "-lm",
                "-o",
                str(executable),
            ]
        )
        for invocation in (command, [str(executable)]):
            result = subprocess.run(invocation, capture_output=True, text=True, check=False)
            print(result.stdout + result.stderr, end="")
            result.check_returncode()


if __name__ == "__main__":
    main()
