"""离线验证真实上电初始化/电流校正生产体；无硬件和持久化入口。"""

import argparse
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
from project_paths import NATIVE_INCLUDE_FLAGS, ROOT
from run_position_servo_tests import function_source


def main() -> None:
    """在两板实际采样配置下执行生产初始化、校正和快环调度。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    output = args.out.resolve()
    output.mkdir(parents=True, exist_ok=True)
    axis = (ROOT / "firmware/app/runtime/motor_axis.c").read_text(encoding="utf-8")
    sensing = (ROOT / "firmware/motor/sensing/motor_current.c").read_text(encoding="utf-8")
    algorithm = (ROOT / "firmware/motor/control/foc.c").read_text(encoding="utf-8")
    production = "CurrentOffsetCalibration s_current_offset_calibration;\n"
    production += "static EncoderCalibrationWorkspace s_encoder_linearization_workspace;\n"
    production += "#if BSP_ENABLED_LINEAR_HALL\nstatic LinearHall s_linear_hall;\n"
    production += "static LinearHallCalibration s_hall_sweep;\n"
    production += "#endif\n"
    production += "#define FOC_MAX_MODULATION (0.95f * 0.866025403f)\n"
    production += function_source(sensing, "MotorCurrentOffsets_IsValid")
    production += function_source(sensing, "MotorCurrent_Update")
    for name in (
        "MotorCurrentOffsets_Init",
        "fail_current_offset_calibration",
        "MotorCurrentOffsets_Calibrate",
    ):
        production += function_source(sensing, name)
    for name in (
        "Foc_MaxVoltage",
        "Foc_ConfigureControllers",
        "Foc_UpdateCurrentLimits",
        "Foc_ResetCurrent",
        "Clarke_Transform",
    ):
        production += function_source(algorithm, name)
    for name in (
        "run_active_calibration_cycle",
        "MotorAxis_Init",
        "MotorAxis_SampleFast",
        "MotorAxis_ControlFast",
        "MotorAxis_OuterTick",
        "MotorAxis_ProcessCalibrationPublication",
    ):
        production += function_source(axis, name)
    start = production.index("    /* 固定启动暂存区")
    end = production.index("    MotorCurrentOffsets_Init(", start)
    production = (
        production[:start] + "    load_parameters(&s_motor_configuration);\n" + production[end:]
    )
    fixture = (ROOT / "tests/unit/startup_current_offsets_test.c").read_text(encoding="utf-8")
    fixture = fixture.replace(
        "/* STARTUP_CURRENT_OFFSET_PRODUCTION：脚本插入真实生产体。 */", production
    )
    source = output / "startup_current_offsets.c"
    source.write_text(fixture, encoding="utf-8")
    compiler = [args.cc] + (["cc"] if Path(args.cc).stem.lower() == "zig" else [])
    for board, shunt in ((1, 6), (2, 4)):
        executable = output / f"startup_current_offsets_{shunt}.exe"
        command = (
            compiler
            + NATIVE_INCLUDE_FLAGS
            + [str(ROOT / "tests/unit/bsp_preemption_stubs.c")]
            + [
                "-std=c99",
                "-O1",
                "-UNDEBUG",
                "-Wall",
                "-Wextra",
                "-Werror",
                "-Wno-unused-parameter",
                f"-DBSP_BOARD={board}",
                str(source),
                str(ROOT / "firmware/motor/control/pi_controller.c"),
                str(ROOT / "firmware/motor/math/control_math.c"),
                str(ROOT / "firmware/motor/feedback/linear_hall.c"),
                str(ROOT / "firmware/motor/protection/motor_thermal.c"),
                str(ROOT / "firmware/motor/protection/motor_thermal_config.c"),
                str(ROOT / "firmware/motor/protection/motor_thermal_control.c"),
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
