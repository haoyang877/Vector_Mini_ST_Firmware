"""使用完整生产运行所有者验证命令、标定保存身份和取消释放；不访问硬件。"""

import argparse
import subprocess
from pathlib import Path

from run_position_servo_tests import NATIVE_INCLUDE_FLAGS, ROOT, function_source


def main() -> None:
    """编译真实所有者与纯核心，仅以桩替换硬件和标定 worker。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", required=True)
    parser.add_argument("--out", type=Path, default=ROOT / "outputs/run_state_tests")
    args = parser.parse_args()
    output = args.out.resolve()
    output.mkdir(parents=True, exist_ok=True)
    fixture = (ROOT / "tests/unit/native/run_state_owner_test.c").read_text(encoding="utf-8")
    # 准入与两项确定性计算沿用生产体；完整状态所有者独立编译，不替换内部状态。
    algorithm = (ROOT / "firmware/motor/foc/foc_algorithm.c").read_text(encoding="utf-8")
    sensing = (ROOT / "firmware/motor/foc/foc_sensing.c").read_text(encoding="utf-8")
    fixture += "\n" + function_source(algorithm, "FOC_MaximumVoltage")
    fixture += (
        "\n#define CURRENT_OFFSET_MIN_COUNTS 1948.0f\n#define CURRENT_OFFSET_MAX_COUNTS 2148.0f\n"
    )
    fixture += function_source(sensing, "MotorCurrentOffsets_Valid")
    execution = (ROOT / "firmware/app/runtime/motor_execution.c").read_text(encoding="utf-8")
    fixture += "\n" + function_source(execution, "MotorExecution_ValidateTarget")
    adapter = (ROOT / "firmware/app/composition/yg_protocol_status_adapter.c").read_text(
        encoding="utf-8"
    )
    fixture += "\n" + function_source(adapter, "protocol_mode")
    fixture += "\n" + function_source(adapter, "protocol_state")
    source = output / "run_state_owner.c"
    source.write_text(fixture, encoding="utf-8")
    compiler = [args.cc] + (["cc"] if Path(args.cc).stem == "zig" else [])
    executable = output / "run_state_owner.exe"
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
            str(source),
            str(ROOT / "firmware/app/motor_state_machine/motor_state_machine.c"),
            str(ROOT / "firmware/app/runtime/motor_maintenance.c"),
            str(ROOT / "firmware/app/motor_state_machine/motor_state_rules.c"),
            str(ROOT / "firmware/motor/protection/foc_errhandle.c"),
            str(ROOT / "firmware/services/parameters/motor_axis_profile.c"),
            str(ROOT / "firmware/services/parameters/parameter_value_service.c"),
            str(ROOT / "firmware/app/runtime/maintenance_workspace.c"),
            str(ROOT / "shared/checksum/crc32.c"),
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
    rules_executable = output / "motor_state_rules.exe"
    rules_command = (
        compiler
        + NATIVE_INCLUDE_FLAGS
        + [
            "-std=c99",
            "-O2",
            "-Wall",
            "-Wextra",
            "-Werror",
            str(ROOT / "tests/unit/native/motor_state_rules_test.c"),
            str(ROOT / "firmware/app/motor_state_machine/motor_state_rules.c"),
            "-o",
            str(rules_executable),
        ]
    )
    for invocation in (rules_command, [str(rules_executable)]):
        result = subprocess.run(invocation, capture_output=True, text=True)
        logs.append(result.stdout + result.stderr)
        (output / "test.log").write_text("\n".join(logs), encoding="utf-8")
        print(logs[-1], end="")
        result.check_returncode()


if __name__ == "__main__":
    main()
