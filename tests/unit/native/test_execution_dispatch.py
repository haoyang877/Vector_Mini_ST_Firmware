"""编译真实快环调度体，核对功能独占、故障优先及零矢量预载；可与旧源码差分。"""

import argparse
import subprocess
from pathlib import Path

from run_position_servo_tests import NATIVE_INCLUDE_FLAGS, ROOT, function_source


def run_variant(compiler: Path, source_dir: Path, output: Path, profile: int) -> str:
    """在相同桩输入下编译旧版或新版生产体，返回可比较的调用轨迹摘要。"""
    execution = (source_dir / "motor_execution.c").read_text(encoding="utf-8")
    state_path = source_dir / "motor_axis.c"
    if not state_path.exists():
        state_path = source_dir / "motor_state.c"
    state = state_path.read_text(encoding="utf-8")
    fixture = (ROOT / "tests/unit/native/execution_dispatch_test.c").read_text(encoding="utf-8")
    if "s_current_offset_calibration.complete" in state:
        context_end = state.index("CurrentOffsetCalibration s_current_offset_calibration;") + len(
            "CurrentOffsetCalibration s_current_offset_calibration;"
        )
        context = state[state.index("#define CURRENT_OFFSET_SAMPLE_COUNT") : context_end]
        fixture = (
            '#include "foc_sensing.h"\n#define STARTUP_CURRENT_OFFSETS_PRESENT 1\n'
            + context
            + "\n"
            + fixture
        )
    if "MotorExecution_ControlStep(" in execution:
        fixture += "\n" + function_source(execution, "MotorExecution_ControlStep")
        fixture += "\n" + function_source(state, "run_fast_task")
        call = "run_fast_task()"
        if "run_fast_task(bool maintenance_runnable)" in state:
            # 夹具单测分发入口：按原故障/零点门禁捕获许可，便于比较功能调用轨迹。
            call = (
                "run_fast_task(s_current_offset_calibration.complete && "
                "s_motor_state.fault_code == MOTOR_FAULT_NONE && "
                "MotorStateMachine_MaintenanceRunnable())"
            )
    else:
        fixture += "\n" + function_source(execution, "MotorExecution_FastStep")
        call = (
            "MotorExecution_FastStep(&foc, &motor_state, &speed_pi, "
            "&rotor_encoder, &flux_observer, &startup)"
        )
    fixture += "\nstatic MotorTaskResult dispatch_step(void) { return " + call + "; }\n"
    output.mkdir(parents=True, exist_ok=True)
    source = output / "dispatch.c"
    source.write_text(fixture, encoding="utf-8")
    executable = output / "dispatch.exe"
    command = [str(compiler)] + (["cc"] if compiler.stem.lower() == "zig" else [])
    command += NATIVE_INCLUDE_FLAGS + [
        "-I",
        str(ROOT / "firmware/services/lifecycle"),
        "-std=c99",
        "-O1",
        "-UNDEBUG",
        "-Wall",
        "-Wextra",
        "-Werror",
        f"-DFAST_LOOP_STAGE_PROFILE={profile}",
        str(source),
        "-o",
        str(executable),
    ]
    logs = []
    for invocation in (command, [str(executable)]):
        result = subprocess.run(invocation, capture_output=True, text=True, check=False)
        logs.append(result.stdout + result.stderr)
        (output / "test.log").write_text("\n".join(logs), encoding="utf-8")
        result.check_returncode()
    return logs[-1]


def main() -> None:
    """执行核心分发检查；功能退役后不与全功能历史轨迹混作等价证据。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", required=True, type=Path)
    parser.add_argument("--out", type=Path, default=ROOT / "outputs/execution_dispatch_tests")
    parser.add_argument("--reference-dir", type=Path)
    args = parser.parse_args()
    for profile in (0,):
        current = run_variant(
            args.cc, ROOT / "firmware/app/runtime", args.out / f"current-{profile}", profile
        )
        print(current, end="")
        if args.reference_dir:
            reference = run_variant(
                args.cc, args.reference_dir, args.out / f"reference-{profile}", profile
            )
            if current != reference:
                raise RuntimeError(f"Dispatch behavior differs with profiling={profile}.")
            print(f"PASS baseline equivalence with profiling={profile}")


if __name__ == "__main__":
    main()
