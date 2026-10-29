"""独立验证标定计算核心，并对完整任务/工作区检查或冻结任务做逐拍比较。"""

import argparse
import os
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
from harness.common import zig_path
from project_paths import NATIVE_INCLUDE_FLAGS, ROOT
from run_position_servo_tests import function_source


def main() -> None:
    """核心仅用自身 include 目录；任务仍使用真实工作区、数学和 PI 及离线控制桩。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", default=None)
    parser.add_argument("--out", type=Path, default=None)
    parser.add_argument("--reference-source", type=Path, default=None)
    parser.add_argument("--reference-core", type=Path, default=None)
    parser.add_argument("--damping-profile", type=int, choices=(0, 1), default=0)
    args = parser.parse_args()
    compiler = Path(args.cc) if args.cc else zig_path()
    if compiler is None:
        parser.error("未找到 Zig/C 编译器，请通过 --cc 指定")
    output = (args.out or ROOT / "outputs/tests/maintenance_workspace").resolve()
    output.mkdir(parents=True, exist_ok=True)
    extension = ".exe" if os.name == "nt" else ""
    command = [str(compiler)] + (["cc"] if compiler.stem.lower() == "zig" else [])
    flags = ["-std=c99", "-O1", "-Wall", "-Wextra", "-Werror"]
    core_directory = ROOT / "firmware/motor/calibration"
    core = core_directory / "encoder_calibration_core.c"
    core_executable = output / ("encoder_calibration_core" + extension)
    logs = []

    def run(arguments: list[str]) -> str:
        """记录真实执行结果，失败时保留编译或运行诊断。"""
        result = subprocess.run(arguments, capture_output=True, text=True, encoding="utf-8")
        logs.append(result.stdout + result.stderr)
        (output / "test.log").write_text("\n".join(logs), encoding="utf-8")
        print(logs[-1], end="")
        result.check_returncode()
        return result.stdout

    core_fixture = ROOT / "tests/unit/encoder_calibration_core_test.c"
    if args.reference_core:
        legacy = function_source(
            args.reference_core.read_text(encoding="utf-8"), "EncoderCalibrationCore_Q15Difference"
        ).replace("EncoderCalibrationCore_Q15Difference", "legacy_q15_difference")
        core_source = output / "core_reference_fixture.c"
        core_source.write_text(
            '#include "encoder_calibration_core.h"\n'
            "#define ENCODER_CALIBRATION_LEGACY_RING_REFERENCE\n"
            + legacy
            + "\n"
            + core_fixture.read_text(encoding="utf-8"),
            encoding="utf-8",
        )
        core_fixture = core_source
    run(
        command
        + flags
        + [
            "-I",
            str(core_directory),
            str(core_fixture),
            str(core),
            "-o",
            str(core_executable),
        ]
    )
    run([str(core_executable)])
    fixture = (ROOT / "tests/unit/maintenance_workspace_test.c").read_text(encoding="utf-8")
    sources = [core_directory / "encoder_calibration_task.c"]
    if args.reference_source:
        sources.append(args.reference_source)
    traces = []
    for index, task_path in enumerate(sources):
        source = output / f"workspace_fixture_{index}.c"
        source.write_text(
            fixture.replace(
                "/* ENCODER_WORKSPACE_REFERENCE：脚本插入真实任务的采样借用和取消函数。 */",
                task_path.read_text(encoding="utf-8"),
            ),
            encoding="utf-8",
        )
        executable = output / (f"maintenance_workspace_{index}" + extension)
        # 保留既有多圈 float→uint16 表达式；此宿主比较不替代 ARM 转换或实机证明。
        task_core = args.reference_core if index == 1 and args.reference_core else core
        reference_header = (
            ["-include", str(task_core.with_suffix(".h"))]
            if index == 1 and args.reference_core
            else []
        )
        run(
            command
            + reference_header
            + NATIVE_INCLUDE_FLAGS
            + [str(ROOT / "tests/unit/bsp_preemption_stubs.c")]
            + flags
            + ["-DMOTOR_HAS_DAMPING_RING=" + str(args.damping_profile)]
            + [
                "-fno-sanitize=float-cast-overflow",
                str(source),
                str(task_core),
                str(ROOT / "firmware/app/runtime/maintenance_workspace.c"),
                str(ROOT / "firmware/motor/foc/foc_pid.c"),
                str(ROOT / "firmware/motor/math/control_math.c"),
                "-o",
                str(executable),
            ]
        )
        traces.append(run([str(executable)]))
    if len(traces) == 2 and traces[0] != traces[1]:
        raise RuntimeError("冻结任务与当前任务的逐拍输出/阶段/工作区轨迹不一致，检查 test.log")
    if len(traces) == 2:
        print("reference equivalence: PASS (identical per-tick trace digest and final outputs)")


if __name__ == "__main__":
    main()
