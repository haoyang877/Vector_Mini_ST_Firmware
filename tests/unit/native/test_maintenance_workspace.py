"""检查实际固定工作区的类型容量、互斥、归还与复用；不接触硬件。"""

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
    """编译真实 APP 工作区与离线临界区桩，证据写入 outputs。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", default=None)
    parser.add_argument("--out", type=Path, default=None)
    args = parser.parse_args()
    compiler = Path(args.cc) if args.cc else zig_path()
    if compiler is None:
        parser.error("未找到 Zig/C 编译器，请通过 --cc 指定")
    output = (args.out or ROOT / "outputs/tests/maintenance_workspace").resolve()
    output.mkdir(parents=True, exist_ok=True)
    executable = output / (
        "maintenance_workspace.exe" if os.name == "nt" else "maintenance_workspace"
    )
    task = (ROOT / "firmware/motor/calibration/encoder_calibration_task.c").read_text(
        encoding="utf-8"
    )
    context = task[
        task.index("static EncoderCalibrationWorkspace *s_encoder_calib_workspace") : task.index(
            "#define ENC_ZERO_ALIGN_TIME"
        )
    ]
    context = context.replace("static uint16_t s_zero_sample_anchor;\n", "")
    functions = "\n".join(
        function_source(task, name)
        for name in (
            "encoder_calib_release_samples",
            "encoder_calib_prepare_samples",
            "EncoderCalibrationTask_Cancel",
            "EncoderCalibrationTask_IsReleased",
        )
    )
    fixture = (ROOT / "tests/unit/maintenance_workspace_test.c").read_text(encoding="utf-8")
    fixture = fixture.replace(
        "/* ENCODER_WORKSPACE_REFERENCE：脚本插入真实任务的采样借用和取消函数。 */",
        context + functions,
    )
    source = output / "workspace_fixture.c"
    source.write_text(fixture, encoding="utf-8")
    command = [str(compiler)] + (["cc"] if compiler.stem.lower() == "zig" else [])
    command += NATIVE_INCLUDE_FLAGS + [
        "-std=c99",
        "-Wall",
        "-Wextra",
        "-Werror",
        str(source),
        str(ROOT / "firmware/app/runtime/maintenance_workspace.c"),
        "-o",
        str(executable),
    ]
    subprocess.run(command, check=True)
    subprocess.run([str(executable)], check=True)


if __name__ == "__main__":
    main()
