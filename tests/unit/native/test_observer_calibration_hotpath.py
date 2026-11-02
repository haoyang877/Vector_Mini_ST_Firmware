"""编译真实观测器与标定准备路径；只检查本次等价优化，不接硬件。"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
from harness.common import zig_path
from project_paths import NATIVE_INCLUDE_FLAGS, ROOT


def main() -> int:
    """运行一次定向检查，保留编译与执行日志。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    compiler = args.cc or zig_path()
    if compiler is None:
        parser.error("未找到本机C编译器")
    output = args.out.resolve()
    if not output.is_relative_to((ROOT / "outputs").resolve()):
        parser.error("证据必须位于本树outputs")
    output.mkdir(parents=True, exist_ok=True)
    (output / "main.h").write_text("#pragma once\n#include <stdint.h>\n", encoding="utf-8")
    summary = []
    for board, damping in ((1, 0), (1, 1), (2, 0), (2, 1)):
        label = f"{board}_{damping}"
        executable = output / f"observer_calibration_{label}.exe"
        command = [str(compiler)] + (["cc"] if compiler.stem.lower() == "zig" else [])
        command += ["-std=c99", "-O1", "-Wall", "-Wextra", "-Werror", "-I", str(output)]
        command += (
            NATIVE_INCLUDE_FLAGS
            + [f"-DBSP_BOARD={board}", "-DFOC_ACTIVE_MOTOR_PROFILE=1"]
            + [str(ROOT / "tests/unit/bsp_preemption_stubs.c")]
        )
        command += [
            f"-DMOTOR_HAS_DAMPING_RING={damping}",
            str(ROOT / "tests/unit/native/observer_calibration_hotpath_test.c"),
            str(ROOT / "firmware/motor/control/pi_controller.c"),
            str(ROOT / "firmware/motor/math/control_math.c"),
            "-lm",
            "-o",
            str(executable),
        ]
        build = subprocess.run(command, capture_output=True, check=False)
        (output / f"compile_{label}.log").write_bytes(build.stdout + build.stderr)
        run = (
            subprocess.run([str(executable)], capture_output=True, check=False)
            if build.returncode == 0
            else None
        )
        (output / f"run_{label}.log").write_bytes(run.stdout + run.stderr if run else b"not run\n")
        summary.append(
            {
                "board": board,
                "damping": damping,
                "compile_exit": build.returncode,
                "run_exit": run.returncode if run else None,
                "command": command,
            }
        )
        if build.returncode != 0 or run is None or run.returncode != 0:
            break
    result = {
        "success": len(summary) == 4
        and all(item["compile_exit"] == 0 and item["run_exit"] == 0 for item in summary),
        "hardware_validation": "not_run",
        "cases": summary,
        "core_included_in_cases": True,
    }
    (output / "summary.json").write_text(
        json.dumps(result, ensure_ascii=True, indent=2) + "\n", encoding="utf-8"
    )
    print(f"observer/calibration: {'PASS' if result['success'] else 'FAIL'}; logs: {output}")
    return 0 if result["success"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
