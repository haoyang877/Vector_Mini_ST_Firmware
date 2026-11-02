"""运行真实测试模块的内存回归；不访问适配器或电机。"""

import argparse
import json
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
from harness.common import zig_path
from project_paths import NATIVE_INCLUDE_FLAGS, ROOT


def main() -> int:
    """用仓库编译器构建并运行有界触发、协议与停止条件检查。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    compiler = args.cc or zig_path()
    if compiler is None:
        parser.error("未找到本机C编译器")
    out = args.out.resolve()
    if not out.is_relative_to((ROOT / "outputs").resolve()):
        parser.error("证据须写入outputs")
    out.mkdir(parents=True, exist_ok=True)
    executable = out / "unified_motor_bench.exe"
    command = [str(compiler)] + (["cc"] if compiler.stem.lower() == "zig" else [])
    command += NATIVE_INCLUDE_FLAGS + [
        "-std=c99",
        "-O2",
        "-Wall",
        "-Wextra",
        "-Werror",
        "-DBSP_BOARD=2",
        "-DFOC_ACTIVE_MOTOR_PROFILE=1",
        "-DAPP_ENABLE_MOTOR_BENCH=1",
        str(ROOT / "tests/unit/native/unified_motor_bench_test.c"),
        str(ROOT / "firmware/motor/math/control_math.c"),
        "-lm",
        "-o",
        str(executable),
    ]
    build = subprocess.run(command, capture_output=True, check=False)
    (out / "compile.log").write_bytes(build.stdout + build.stderr)
    run = (
        subprocess.run([str(executable)], capture_output=True, check=False)
        if build.returncode == 0
        else None
    )
    (out / "run.log").write_bytes(run.stdout + run.stderr if run else b"NOT_RUN\n")
    passed = build.returncode == 0 and run is not None and run.returncode == 0
    (out / "summary.json").write_text(
        json.dumps(
            {
                "status": "PASS" if passed else "FAIL",
                "hardware_contacted": False,
                "compile_exit": build.returncode,
                "run_exit": run.returncode if run else None,
                "command": command,
            },
            indent=2,
        ),
        encoding="utf-8",
    )
    print(
        (run.stdout + run.stderr).decode(errors="replace")
        if run
        else (build.stdout + build.stderr).decode(errors="replace")
    )
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
