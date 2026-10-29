"""编译真实 CAN 端口和窄 HAL 状态桩，日志只写 outputs，不接硬件。"""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
from harness.common import zig_path
from project_paths import ROOT


def main() -> int:
    """运行单个通信恢复反例，保留构建、执行及输入哈希。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", type=Path)
    parser.add_argument("--out", type=Path, default=ROOT / "outputs/tests/comm_bus_off")
    args = parser.parse_args()
    compiler = args.cc or zig_path()
    if compiler is None:
        parser.error("未找到本机 C 编译器")
    output = args.out.resolve()
    if not output.is_relative_to((ROOT / "outputs").resolve()):
        parser.error("测试证据必须位于本树 outputs")
    output.mkdir(parents=True, exist_ok=True)
    for name in ("fdcan.h", "main.h"):
        (output / name).write_text("/* 仅测试桩：类型与函数在夹具定义。 */\n", encoding="utf-8")
    source = ROOT / "firmware/platform/bsp/mcu/stm32g4/can.c"
    fixture = ROOT / "tests/unit/native/comm_bus_off_test.c"
    executable = output / ("comm_bus_off.exe" if sys.platform == "win32" else "comm_bus_off")
    command = [str(compiler)] + (["cc"] if compiler.stem.lower() == "zig" else [])
    command += [
        "-std=c99",
        "-Wall",
        "-Wextra",
        "-Werror",
        "-I",
        str(output),
        "-I",
        str(ROOT / "firmware/platform/bsp/include"),
        "-I",
        str(ROOT / "firmware/platform/config"),
        "-I",
        str(ROOT / "firmware/app/config"),
        "-I",
        str(ROOT / "firmware/platform/bsp"),
        "-I",
        str(source.parent),
        str(fixture),
        "-o",
        str(executable),
    ]
    build = subprocess.run(command, capture_output=True, check=False)
    (output / "compile.log").write_bytes(build.stdout + build.stderr)
    result = (
        subprocess.run([str(executable)], capture_output=True, check=False)
        if build.returncode == 0
        else None
    )
    (output / "run.log").write_bytes(result.stdout + result.stderr if result else b"not run\n")
    summary = {
        "success": build.returncode == 0 and result is not None and result.returncode == 0,
        "hardware_validation": "not_run",
        "command": command,
        "compile_exit": build.returncode,
        "run_exit": result.returncode if result else None,
        "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "fixture_sha256": hashlib.sha256(fixture.read_bytes()).hexdigest(),
    }
    (output / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=True, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary, ensure_ascii=True))
    if result:
        print(result.stdout.decode("utf-8", errors="replace"), end="")
    return 0 if summary["success"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
