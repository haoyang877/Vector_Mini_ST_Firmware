"""编译真实APP Flash端口与进程内寄存器模型；不访问硬件。"""

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
    """仅运行Flash失败路径定向夹具，记录源码、寄存器定义和结果。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", type=Path)
    parser.add_argument("--source", type=Path)
    parser.add_argument("--header", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    compiler = args.cc or zig_path()
    if compiler is None:
        parser.error("未找到本机C编译器")
    output = args.out.resolve()
    if not output.is_relative_to((ROOT / "outputs").resolve()):
        parser.error("测试证据必须位于本树outputs")
    source = (args.source or ROOT / "firmware/platform/bsp/mcu/stm32g4/flash.c").resolve()
    header = (args.header or ROOT / "firmware/platform/bsp/include/bsp_flash.h").resolve()
    fixture = ROOT / "tests/unit/native/app_flash_failure_test.c"
    vendor = ROOT / (
        "firmware/platform/sdk/stm32g4/Drivers/CMSIS/Device/ST/STM32G4xx/Include/stm32g431xx.h"
    )
    target = ROOT / "shared/targets/vector_mini_st_g431.h"
    output.mkdir(parents=True, exist_ok=True)
    (output / "main.h").write_text("#include <stdint.h>\n", encoding="utf-8")
    # 寄存器位来自本仓库vendor头，避免模型自造位值。
    bit_definitions = "\n".join(
        line
        for line in vendor.read_text(encoding="utf-8").splitlines()
        if line.startswith(("#define FLASH_CR_", "#define FLASH_SR_", "#define FLASH_ACR_"))
    )
    (output / "flash_register_bits.h").write_text(bit_definitions + "\n", encoding="utf-8")
    (output / "flash_under_test.c").write_bytes(source.read_bytes())
    (output / "bsp_flash.h").write_bytes(header.read_bytes())
    executable = output / (
        "app_flash_failure.exe" if sys.platform == "win32" else "app_flash_failure"
    )
    command = [str(compiler)] + (["cc"] if compiler.stem.lower() == "zig" else [])
    command += [
        "-std=c99",
        "-O1",
        "-UNDEBUG",
        "-Wall",
        "-Wextra",
        "-Werror",
        # MCU地址为32位；宿主仅映射同一低地址，保留生产强转不改被测源码。
        "-Wno-int-to-pointer-cast",
        "-I",
        str(output),
        "-I",
        str(ROOT / "firmware/platform/bsp/mcu/stm32g4"),
        "-I",
        str(ROOT / "firmware/platform/bsp/include"),
        "-I",
        str(ROOT / "firmware/platform/config"),
        "-I",
        str(ROOT / "firmware/app/config"),
        "-I",
        str(ROOT / "firmware/platform/bsp"),
        "-I",
        str(target.parent),
        str(fixture),
        "-o",
        str(executable),
    ]
    build = subprocess.run(command, capture_output=True, check=False)
    (output / "compile.log").write_bytes(build.stdout + build.stderr)
    result = None
    timed_out = False
    if build.returncode == 0:
        try:
            result = subprocess.run([str(executable)], capture_output=True, check=False, timeout=10)
        except subprocess.TimeoutExpired as error:
            timed_out = True
            (output / "run.log").write_bytes((error.stdout or b"") + (error.stderr or b""))
    if not timed_out:
        (output / "run.log").write_bytes(result.stdout + result.stderr if result else b"not run\n")
    summary = {
        "success": build.returncode == 0 and result is not None and result.returncode == 0,
        "hardware_validation": "not_run",
        "single_bank_stall_validation": "not_run",
        "compile_exit": build.returncode,
        "run_exit": result.returncode if result else None,
        "timed_out": timed_out,
        "command": command,
        "inputs": {
            str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in (source, header, fixture, vendor, target)
        },
    }
    (output / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=True, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary, ensure_ascii=True))
    if result:
        print((result.stdout + result.stderr).decode("utf-8", errors="replace"), end="")
    return 0 if summary["success"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
