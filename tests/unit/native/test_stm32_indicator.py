"""对照重构前真实指示器输出的固定签名，覆盖GPIO和DMA数据。"""

import argparse
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
from project_paths import ROOT


def main() -> None:
    """使用完整生产指示器实现和最小硬件桩，不连接设备。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--baseline", type=Path)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    base = (args.baseline or ROOT) / "firmware/platform/bsp/mcu/stm32g4"
    native = ROOT / "tests/unit/native"
    sources = (
        [base / "indicator_stm32g4.c"]
        if args.baseline
        else [base / "led.c", ROOT / "firmware/platform/bsp/bsp_led.c"]
    )
    executable = args.out.resolve() / "indicator.exe"
    command = [args.cc] + (["cc"] if Path(args.cc).stem == "zig" else [])
    command += ["-std=c11", "-O2", "-UNDEBUG", "-Wall", "-Wextra", "-Werror"]
    if args.baseline:
        command += ["-Wno-parentheses", "-DLEGACY_INDICATOR=1"]
    for directory in (
        native / "stm32_indicator_fake",
        (args.baseline or ROOT) / "firmware/platform/bsp/include",
        ROOT / "firmware/platform/bsp",
        ROOT / "firmware/platform/config",
        ROOT / "firmware/app/config",
    ):
        command += ["-I", str(directory)]
    command += [str(p) for p in (*sources, native / "stm32_indicator_test.c")]
    command += ["-o", str(executable)]
    build = subprocess.run(command, capture_output=True)
    (args.out / "compile.log").write_bytes(build.stdout + build.stderr)
    build.check_returncode()
    result = subprocess.run([str(executable)], capture_output=True, check=True, text=True)
    (args.out / "run.log").write_text(result.stdout, encoding="utf8")
    # 原始签名在首次基线运行后冻结；候选不得自行更新。
    expected = "f4329c0d 114688"
    # 基线12f346c5：原indicator_stm32g4.c在同一2kHz调度、5/20Hz节拍和合法颜色下取得。
    if not args.baseline and result.stdout.strip() != expected:
        raise RuntimeError("Indicator differs from frozen GPIO/DMA baseline: " + result.stdout)
    print(result.stdout, end="")


if __name__ == "__main__":
    main()
