"""编译完整AT32 ADC驱动与真实SDK声明，验证采样、DMA和模拟量行为。"""

import subprocess
from pathlib import Path

from _bsp_test_support import ROOT, arguments


def main() -> None:
    """保留已实现的ADC路径；CAN与编码器在各自真实硬件夹具中覆盖。"""
    args = arguments(__doc__)
    native = ROOT / "tests/unit/native"
    compiler = [args.cc] + (["cc"] if Path(args.cc).stem.lower() == "zig" else [])
    includes = [
        native / "r40_io_fake",
        ROOT / "firmware/platform/sdk/at32f45x/drivers/inc",
        ROOT / "firmware/platform/bsp/include",
        ROOT / "firmware/config",
        ROOT / "firmware/platform/bsp",
    ]
    command = compiler + [
        "-DBSP_BOARD=2",
        "-std=c11",
        "-O2",
        "-UNDEBUG",
        "-Wall",
        "-Wextra",
        "-Werror",
    ]
    for directory in includes:
        command += ["-I", str(directory)]
    executable = args.out / "r40_adc.exe"
    logs = []
    for invocation in (
        command
        + [
            str(native / "r40_adc_test.c"),
            str(native / "r40_io_fake/sdk.c"),
            "-lm",
            "-o",
            str(executable),
        ],
        [str(executable)],
    ):
        result = subprocess.run(invocation, capture_output=True, text=True)
        logs.append(result.stdout + result.stderr)
        (args.out / "test.log").write_text("\n".join(logs), encoding="utf-8")
        print(logs[-1], end="")
        result.check_returncode()


if __name__ == "__main__":
    main()
