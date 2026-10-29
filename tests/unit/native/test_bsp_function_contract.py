"""执行真实共用功能接口和MCU边界故障注入，不连接硬件。"""

import argparse
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
from project_paths import NATIVE_INCLUDE_FLAGS, ROOT


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    compiler = [args.cc] + (["cc"] if Path(args.cc).stem == "zig" else [])
    for board, internal in ((1, False), (2, False), (2, True)):
        executable = args.out.resolve() / f"bsp_contract_{board}_{internal}.exe"
        subprocess.run(
            compiler
            + NATIVE_INCLUDE_FLAGS
            + [
                "-std=c11",
                "-O2",
                "-UNDEBUG",
                "-Wall",
                "-Wextra",
                "-Werror",
                f"-DBSP_BOARD={board}",
                *(["-DBSP_ENABLED_MCU_TEMPERATURE=1"] if internal else []),
                str(ROOT / "tests/unit/native/bsp_function_contract_test.c"),
                *[
                    str(ROOT / "firmware/platform/bsp" / name)
                    for name in ("bsp_adc.c", "bsp_pwm.c", "bsp_uart.c")
                ],
                "-lm",
                "-o",
                str(executable),
            ],
            check=True,
        )
        for scenario in range(5):
            subprocess.run([str(executable), str(scenario)], check=True)


if __name__ == "__main__":
    main()
