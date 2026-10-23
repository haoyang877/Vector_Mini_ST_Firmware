"""以真实SDK声明编译ADC/CAN端口，执行有错误注入的离线回归。"""

import sys as _sys
from pathlib import Path as _Path

_sys.path.insert(0, str(_Path(__file__).resolve().parents[3] / "tools"))
import argparse
import subprocess
from pathlib import Path

from project_paths import ROOT


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    native = ROOT / "tests/unit/native"
    command = [args.cc] + (["cc"] if Path(args.cc).stem == "zig" else [])
    command += ["-std=c11", "-O2", "-UNDEBUG", "-Wall", "-Wextra", "-Werror"]
    for directory in (
        native / "r40_io_fake",
        ROOT / "firmware/third_party/at32f45x/drivers/inc",
        ROOT / "firmware/platform/at32f456/bsp",
        ROOT / "firmware/platform/api",
    ):
        command += ["-I", str(directory)]
    for module in (
        "adc",
        "adc_sincos",
        "adc_keil_sincos",
        "adc_disabled",
        "adc_disabled_sincos",
        "can",
        "motor_ntc",
        "motor_ntc_bad_config",
    ):
        source_module = module.removesuffix("_sincos").replace("_keil", "")
        source_module = "motor_ntc" if source_module == "motor_ntc_bad_config" else source_module
        flags = []
        if source_module == "adc":
            flags = ["-DR40_TEMPERATURE_SENSING_ENABLED=1"]
        elif source_module == "adc_disabled":
            flags = ["-DR40_MOTOR_NTC_ENABLED=0"]
        elif module == "motor_ntc_bad_config":
            flags = ["-DR40_MOTOR_NTC_BETA_K=0.0f", "-DEXPECT_BAD_NTC_CONFIG=1"]
        if module == "motor_ntc":
            flags = ["-DEXPECT_BAD_NTC_CONFIG=0"]
        if module.endswith("_sincos"):
            flags.append("-DR40_OUTPUT_SINCOS_DIAGNOSTICS=1")
        if module == "adc_keil_sincos":
            flags.append("-DR40_KEIL_OPENLOOP=1")
        executable = args.out.resolve() / f"r40_{module}.exe"
        subprocess.run(
            command
            + flags
            + [
                str(native / f"r40_{source_module}_test.c"),
                str(native / "r40_io_fake/sdk.c"),
                "-lm",
                "-o",
                str(executable),
            ],
            check=True,
        )
        scenarios = (
            range(19)
            if module in ("adc_sincos", "adc_keil_sincos")
            else (*range(15), 17, 18)
            if module == "adc"
            else range(1)
        )
        for scenario in scenarios:
            subprocess.run([str(executable), str(scenario)], check=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
