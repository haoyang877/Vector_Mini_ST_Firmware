"""以真实SDK声明编译ADC/CAN端口，执行有错误注入的离线回归。"""

import sys as _sys
from pathlib import Path as _Path

_sys.path.insert(0, str(_Path(__file__).resolve().parents[3] / "tools"))
import argparse
import subprocess
from pathlib import Path

from project_paths import NATIVE_INCLUDE_FLAGS, ROOT
from run_position_servo_tests import function_source


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    native = ROOT / "tests/unit/native"
    command = [args.cc] + (["cc"] if Path(args.cc).stem == "zig" else [])
    command += ["-DBSP_BOARD=2", "-std=c11", "-O2", "-UNDEBUG", "-Wall", "-Wextra", "-Werror"]
    for directory in (
        native / "r40_io_fake",
        ROOT / "firmware/platform/sdk/at32f45x/drivers/inc",
        ROOT / "firmware/platform/bsp/mcu/at32f456",
        ROOT / "firmware/platform/bsp/include",
        ROOT / "firmware/platform/config",
        ROOT / "firmware/app/config",
        ROOT / "firmware/platform/bsp",
        ROOT / "firmware/platform/devices",
    ):
        command += ["-I", str(directory)]
    for module in (
        "adc",
        "adc_disabled",
        "sincos",
        "can",
        "motor_ntc",
        "motor_ntc_bad_config",
    ):
        source_module = module.removesuffix("_sincos").replace("_keil", "")
        source_module = "motor_ntc" if source_module == "motor_ntc_bad_config" else source_module
        flags = []
        if source_module == "adc":
            flags = ["-DBSP_ENABLED_MCU_TEMPERATURE=1"]
        elif source_module == "sincos":
            flags = ["-DBSP_ENABLED_MCU_TEMPERATURE=1", "-DBSP_ENABLED_OUTPUT_SINCOS=1"]
        elif source_module == "adc_disabled":
            flags = ["-DBSP_ENABLED_MOTOR_TEMPERATURE=0"]
        elif module == "motor_ntc_bad_config":
            flags = ["-DR40_MOTOR_NTC_BETA_K=0.0f", "-DEXPECT_BAD_NTC_CONFIG=1"]
        if module == "motor_ntc":
            flags = ["-DEXPECT_BAD_NTC_CONFIG=0"]
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
            range(7)
            if module == "sincos"
            else range(22)
            if module in ("adc_sincos", "adc_keil_sincos")
            else (*range(15), 17, 18, 19, 20, 21, 22, 23)
            if module == "adc"
            else range(1)
        )
        for scenario in scenarios:
            subprocess.run([str(executable), str(scenario)], check=True)
    stm32 = ROOT / "firmware/platform/bsp/mcu/stm32g4"
    sampling = (stm32 / "adc.c").read_text(encoding="utf8")
    names = (
        "McuAdc_Init",
        "McuAdc_Start",
        "McuAdc_CompleteFast",
        "Adc_IsHealthy",
        "McuAdc_ReadRank",
        "McuAdc_ReadBusCurrent",
        "McuAdc_PollTemperature",
        "McuAdc_ContinueTemperature",
    )
    production = "\n".join(function_source(sampling, name) for name in names)
    production += '\n#include "' + (ROOT / "firmware/platform/bsp/bsp_adc.c").as_posix() + '"\n'
    for path, functions in (
        (
            "temperature.c",
            ("convert_sample", "McuAdc_ConvertTemperature"),
        ),
        ("system.c", ("Bsp_Start",)),
    ):
        source = (stm32 / path).read_text(encoding="utf8")
        production += "\n" + "\n".join(function_source(source, name) for name in functions)
    fixture = (native / "stm32_sampling_test.c").read_text(encoding="utf8")
    source = args.out.resolve() / "stm32_sampling_production.c"
    source.write_text(
        fixture.replace("/* STM32_SAMPLING_PRODUCTION */", production), encoding="utf8"
    )
    executable = args.out.resolve() / "stm32_sampling.exe"
    subprocess.run(
        [item for item in command[: command.index("-I")] if item != "-DBSP_BOARD=2"]
        + NATIVE_INCLUDE_FLAGS
        + [str(source), "-lm", "-o", str(executable)],
        check=True,
    )
    subprocess.run([str(executable)], check=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
