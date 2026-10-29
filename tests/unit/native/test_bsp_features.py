"""编译并执行真实能力组合与可选ADC路径；非法组合必须由编译器拒绝。"""

import argparse
import itertools
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
from project_paths import ROOT


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    native = ROOT / "tests/unit/native"
    platform = ROOT / "firmware/platform"
    compiler = [args.cc] + (["cc"] if Path(args.cc).stem == "zig" else [])
    common = compiler + ["-std=c11", "-O2", "-UNDEBUG", "-Wall", "-Wextra", "-Werror"]
    api = [
        "-I",
        str(platform / "bsp/include"),
        "-I",
        str(platform / "config"),
        "-I",
        str(platform / "bsp"),
        "-I",
        str(platform / "devices"),
        "-I",
        str(ROOT / "firmware/app/config"),
    ]
    subject = native / "bsp_feature_config_test.c"
    for board in (1, 2):
        combinations = (
            itertools.product((0, 1), repeat=4) if board == 2 else ((1, 0, 0, 0), (1, 0, 0, 0))
        )
        for index, (temperature, bus_current, motor_temperature, sincos) in enumerate(combinations):
            flags = [
                f"-DBSP_BOARD={board}",
                f"-DBSP_REQUIRED_MCU_TEMPERATURE={int(board == 1)}",
                f"-DBSP_ENABLED_MCU_TEMPERATURE={temperature}",
                f"-DBSP_ENABLED_BUS_CURRENT={bus_current}",
                f"-DBSP_ENABLED_MOTOR_TEMPERATURE={motor_temperature}",
                f"-DBSP_ENABLED_OUTPUT_SINCOS={sincos}",
            ]
            if board == 1:
                flags += [f"-DBSP_ENABLED_FAST_LOOP_PROFILE={index}"]
            exe = args.out / f"config_{board}_{index}.exe"
            subprocess.run(common + api + flags + [str(subject), "-o", str(exe)], check=True)
            subprocess.run([str(exe)], check=True)
            if board == 2:
                exe = args.out / f"aux_{index}.exe"
                sources = [native / "bsp_aux_configuration_test.c", native / "r40_io_fake/sdk.c"]
                includes = api + [
                    "-I",
                    str(native / "r40_io_fake"),
                    "-I",
                    str(platform / "bsp/mcu/at32f456"),
                    "-I",
                    str(ROOT / "firmware/platform/sdk/at32f45x/drivers/inc"),
                ]
                subprocess.run(
                    common + includes + flags + [str(p) for p in sources] + ["-o", str(exe)],
                    check=True,
                )
                subprocess.run([str(exe)], check=True)
    for index, (target, board, temperature, required, can) in enumerate(
        (
            (None, 1, 1, 1, 1),
            ("r40_app_off.h", 2, 1, 0, 1),
            ("r40_app_commissioning.h", 2, 0, 0, 1),
            (None, 2, 0, 0, 1),
            ("at32_motor_bench.h", 2, 0, 0, 0),
        )
    ):
        flags = [
            f"-DBSP_BOARD={board}",
            f"-DEXPECT_TEMPERATURE_ENABLED={temperature}",
            f"-DEXPECT_TEMPERATURE_REQUIRED={required}",
            f"-DEXPECT_CAN_ENABLED={can}",
        ]
        if target:
            flags += ["-include", str(ROOT / "firmware/app/config/targets" / target)]
        exe = args.out / f"target_{index}.exe"
        subprocess.run(common + api + flags + [str(subject), "-o", str(exe)], check=True)
        subprocess.run([str(exe)], check=True)
    for board in ("at32f456", "stm32g4"):
        exe = args.out / f"can_disabled_{board}.exe"
        sources = [
            native / "bsp_can_disabled_test.c",
            platform / "bsp/bsp_can.c",
        ]
        subprocess.run(
            common
            + api
            + [f"-DBSP_BOARD={2 if board == 'at32f456' else 1}", "-DBSP_ENABLED_CAN=0"]
            + [str(p) for p in sources]
            + ["-o", str(exe)],
            check=True,
        )
        subprocess.run([str(exe)], check=True)
    invalid = [
        ["BSP_BOARD=9"],
        ["BSP_BOARD=1", "FOC_ACTIVE_HW_PROFILE=2"],
        ["BSP_BOARD=1", "AT32F456xx=1"],
        ["BSP_BOARD=2", "STM32G431xx=1"],
        ["BSP_BOARD=1", "BSP_ENABLED_BUS_CURRENT=1"],
        ["BSP_BOARD=1", "BSP_ENABLED_MOTOR_TEMPERATURE=1"],
        ["BSP_BOARD=1", "BSP_ENABLED_OUTPUT_SINCOS=1"],
        ["BSP_BOARD=2", "BSP_ENABLED_OUTPUT_SINCOS=2"],
        ["BSP_ENABLED_PHASE_VOLTAGE=1"],
        ["BSP_BOARD=2", "BSP_ENABLED_FAST_LOOP_PROFILE=1"],
        ["BSP_ENABLED_PWM=2"],
        ["BSP_PWM_HARD_LOCK=2"],
        ["BSP_ENABLED_MCU_TEMPERATURE=0"],
        ["BSP_REQUIRED_MCU_TEMPERATURE=2"],
        ["BSP_BOARD=1", "BSP_REQUIRED_MCU_TEMPERATURE=0", "BSP_ENABLED_MCU_TEMPERATURE=0"],
        ["APP_TARGET_R40_F456=1", "BSP_BOARD=2", "BSP_REQUIRED_MCU_TEMPERATURE=0"],
        ["BSP_ENABLED_FAST_LOOP_PROFILE=2"],
        ["BSP_TARGET_MOTOR_ONLY=1"],
        ["BSP_TARGET_MOTOR_ONLY=1", "BSP_BOARD=2", "APP_TARGET_R40_F456=1"],
    ]
    for feature in ("PHASE_CURRENT", "BUS_VOLTAGE", "ROTOR_ENCODER", "PWM", "CAN"):
        invalid += [[f"BSP_ENABLED_{feature}=0"], [f"BSP_REQUIRED_{feature}=0"]]
    for index, flags in enumerate(invalid):
        result = subprocess.run(
            common
            + api
            + ["-D" + flag for flag in flags]
            + ["-c", str(subject), "-o", str(args.out / f"invalid_{index}.o")],
            capture_output=True,
            text=True,
        )
        (args.out / f"invalid_{index}.log").write_text(
            result.stdout + result.stderr, encoding="utf8"
        )
        if result.returncode == 0 or "error:" not in result.stderr:
            raise RuntimeError(f"非法能力配置未拒绝: {flags}")
    print(
        f"PASS 15 capability/target combinations, 8 actual ADC configurations, 2 CAN disabled implementations, {len(invalid)} compile rejections"
    )


if __name__ == "__main__":
    main()
