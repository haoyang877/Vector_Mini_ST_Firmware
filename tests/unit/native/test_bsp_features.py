"""验证当前固定板卡能力；电机配置独立，退役的逐项开关不再作为合法组合。"""

import subprocess
from pathlib import Path

from _bsp_test_support import NATIVE_INCLUDE_FLAGS, arguments, run_c

SOURCE = r"""
#include <assert.h>
#include "application_config.h"
#include "bsp.h"
#include "bsp_adc.h"
#include "bsp_can.h"
#include "bsp_encoder.h"
#include "bsp_pwm.h"
int main(void) {
    assert(BSP_ENABLED_PHASE_CURRENT && BSP_ENABLED_BUS_VOLTAGE);
    assert(BSP_ENABLED_PWM && BSP_ENABLED_CAN && BSP_ENABLED_ROTOR_ENCODER);
    assert(BSP_ENABLED_MCU_TEMPERATURE==(BSP_BOARD==1));
    assert(BSP_ENABLED_INTERNAL_ANALOG_SAMPLING==(BSP_BOARD==1));
    assert(BSP_ENABLED_BUS_CURRENT==(BSP_BOARD==2));
    assert(BSP_ENABLED_MOTOR_TEMPERATURE==(BSP_BOARD==2));
    assert(BSP_ENABLED_LINEAR_HALL==(BSP_BOARD==2));
    assert(BSP_REQUIRED_MCU_TEMPERATURE==(BSP_BOARD==1));
    assert(PARAM_MOTOR_POLE_PAIRS==(FOC_ACTIVE_MOTOR_PROFILE==1 ? 21 : 14));
    return 0;
}
"""


def main() -> None:
    """编译两板乘两电机矩阵，拒绝缺失板卡、未知板卡或未知电机。"""
    args = arguments(__doc__)
    for board in (1, 2):
        for profile in (1, 2):
            run_c(
                args,
                f"features_{board}_{profile}",
                f"#define FOC_ACTIVE_MOTOR_PROFILE {profile}\n" + SOURCE,
                board,
            )
    fixture = args.out / "invalid.c"
    fixture.write_text(SOURCE, encoding="utf-8")
    compiler = [args.cc] + (["cc"] if Path(args.cc).stem.lower() == "zig" else [])
    for index, flags in enumerate(
        ([], ["-DBSP_BOARD=9"], ["-DBSP_BOARD=1", "-DFOC_ACTIVE_MOTOR_PROFILE=9"])
    ):
        result = subprocess.run(
            compiler
            + NATIVE_INCLUDE_FLAGS
            + [
                "-std=c11",
                "-Werror",
                *flags,
                "-c",
                str(fixture),
                "-o",
                str(args.out / f"invalid_{index}.o"),
            ],
            capture_output=True,
            text=True,
        )
        (args.out / f"invalid_{index}.log").write_text(
            result.stdout + result.stderr, encoding="utf-8"
        )
        if result.returncode == 0 or "error:" not in result.stderr:
            raise RuntimeError(f"非法配置未拒绝: {flags}")
    print(
        "PASS fixed board capabilities, independent motor selection, invalid configuration rejection"
    )


if __name__ == "__main__":
    main()
