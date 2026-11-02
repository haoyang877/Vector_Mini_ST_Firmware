"""编译真实热纯核及当前关闭热调度的兼容入口；不强开退役运行功能。"""

from _bsp_test_support import ROOT, arguments, run_c


def main() -> None:
    """保留物理配置、热历史和瞬时电压模式限流保护检查。"""
    args = arguments(__doc__)
    sources = [
        ROOT / path
        for path in (
            "tests/unit/motor_thermal_test.c",
            "firmware/motor/protection/motor_thermal.c",
            "firmware/motor/protection/motor_thermal_config.c",
            "firmware/motor/protection/motor_thermal_control.c",
        )
    ]
    run_c(
        args,
        "motor_thermal",
        "#define FOC_ACTIVE_MOTOR_PROFILE 1\n",
        1,
        ["-DFOC_ACTIVE_MOTOR_PROFILE=1", *sources],
    )
    run_c(
        args,
        "unconfigured_thermal",
        r"""
#include <assert.h>
#include <stdio.h>
#include "motor_thermal_config.h"
int main(void) {
    MotorWindingThermalConfig config;
    MotorThermalConfig derived;
    MotorWindingThermal_Default(&config);
    assert(config.continuous_vector_a==0 && config.peak_a==0);
    assert(!MotorWindingThermal_Derive(&config,&derived));
    puts("PASS YQ missing thermal inputs rejected; no invented calibration defaults");
    return 0;
}
""",
        2,
        [
            ROOT / "firmware/motor/protection/motor_thermal_config.c",
            ROOT / "firmware/motor/protection/motor_thermal.c",
        ],
    )


if __name__ == "__main__":
    main()
