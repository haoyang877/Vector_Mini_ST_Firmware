"""离线对照实际 HAL 模拟模式与六路功率引脚高阻端口，不访问硬件。"""

import argparse
from pathlib import Path
import subprocess
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
from project_paths import ROOT, NATIVE_INCLUDE_FLAGS
from run_position_servo_tests import function_source


PRELUDE = r'''
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "motor_hw.h"
static GPIO_TypeDef gpioa, gpiob;
static TIM_TypeDef timer;
#undef GPIOA
#undef GPIOB
#undef TIM1
#define GPIOA (&gpioa)
#define GPIOB (&gpiob)
#define TIM1 (&timer)
'''

CASES = r'''
static uint32_t random_state = 0x13579BDFU;
static void random_bytes(void *object, size_t bytes)
{
    unsigned char *out = object;
    for (size_t i = 0; i < bytes; ++i)
    {
        random_state = random_state * 1664525U + 1013904223U;
        out[i] = (unsigned char)(random_state >> 24);
    }
}

static void check_case(void)
{
    GPIO_TypeDef expected_a, expected_b, before_a, before_b;
    TIM_TypeDef before_timer;
    GPIO_InitTypeDef config = {0};
    const uint32_t mask_a = (3U << 14) | (3U << 16) | (3U << 18) | (3U << 20);
    const uint32_t mask_b = 15U;
    memcpy(&before_a, &gpioa, sizeof(gpioa));
    memcpy(&before_b, &gpiob, sizeof(gpiob));
    memcpy(&before_timer, &timer, sizeof(timer));
    memcpy(&expected_a, &gpioa, sizeof(gpioa));
    memcpy(&expected_b, &gpiob, sizeof(gpiob));

    /* 参考路径执行仓库内原版 HAL，而非复制待测位运算。 */
    config.Pin = PWM_AL_Pin | PWM_AH_Pin | PWM_BH_Pin | PWM_CH_Pin;
    config.Mode = GPIO_MODE_ANALOG;
    config.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(&expected_a, &config);
    config.Pin = PWM_BL_Pin | PWM_CL_Pin;
    HAL_GPIO_Init(&expected_b, &config);
    PWM_Outputs_HiZ();
    assert(memcmp(&gpioa, &expected_a, sizeof(gpioa)) == 0);
    assert(memcmp(&gpiob, &expected_b, sizeof(gpiob)) == 0);
    assert(memcmp(&timer, &before_timer, sizeof(timer)) == 0);
    assert((gpioa.MODER & mask_a) == mask_a && (gpiob.MODER & mask_b) == mask_b);
    assert((gpioa.MODER & ~mask_a) == (before_a.MODER & ~mask_a));
    assert((gpiob.MODER & ~mask_b) == (before_b.MODER & ~mask_b));

    /* 重复释放幂等；定时器启停、CH4 与 MOE 不归此接口修改。 */
    PWM_Outputs_HiZ();
    assert(memcmp(&gpioa, &expected_a, sizeof(gpioa)) == 0);
    assert(memcmp(&gpiob, &expected_b, sizeof(gpiob)) == 0);
    assert(memcmp(&timer, &before_timer, sizeof(timer)) == 0);
}

int main(void)
{
    unsigned cases = 0U;
    for (unsigned modes = 0U; modes < 4096U; ++modes)
    {
        random_bytes(&gpioa, sizeof(gpioa));
        random_bytes(&gpiob, sizeof(gpiob));
        random_bytes(&timer, sizeof(timer));
        gpioa.MODER = (gpioa.MODER & ~(255U << 14)) | ((modes & 255U) << 14);
        gpiob.MODER = (gpiob.MODER & ~15U) | (modes >> 8);
        check_case();
        ++cases;
    }
    for (unsigned i = 0U; i < 20000U; ++i)
    {
        random_bytes(&gpioa, sizeof(gpioa));
        random_bytes(&gpiob, sizeof(gpiob));
        random_bytes(&timer, sizeof(timer));
        check_case();
        ++cases;
    }
    printf("PASS %u cases: actual HAL equivalence, six pins only, unrelated GPIO/TIM preserved, idempotent\n", cases);
    return 0;
}
'''


def main() -> None:
    """编译并执行生产端口和旧 HAL 路径的寄存器对照。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", required=True)
    parser.add_argument("--out", type=Path, default=ROOT / "outputs/motor_gpio_hiz/host")
    args = parser.parse_args()
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    platform = (ROOT / "firmware/platform/stm32g4/ports/motor/motor_current_stm32g4.c").read_text(encoding="utf-8")
    errors = (ROOT / "firmware/motor/protection/foc_errhandle.c").read_text(encoding="utf-8")
    cubemx = ROOT / "firmware/platform/stm32g4/cubemx"
    vendor = (cubemx / "Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_gpio.c").read_text(encoding="utf-8")
    fixture = PRELUDE + "\n".join([
        function_source(vendor, "HAL_GPIO_Init"),
        function_source(platform, "motor_hw_phase_outputs_high_impedance"),
        function_source(errors, "PWM_Outputs_HiZ"),
    ]) + CASES
    source = out / "motor_gpio_hiz.c"
    source.write_text(fixture, encoding="utf-8")
    compiler = [args.cc] + (["cc"] if Path(args.cc).stem == "zig" else [])
    command = compiler + NATIVE_INCLUDE_FLAGS + [
        "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror",
        "-Wno-pointer-to-int-cast", "-Wno-int-to-pointer-cast",
        "-DSTM32G431xx", "-DUSE_HAL_DRIVER",
    ]
    for path in ("Core/Inc", "Drivers/STM32G4xx_HAL_Driver/Inc",
                 "Drivers/CMSIS/Device/ST/STM32G4xx/Include", "Drivers/CMSIS/Include"):
        command += ["-I", str(cubemx / path)]
    exe = out / "motor_gpio_hiz.exe"
    subprocess.run(command + [str(source), "-o", str(exe)], cwd=ROOT, check=True)
    subprocess.run([str(exe)], cwd=ROOT, check=True)


if __name__ == "__main__":
    main()
