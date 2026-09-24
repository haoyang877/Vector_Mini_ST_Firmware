/** @file
 * @brief STM32G4 电流采样扇区适配，只读上一拍 PWM 比较值。
 * @note 依赖当前中心对齐 PWM 的预装载与 ADC 快中断写入顺序。
 */
#include "motor_hw.h"
#include "stm32g4xx.h"

unsigned motor_hw_current_sample_sector(void)
{
    const uint32_t phase_enable_mask = TIM_CCER_CC1E | TIM_CCER_CC1NE |
                                       TIM_CCER_CC2E | TIM_CCER_CC2NE |
                                       TIM_CCER_CC3E | TIM_CCER_CC3NE;
    uint32_t compare_a;
    uint32_t compare_b;
    uint32_t compare_c;
    uint32_t period;

    if ((TIM1->CCER & phase_enable_mask) != phase_enable_mask ||
        (TIM1->BDTR & TIM_BDTR_MOE) == 0U)
    {
        return 0U;
    }

    /* PWM 仅由快中断写入：上一拍 CCR 在计数谷值已装载，本拍尚未写入。
     * CCR 读回本身不是影子值接口；更改调度/预装载时必须重新验证此约束。 */
    compare_a = TIM1->CCR1;
    compare_b = TIM1->CCR2;
    compare_c = TIM1->CCR3;
    period = TIM1->ARR;
    if (period == 0U || compare_a > period || compare_b > period ||
        compare_c > period || (compare_a == compare_b && compare_b == compare_c))
    {
        return 0U;
    }

    /* 当前 SVM：1/2 的 C、3/4 的 A、5/6 的 B 占空比最大。
     * 边界并列时优先舍弃采样较晚的相：C、B、A。 */
    if (compare_c >= compare_a && compare_c >= compare_b)
    {
        return compare_a <= compare_b ? 1U : 2U;
    }
    if (compare_b >= compare_a && compare_b >= compare_c)
    {
        return compare_a <= compare_c ? 6U : 5U;
    }
    return compare_b <= compare_c ? 3U : 4U;
}
