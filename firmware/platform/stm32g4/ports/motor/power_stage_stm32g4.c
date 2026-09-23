#include "power_stage_hw.h"

#include "tim.h"

/* 功率级输出端口：唯一直接调用 TIM1 HAL 启停接口的实现。
 * 六路调用顺序与去耦前的 foc_errhandle.c 完全一致；HAL 启停会同步切换
 * MOE 主输出使能，调整顺序会改变输出切换过程中的中间状态。 */

void power_stage_hw_stop(void)
{
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_OCN_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_OCN_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_OCN_Stop(&htim1, TIM_CHANNEL_3);
}

void power_stage_hw_start(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_OCN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_OCN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_OCN_Start(&htim1, TIM_CHANNEL_3);
}

bool power_stage_hw_phases_disabled(void)
{
    const uint32_t phase_enable_mask = TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E |
                                       TIM_CCER_CC2NE | TIM_CCER_CC3E | TIM_CCER_CC3NE;
    return (htim1.Instance->CCER & phase_enable_mask) == 0U;
}
