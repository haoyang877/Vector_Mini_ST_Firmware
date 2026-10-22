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

/* 固定TIM1的HAL成功分支；每步仍先清再置通道位、置MOE，再按完整SMS决定CEN。
 * 调用方在原临界区内启相；不合并六步，不改变Break、CH4或HAL全局State/Lock。 */
__STATIC_FORCEINLINE void power_stage_tim1_start_channel(
    volatile HAL_TIM_ChannelStateTypeDef *channel_state, uint32_t enable_mask)
{
    *channel_state = HAL_TIM_CHANNEL_STATE_BUSY;
    TIM1->CCER &= ~enable_mask;
    TIM1->CCER |= enable_mask;
    TIM1->BDTR |= TIM_BDTR_MOE;
    uint32_t slave_mode = TIM1->SMCR & TIM_SMCR_SMS;
    if (!IS_TIM_SLAVEMODE_TRIGGER_ENABLED(slave_mode))
    {
        TIM1->CR1 |= TIM_CR1_CEN;
    }
}

void power_stage_hw_start(void)
{
    if (htim1.Instance == TIM1 &&
        htim1.ChannelState[0] == HAL_TIM_CHANNEL_STATE_READY &&
        htim1.ChannelState[1] == HAL_TIM_CHANNEL_STATE_READY &&
        htim1.ChannelState[2] == HAL_TIM_CHANNEL_STATE_READY &&
        htim1.ChannelNState[0] == HAL_TIM_CHANNEL_STATE_READY &&
        htim1.ChannelNState[1] == HAL_TIM_CHANNEL_STATE_READY &&
        htim1.ChannelNState[2] == HAL_TIM_CHANNEL_STATE_READY)
    {
        power_stage_tim1_start_channel(&htim1.ChannelState[0], TIM_CCER_CC1E);
        power_stage_tim1_start_channel(&htim1.ChannelState[1], TIM_CCER_CC2E);
        power_stage_tim1_start_channel(&htim1.ChannelState[2], TIM_CCER_CC3E);
        power_stage_tim1_start_channel(&htim1.ChannelNState[0], TIM_CCER_CC1NE);
        power_stage_tim1_start_channel(&htim1.ChannelNState[1], TIM_CCER_CC2NE);
        power_stage_tim1_start_channel(&htim1.ChannelNState[2], TIM_CCER_CC3NE);
        return;
    }
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
