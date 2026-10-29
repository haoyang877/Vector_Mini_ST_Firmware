#include "bsp_config.h"
#include "bsp_pwm.h"
#include "tim.h"
#include "control_config.h"
#define PWM_TIM_PERIOD (170000000 / FOC_FREQ / 2)
#include "bsp_system.h"
#include "bsp_internal.h"

#if BSP_PWM_PHASE_A_CHANNEL == 1
#define PWM_COMPARE_A TIM1->CCR1
#elif BSP_PWM_PHASE_A_CHANNEL == 2
#define PWM_COMPARE_A TIM1->CCR2
#elif BSP_PWM_PHASE_A_CHANNEL == 3
#define PWM_COMPARE_A TIM1->CCR3
#elif BSP_PWM_PHASE_A_CHANNEL == 4
#define PWM_COMPARE_A TIM1->CCR4
#else
#error "Unsupported PWM compare channel"
#endif
#if BSP_PWM_PHASE_B_CHANNEL == 1
#define PWM_COMPARE_B TIM1->CCR1
#elif BSP_PWM_PHASE_B_CHANNEL == 2
#define PWM_COMPARE_B TIM1->CCR2
#elif BSP_PWM_PHASE_B_CHANNEL == 3
#define PWM_COMPARE_B TIM1->CCR3
#elif BSP_PWM_PHASE_B_CHANNEL == 4
#define PWM_COMPARE_B TIM1->CCR4
#else
#error "Unsupported PWM compare channel"
#endif
#if BSP_PWM_PHASE_C_CHANNEL == 1
#define PWM_COMPARE_C TIM1->CCR1
#elif BSP_PWM_PHASE_C_CHANNEL == 2
#define PWM_COMPARE_C TIM1->CCR2
#elif BSP_PWM_PHASE_C_CHANNEL == 3
#define PWM_COMPARE_C TIM1->CCR3
#elif BSP_PWM_PHASE_C_CHANNEL == 4
#define PWM_COMPARE_C TIM1->CCR4
#else
#error "Unsupported PWM compare channel"
#endif

/* 定时器原有寄存器顺序和HAL状态更新保持。 */
__STATIC_FORCEINLINE void pwm_start_channel(volatile HAL_TIM_ChannelStateTypeDef *channel_state,
                                            uint32_t enable_mask)
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
void McuPwm_EnableOutputs(void)
{
    if (htim1.Instance == TIM1 && htim1.ChannelState[0] == HAL_TIM_CHANNEL_STATE_READY &&
        htim1.ChannelState[1] == HAL_TIM_CHANNEL_STATE_READY &&
        htim1.ChannelState[2] == HAL_TIM_CHANNEL_STATE_READY &&
        htim1.ChannelNState[0] == HAL_TIM_CHANNEL_STATE_READY &&
        htim1.ChannelNState[1] == HAL_TIM_CHANNEL_STATE_READY &&
        htim1.ChannelNState[2] == HAL_TIM_CHANNEL_STATE_READY)
    {
        pwm_start_channel(&htim1.ChannelState[0], TIM_CCER_CC1E);
        pwm_start_channel(&htim1.ChannelState[1], TIM_CCER_CC2E);
        pwm_start_channel(&htim1.ChannelState[2], TIM_CCER_CC3E);
        pwm_start_channel(&htim1.ChannelNState[0], TIM_CCER_CC1NE);
        pwm_start_channel(&htim1.ChannelNState[1], TIM_CCER_CC2NE);
        pwm_start_channel(&htim1.ChannelNState[2], TIM_CCER_CC3NE);
        return;
    }
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_OCN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_OCN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_OCN_Start(&htim1, TIM_CHANNEL_3);
}

void McuPwm_DisableOutputs(void)
{
    if (htim1.Instance != TIM1)
    {
        TIM1->BDTR &= ~TIM_BDTR_MOE;
        TIM1->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE |
                        TIM_CCER_CC3E | TIM_CCER_CC3NE);
        return;
    }
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_OCN_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_OCN_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_OCN_Stop(&htim1, TIM_CHANNEL_3);
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    TIM1->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE |
                    TIM_CCER_CC3E | TIM_CCER_CC3NE);
}
bool BspPwm_ArePhasesDisabled(void)
{
    const uint32_t phase_enable_mask = TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E |
                                       TIM_CCER_CC2NE | TIM_CCER_CC3E | TIM_CCER_CC3NE;
    return (TIM1->CCER & phase_enable_mask) == 0U;
}
float McuPwm_PeriodCounts(void)
{
    return PWM_TIM_PERIOD;
}
/* 仅运行/部分输出路径检查期限；剩余计数复用原计数寄存器值，减少热路径活跃变量。 */
static inline bool pwm_can_commit(uint32_t control)
{
    if ((control & TIM_CR1_CEN) == 0U)
    {
        return true;
    }
    uint32_t remaining = TIM1->CNT, period = TIM1->ARR;
    if ((control & TIM_CR1_DIR) == 0U)
    {
        remaining = period - remaining;
    }
    /* 上数时原count>period会无符号回绕为大于period；两方向共用同一范围检查。 */
    return remaining <= period && remaining > BSP_PWM_COMMIT_GUARD_COUNTS;
}

bool McuPwm_CommitThree(uint16_t a, uint16_t b, uint16_t c)
{
    if (BspPwm_ArePhasesDisabled() && (TIM1->BDTR & TIM_BDTR_MOE) == 0U)
    {
        /* 关相预置单独完成更新屏蔽/恢复，不把该状态带入正常提交路径。 */
        uint32_t control = TIM1->CR1;
        TIM1->CR1 = control | (1U << 1U);
        PWM_COMPARE_A = a;
        PWM_COMPARE_B = b;
        PWM_COMPARE_C = c;
        TIM1->CR1 = control;
        return true;
    }
    if (!pwm_can_commit(TIM1->CR1))
    {
        return false;
    }
    PWM_COMPARE_A = a;
    PWM_COMPARE_B = b;
    PWM_COMPARE_C = c;
    return true;
}

bool McuPwm_CommitPhase(BspPwmPhase phase, uint16_t counts)
{
    if (!(BspPwm_ArePhasesDisabled() && (TIM1->BDTR & TIM_BDTR_MOE) == 0U) &&
        !pwm_can_commit(TIM1->CR1))
    {
        return false;
    }
    switch (phase)
    {
    case BSP_PWM_PHASE_A:
        PWM_COMPARE_A = counts;
        break;
    case BSP_PWM_PHASE_B:
        PWM_COMPARE_B = counts;
        break;
    case BSP_PWM_PHASE_C:
        PWM_COMPARE_C = counts;
        break;
    default:
        return false;
    }
    return true;
}

void McuPwm_ReadOutputs(bool *any_channel, bool *all_channels, bool *master)
{
    const uint32_t mask = TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE |
                          TIM_CCER_CC3E | TIM_CCER_CC3NE;
    uint32_t channels = TIM1->CCER & mask;
    *any_channel = channels != 0U;
    *all_channels = channels == mask;
    *master = (TIM1->BDTR & TIM_BDTR_MOE) != 0U;
}

/* 上电配置与运行操作同属本定时器实现，保持原校准/使能顺序。 */
void McuPwm_Init(void)
{
    MX_TIM1_Init();
}

void McuPwm_Fatal(void)
{
    __disable_irq();
    for (;;)
    {
        __WFI();
    }
}
