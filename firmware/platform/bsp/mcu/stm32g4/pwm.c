/**
 * @file pwm.c
 * @brief STM32G4三相PWM初始化、寄存器读写和输出启停。
 */
#include "../../bsp_internal.h"
#include "bsp_pwm.h"
#include "board_config.h"
#include "tim.h"

#define PWM_PHASE_CHANNEL_MASK                                                                     \
    (TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC3E |             \
     TIM_CCER_CC3NE)

void McuPwm_Init(void)
{
    MX_TIM1_Init();
}

void BspPwm_SetDuty(float phase_a, float phase_b, float phase_c)
{
    float period_counts = (float)TIM1->ARR;
    TIM1->CCR1 = (uint16_t)(phase_a * period_counts);
    TIM1->CCR2 = (uint16_t)(phase_b * period_counts);
    TIM1->CCR3 = (uint16_t)(phase_c * period_counts);
}

void BspPwm_SetPhaseDuty(BspPwmPhase phase, float duty)
{
    uint16_t counts = (uint16_t)(duty * (float)TIM1->ARR);
    switch (phase)
    {
    case BSP_PWM_PHASE_A:
        TIM1->CCR1 = counts;
        break;
    case BSP_PWM_PHASE_B:
        TIM1->CCR2 = counts;
        break;
    case BSP_PWM_PHASE_C:
        TIM1->CCR3 = counts;
        break;
    }
}

void BspPwm_Enable(void)
{
    /* 六路相通道及互补输出，最后开放主输出。 */
    TIM1->CCER |= PWM_PHASE_CHANNEL_MASK;
    TIM1->BDTR |= TIM_BDTR_MOE;
}

void BspPwm_Disable(void)
{
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    TIM1->CCER &= ~PWM_PHASE_CHANNEL_MASK;
}

bool BspPwm_ArePhasesDisabled(void)
{
    return (TIM1->CCER & PWM_PHASE_CHANNEL_MASK) == 0U;
}
