/**
 * @file pwm.c
 * @brief STM32G4三相PWM初始化、寄存器读写和输出启停。
 */
#include "../../bsp_internal.h"
#include "bsp_pwm.h"
#include "mcu_resources.h"
#include "tim.h"

void McuPwm_Init(void)
{
    MCU_PWM_INIT();
}

void BspPwm_SetDuty(float phase_a, float phase_b, float phase_c)
{
    float period_counts = (float)MCU_PWM_TIMER->ARR;
    MCU_PWM_PHASE_A_COMPARE = (uint16_t)(phase_a * period_counts);
    MCU_PWM_PHASE_B_COMPARE = (uint16_t)(phase_b * period_counts);
    MCU_PWM_PHASE_C_COMPARE = (uint16_t)(phase_c * period_counts);
}

void BspPwm_SetPhaseDuty(BspPwmPhase phase, float duty)
{
    uint16_t counts = (uint16_t)(duty * (float)MCU_PWM_TIMER->ARR);
    switch (phase)
    {
    case BSP_PWM_PHASE_A:
        MCU_PWM_PHASE_A_COMPARE = counts;
        break;
    case BSP_PWM_PHASE_B:
        MCU_PWM_PHASE_B_COMPARE = counts;
        break;
    case BSP_PWM_PHASE_C:
        MCU_PWM_PHASE_C_COMPARE = counts;
        break;
    }
}

void BspPwm_Enable(void)
{
    /* 六路相通道及互补输出，最后开放主输出。 */
    MCU_PWM_TIMER->CCER |= MCU_PWM_PHASE_CHANNEL_MASK;
    MCU_PWM_TIMER->BDTR |= TIM_BDTR_MOE;
}

void BspPwm_Disable(void)
{
    MCU_PWM_TIMER->BDTR &= ~TIM_BDTR_MOE;
    MCU_PWM_TIMER->CCER &= ~MCU_PWM_PHASE_CHANNEL_MASK;
}

bool BspPwm_ArePhasesDisabled(void)
{
    return (MCU_PWM_TIMER->CCER & MCU_PWM_PHASE_CHANNEL_MASK) == 0U;
}
