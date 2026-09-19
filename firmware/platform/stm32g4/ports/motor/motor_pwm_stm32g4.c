#include "motor_hw.h"

#include "hw_conf.h"

/* 三相 PWM 输出端口：实现 platform/api/motor_hw.h 的 PWM 契约，只写 TIM1
 * 比较寄存器，不改变定时器与功率级使能状态。占空比按 hw_conf.h 的
 * PWM_TIM_PERIOD 折算为计数值；数值有限性由调用方保证。 */

void motor_hw_pwm_set_duty(float dtc_a, float dtc_b, float dtc_c)
{
    TIM1->CCR1 = (uint16_t)(dtc_a * PWM_TIM_PERIOD);
    TIM1->CCR2 = (uint16_t)(dtc_b * PWM_TIM_PERIOD);
    TIM1->CCR3 = (uint16_t)(dtc_c * PWM_TIM_PERIOD);
}

void motor_hw_pwm_set_phase_duty(MotorHwPwmPhase phase, float duty)
{
    switch (phase)
    {
    case MOTOR_HW_PWM_PHASE_A:
        TIM1->CCR1 = (uint16_t)(duty * PWM_TIM_PERIOD);
        break;
    case MOTOR_HW_PWM_PHASE_B:
        TIM1->CCR2 = (uint16_t)(duty * PWM_TIM_PERIOD);
        break;
    case MOTOR_HW_PWM_PHASE_C:
        TIM1->CCR3 = (uint16_t)(duty * PWM_TIM_PERIOD);
        break;
    default:
        break;
    }
}
