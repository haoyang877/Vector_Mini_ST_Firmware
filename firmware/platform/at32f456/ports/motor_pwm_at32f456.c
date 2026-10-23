#include "motor_hw.h"

#include "at32f45x.h"
#include "critical_hw.h"

/* TMR1 CH2/3/4对应A/B/C；CH1只作ADC触发。所有通道已在平台初始化时预装载。
 * 本端口只写比较值，不产生软件更新事件、不改变功率使能或定时器运行状态。 */
void motor_hw_pwm_set_duty(float duty_a, float duty_b, float duty_c)
{
    float period = (float)TMR1->pr;
    uint16_t a = (uint16_t)(duty_a * period);
    uint16_t b = (uint16_t)(duty_b * period);
    uint16_t c = (uint16_t)(duty_c * period);
    uint32_t mask = critical_hw_enter();
    TMR1->c2dt = a;
    TMR1->c3dt = b;
    TMR1->c4dt = c;
    critical_hw_exit(mask);
}

void motor_hw_pwm_set_phase_duty(MotorHwPwmPhase phase, float duty)
{
    uint16_t counts = (uint16_t)(duty * (float)TMR1->pr);
    switch (phase)
    {
    case MOTOR_HW_PWM_PHASE_A:
        TMR1->c2dt = counts;
        break;
    case MOTOR_HW_PWM_PHASE_B:
        TMR1->c3dt = counts;
        break;
    case MOTOR_HW_PWM_PHASE_C:
        TMR1->c4dt = counts;
        break;
    default:
        break;
    }
}

void motor_hw_pwm_force_high_sides(void)
{
    motor_hw_pwm_set_duty(1.0f, 1.0f, 1.0f);
}
