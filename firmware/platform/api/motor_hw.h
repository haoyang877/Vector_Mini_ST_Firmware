#ifndef MOTOR_HW_H
#define MOTOR_HW_H

/** PWM 输出相选择。 */
typedef enum
{
    MOTOR_HW_PWM_PHASE_A = 0,
    MOTOR_HW_PWM_PHASE_B,
    MOTOR_HW_PWM_PHASE_C
} MotorHwPwmPhase;

/**
 * @brief 同时更新三相互补 PWM 占空比。
 * @param dtc_a A 相占空比，范围 0.0~1.0。
 * @param dtc_b B 相占空比，范围 0.0~1.0。
 * @param dtc_c C 相占空比，范围 0.0~1.0。
 * @note 只写定时器比较寄存器，不改变定时器与功率级使能状态；数值有限性由调用方保证。
 */
void motor_hw_pwm_set_duty(float dtc_a, float dtc_b, float dtc_c);

/**
 * @brief 更新单相 PWM 占空比，其他两相保持不变。
 * @param phase 目标相，取 MotorHwPwmPhase。
 * @param duty 占空比，范围 0.0~1.0。
 * @note 供标定等逐相操作使用；常规控制环应先算完三相再调用三相版本。
 */
void motor_hw_pwm_set_phase_duty(MotorHwPwmPhase phase, float duty);

/**
 * @brief 三相上桥臂常开，占空比 1.0。
 * @note 供标定短接测试使用；只写比较寄存器，不改变定时器与驱动使能状态。
 */
void motor_hw_pwm_force_high_sides(void);

#endif
