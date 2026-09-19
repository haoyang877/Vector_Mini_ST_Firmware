#ifndef MOTOR_HW_H
#define MOTOR_HW_H

/**
 * @brief 在采样开始前初始化外环延迟执行上下文。
 * @note 快速电机中断必须能够抢占此上下文；本接口不包含控制算法，也不使能电机。
 */
void motor_hw_outer_init(void);
/**
 * @brief 请求执行一次外环服务，不等待执行完成。
 * @note 调用方拥有单任务邮箱；前一次完成被消费前不得再次调度。
 */
void motor_hw_outer_schedule(void);
/**
 * @brief 在快速中断与延迟上下文之间发布或获取共享电机数据。
 * @note 仅提供编译器和硬件内存屏障，不是中断锁，也不建立多写者互斥。
 */
void motor_hw_outer_barrier(void);

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
