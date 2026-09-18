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

#endif
