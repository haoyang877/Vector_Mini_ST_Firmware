#ifndef MOTOR_LIFECYCLE_EXCHANGE_H
#define MOTOR_LIFECYCLE_EXCHANGE_H

#include "motor_lifecycle.h"

/* 生命周期模块内部接口；板级、协议与标定使用 motor_lifecycle.h。 */

/**
 * @brief 发布本次外环的有效字段及启相意图，作废上一次未消费的结果。
 * @param life 当前轴。
 * @param work 属于此轴的本拍结果。
 * @param update_control 是否提交电流参考和速度 PI。
 * @param request_phase_enable 是否请求 ADC 复核后启相。
 * @note 由外环单写。两个标志均为 false 时只撤销旧结果。
 */
void MotorLifecycle_PublishOuterResult(MotorLifecycle *life,
                                       const MotorOuterWork *work,
                                       bool update_control,
                                       bool request_phase_enable);

/**
 * @brief 在 ADC 拍首消费本轴的外环结果和控制目标。
 * @param life 当前轴。
 * @return 待生命周期复核的启相会话号，无有效请求时返回零。
 * @note 只在最高优先级 ADC 中执行；过期结果不会恢复输出。
 */
uint32_t MotorLifecycle_ApplyPendingControl(MotorLifecycle *life);

#endif
