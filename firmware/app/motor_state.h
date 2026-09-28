#ifndef MOTOR_STATE_H
#define MOTOR_STATE_H
#include <stdbool.h>
#include "data_type.h"
#include "motor_status.h"
#include "app_lifecycle.h"
/**
 * @brief 装载参数并初始化唯一电机执行对象。
 * @note 仅启动组合层在中断启用前调用一次，不自动使能。
 */
void MotorControl_Init(void);
/**
 * @brief 执行快速采样、控制和结果发布。
 * @note 20 kHz ADC2 JEOS 上下文，执行有界。
 */
void MotorControl_FastTick(void);
/**
 * @brief 执行慢估计、外环和生命周期监督。
 * @return 本次提交后的模式，仅用于指示器。
 * @note 2 kHz TIM7 上下文。
 */
ModeNow_TypeDef MotorControl_SlowTick(void);
/**
 * @brief 采集一致的只读电机测量值。
 * @param sample 调用方快照空间，不保留指针。
 * @param valid_bits 接收既有测量有效位。
 * @return 关联的生命周期状态。
 * @note 前台调用；内部短临界区，不暴露控制对象地址。
 */
AppLifecycleState MotorControl_ReadStatus(MotorStatus *sample, uint8_t *valid_bits);
/**
 * @brief 执行受理的停机前台维护。
 * @note 主循环调用；表整理和 Flash 操作禁止在 ISR 执行。
 */
void MotorControl_ServiceBackground(void);
#endif
