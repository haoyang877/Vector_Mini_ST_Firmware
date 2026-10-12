#ifndef __FOC_TASK_H__
#define __FOC_TASK_H__

#include <stdbool.h>
#include "speed_feedforward.h"

void MotorControl_Init(void);
/** Startup readiness for board glue; querying does not enable the motor. */
bool MotorControl_IsConfigurationValid(void);
/** @brief 在停机且已关相时更新速度前馈配置并清除规划状态。
 * @param config 完整待应用配置，非空；单位见SpeedFeedforwardConfig。
 * @return 已应用返回true；运行、故障、待切档或非法配置返回false。
 * @note CAN参数中断调用；不使能电机、不保存Flash、不修改PI增益。
 */
bool MotorControl_SetSpeedFeedforward(const SpeedFeedforwardConfig *config);
void FOC20kHzIRQHandler(void);
/** Run temperature conversion/protection from the existing 1 kHz supervisor. */
void FOC1kHzSupervisor(void);

#endif
