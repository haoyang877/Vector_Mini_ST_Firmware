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
/**
 * @brief 编码并尝试发送最近一次 2 kHz 快环遥测快照，单位和通道布局保持原协议。
 * @note 仅由低优先级 PendSV 调用，是 RTT 通道 1 的唯一写入者；不等待缓冲、不关中断。
 *       快环只发布 float 快照；服务读完前禁止覆盖，满槽或缓冲不足时整帧丢弃。
 */
void FOC_TelemetryService(void);
/** Run temperature conversion/protection from the existing 1 kHz supervisor. */
void FOC1kHzSupervisor(void);

#endif
