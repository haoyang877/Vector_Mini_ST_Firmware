#ifndef MOTOR_EXECUTION_H
#define MOTOR_EXECUTION_H

#include "motor_control_command.h"
#include "motor_control_execution.h"

/* APP控制目标准入接口；算法与运行状态通过motor执行接口显式传递。 */

/**
 * @brief 验证控制目标格式及当前参数允许的数值范围，不决定运行许可。
 * @param configuration 只读活动配置。
 * @param command SI 值命令；未使用字段须为零，所有浮点字段须有限。
 * @param foc 只读母线滤波电压，单位 V。
 * @param encoder 只读当前机械位置，单位 rad。
 * @return 格式错误返回 INVALID，越界返回 OUT_OF_RANGE，否则 ACCEPTED。
 * @note 不修改对象、不访问硬件；调用者同步配置和测量，另行检查故障、状态与启相许可。
 */
MotorControlResult MotorExecution_ValidateTarget(const MotorConfiguration *configuration,
                                                 const MotorControlCommand *command,
                                                 const FocState *foc,
                                                 const EncoderState *encoder);

#endif
