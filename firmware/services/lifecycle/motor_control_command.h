#ifndef MOTOR_CONTROL_COMMAND_H
#define MOTOR_CONTROL_COMMAND_H

#include <stdint.h>

#include "motor_control_mode.h"

/** @brief 已转换到 SI 单位的单轴值命令；未使用的目标字段必须为零。 */
typedef struct
{
    MotorControlMode mode;
    float position_rad;
    float speed_rad_s;
    float iq_a;
    float vq_v;
    /* 来源 0 表示本地调用；身份不参与控制计算，也不是存储 ABI。 */
    uint8_t source_node;
    uint16_t sequence;
} MotorControlCommand;

/** @brief 目标提交结果；ACCEPTED 仅表示已复制到控制所有者的邮箱。 */
typedef enum
{
    MOTOR_CONTROL_ACCEPTED = 0,
    MOTOR_CONTROL_INVALID,
    MOTOR_CONTROL_DENIED,
    MOTOR_CONTROL_BUSY,
    MOTOR_CONTROL_OUT_OF_RANGE,
    MOTOR_CONTROL_FAULT,
    MOTOR_CONTROL_STALE_SEQUENCE,
    MOTOR_CONTROL_CONFLICTING_RETRY
} MotorControlResult;

/** @brief 单槽控制请求的实际进展；APPLIED 仅证明目标已进入控制参考。 */
typedef enum
{
    MOTOR_CONTROL_PROGRESS_NONE = 0,
    MOTOR_CONTROL_PROGRESS_PENDING,
    MOTOR_CONTROL_PROGRESS_APPLIED,
    MOTOR_CONTROL_PROGRESS_REJECTED
} MotorControlProgressState;

/** @brief 控制所有者发布的只读身份与终态，不代表机械到位或功率波形验证。 */
typedef struct
{
    MotorControlProgressState state;
    MotorControlResult result;
    uint8_t source_node;
    uint16_t sequence;
} MotorControlProgress;

/**
 * @brief 控制所有者目标提交端口；返回前必须复制命令，不得保留指针。
 * @param context 组合层注入的控制所有者上下文。
 * @param command SI 单位命令；不得隐式使能或清故障。
 * @return 拒绝原因或 ACCEPTED；受理不表示目标已应用或电机已到位。
 * @note 前台串行调用，非阻塞；控制周期再次检查状态与限值后才能应用。
 */
typedef MotorControlResult (*MotorControlSubmit)(void *context, const MotorControlCommand *command);

#endif
