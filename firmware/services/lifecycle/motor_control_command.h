#ifndef MOTOR_CONTROL_COMMAND_H
#define MOTOR_CONTROL_COMMAND_H

#include <stdint.h>

/** @brief 电机业务模式，独立于线路编号和历史 ModeNow 枚举。 */
typedef enum
{
    MOTOR_CONTROL_CURRENT = 0,
    MOTOR_CONTROL_SPEED,
    MOTOR_CONTROL_POSITION,
    MOTOR_CONTROL_VQ
} MotorControlMode;

/** @brief 已转换到 SI 单位的单轴值命令；未使用的目标字段必须为零。 */
typedef struct
{
    MotorControlMode mode;
    float position_rad;
    float speed_rad_s;
    float iq_a;
    float vq_v;
} MotorControlCommand;

/** @brief 目标提交结果；ACCEPTED 仅表示已复制到控制所有者的邮箱。 */
typedef enum
{
    MOTOR_CONTROL_ACCEPTED = 0,
    MOTOR_CONTROL_INVALID,
    MOTOR_CONTROL_DENIED,
    MOTOR_CONTROL_BUSY,
    MOTOR_CONTROL_OUT_OF_RANGE,
    MOTOR_CONTROL_FAULT
} MotorControlResult;

/**
 * @brief 控制所有者目标提交端口；返回前必须复制命令，不得保留指针。
 * @param context 组合层注入的控制所有者上下文。
 * @param command SI 单位命令；不得隐式使能或清故障。
 * @return 拒绝原因或 ACCEPTED；受理不表示目标已应用或电机已到位。
 * @note 前台串行调用，非阻塞；控制周期再次检查状态与限值后才能应用。
 */
typedef MotorControlResult (*MotorControlSubmit)(void *context, const MotorControlCommand *command);

#endif
