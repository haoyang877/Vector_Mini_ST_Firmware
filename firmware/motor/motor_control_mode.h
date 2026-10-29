#ifndef MOTOR_CONTROL_MODE_H
#define MOTOR_CONTROL_MODE_H
#include <stdbool.h>

/* 内部控制选择，独立于线上编号；NONE 表示不运行控制，不表示硬件已关断。 */
typedef enum
{
    MOTOR_CONTROL_CURRENT = 0,
    MOTOR_CONTROL_SPEED,
    MOTOR_CONTROL_POSITION,
    MOTOR_CONTROL_VQ,
    MOTOR_CONTROL_NONE,
    MOTOR_CONTROL_OPEN_VOLTAGE,
    MOTOR_CONTROL_OPEN_CURRENT
} MotorControlMode;

/** @brief 开环电角频率硬上限，单位Hz；实际命令还受机械速度包络限制。 */
#define MOTOR_OPEN_LOOP_MAX_ELECTRICAL_HZ 200.0f

/**
 * @brief 判断可启动的控制模式，NONE不属于可启动模式。
 * @param mode 内部模式值。
 * @return 已实现的普通控制模式返回true。
 */
static inline bool MotorControlMode_IsValid(MotorControlMode mode)
{
    return (unsigned)mode <= MOTOR_CONTROL_VQ || mode == MOTOR_CONTROL_OPEN_VOLTAGE ||
           mode == MOTOR_CONTROL_OPEN_CURRENT;
}

/**
 * @brief 判断使用指定电角频率而非反馈角度的模式。
 * @param mode 内部模式值。
 * @return 使用开环角度返回true。
 */
static inline bool MotorControlMode_IsOpenLoop(MotorControlMode mode)
{
    return mode == MOTOR_CONTROL_OPEN_VOLTAGE || mode == MOTOR_CONTROL_OPEN_CURRENT;
}

/**
 * @brief 判断电压给定模式，用于保持不可按电流参考降额的保护策略。
 * @param mode 内部模式值。
 * @return 直接给定电压的模式返回true。
 */
static inline bool MotorControlMode_IsVoltage(MotorControlMode mode)
{
    return mode == MOTOR_CONTROL_VQ || mode == MOTOR_CONTROL_OPEN_VOLTAGE;
}

#endif
