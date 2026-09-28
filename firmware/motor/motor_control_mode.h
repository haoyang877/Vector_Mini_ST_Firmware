#ifndef MOTOR_CONTROL_MODE_H
#define MOTOR_CONTROL_MODE_H

/* 内部控制选择，独立于线上编号；NONE 表示不运行控制，不表示硬件已关断。 */
typedef enum
{
    MOTOR_CONTROL_CURRENT = 0,
    MOTOR_CONTROL_SPEED,
    MOTOR_CONTROL_POSITION,
    MOTOR_CONTROL_VQ,
    MOTOR_CONTROL_NONE
} MotorControlMode;

#endif
