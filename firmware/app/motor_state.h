#ifndef MOTOR_STATE_H
#define MOTOR_STATE_H

/** @brief 本轴状态机已提交的主状态。 */
typedef enum
{
    MOTOR_STATE_IDLE,
    MOTOR_STATE_SELF_TEST,
    MOTOR_STATE_CALIBRATION,
    MOTOR_STATE_RUNNING,
    MOTOR_STATE_FAULT
} MotorState;

#endif
