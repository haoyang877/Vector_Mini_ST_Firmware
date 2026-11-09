#ifndef MOTOR_STATE_MACHINE_H
#define MOTOR_STATE_MACHINE_H

typedef enum
{
    MOTOR_STATE_IDLE,
    MOTOR_STATE_SELF_TEST,
    MOTOR_STATE_CALIBRATION,
    MOTOR_STATE_RUNNING,
    MOTOR_STATE_FAULT
} MotorState;

/* 每次只处理一个已由运行层校验的事件；无硬件、配置或任务依赖。 */
typedef enum
{
    MOTOR_STATE_EVENT_RUN,
    MOTOR_STATE_EVENT_CALIBRATE,
    MOTOR_STATE_EVENT_COMPLETE,
    MOTOR_STATE_EVENT_STOP,
    MOTOR_STATE_EVENT_FAULT,
    MOTOR_STATE_EVENT_CLEAR
} MotorStateEvent;

/**
 * @brief 按单个事件计算下一主状态。
 * @param state 调用方持有的当前状态，每根轴独立。
 * @param event 已满足执行条件的事件，非法事件进入故障。
 * @return 迁移后的状态。没有对应迁移时保持原状态，非法状态进入故障。
 */
MotorState MotorStateMachine_NextState(MotorState state, MotorStateEvent event);

#endif
