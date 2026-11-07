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
 * @brief 计算五态的下一状态，非法状态或事件统一进入故障。
 * @param state 当前状态；上电由运行层置为IDLE，自检仅预留。
 * @param event 单个事件；运行层负责健康校验、关相和资源释放。
 * @return 下一状态；清错回空闲，自检占位保持原状态。
 * @note 纯函数；RUN事件仅在实际启相确认后提交，完成/停止前必须关闭输出。
 */
MotorState MotorStateMachine_Step(MotorState state, MotorStateEvent event);

#endif
