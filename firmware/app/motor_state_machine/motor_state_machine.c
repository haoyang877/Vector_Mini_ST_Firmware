#include "motor_state_machine.h"

/* 五态纯迁移；自检仅占位，所有输出和请求资格由APP运行层处理。 */
MotorState MotorStateMachine_Step(MotorState state, MotorStateEvent event)
{
    if ((unsigned)state > MOTOR_STATE_FAULT || (unsigned)event > MOTOR_STATE_EVENT_CLEAR ||
        event == MOTOR_STATE_EVENT_FAULT)
    {
        return MOTOR_STATE_FAULT;
    }
    switch (state)
    {
    case MOTOR_STATE_IDLE:
        if (event == MOTOR_STATE_EVENT_RUN)
        {
            return MOTOR_STATE_RUNNING;
        }
        if (event == MOTOR_STATE_EVENT_CALIBRATE)
        {
            return MOTOR_STATE_CALIBRATION;
        }
        break;
    case MOTOR_STATE_SELF_TEST:
        break;
    case MOTOR_STATE_CALIBRATION:
    case MOTOR_STATE_RUNNING:
        if (event == MOTOR_STATE_EVENT_COMPLETE || event == MOTOR_STATE_EVENT_STOP)
        {
            return MOTOR_STATE_IDLE;
        }
        break;
    case MOTOR_STATE_FAULT:
        if (event == MOTOR_STATE_EVENT_CLEAR)
        {
            return MOTOR_STATE_IDLE;
        }
        break;
    default:
        return MOTOR_STATE_FAULT;
    }
    return state;
}
