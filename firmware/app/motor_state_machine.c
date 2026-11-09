#include "motor_state_machine.h"

MotorState MotorStateMachine_NextState(MotorState state, MotorStateEvent event)
{
    /* 故障事件优先于任何状态的正常出口。无法识别的状态或事件也进入故障。 */
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
