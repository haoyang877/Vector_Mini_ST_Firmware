#include <assert.h>
#include <stdio.h>
#include "motor_state_machine.h"

int main(void)
{
    /* 各行按 RUN / CALIBRATE / COMPLETE / STOP / FAULT / CLEAR 对照五态契约。 */
    static const MotorState expected[][MOTOR_STATE_EVENT_CLEAR + 1] = {
        [MOTOR_STATE_IDLE] = {MOTOR_STATE_RUNNING,
                              MOTOR_STATE_CALIBRATION,
                              MOTOR_STATE_IDLE,
                              MOTOR_STATE_IDLE,
                              MOTOR_STATE_FAULT,
                              MOTOR_STATE_IDLE},
        [MOTOR_STATE_SELF_TEST] = {MOTOR_STATE_SELF_TEST,
                                   MOTOR_STATE_SELF_TEST,
                                   MOTOR_STATE_SELF_TEST,
                                   MOTOR_STATE_SELF_TEST,
                                   MOTOR_STATE_FAULT,
                                   MOTOR_STATE_SELF_TEST},
        [MOTOR_STATE_CALIBRATION] = {MOTOR_STATE_CALIBRATION,
                                     MOTOR_STATE_CALIBRATION,
                                     MOTOR_STATE_IDLE,
                                     MOTOR_STATE_IDLE,
                                     MOTOR_STATE_FAULT,
                                     MOTOR_STATE_CALIBRATION},
        [MOTOR_STATE_RUNNING] = {MOTOR_STATE_RUNNING,
                                 MOTOR_STATE_RUNNING,
                                 MOTOR_STATE_IDLE,
                                 MOTOR_STATE_IDLE,
                                 MOTOR_STATE_FAULT,
                                 MOTOR_STATE_RUNNING},
        [MOTOR_STATE_FAULT] = {MOTOR_STATE_FAULT,
                               MOTOR_STATE_FAULT,
                               MOTOR_STATE_FAULT,
                               MOTOR_STATE_FAULT,
                               MOTOR_STATE_FAULT,
                               MOTOR_STATE_IDLE},
    };
    for (unsigned state = 0; state <= MOTOR_STATE_FAULT; ++state)
    {
        for (unsigned event = 0; event <= MOTOR_STATE_EVENT_CLEAR; ++event)
        {
            assert(MotorStateMachine_NextState((MotorState)state, (MotorStateEvent)event) ==
                   expected[state][event]);
        }
    }
    assert(MotorStateMachine_NextState((MotorState)99, MOTOR_STATE_EVENT_RUN) == MOTOR_STATE_FAULT);
    assert(MotorStateMachine_NextState(MOTOR_STATE_IDLE, (MotorStateEvent)99) == MOTOR_STATE_FAULT);
    puts("PASS 30 state/event transitions and invalid inputs");
    return 0;
}
