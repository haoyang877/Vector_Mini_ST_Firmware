#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <stdio.h>
#include "../../System/servo_hil.c"
static ServoHilCommand request(uint32_t op, float value, uint32_t mode)
{
    servo_hil_mailbox.opcode = op;
    servo_hil_mailbox.value = value;
    servo_hil_mailbox.heartbeat++;
    servo_hil_mailbox.sequence++;
    return ServoHil_Poll(0, 0, 0, mode, 0, 20000, 1);
}
static void test_phase_burst_guard(void)
{
    ServoHilCommand c;
    unsigned i;
    c = request(SERVO_HIL_ARM, 0, 0); ServoHil_Complete(true);
    ServoHil_ObservePhaseCurrents(4, -4, 0);
    assert(phase_guard_state.exposure_ticks == 0 && armed);
    for (i=0; i<4000; ++i) ServoHil_ObservePhaseCurrents(0, -5, 5);
    assert(phase_guard_state.exposure_ticks == 4000);
    ServoHil_ObservePhaseCurrents(0, 0, 0);
    assert(phase_guard_state.exposure_ticks == 4000); /* Dips do not reset budget. */
    for (i=0; i<25999; ++i) ServoHil_ObservePhaseCurrents(5, -2, -3);
    assert(phase_guard_state.trip == 0);
    c = ServoHil_Poll(0,0,0,3,0,1000,1);
    assert(c.action == SERVO_HIL_NONE && armed); /* 29.999 s remains allowed. */
    ServoHil_ObservePhaseCurrents(5, -2, -3);
    c = ServoHil_Poll(0,0,0,3,0,1000,1);
    assert(c.action == SERVO_HIL_STOP && !armed);
    ServoHil_ObservePhaseCurrents(6,0,0);
    assert(phase_guard_state.trip == 6); /* Preserve diagnosis while disabled. */
    for (i=0; i<3; ++i) {
        request(SERVO_HIL_ARM, 0, 0); ServoHil_Complete(true);
        assert(phase_guard_state.exposure_ticks == 0 && phase_guard_state.trip == 0);
        ServoHil_ObservePhaseCurrents(i==0 ? 6:0, i==1 ? -6:0, i==2 ? 6:0);
        assert(phase_guard_state.trip == 5);
        c=ServoHil_Poll(0,0,0,3,0,20000,1); assert(c.action == SERVO_HIL_STOP);
    }
    request(SERVO_HIL_ARM, 0, 0); ServoHil_Complete(true);
    ServoHil_ObservePhaseCurrents(NAN,0,0);
    c=ServoHil_Poll(0,0,0,3,0,20000,1); assert(c.action == SERVO_HIL_STOP);
}
int main(void)
{
    ServoHilCommand c;
    unsigned i;
    c = request(SERVO_HIL_POSITION, .1f, 0);
    assert(c.action == SERVO_HIL_STOP && !armed);
    c = request(SERVO_HIL_ARM, 0, 0);
    assert(c.action == SERVO_HIL_ARM); ServoHil_Complete(true);
    assert(armed);
    c = request(SERVO_HIL_HOLD_FILTER, 0, 3);
    assert(c.action == SERVO_HIL_HOLD_FILTER && c.value == 0); ServoHil_Complete(true);
    c = request(SERVO_HIL_HOLD_FILTER, 1, 3);
    assert(c.action == SERVO_HIL_HOLD_FILTER && c.value == 1); ServoHil_Complete(true);
    c = request(SERVO_HIL_HOLD_FILTER, 2, 3);
    assert(c.action == SERVO_HIL_HOLD_FILTER && c.value == 2); ServoHil_Complete(true);
    c = request(SERVO_HIL_POSITION, .1f, 3);
    assert(c.action == SERVO_HIL_POSITION); ServoHil_Complete(true);
    for(i=0; i<10001; ++i) {
        c = ServoHil_Poll(0,0,0,3,0,20000,1);
        if(c.action == SERVO_HIL_STOP) break;
    }
    assert(i<=10000 && c.action == SERVO_HIL_STOP && !armed);
    c = request(SERVO_HIL_ARM, 0, 0); ServoHil_Complete(true);
    c = ServoHil_Poll(SERVO_HIL_TRAVEL_LIMIT_RAD,0,0,3,0,20000,1);
    assert(c.action == SERVO_HIL_STOP && !armed);
    c = request(SERVO_HIL_ARM, 0, 0); ServoHil_Complete(true);
    c = request(SERVO_HIL_POSITION, 1.6f, 3);
    assert(c.action == SERVO_HIL_STOP && !armed);
    c = request(SERVO_HIL_POSITION_KP, 1.0f, 0);
    assert(c.action == SERVO_HIL_POSITION_KP); ServoHil_Complete(true);
    c = request(SERVO_HIL_POSITION_KP, 1.0f, 3);
    assert(c.action == SERVO_HIL_STOP);
    c = request(SERVO_HIL_ARM, 0, 0); ServoHil_Complete(false);
    assert(!armed);
    c = request(SERVO_HIL_ARM, 0, 0); ServoHil_Complete(true);
    for(i=0; i<4999; ++i) {
        c = ServoHil_Poll(0,0,0,3,0,20000,2);
        assert(c.action == SERVO_HIL_NONE);
    }
    c = ServoHil_Poll(0,0,0,3,0,20000,2);
    assert(c.action == SERVO_HIL_STOP && !armed);
    c = request(SERVO_HIL_MAX_SPEED, SERVO_HIL_MAX_CRUISE_RAD_S, 0);
    assert(c.action == SERVO_HIL_MAX_SPEED); ServoHil_Complete(true);
    c = request(SERVO_HIL_MAX_SPEED, .79f, 0);
    assert(c.action == SERVO_HIL_STOP);
    c = request(SERVO_HIL_ARM, 0, 0); ServoHil_Complete(true);
    c = request(SERVO_HIL_POSITION, SERVO_HIL_TARGET_LIMIT_RAD, 3);
    assert(c.action == SERVO_HIL_POSITION); ServoHil_Complete(true);
    c = request(SERVO_HIL_POSITION, -SERVO_HIL_TARGET_LIMIT_RAD, 3);
    assert(c.action == SERVO_HIL_POSITION); ServoHil_Complete(true);
    c = ServoHil_Poll(-1.5f, 0, 0, 3, 0, 20000, 1);
    assert(c.action == SERVO_HIL_NONE && armed);
    c = ServoHil_Poll(-SERVO_HIL_TRAVEL_LIMIT_RAD, 0, 0, 3, 0, 20000, 1);
    assert(c.action == SERVO_HIL_STOP && !armed);
    c = request(SERVO_HIL_ARM, 0, 0); ServoHil_Complete(true);
    c = ServoHil_Poll(0, 1.6f, 0, 3, 0, 20000, 1);
    assert(c.action == SERVO_HIL_STOP && !armed);
    c = request(SERVO_HIL_HOLD_FILTER, .5f, 3); assert(c.action == SERVO_HIL_STOP);
    c = request(SERVO_HIL_HOLD_FILTER, 3, 0); assert(c.action == SERVO_HIL_STOP);
    c = request(SERVO_HIL_HOLD_FILTER, 0, 2); assert(c.action == SERVO_HIL_STOP);
    c = request(SERVO_HIL_HOLD_FILTER, 1, 0);
    assert(c.action == SERVO_HIL_HOLD_FILTER); ServoHil_Complete(true);
    c = request(999U, 0, 0); assert(c.action == SERVO_HIL_STOP);
    c = request(SERVO_HIL_VELOCITY_FILTER, 0, 0);
    assert(c.action == SERVO_HIL_VELOCITY_FILTER); ServoHil_Complete(true);
    c = request(SERVO_HIL_VELOCITY_FILTER, 1, 0);
    assert(c.action == SERVO_HIL_VELOCITY_FILTER); ServoHil_Complete(true);
    c = request(SERVO_HIL_VELOCITY_FILTER, .5f, 0); assert(c.action == SERVO_HIL_STOP);
    c = request(SERVO_HIL_VELOCITY_FILTER, 2, 0); assert(c.action == SERVO_HIL_STOP);
    c = request(SERVO_HIL_VELOCITY_FILTER, NAN, 0); assert(c.action == SERVO_HIL_STOP);
    request(SERVO_HIL_ARM, 0, 0); ServoHil_Complete(true);
    c = request(SERVO_HIL_VELOCITY_FILTER, 1, 3);
    assert(c.action == SERVO_HIL_STOP && !armed);
    test_phase_burst_guard();
    puts("PASS phase-current burst guard: 6 A all phases, 30 s cumulative, dips, re-arm, invalid sample");
    puts("PASS HIL command service: arm, timeout, travel bound, command validation, adapter failure");
    return 0;
}
