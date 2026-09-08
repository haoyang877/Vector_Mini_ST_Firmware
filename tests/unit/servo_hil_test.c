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
int main(void)
{
    ServoHilCommand c;
    unsigned i;
    c = request(SERVO_HIL_POSITION, .1f, 0);
    assert(c.action == SERVO_HIL_STOP && !armed);
    c = request(SERVO_HIL_ARM, 0, 0);
    assert(c.action == SERVO_HIL_ARM); ServoHil_Complete(true);
    assert(armed);
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
    c = request(999U, 0, 0); assert(c.action == SERVO_HIL_STOP);
    puts("PASS HIL command service: arm, timeout, travel bound, command validation, adapter failure");
    return 0;
}
