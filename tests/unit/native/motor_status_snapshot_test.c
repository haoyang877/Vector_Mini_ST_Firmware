/* 只检查现有状态元数据接口，不再构造测量快照。 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "motor_lifecycle.h"
#include "bsp_system.h"
static MotorControlState motor;
static FocState foc;
static EncoderState encoder;
bool McuTemperature_IsFresh(void)
{
    return false;
}
static struct
{
    MotorControlState *motor;
    MotorState state;
    FocState *foc;
    EncoderState *rotor_encoder;
    bool fast_samples_fresh;
} g_motor_context = {&motor, MOTOR_STATE_IDLE, &foc, &encoder, true};
static unsigned depth;
uint32_t BspSystem_EnterCritical(void)
{
    return depth++;
}
void BspSystem_ExitCritical(uint32_t saved)
{
    assert(depth == saved + 1U);
    depth = saved;
}
/* REAL_FUNCTIONS */
int main(void)
{
    for (unsigned i = 0U; i <= MOTOR_STATE_FAULT; ++i)
    {
        g_motor_context.state = (MotorState)i;
        motor.control_mode = MOTOR_CONTROL_SPEED;
        motor.fault_code = MOTOR_FAULT_OVERCURRENT;
        MotorStatus status = MotorLifecycle_ReadStatus();
        assert(status.state == g_motor_context.state && status.mode == MOTOR_CONTROL_SPEED &&
               status.fault == MOTOR_FAULT_OVERCURRENT && depth == 0U);
    }
    g_motor_context.state = MOTOR_STATE_RUNNING;
    motor.control_mode = MOTOR_CONTROL_POSITION;
    motor.position_target_rad = 1.5f;
    motor.position_reference_rad = 0.3f;
    assert(MotorLifecycle_ReadStatus().position_reference_rad == 0.3f);
    assert(protocol_state(MOTOR_STATE_CALIBRATION) == 4U);
    assert(protocol_state(MOTOR_STATE_IDLE) == 5U);
    assert(protocol_state(MOTOR_STATE_RUNNING) == 6U);
    assert(protocol_state(MOTOR_STATE_FAULT) == 8U);
    puts("PASS minimal status metadata: state, mode, fault and invalid measurement channels");
    return 0;
}
