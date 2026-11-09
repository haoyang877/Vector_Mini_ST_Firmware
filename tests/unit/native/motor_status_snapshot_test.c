/* 只检查现有状态元数据接口，不再构造测量快照。 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "motor_state_machine.h"
#include "bsp_system.h"
static MotorControlState motor;
static FocState foc;
static EncoderState encoder;
bool McuTemperature_IsFresh(const volatile McuTemperatureTelemetry *temperature)
{
    (void)temperature;
    return false;
}
static MotorStateMachine machine = {
    .drive = {.motor = &motor, .foc = &foc, .rotor_encoder = &encoder},
    .state = MOTOR_STATE_IDLE,
    .fast_samples_fresh = true};
static unsigned barriers, interrupts;
void BspSystem_MemoryBarrier(void)
{
    if (++barriers % 2U == 0U && interrupts != 0U)
    {
        --interrupts;
        ++machine.fast_sample_ticks;
        motor.control_mode = MOTOR_CONTROL_CURRENT;
    }
}
/* REAL_FUNCTIONS */
int main(void)
{
    for (unsigned i = 0U; i <= MOTOR_STATE_FAULT; ++i)
    {
        machine.state = (MotorState)i;
        motor.control_mode = MOTOR_CONTROL_SPEED;
        motor.fault_code = MOTOR_FAULT_OVERCURRENT;
        MotorStatus status;
        assert(MotorStateMachine_ReadStatus(&machine, &status));
        assert(status.state == machine.state && status.mode == MOTOR_CONTROL_SPEED &&
               status.fault == MOTOR_FAULT_OVERCURRENT);
    }
    machine.state = MOTOR_STATE_RUNNING;
    motor.control_mode = MOTOR_CONTROL_POSITION;
    motor.position_target_rad = 1.5f;
    motor.position_reference_rad = 0.3f;
    MotorStatus sample;
    assert(MotorStateMachine_ReadStatus(&machine, &sample) &&
           sample.position_reference_rad == 0.3f);
    interrupts = 1U;
    assert(MotorStateMachine_ReadStatus(&machine, &sample) && sample.mode == MOTOR_CONTROL_CURRENT);
    interrupts = 2U;
    sample.mode = MOTOR_CONTROL_SPEED;
    assert(!MotorStateMachine_ReadStatus(&machine, &sample) && sample.mode == MOTOR_CONTROL_SPEED);
    assert(encode_motor_state(MOTOR_STATE_CALIBRATION) == 4U);
    assert(encode_motor_state(MOTOR_STATE_IDLE) == 5U);
    assert(encode_motor_state(MOTOR_STATE_RUNNING) == 6U);
    assert(encode_motor_state(MOTOR_STATE_FAULT) == 8U);
    puts("PASS minimal status metadata: state, mode, fault and invalid measurement channels");
    return 0;
}
