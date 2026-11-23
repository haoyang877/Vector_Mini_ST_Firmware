/* 验证整帧状态重试和目标坐标直接读取各自的行为。 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "motor_state_machine.h"
#include "bsp_system.h"
static MotorControlState motor;
static FocState foc;
static EncoderState encoder;
static MotorOutputTelemetry s_output;
static uint32_t s_now_ms;
static bool s_interrupt_on_time_read;
bool McuTemperature_IsFresh(const volatile McuTemperatureTelemetry *temperature)
{
    (void)temperature;
    return false;
}
static MotorStateMachine machine = {
    .drive = {.motor = &motor, .foc = &foc, .rotor_encoder = &encoder, .output = &s_output},
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
uint32_t BspSystem_TimeMs(void)
{
    if (s_interrupt_on_time_read)
    {
        ++machine.fast_sample_ticks;
        ++machine.outer_ticks;
        ++machine.supervisor_ticks;
        s_interrupt_on_time_read = false;
    }
    return s_now_ms;
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
    MotorOutputCoordinates coordinates;
    assert(!MotorStateMachine_ReadOutputCoordinates(&machine, &coordinates));
    encoder.has_valid_sample = true;
    encoder.mechanical_position_rad = 8.0f;
    s_output = (MotorOutputTelemetry){.tracked_position_rad = -1.0f,
                                      .gear_ratio = 4.0f,
                                      .sample_ms = UINT32_MAX,
                                      .sample_counter = 1U,
                                      .flags = 2U,
                                      .direction = -1};
    s_now_ms = 9U;
    unsigned previous_barriers = barriers;
    s_interrupt_on_time_read = true;
    assert(MotorStateMachine_ReadOutputCoordinates(&machine, &coordinates));
    assert(barriers == previous_barriers && !s_interrupt_on_time_read);
    assert(coordinates.motor_position_rad == 8.0f && coordinates.output_position_rad == -1.0f);
    assert(coordinates.gear_ratio == 4.0f && coordinates.direction == -1);
    machine.fast_samples_fresh = false;
    assert(!MotorStateMachine_ReadOutputCoordinates(&machine, &coordinates));
    machine.fast_samples_fresh = true;
    ++s_now_ms;
    assert(!MotorStateMachine_ReadOutputCoordinates(&machine, &coordinates));
    assert(coordinates.motor_position_rad == 8.0f);
    assert(encode_motor_state(MOTOR_STATE_CALIBRATION) == 4U);
    assert(encode_motor_state(MOTOR_STATE_IDLE) == 5U);
    assert(encode_motor_state(MOTOR_STATE_RUNNING) == 6U);
    assert(encode_motor_state(MOTOR_STATE_FAULT) == 8U);
    puts("PASS status retry and direct output-coordinate validity");
    return 0;
}
