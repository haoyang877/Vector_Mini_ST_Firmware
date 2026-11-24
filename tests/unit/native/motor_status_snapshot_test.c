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
static unsigned s_fast_on_clock_read, s_outer_on_clock_read, s_supervisor_on_clock_read;
static bool s_mcu_fresh;
static MotorTemperatureTelemetry s_ntc;
bool McuTemperature_IsFresh(const volatile McuTemperatureTelemetry *temperature)
{
    (void)temperature;
    return s_mcu_fresh;
}
static MotorStateMachine machine = {.drive = {.motor = &motor,
                                              .foc = &foc,
                                              .rotor_encoder = &encoder,
                                              .output = &s_output,
                                              .motor_temperature = &s_ntc},
                                    .state = MOTOR_STATE_IDLE,
                                    .fast_samples_fresh = true};
static unsigned barriers, interrupts;
void BspSystem_MemoryBarrier(void)
{
    if (++barriers % 3U == 0U && interrupts != 0U)
    {
        --interrupts;
        ++machine.fast_sample_ticks;
        machine.state = MOTOR_STATE_FAULT;
        motor.control_mode = MOTOR_CONTROL_NONE;
        motor.fault_code = MOTOR_FAULT_OVERCURRENT;
        foc.bus_voltage_filtered_v = 35.0f;
    }
}
uint32_t BspSystem_TimeMs(void)
{
    if (s_fast_on_clock_read != 0U)
    {
        --s_fast_on_clock_read;
        ++machine.fast_sample_ticks;
        foc.bus_current_filtered_a += 1.0f;
    }
    if (s_outer_on_clock_read != 0U)
    {
        --s_outer_on_clock_read;
        ++machine.outer_ticks;
        s_output.tracked_position_rad += 1.0f;
    }
    if (s_supervisor_on_clock_read != 0U)
    {
        --s_supervisor_on_clock_read;
        ++machine.supervisor_ticks;
        machine.fast_samples_fresh = false;
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
    /* ADC 在慢域复制过程中抢占，不影响外环/监督数据，快域要读到中断后的值。 */
    s_fast_on_clock_read = 3U;
    assert(MotorStateMachine_ReadStatus(&machine, &sample));
    assert(s_fast_on_clock_read == 2U && sample.bus_current_a == 1.0f);
    s_fast_on_clock_read = 0U;

    /* 外环、监督分别使旧慢域副本失效。第二次读取必须观察到最新数据。 */
    s_outer_on_clock_read = 1U;
    assert(MotorStateMachine_ReadStatus(&machine, &sample));
    assert(sample.output.tracked_position_rad == 1.0f);
    s_supervisor_on_clock_read = 1U;
    assert(MotorStateMachine_ReadStatus(&machine, &sample));
    assert(!sample.measurements_valid);
    machine.fast_samples_fresh = true;

    motor.fault_code = MOTOR_FAULT_NONE;
    interrupts = 2U;
    assert(MotorStateMachine_ReadStatus(&machine, &sample));
    assert(sample.state == MOTOR_STATE_FAULT && sample.mode == MOTOR_CONTROL_NONE &&
           sample.fault == MOTOR_FAULT_OVERCURRENT && sample.bus_voltage_v == 35.0f &&
           isnan(sample.iq_a) && isnan(sample.position_reference_rad));
    memset(&sample, 0x45, sizeof(sample));
    unsigned char saved[sizeof(sample)];
    memcpy(saved, &sample, sizeof(sample));
    interrupts = 3U;
    assert(!MotorStateMachine_ReadStatus(&machine, &sample));
    assert(memcmp(saved, &sample, sizeof(sample)) == 0);
    s_outer_on_clock_read = 3U;
    assert(!MotorStateMachine_ReadStatus(&machine, &sample));
    assert(memcmp(saved, &sample, sizeof(sample)) == 0);
    s_supervisor_on_clock_read = 3U;
    assert(!MotorStateMachine_ReadStatus(&machine, &sample));
    assert(memcmp(saved, &sample, sizeof(sample)) == 0);
    machine.fast_samples_fresh = true;

    unsigned metadata_barriers = barriers;
    MotorStateMachine_ReadStatusMetadata(&machine, &sample);
    assert(sample.state == MOTOR_STATE_FAULT && sample.mode == MOTOR_CONTROL_NONE &&
           sample.fault == MOTOR_FAULT_OVERCURRENT && barriers == metadata_barriers);
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
    s_fast_on_clock_read = 1U;
    assert(MotorStateMachine_ReadOutputCoordinates(&machine, &coordinates));
    assert(barriers == previous_barriers && s_fast_on_clock_read == 0U);
    assert(coordinates.motor_position_rad == 8.0f && coordinates.output_position_rad == -1.0f);
    assert(coordinates.gear_ratio == 4.0f && coordinates.direction == -1);
    machine.fast_samples_fresh = false;
    assert(!MotorStateMachine_ReadOutputCoordinates(&machine, &coordinates));
    machine.fast_samples_fresh = true;
    ++s_now_ms;
    assert(!MotorStateMachine_ReadOutputCoordinates(&machine, &coordinates));
    assert(coordinates.motor_position_rad == 8.0f);
    /* 每次用当前时基检查源样本，包括毫秒回绕和丢采样。 */
    s_ntc =
        (MotorTemperatureTelemetry){.celsius = 28.0f, .sample_ms = UINT32_MAX - 20U, .valid = true};
    s_now_ms = 9U;
    s_mcu_fresh = true;
    foc.temperature_c = 31.0f;
    assert(MotorStateMachine_ReadStatus(&machine, &sample));
    assert(sample.output_age_ms == 10U && (sample.output.flags & 2U) != 0U &&
           sample.motor_temperature_valid && sample.motor_temperature_c == 28.0f &&
           sample.temperature_c == 31.0f);
    ++s_now_ms;
    s_mcu_fresh = false;
    assert(MotorStateMachine_ReadStatus(&machine, &sample));
    assert(sample.output_age_ms == 11U && (sample.output.flags & 2U) == 0U &&
           !sample.motor_temperature_valid && isnan(sample.motor_temperature_c) &&
           isnan(sample.temperature_c));
    s_output.sample_counter = 0U;
    machine.fast_samples_fresh = false;
    assert(MotorStateMachine_ReadStatus(&machine, &sample));
    assert(!sample.measurements_valid && sample.output_age_ms == UINT32_MAX &&
           (sample.output.flags & 3U) == 0U);
    assert(encode_motor_state(MOTOR_STATE_CALIBRATION) == 4U);
    assert(encode_motor_state(MOTOR_STATE_IDLE) == 5U);
    assert(encode_motor_state(MOTOR_STATE_RUNNING) == 6U);
    assert(encode_motor_state(MOTOR_STATE_FAULT) == 8U);
    puts("PASS status retry and direct output-coordinate validity");
    return 0;
}
