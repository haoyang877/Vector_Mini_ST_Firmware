/* 状态读取抢占夹具：实际读取实现由Python拼入，更新模拟完整ISR发布。 */
#include "motor_axis.h"
#include "foc_algorithm.h"
#include "angle_feedback.h"
#include "bsp_system.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static MotorConfiguration s_motor_configuration;
static MotorControlState s_motor_state;
static FocState s_foc_state;
static EncoderState s_rotor_encoder;
static volatile uint32_t s_status_revision;
static unsigned s_depth, s_budget, s_getter_calls, s_locked_getter_calls, s_value;

static void publish(unsigned value)
{
    s_value = value;
    s_motor_state.control_mode = value & 1U ? MOTOR_CONTROL_CURRENT : MOTOR_CONTROL_POSITION;
    s_motor_state.position_target_rad = (float)value;
    s_motor_state.position_velocity_filtered_rad_s = (float)value;
    s_motor_state.speed_target_rad_s = (float)value;
    s_motor_state.iq_target_a = (float)value;
    s_foc_state.iq_a = (float)value;
    s_foc_state.temperature_c = (float)value;
    s_foc_state.bus_voltage_filtered_v = (float)value;
    s_foc_state.bus_current_filtered_a = (float)value;
    s_motor_configuration.application.axis_profile_valid = true;
    ++s_status_revision;
}

uint32_t BspSystem_EnterCritical(void)
{
    return s_depth++;
}

void BspSystem_ExitCritical(uint32_t key)
{
    assert(s_depth > key);
    s_depth = key;
}

float Encoder_GetMecPos(const EncoderState *encoder)
{
    (void)encoder;
    ++s_getter_calls;
    if (s_depth != 0U)
    {
        ++s_locked_getter_calls;
    }
    else if (s_budget != 0U)
    {
        --s_budget;
        publish(s_value + 1U);
    }
    return (float)s_value;
}

float Encoder_GetMecVel(const EncoderState *encoder)
{
    (void)encoder;
    return (float)s_value;
}

uint16_t Encoder_GetBadFrameStreak(const EncoderState *encoder)
{
    (void)encoder;
    return 0U;
}

uint8_t Encoder_GetCalibFlag(const EncoderCalibration *calibration)
{
    (void)calibration;
    return 3U;
}

bool MotorTemperature_IsFresh(void)
{
    return (s_value & 1U) != 0U;
}

MotorMaintenanceKind MotorStateMachine_GetMaintenanceKind(void)
{
    return s_value & 1U ? MOTOR_MAINTENANCE_NONE : MOTOR_MAINTENANCE_ENCODER_LUT;
}

void MotorStateMachine_ReadControlProgress(MotorControlProgress *progress)
{
    uint32_t key = BspSystem_EnterCritical();
    memset(progress, 0, sizeof(*progress));
    progress->sequence = (uint16_t)s_value;
    BspSystem_ExitCritical(key);
}

MotorState MotorStateMachine_GetState(void)
{
    return s_value & 1U ? MOTOR_STATE_RUNNING : MOTOR_STATE_MAINTENANCE;
}

/* REAL_FUNCTIONS */

int main(void)
{
    for (unsigned budget = 0U; budget < 10U; ++budget)
    {
        for (unsigned wrap = 0U; wrap < 2U; ++wrap)
        {
            s_status_revision = wrap ? UINT32_MAX - 1U : 0U;
            publish(1U);
            s_budget = budget;
            s_getter_calls = s_locked_getter_calls = 0U;
            MotorStatus actual = {0}, expected = {0};
            MotorControlProgress progress = {0}, expected_progress = {0};
            MotorMaintenanceKind kind, expected_kind;
            uint8_t valid, expected_valid;
            MotorState state = MotorAxis_ReadStatus(&actual, &valid, &kind, &progress);
            assert(s_depth == 0U);
            assert(s_getter_calls == (budget < 3U ? budget + 1U : 3U));
            assert(s_locked_getter_calls == 0U);
            if (budget >= 3U)
            {
                assert(valid == 0U);
                continue;
            }
            s_budget = 0U;
            uint32_t key = BspSystem_EnterCritical();
            MotorState expected_state = read_status_snapshot(
                &expected, &expected_valid, &expected_kind, &expected_progress);
            BspSystem_ExitCritical(key);
            assert(state == expected_state && kind == expected_kind && valid == expected_valid);
            assert(memcmp(&actual, &expected, sizeof(actual)) == 0);
            assert(memcmp(&progress, &expected_progress, sizeof(progress)) == 0);
        }
    }
    puts(
        "PASS coherent motor snapshots: fast/slow publication, rollover, bounded retry and invalid "
        "fallback");
    return 0;
}

uint32_t BspSystem_EnterControlCritical(void)
{
    return BspSystem_EnterCritical();
}
void BspSystem_ExitControlCritical(uint32_t state)
{
    BspSystem_ExitCritical(state);
}
