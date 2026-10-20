#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "motor_axis.h"
#include "motor_maintenance.h"
#include "foc_algorithm.h"
#include "foc_errhandle.h"
#include "foc_sensing.h"
#include "foc_sensorless.h"
#include "motor_sensing.h"
#include "power_stage_hw.h"
#include "motor_execution.h"
#include "control_math.h"
#include "motor_state_machine.h"
#include "motor_hw.h"
#include "motor_parameter_transaction.h"
#include "motor_thermal_control.h"
#include "time_hw.h"
#include "application_profile_defaults.h"

/* 只替换启动外设/存储边界；初始化、校正、换算和PI实现采用生产体。 */
static MotorControlState s_motor_state;
static FocState s_foc_state;
static PiController s_speed_pi;
static EncoderState s_rotor_encoder;
static FluxObserverState s_flux_observer;
static SensorlessStartupState s_sensorless_startup;
static uint32_t s_config_revision, s_revision_work_epoch;
static struct
{
    bool valid;
} s_last_read, s_last_write, s_save_job;
static uint16_t samples[3];
static bool output_off = true;
static float simulated_bus_voltage_v = 30.0f;
static unsigned current_reads, state_initializations, maintenance_queries;
static unsigned observer_calls, progress_calls, protection_calls, control_calls;
static MotorState simulated_state = MOTOR_STATE_INIT;

static void load_parameters(MotorControlState *motor, EncoderState *encoder)
{
    assert(motor == &s_motor_state && encoder == &s_rotor_encoder);
    /* 模拟已存储的不同d/q增益，确保初始化不会覆盖为编译默认值。 */
    MotorWindingThermal_Default(&s_motor_state.model.thermal);
    s_motor_state.application.current_limit_a = 6.0f;
    s_motor_state.model.id_kp = 2.0f;
    s_motor_state.model.id_ki = 3.0f;
    s_motor_state.model.iq_kp = 4.0f;
    s_motor_state.model.iq_ki = 5.0f;
    s_motor_state.application.speed_kp = 0.2f;
    s_motor_state.application.speed_ki = 0.3f;
    s_motor_state.application.axis_profile_valid = true;
    s_motor_state.device.phase_a_offset_counts = 2048.0f;
    s_motor_state.device.phase_b_offset_counts = 2048.0f;
    s_motor_state.device.phase_c_offset_counts = 2048.0f;
}

void MotorStateMachine_Init(MotorControlState *motor,
                            FocState *foc,
                            PiController *pi,
                            EncoderState *encoder,
                            SensorlessStartupState *startup)
{
    (void)pi;
    (void)encoder;
    (void)startup;
    ++state_initializations;
    assert(!MotorCurrentOffsets_Valid(motor) && isnan(foc->phase_a_current_a));
}
void Encoder_ParamInit(EncoderState *encoder)
{
    (void)encoder;
}
void FluxObserver_ParamInit(FluxObserverState *observer)
{
    (void)observer;
}
void SensorlessStartup_Reset(SensorlessStartupState *startup)
{
    (void)startup;
}
void MotorOuterLoop_RequestReset(void)
{
}
void MotorMaintenance_Init(void)
{
}
bool power_stage_hw_phases_disabled(void)
{
    return output_off;
}
uint16_t motor_hw_current_sample_raw(MotorHwCurrentPhase phase)
{
    ++current_reads;
    assert((unsigned)phase < 3U);
    return samples[phase];
}
void MotorFault_Report(MotorControlState *motor, MotorFaultCode error)
{
    if (motor->fault_code == MOTOR_FAULT_NONE)
    {
        motor->fault_code = error;
    }
}

bool Encoder_BeginSample(void)
{
    return true;
}
uint8_t Encoder_BeginReadback(bool sample_started)
{
    return sample_started ? 1U : 0U;
}
void Encoder_CompleteSample(MotorControlState *motor, EncoderState *encoder, uint8_t sample_stage)
{
    assert(sample_stage == 1U);
}
uint32_t time_hw_now_ms(void)
{
    return 0U;
}
float Encoder_GetMecPos(const EncoderState *encoder)
{
    return 0.0f;
}
float Encoder_GetMecVel(const EncoderState *encoder)
{
    return 0.0f;
}
void Vbus_Update(FocState *foc, MotorControlState *motor, bool output_expected)
{
    foc->bus_voltage_v = foc->bus_voltage_filtered_v = simulated_bus_voltage_v;
}
void FluxObserver_Update(FocState *foc, MotorControlState *motor, FluxObserverState *observer)
{
    assert(isfinite(foc->phase_a_current_a) && isfinite(foc->phase_b_current_a) &&
           isfinite(foc->phase_c_current_a));
    ++observer_calls;
}
bool MotorStateMachine_OutputExpected(void)
{
    return !output_off;
}
bool MotorStateMachine_MaintenanceRunnable(void)
{
    ++maintenance_queries;
    return false;
}
MotorMaintenanceKind MotorStateMachine_GetMaintenanceKind(void)
{
    return MOTOR_MAINTENANCE_ENCODER_LUT;
}
MotorState MotorStateMachine_GetState(void)
{
    return simulated_state;
}
void MotorStateMachine_CheckFastFaults(void)
{
    ++protection_calls;
}
void MotorStateMachine_FastFaultStop(void)
{
    if (s_motor_state.fault_code != MOTOR_FAULT_NONE)
    {
        output_off = true;
    }
}
uint32_t MotorStateMachine_WorkEpoch(void)
{
    return 1U;
}
void MotorStateMachine_PostOutcome(MotorTaskResult outcome, uint32_t epoch)
{
    assert(outcome.result == MOTOR_TASK_RUNNING && epoch == 1U);
    ++progress_calls;
}
void motor_hw_pwm_set_duty(float duty_a, float duty_b, float duty_c)
{
    assert(duty_a == 0.5f && duty_b == 0.5f && duty_c == 0.5f);
}
MotorTaskResult MotorExecution_ControlStep(FocState *foc,
                                           MotorControlState *motor,
                                           EncoderState *encoder,
                                           FluxObserverState *observer)
{
    ++control_calls;
    return (MotorTaskResult){MOTOR_TASK_RUNNING, MOTOR_FAULT_NONE};
}
MotorTaskResult MotorMaintenance_Step(MotorMaintenanceKind kind,
                                      FocState *foc,
                                      MotorControlState *motor,
                                      PiController *pi,
                                      EncoderState *encoder,
                                      FluxObserverState *observer,
                                      SensorlessStartupState *startup)
{
    assert(false);
    return (MotorTaskResult){MOTOR_TASK_RUNNING, MOTOR_FAULT_NONE};
}
/* STARTUP_CURRENT_OFFSET_PRODUCTION：脚本插入真实生产体。 */

static void initialize(void)
{
    memset(&s_motor_state, 0, sizeof(s_motor_state));
    memset(&s_foc_state, 0, sizeof(s_foc_state));
    memset(&s_speed_pi, 0, sizeof(s_speed_pi));
    output_off = true;
    simulated_bus_voltage_v = 30.0f;
    current_reads = 0U;
    maintenance_queries = 0U;
    observer_calls = progress_calls = protection_calls = control_calls = 0U;
    simulated_state = MOTOR_STATE_INIT;
    samples[0] = 8131U;
    samples[1] = 8205U;
    samples[2] = 8146U;
    s_foc_state.id_pi.integral_output = s_foc_state.iq_pi.integral_output = 2.0f;
    s_speed_pi.integral_output = 2.0f;
    MotorAxis_Init();
    assert(s_motor_state.control_mode == MOTOR_CONTROL_NONE);
    assert(s_foc_state.id_pi.proportional_gain == 2.0f && s_foc_state.id_pi.integral_gain == 3.0f);
    assert(s_foc_state.iq_pi.proportional_gain == 4.0f && s_foc_state.iq_pi.integral_gain == 5.0f);
    assert(s_speed_pi.proportional_gain == 0.2f && s_speed_pi.integral_gain == 0.3f);
    assert(s_foc_state.id_pi.sample_period_s == Current_Ts);
    assert(s_foc_state.iq_pi.sample_period_s == Current_Ts &&
           s_speed_pi.sample_period_s == Speed_Ts);
    assert(s_foc_state.id_pi.output_max == 0.0f && s_foc_state.iq_pi.output_min == 0.0f);
    assert(s_speed_pi.output_min == -1.0f && s_speed_pi.output_max == 1.0f);
    assert(s_foc_state.id_pi.integral_output == 0.0f && s_foc_state.iq_pi.integral_output == 0.0f);
    assert(s_speed_pi.integral_output == 0.0f && s_config_revision == 1U);
    assert(!MotorCurrentOffsets_Valid(&s_motor_state));
}

static void test_full_window(void)
{
    initialize();
    for (unsigned i = 0U; i < CURRENT_OFFSET_SAMPLE_COUNT; ++i)
    {
        samples[0] = (i & 1U) ? 8131U : 8130U;
        assert(!s_current_offset_calibration.complete && isnan(s_foc_state.phase_a_current_a));
        MotorAxis_FastTick();
        assert(output_off && s_motor_state.fault_code == MOTOR_FAULT_NONE);
        assert(s_foc_state.id_pi.output_min == -FOC_MaximumVoltage(30.0f));
        assert(s_foc_state.iq_pi.output_max == FOC_MaximumVoltage(30.0f));
    }
    assert(s_current_offset_calibration.complete && !s_current_offset_calibration.failed);
    assert(MotorCurrentOffsets_Valid(&s_motor_state));
    assert(fabsf(s_motor_state.device.phase_a_offset_counts - 2032.625f) < 0.0005f);
    assert(fabsf(s_motor_state.device.phase_b_offset_counts - 2051.25f) < 0.0005f);
    assert(fabsf(s_motor_state.device.phase_c_offset_counts - 2036.5f) < 0.0005f);
    assert(fabsf(s_foc_state.phase_a_current_a + 0.125f * MOTOR_SENSING_CURRENT_A_PER_COUNT) <
           0.00002f);
    assert(fabsf(s_foc_state.phase_b_current_a) < 0.00002f &&
           fabsf(s_foc_state.phase_c_current_a) < 0.00002f);
    assert(s_config_revision == 2U);
    assert(observer_calls == 1U && control_calls == 0U);
    assert(progress_calls == CURRENT_OFFSET_SAMPLE_COUNT &&
           protection_calls == CURRENT_OFFSET_SAMPLE_COUNT);
    /* 完成零点的这一拍也只捕获一次维护许可，分发与修订使用同一事实。 */
    assert(maintenance_queries == CURRENT_OFFSET_SAMPLE_COUNT);
    uint32_t old_sum = s_current_offset_calibration.phase_sum_raw[0];
    samples[0] += 4U;
    sample_phase_currents();
    assert(s_current_offset_calibration.sample_count == CURRENT_OFFSET_SAMPLE_COUNT);
    assert(s_current_offset_calibration.phase_sum_raw[0] == old_sum && s_config_revision == 2U);
    assert(fabsf(s_foc_state.phase_a_current_a + 1.125f * MOTOR_SENSING_CURRENT_A_PER_COUNT) <
           0.00002f);
}

static void test_rejections(void)
{
    initialize();
    output_off = false;
    MotorAxis_FastTick();
    assert(output_off && progress_calls == 1U && protection_calls == 1U && observer_calls == 0U);
    assert(s_current_offset_calibration.failed && current_reads == 0U);
    assert(s_motor_state.fault_code == MOTOR_FAULT_CURRENT_OFFSET && s_config_revision == 1U);
    output_off = true;
    sample_phase_currents();
    assert(current_reads == 0U && !s_current_offset_calibration.complete);

    initialize();
    s_motor_state.control_mode = MOTOR_CONTROL_CURRENT;
    sample_phase_currents();
    assert(s_current_offset_calibration.failed && current_reads == 0U);

    initialize();
    samples[1] = 65535U;
    sample_phase_currents();
    assert(s_current_offset_calibration.failed && s_current_offset_calibration.sample_count == 0U);
    assert(!MotorCurrentOffsets_Valid(&s_motor_state));

    initialize();
    samples[2] = 0U;
    for (unsigned i = 0U; i < CURRENT_OFFSET_SAMPLE_COUNT; ++i)
    {
        sample_phase_currents();
    }
    assert(s_current_offset_calibration.failed && !s_current_offset_calibration.complete);
    assert(s_motor_state.fault_code == MOTOR_FAULT_CURRENT_OFFSET);
    assert(isnan(s_motor_state.device.phase_a_offset_counts) &&
           isnan(s_foc_state.phase_a_current_a));
    assert(s_config_revision == 1U);

    initialize();
    s_motor_state.fault_code = MOTOR_FAULT_ENCODER;
    samples[0] = 65535U;
    sample_phase_currents();
    assert(s_motor_state.fault_code == MOTOR_FAULT_ENCODER);
}

static void test_voltage_limits_refresh_only(void)
{
    /* 母线有效、升降和失效均经真实FastTick；校正pending不执行电流控制。 */
    static const float voltages_v[] = {32.84f, 26.0f, 33.0f, 0.0f, -1.0f, NAN, INFINITY};
    initialize();
    s_foc_state.id_pi =
        (PiController){7.0f, 8.0f, 0.123f, -3.0f, 4.0f, 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};
    s_foc_state.iq_pi =
        (PiController){9.0f, 10.0f, 0.124f, -5.0f, 6.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f};
    s_speed_pi.output_min = -0.25f;
    s_speed_pi.output_max = 0.75f;
    s_speed_pi.integral_output = 0.4f;
    PiController expected_speed = s_speed_pi;
    for (unsigned i = 0U; i < sizeof(voltages_v) / sizeof(voltages_v[0]); ++i)
    {
        PiController expected_id = s_foc_state.id_pi;
        PiController expected_iq = s_foc_state.iq_pi;
        simulated_bus_voltage_v = voltages_v[i];
        float maximum_voltage_v = FOC_MaximumVoltage(simulated_bus_voltage_v);
        expected_id.output_min = expected_iq.output_min = -maximum_voltage_v;
        expected_id.output_max = expected_iq.output_max = maximum_voltage_v;
        MotorAxis_FastTick();
        assert(memcmp(&s_foc_state.id_pi, &expected_id, sizeof(expected_id)) == 0);
        assert(memcmp(&s_foc_state.iq_pi, &expected_iq, sizeof(expected_iq)) == 0);
        assert(memcmp(&s_speed_pi, &expected_speed, sizeof(expected_speed)) == 0);
        assert(control_calls == 0U && observer_calls == 0U && output_off);
        assert(s_config_revision == 1U && s_motor_state.fault_code == MOTOR_FAULT_NONE);
    }
}

int main(void)
{
    assert(CURRENT_OFFSET_SAMPLE_COUNT == 20000U);
    test_full_window();
    test_rejections();
    test_voltage_limits_refresh_only();
    initialize();
    simulated_state = MOTOR_STATE_RUNNING;
    MotorAxis_FastTick();
    assert(control_calls == 0U && observer_calls == 0U && progress_calls == 1U);
    initialize();
    s_foc_state.bus_voltage_filtered_v = 30.0f;
    configure_controllers();
    assert(s_foc_state.id_pi.output_max == FOC_MaximumVoltage(30.0f));
    assert(s_foc_state.iq_pi.output_min == -FOC_MaximumVoltage(30.0f));
    assert(state_initializations >= 7U);
    puts("PASS startup offset/PI initialization and sampled voltage limits preserve PI state");
    return 0;
}
