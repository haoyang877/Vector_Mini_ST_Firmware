#include <assert.h>
#include <stddef.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "motor_axis.h"
#include "foc.h"
#include "motor_fault.h"
#include "motor_sensing.h"
#include "flux_observer.h"
#include "board_config.h"
#include "bsp_adc.h"
#include "bsp_pwm.h"
#include "motor_lifecycle.h"
#include "encoder_calibration.h"
#include "motor_control.h"
#include "control_math.h"
#include "motor_lifecycle.h"
#include "bsp_pwm.h"
#include "motor_axis.h"
#include "motor_thermal_control.h"
#include "bsp_system.h"
#include "application_config.h"

/* 只替换启动外设/存储边界；初始化、校正、换算和PI实现采用生产体。 */
static MotorConfiguration s_motor_configuration;
static MotorControlState s_motor_state;
static FocState s_foc_state;
static PiController s_speed_pi;
static EncoderState s_rotor_encoder;
static FluxObserverState s_flux_observer;
static SensorlessStartupState s_sensorless_startup;
static uint16_t samples[3];
static float bus_sample_raw, bsp_bus_offset;
static unsigned bus_raw_reads, phase_offset_writes, bus_offset_writes;
static bool enable_on_commit, mode_on_commit;
static bool inject_raw;
static float injected_raw;
static bool output_off = true;
static float simulated_bus_voltage_v = 30.0f;
static unsigned current_reads, state_initializations;
static unsigned fast_frame_reads;
static bool frame_available = true;
static unsigned observer_calls, progress_calls, protection_calls, control_calls;
static MotorState simulated_state = MOTOR_STATE_IDLE;
static MotorCalibrationType simulated_calibration_type = MOTOR_CALIBRATION_NONE;
static MotorTaskResult simulated_calibration_outcome;
static bool calibration_publication_pending, calibration_publication_succeeds;
static unsigned calibration_cycle_calls, calibration_publication_calls;

MotorCalibrationType MotorLifecycle_GetCalibrationType(void)
{
    return simulated_calibration_type;
}

bool MotorLifecycle_IsCalibrationRunnable(void)
{
    return simulated_state == MOTOR_STATE_CALIBRATION &&
           simulated_calibration_type != MOTOR_CALIBRATION_NONE;
}

bool MotorLifecycle_ReadPendingCalibrationPublication(uint32_t *calibration_epoch)
{
    if (!calibration_publication_pending)
    {
        return false;
    }
    *calibration_epoch = 1U;
    return true;
}

void MotorLifecycle_CompleteCalibrationPublication(uint32_t calibration_epoch,
                                                   bool publication_succeeded)
{
    assert(calibration_epoch == 1U && output_off);
    assert(publication_succeeded == calibration_publication_succeeds);
    calibration_publication_pending = false;
    ++calibration_publication_calls;
}

MotorTaskResult EncoderCalibration_StepLut(const EncoderCalibrationControl *control,
                                           EncoderCalibrationWorkspace *workspace)
{
    assert(control->configuration == &s_motor_configuration);
    assert(control->foc == &s_foc_state && control->motor == &s_motor_state &&
           control->speed_pi == &s_speed_pi);
    assert(control->encoder == &s_rotor_encoder && control->observer == &s_flux_observer &&
           control->startup == &s_sensorless_startup && workspace != NULL);
    ++calibration_cycle_calls;
    return simulated_calibration_outcome;
}

MotorTaskResult EncoderCalibration_StepElecZero(const EncoderCalibrationControl *control)
{
    assert(control->configuration == &s_motor_configuration);
    assert(control->foc == &s_foc_state && control->motor == &s_motor_state &&
           control->encoder == &s_rotor_encoder);
    ++calibration_cycle_calls;
    return simulated_calibration_outcome;
}

bool Encoder_SetMechanicalZero(EncoderCalibration *calibration, EncoderState *encoder)
{
    assert(calibration == &s_motor_configuration.device.encoder && encoder == &s_rotor_encoder);
    ++calibration_cycle_calls;
    return true;
}

bool EncoderCalibration_PublishLut(EncoderCalibration *calibration, EncoderState *encoder)
{
    assert(output_off && calibration == &s_motor_configuration.device.encoder);
    assert(encoder == &s_rotor_encoder);
    return calibration_publication_succeeds;
}
static bool simulated_defaults;
static bool simulated_save;
static unsigned velocity_restarts;
static unsigned critical_depth;
static bool trace_enabled, inject_control_fault;
static char fast_trace[32];
static unsigned fast_trace_length;

/* 只在指定场景记录调度边界，正常零偏窗口不产生额外记录。 */
static void record_fast_call(char event)
{
    if (trace_enabled)
    {
        assert(fast_trace_length + 1U < sizeof(fast_trace));
        fast_trace[fast_trace_length++] = event;
        fast_trace[fast_trace_length] = '\0';
    }
}

uint32_t BspSystem_EnterCritical(void)
{
    if (enable_on_commit)
    {
        output_off = false;
    }
    if (mode_on_commit)
    {
        s_motor_state.control_mode = MOTOR_CONTROL_CURRENT;
    }
    return critical_depth++;
}

void BspSystem_ExitCritical(uint32_t previous)
{
    assert(critical_depth == previous + 1U);
    critical_depth = previous;
}

static void load_parameters(MotorConfiguration *configuration)
{
    assert(configuration == &s_motor_configuration);
    /* 模拟已存储的不同d/q增益，确保初始化不会覆盖为编译默认值。 */
    MotorWindingThermal_Default(&s_motor_configuration.model.thermal);
    s_motor_configuration.application.current_limit_a = 6.0f;
    s_motor_configuration.model.id_kp = 2.0f;
    s_motor_configuration.model.id_ki = 3.0f;
    s_motor_configuration.model.iq_kp = 4.0f;
    s_motor_configuration.model.iq_ki = 5.0f;
    s_motor_configuration.application.speed_kp = 0.2f;
    s_motor_configuration.application.speed_ki = 0.3f;
    s_motor_configuration.application.axis_profile_valid = true;
    s_motor_configuration.device.phase_a_offset_counts = 2048.0f;
    s_motor_configuration.device.phase_b_offset_counts = 2048.0f;
    s_motor_configuration.device.phase_c_offset_counts = 2048.0f;
}

void MotorLifecycle_Init(const MotorConfiguration *configuration,
                         MotorControlState *motor,
                         FocState *foc,
                         PiController *pi,
                         EncoderState *encoder,
                         SensorlessStartupState *startup)
{
    (void)pi;
    (void)encoder;
    (void)startup;
    ++state_initializations;
    motor->control_mode = MOTOR_CONTROL_NONE;
    assert(!MotorCurrentOffsets_IsValid(configuration) && isnan(foc->phase_current_a.a));
}
void Encoder_InitFeedback(EncoderCalibration *calibration,
                          const int32_t *pole_pairs,
                          const MotorAxisProfile *axis_profile,
                          const bool *axis_profile_valid,
                          EncoderState *encoder)
{
    assert(calibration == &s_motor_configuration.device.encoder);
    assert(pole_pairs == &s_motor_configuration.model.motor_pole_pairs);
    assert(axis_profile == &s_motor_configuration.application.axis_profile);
    assert(axis_profile_valid == &s_motor_configuration.application.axis_profile_valid);
    (void)encoder;
}
void FluxObserver_Init(FluxObserverState *observer)
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

bool BspPwm_ArePhasesDisabled(void)
{
    return output_off;
}
static float bsp_offsets[3];
float BspAdc_ReadRawChannel(BspAdcChannel channel)
{
    if (channel == BSP_ADC_BUS_CURRENT)
    {
        ++bus_raw_reads;
        return bus_sample_raw;
    }
    assert(channel <= BSP_ADC_PHASE_CURRENT_C);
    if (channel == BSP_ADC_PHASE_CURRENT_A)
    {
        ++current_reads;
    }
    return inject_raw && channel == BSP_ADC_PHASE_CURRENT_B ? injected_raw
                                                            : samples[channel] * .25f;
}
void BspAdc_SetPhaseOffsets(const float offsets[3])
{
    assert(output_off && critical_depth == 1U);
    ++phase_offset_writes;
    memcpy(bsp_offsets, offsets, sizeof(bsp_offsets));
}
void BspAdc_SetBusCurrentOffset(float offset_counts)
{
    assert(output_off && critical_depth == 1U && phase_offset_writes == 1U);
    ++bus_offset_writes;
    bsp_bus_offset = offset_counts;
}
float BspAdc_ReadChannel(BspAdcChannel channel)
{
    if (channel == BSP_ADC_BUS_VOLTAGE)
    {
        record_fast_call('A');
        ++fast_frame_reads;
        return frame_available ? simulated_bus_voltage_v : NAN;
    }
    if (channel == BSP_ADC_BUS_CURRENT)
    {
        return BSP_ENABLED_BUS_CURRENT && frame_available ? (bus_sample_raw - bsp_bus_offset) * 0.1f
                                                          : NAN;
    }
    assert(channel <= BSP_ADC_PHASE_CURRENT_C);
    return frame_available
               ? -(samples[channel] * .25f - bsp_offsets[channel]) * BSP_ADC_CURRENT_A_PER_COUNT
               : NAN;
}

void MotorFault_Report(MotorControlState *motor, MotorFaultCode error)
{
    if (motor->fault_code == MOTOR_FAULT_NONE)
    {
        motor->fault_code = error;
    }
}

void BspEncoder_Begin(void)
{
    record_fast_call('B');
}
void Encoder_Update(EncoderState *encoder)
{
    record_fast_call('E');
    assert(!encoder->velocity_restart_requested);
    encoder->velocity_restart_requested = false;
}
void Encoder_ResetVelocity(EncoderState *encoder)
{
    record_fast_call('Z');
    assert(output_off && simulated_save);
    ++velocity_restarts;
    encoder->velocity_restart_requested = true;
}
uint32_t BspSystem_TimeMs(void)
{
    return 0U;
}
float Encoder_GetMechPos(const EncoderState *encoder)
{
    return 0.0f;
}
float Encoder_GetMechVel(const EncoderState *encoder)
{
    return 0.0f;
}
void MotorVoltage_Update(FocState *foc)
{
    float bus_voltage_v = BspAdc_ReadChannel(BSP_ADC_BUS_VOLTAGE);
    record_fast_call('V');
    foc->bus_voltage_v = foc->bus_voltage_filtered_v = bus_voltage_v;
}
void FluxObserver_Update(const MotorModelConfig *model,
                         const AlphaBetaVector *current_ab_a,
                         const AlphaBetaVector *voltage_ab_v,
                         FluxObserverState *observer)
{
    (void)model;
    (void)current_ab_a;
    (void)voltage_ab_v;
    (void)observer;
    record_fast_call('O');
    ++observer_calls;
}
bool MotorLifecycle_OutputExpected(void)
{
    return !output_off;
}

MotorState MotorLifecycle_GetState(void)
{
    return simulated_state;
}
static void check_fast_faults(void)
{
    record_fast_call('F');
    ++protection_calls;
}
static void fast_fault_stop(void)
{
    record_fast_call('S');
    if (s_motor_state.fault_code != MOTOR_FAULT_NONE)
    {
        output_off = true;
    }
}
uint32_t MotorLifecycle_WorkEpoch(void)
{
    record_fast_call('W');
    return 1U;
}
void MotorLifecycle_PostOutcome(MotorTaskResult outcome, uint32_t epoch)
{
    record_fast_call('T');
    assert(epoch == 1U);
    assert(outcome.result == MOTOR_TASK_RUNNING ||
           simulated_calibration_type != MOTOR_CALIBRATION_NONE);
    ++progress_calls;
    fast_fault_stop();
}
void BspPwm_SetDuty(float duty_a, float duty_b, float duty_c)
{
    record_fast_call('P');
    assert(duty_a == 0.5f && duty_b == 0.5f && duty_c == 0.5f);
}
MotorTaskResult MotorControl_Step(const MotorConfiguration *configuration,
                                  FocState *foc,
                                  MotorControlState *motor,
                                  EncoderState *encoder,
                                  FluxObserverState *observer)
{
    record_fast_call('C');
    if (inject_control_fault)
    {
        MotorFault_Report(motor, MOTOR_FAULT_OVERCURRENT);
    }
    ++control_calls;
    return (MotorTaskResult){MOTOR_TASK_RUNNING, MOTOR_FAULT_NONE};
}

/* STARTUP_CURRENT_OFFSET_PRODUCTION：脚本插入真实生产体。 */

static void sample_test_currents(void)
{
    MotorAxis_SampleFast();
    MotorAxis_ControlFast();
}

static void initialize(void)
{
    memset(&s_motor_state, 0, sizeof(s_motor_state));
    memset(&s_foc_state, 0, sizeof(s_foc_state));
    memset(&s_speed_pi, 0, sizeof(s_speed_pi));
    output_off = true;
    simulated_defaults = simulated_save = false;
    trace_enabled = inject_control_fault = false;
    critical_depth = 0U;
    enable_on_commit = mode_on_commit = false;
    bus_sample_raw = 2049.0f;
    bsp_bus_offset = NAN;
    bus_raw_reads = phase_offset_writes = bus_offset_writes = 0U;
    simulated_bus_voltage_v = 30.0f;
    current_reads = 0U;
    inject_raw = false;
    fast_frame_reads = 0U;
    frame_available = true;
    observer_calls = progress_calls = protection_calls = control_calls = 0U;
    simulated_state = MOTOR_STATE_IDLE;
    simulated_calibration_type = MOTOR_CALIBRATION_NONE;
    simulated_calibration_outcome = (MotorTaskResult){MOTOR_TASK_RUNNING, MOTOR_FAULT_NONE};
    calibration_publication_pending = false;
    calibration_publication_succeeds = true;
    calibration_cycle_calls = calibration_publication_calls = 0U;
    samples[0] = 8131U;
    samples[1] = 8205U;
    samples[2] = 8146U;
    s_foc_state.id_pi.integral_output = s_foc_state.iq_pi.integral_output = 2.0f;
    s_speed_pi.integral_output = 2.0f;
    MotorAxis_Init();
    assert(s_foc_state.bus_voltage_filter_alpha == 0.05f);
    assert(s_foc_state.bus_current_filter_alpha == 0.05f);
    assert(isnan(s_foc_state.bus_current_a));
    assert(isnan(s_foc_state.bus_current_filtered_a));
    assert(s_motor_state.control_mode == MOTOR_CONTROL_NONE);
    assert(s_foc_state.id_pi.proportional_gain == 2.0f && s_foc_state.id_pi.integral_gain == 3.0f);
    assert(s_foc_state.iq_pi.proportional_gain == 4.0f && s_foc_state.iq_pi.integral_gain == 5.0f);
    assert(s_speed_pi.proportional_gain == 0.2f && s_speed_pi.integral_gain == 0.3f);
    assert(s_foc_state.id_pi.sample_period_s == FOC_TS_S);
    assert(s_foc_state.iq_pi.sample_period_s == FOC_TS_S &&
           s_speed_pi.sample_period_s == SPEED_LOOP_TS_S);
    assert(s_foc_state.id_pi.output_max == 0.0f && s_foc_state.iq_pi.output_min == 0.0f);
    assert(s_speed_pi.output_min == -1.0f && s_speed_pi.output_max == 1.0f);
    assert(s_foc_state.id_pi.integral_output == 0.0f && s_foc_state.iq_pi.integral_output == 0.0f);
    assert(s_speed_pi.integral_output == 0.0f);
    assert(!MotorCurrentOffsets_IsValid(&s_motor_configuration));
}

static void test_full_window(void)
{
    initialize();
    for (unsigned i = 0U; i < CURRENT_OFFSET_SAMPLE_COUNT; ++i)
    {
        samples[0] = (i & 1U) ? 8131U : 8130U;
        bus_sample_raw = BSP_ENABLED_BUS_CURRENT ? ((i & 1U) ? 2050.0f : 2049.0f) : NAN;
        assert(!s_current_offset_calibration.complete);
        MotorAxis_SampleFast();
        MotorAxis_ControlFast();
        if (i < CURRENT_OFFSET_SAMPLE_COUNT - 1U)
        {
            assert(phase_offset_writes == 0U && bus_offset_writes == 0U);
            assert(isnan(s_foc_state.phase_current_a.a));
            assert(isnan(s_foc_state.bus_current_a));
        }
        assert(output_off && s_motor_state.fault_code == MOTOR_FAULT_NONE);
    }
    assert(s_current_offset_calibration.complete && !s_current_offset_calibration.failed);
    assert(phase_offset_writes == 1U && bus_offset_writes == (BSP_ENABLED_BUS_CURRENT ? 1U : 0U));
    assert(bus_raw_reads == (BSP_ENABLED_BUS_CURRENT ? CURRENT_OFFSET_SAMPLE_COUNT : 0U));
    assert(critical_depth == 0U);
    assert(MotorCurrentOffsets_IsValid(&s_motor_configuration));
    assert(s_motor_configuration.device.phase_a_offset_counts == 2032.625f);
    assert(s_motor_configuration.device.phase_b_offset_counts == 2051.25f);
    assert(s_motor_configuration.device.phase_c_offset_counts == 2036.5f);
    if (BSP_ENABLED_BUS_CURRENT)
    {
        assert(bsp_bus_offset == 2049.5f);
        assert(fabsf(s_foc_state.bus_current_a - 0.05f) < 1e-6f);
    }
    else
    {
        assert(isnan(s_foc_state.bus_current_a));
    }
    unsigned prior_raw_reads = current_reads + bus_raw_reads;
    uint32_t old_sum = s_current_offset_calibration.phase_sum_raw[0];
    for (unsigned i = 0U; i < 20U; ++i)
    {
        MotorAxis_SampleFast();
        MotorAxis_ControlFast();
    }
    assert(current_reads + bus_raw_reads == prior_raw_reads);
    assert(s_current_offset_calibration.phase_sum_raw[0] == old_sum);
    assert(phase_offset_writes == 1U && bus_offset_writes == (BSP_ENABLED_BUS_CURRENT ? 1U : 0U));

    /* 控制器复位只清PI，不能清除同一FOC对象中已发布的测量。 */
    FocState measured = s_foc_state;
    s_foc_state.id_pi.integral_output = 1.0f;
    s_foc_state.iq_pi.integral_output = -1.0f;
    Foc_ResetCurrent(&s_foc_state);
    measured.id_pi = s_foc_state.id_pi;
    measured.iq_pi = s_foc_state.iq_pi;
    assert(memcmp(&measured, &s_foc_state, sizeof(measured)) == 0);
    assert(s_foc_state.id_pi.integral_output == 0.0f);
    assert(s_foc_state.iq_pi.integral_output == 0.0f);
}

static void test_rejections(void)
{
    const float invalid_raw[] = {NAN, INFINITY, -INFINITY, -1.0f, 4096.0f};
    for (unsigned i = 0U; i < sizeof(invalid_raw) / sizeof(invalid_raw[0]); ++i)
    {
        initialize();
        inject_raw = true;
        injected_raw = invalid_raw[i];
        sample_test_currents();
        assert(s_current_offset_calibration.failed &&
               s_current_offset_calibration.sample_count == 0U);
        assert(s_motor_state.fault_code == MOTOR_FAULT_CURRENT_OFFSET);
        unsigned reads = current_reads + bus_raw_reads;
        inject_raw = false;
        sample_test_currents();
        assert(current_reads + bus_raw_reads == reads);
        assert(!s_current_offset_calibration.complete && phase_offset_writes == 0U &&
               bus_offset_writes == 0U);
        if (BSP_ENABLED_BUS_CURRENT)
        {
            initialize();
            bus_sample_raw = invalid_raw[i];
            sample_test_currents();
            assert(s_current_offset_calibration.failed && !s_current_offset_calibration.complete);
            assert(phase_offset_writes == 0U && bus_offset_writes == 0U);
        }
    }
    initialize();
    output_off = false;
    sample_test_currents();
    assert(s_current_offset_calibration.failed && current_reads == 0U && bus_raw_reads == 0U);
    assert(output_off && s_motor_state.fault_code == MOTOR_FAULT_CURRENT_OFFSET);

    initialize();
    s_motor_state.control_mode = MOTOR_CONTROL_CURRENT;
    sample_test_currents();
    assert(s_current_offset_calibration.failed && current_reads == 0U);

    for (unsigned race = 0U; race < 2U; ++race)
    {
        initialize();
        for (unsigned i = 0U; i < CURRENT_OFFSET_SAMPLE_COUNT - 1U; ++i)
        {
            sample_test_currents();
        }
        enable_on_commit = race == 0U;
        mode_on_commit = race == 1U;
        sample_test_currents();
        assert(s_current_offset_calibration.failed && !s_current_offset_calibration.complete);
        assert(phase_offset_writes == 0U && bus_offset_writes == 0U);
        assert(critical_depth == 0U && isnan(s_motor_configuration.device.phase_a_offset_counts));
    }
    initialize();
    s_motor_state.fault_code = MOTOR_FAULT_ENCODER;
    inject_raw = true;
    injected_raw = NAN;
    sample_test_currents();
    assert(s_motor_state.fault_code == MOTOR_FAULT_ENCODER);
}

static void test_idle_only(void)
{
    for (MotorState state = MOTOR_STATE_SELF_TEST; state <= MOTOR_STATE_FAULT; ++state)
    {
        initialize();
        simulated_state = state;
        for (unsigned i = 0U; i < 10U; ++i)
        {
            MotorAxis_SampleFast();
            MotorAxis_ControlFast();
        }
        assert(current_reads == 0U && bus_raw_reads == 0U);
        assert(phase_offset_writes == 0U && bus_offset_writes == 0U);
        assert(!s_current_offset_calibration.complete &&
               s_current_offset_calibration.sample_count == 0U);
    }
    initialize();
    for (unsigned i = 0U; i < CURRENT_OFFSET_SAMPLE_COUNT; ++i)
    {
        sample_test_currents();
    }
    unsigned reads = current_reads + bus_raw_reads;
    for (MotorState state = MOTOR_STATE_IDLE; state <= MOTOR_STATE_FAULT; ++state)
    {
        simulated_state = state;
        MotorAxis_SampleFast();
        MotorAxis_ControlFast();
    }
    assert(current_reads + bus_raw_reads == reads);
    assert(phase_offset_writes == 1U && bus_offset_writes == (BSP_ENABLED_BUS_CURRENT ? 1U : 0U));
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
        float maximum_voltage_v = Foc_MaxVoltage(simulated_bus_voltage_v);
        expected_id.output_min = expected_iq.output_min = -maximum_voltage_v;
        expected_id.output_max = expected_iq.output_max = maximum_voltage_v;
        MotorAxis_SampleFast();
        MotorAxis_ControlFast();
        assert(memcmp(&s_foc_state.id_pi, &expected_id, sizeof(expected_id)) == 0);
        assert(memcmp(&s_foc_state.iq_pi, &expected_iq, sizeof(expected_iq)) == 0);
        assert(memcmp(&s_speed_pi, &expected_speed, sizeof(expected_speed)) == 0);
        assert(control_calls == 0U && observer_calls == 0U && output_off);
        assert(s_motor_state.fault_code == MOTOR_FAULT_NONE);
    }
}

/* B/A/V/R/O/Z/E/F/S/W/Q/C/M/P/T依次表示事务、ADC、电压、读回、观测器、
 * 测速重置、编码器完成、故障检查、关断检查、代际、许可、控制、维护、预载、发布。 */
static void expect_fast_tick_order(const char *expected)
{
    unsigned prior_reads = fast_frame_reads;
    fast_trace_length = 0U;
    trace_enabled = true;
    MotorAxis_SampleFast();
    MotorAxis_ControlFast();
    trace_enabled = false;
    assert(strcmp(fast_trace, expected) == 0);
    assert(fast_frame_reads == prior_reads + 1U);
}

static void prepare_fast_tick_order(MotorState state, MotorControlMode mode)
{
    initialize();
    s_current_offset_calibration.complete = true;
    s_motor_configuration.device.phase_a_offset_counts = 2048.0f;
    s_motor_configuration.device.phase_b_offset_counts = 2048.0f;
    s_motor_configuration.device.phase_c_offset_counts = 2048.0f;
    for (unsigned phase = 0U; phase < 3U; ++phase)
    {
        bsp_offsets[phase] = 2048.0f;
        samples[phase] = 8192U;
    }
    simulated_state = state;
    s_motor_state.control_mode = mode;
}

static void test_fast_tick_order(void)
{
    initialize();
    expect_fast_tick_order("BAVEFSWTS");

    prepare_fast_tick_order(MOTOR_STATE_RUNNING, MOTOR_CONTROL_CURRENT);
    expect_fast_tick_order("BAVOEFSWCTS");
    prepare_fast_tick_order(MOTOR_STATE_RUNNING, MOTOR_CONTROL_SPEED);
    expect_fast_tick_order("BAVOEFSWCTS");
    prepare_fast_tick_order(MOTOR_STATE_RUNNING, MOTOR_CONTROL_POSITION);
    expect_fast_tick_order("BAVEFSWCTS");
    prepare_fast_tick_order(MOTOR_STATE_IDLE, MOTOR_CONTROL_NONE);
    expect_fast_tick_order("BAVOEFSWPTS");

    /* 故障前后均保留采样收尾；执行中新故障必须由末尾的关断检查接住。 */
    prepare_fast_tick_order(MOTOR_STATE_RUNNING, MOTOR_CONTROL_CURRENT);
    output_off = false;
    s_motor_state.fault_code = MOTOR_FAULT_ENCODER;
    expect_fast_tick_order("BAVOEFSWTS");
    assert(output_off && s_motor_state.fault_code == MOTOR_FAULT_ENCODER);
    prepare_fast_tick_order(MOTOR_STATE_RUNNING, MOTOR_CONTROL_CURRENT);
    output_off = false;
    inject_control_fault = true;
    expect_fast_tick_order("BAVOEFSWCTS");
    assert(output_off && s_motor_state.fault_code == MOTOR_FAULT_OVERCURRENT);
}

static void test_offset_limits(void)
{
    const uint16_t raw_sums[] = {1948U * 4U, 2148U * 4U, 1948U * 4U - 1U, 2148U * 4U + 1U};
    for (unsigned test = 0U; test < sizeof(raw_sums) / sizeof(raw_sums[0]); ++test)
    {
        initialize();
        samples[0] = samples[1] = samples[2] = raw_sums[test];
        for (unsigned sample = 0U; sample < CURRENT_OFFSET_SAMPLE_COUNT; ++sample)
        {
            sample_test_currents();
        }
        bool valid = test < 2U;
        assert(s_current_offset_calibration.complete == valid);
        assert(s_current_offset_calibration.failed != valid);
        assert(MotorCurrentOffsets_IsValid(&s_motor_configuration) == valid);
    }
}

static void test_bus_offset_limits(void)
{
    if (!BSP_ENABLED_BUS_CURRENT)
    {
        return;
    }
    const float bus_offsets[] = {1948.0f, 2148.0f, 1947.0f, 2149.0f};
    for (unsigned test = 0U; test < sizeof(bus_offsets) / sizeof(bus_offsets[0]); ++test)
    {
        initialize();
        bus_sample_raw = bus_offsets[test];
        for (unsigned i = 0U; i < CURRENT_OFFSET_SAMPLE_COUNT; ++i)
        {
            sample_test_currents();
        }
        bool valid = test < 2U;
        assert(s_current_offset_calibration.complete == valid);
        assert(s_current_offset_calibration.failed != valid);
        assert(phase_offset_writes == (valid ? 1U : 0U));
        assert(bus_offset_writes == (valid ? 1U : 0U));
        assert(MotorCurrentOffsets_IsValid(&s_motor_configuration) == valid);
    }
    initialize();
    for (unsigned i = 0U; i < CURRENT_OFFSET_SAMPLE_COUNT - 1U; ++i)
    {
        sample_test_currents();
    }
    bus_sample_raw = NAN;
    sample_test_currents();
    assert(s_current_offset_calibration.failed && !s_current_offset_calibration.complete);
    assert(phase_offset_writes == 0U && bus_offset_writes == 0U);
}

static void test_calibration_dispatch_and_publication(void)
{
    for (MotorCalibrationType calibration_type = MOTOR_CALIBRATION_ENCODER_LINEARIZATION;
         calibration_type <= MOTOR_CALIBRATION_MECHANICAL_ZERO;
         ++calibration_type)
    {
        prepare_fast_tick_order(MOTOR_STATE_CALIBRATION, MOTOR_CONTROL_NONE);
        simulated_calibration_type = calibration_type;
        simulated_calibration_outcome = (MotorTaskResult){
            calibration_type == MOTOR_CALIBRATION_ENCODER_LINEARIZATION ? MOTOR_TASK_FINALIZE
                                                                        : MOTOR_TASK_COMPLETED,
            MOTOR_FAULT_NONE};
        MotorAxis_SampleFast();
        MotorAxis_ControlFast();
        assert(calibration_cycle_calls == 1U && control_calls == 0U);
    }
    prepare_fast_tick_order(MOTOR_STATE_IDLE, MOTOR_CONTROL_NONE);
    calibration_publication_pending = true;
    MotorAxis_ProcessCalibrationPublication();
    assert(calibration_publication_calls == 1U);
    MotorAxis_ProcessCalibrationPublication();
    assert(calibration_publication_calls == 1U);
    calibration_publication_pending = true;
    calibration_publication_succeeds = false;
    MotorAxis_ProcessCalibrationPublication();
    assert(calibration_publication_calls == 2U);
    puts("PASS calibration dispatch: linearization/electrical/mechanical, publication");
}

int main(void)
{
    /* Watch类型与对象布局不随零偏计算的所属目录迁移而变化。 */
    assert(sizeof(CurrentOffsetCalibration) == 24U);
    assert(offsetof(CurrentOffsetCalibration, phase_sum_raw) == 0U);
    assert(offsetof(CurrentOffsetCalibration, bus_sum_raw) == 12U);
    assert(offsetof(CurrentOffsetCalibration, sample_count) == 16U);
    assert(offsetof(CurrentOffsetCalibration, complete) == 20U);
    assert(offsetof(CurrentOffsetCalibration, failed) == 21U);
    assert(CURRENT_OFFSET_SAMPLE_COUNT == 2048U);
    assert(MOTOR_CURRENT_OFFSET_SAMPLE_TIME_MS == 103U);
    test_full_window();
    test_rejections();
    test_idle_only();
    test_offset_limits();
    test_bus_offset_limits();
    test_voltage_limits_refresh_only();
    test_fast_tick_order();
    simulated_defaults = false;
    simulated_save = true;
    output_off = true;
    velocity_restarts = 0U;
    MotorAxis_SampleFast();
    MotorAxis_ControlFast();
    MotorAxis_SampleFast();
    MotorAxis_ControlFast();
    assert(velocity_restarts == 0U);
    simulated_save = false;
    MotorAxis_SampleFast();
    MotorAxis_ControlFast();
    assert(velocity_restarts == 0U);
    initialize();
    simulated_state = MOTOR_STATE_RUNNING;
    MotorAxis_SampleFast();
    MotorAxis_ControlFast();
    assert(control_calls == 0U && observer_calls == 0U && progress_calls == 1U);
    initialize();
    s_foc_state.bus_voltage_filtered_v = 30.0f;
    Foc_ConfigureControllers(&s_motor_configuration, &s_foc_state, &s_speed_pi);
    Foc_UpdateCurrentLimits(&s_foc_state);
    assert(s_foc_state.id_pi.output_max == Foc_MaxVoltage(30.0f));
    assert(s_foc_state.iq_pi.output_min == -Foc_MaxVoltage(30.0f));
    assert(state_initializations >= 7U);
    /* 无效通道仍发布NaN测量，不由采样搬运函数伪造故障或有效电流。 */
    s_current_offset_calibration.complete = true;
    s_motor_configuration.device.phase_a_offset_counts = 2048.0f;
    s_motor_configuration.device.phase_b_offset_counts = 2048.0f;
    s_motor_configuration.device.phase_c_offset_counts = 2048.0f;
    frame_available = false;
    output_off = false;
    unsigned prior_reads = fast_frame_reads;
    expect_fast_tick_order("BAVOEFSWPTS");
    assert(fast_frame_reads == prior_reads + 1U && !output_off);
    assert(s_motor_state.fault_code == MOTOR_FAULT_NONE);
    assert(isnan(s_foc_state.phase_current_a.a) && isnan(s_foc_state.phase_current_a.b) &&
           isnan(s_foc_state.phase_current_a.c));
    test_calibration_dispatch_and_publication();
    puts("PASS FastTick order: channel ADC, SPI overlap, core dispatch and two fault stops");
    puts("PASS IDLE-only shared current-offset window, phase/bus scale, atomic publish and failure "
         "freeze");
    puts("PASS startup offset/PI initialization and sampled voltage limits preserve PI state");
    return 0;
}

void MotorLifecycle_BeginFastTick(void)
{
    check_fast_faults();
    fast_fault_stop();
}
uint32_t BspSystem_EnterControlCritical(void)
{
    return BspSystem_EnterCritical();
}
void BspSystem_ExitControlCritical(uint32_t state)
{
    BspSystem_ExitCritical(state);
}
