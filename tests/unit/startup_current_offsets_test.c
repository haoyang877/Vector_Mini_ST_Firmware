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
#include "parameter_record.h"
#include "bsp_flash.h"

/* 只替换启动外设/存储边界；初始化、校正、换算和PI实现采用生产体。 */
static MotorAxis axis;
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
static uint32_t publication_epoch = 1U;
static EncoderCalibrationPhase simulated_phase;
EncoderCalibrationPhase EncoderCalibration_GetPhase(const EncoderCalibrationSession *instance)
{
    (void)instance;
    return simulated_phase;
}
static float hall_raw_sin = 2048.0f, hall_raw_cos = 2048.0f;
#if BSP_ENABLED_LINEAR_HALL
static uint32_t test_ms;
void MotorLifecycle_ContinueElectricalZero(MotorLifecycle *instance)
{
    (void)instance;
    assert(calibration_publication_pending);
    calibration_publication_pending = false;
    simulated_calibration_type = MOTOR_CALIBRATION_JOINT_ELECTRICAL_ZERO;
}
float Encoder_GetMechVelContinuous(const EncoderState *encoder)
{
    return 0.0f;
}
#endif

bool Encoder_IsOnline(const EncoderState *encoder)
{
    return encoder->has_valid_sample;
}
void Param_StoreOutputEncoder(MotorAxisRecord *record,
                              const OutputEncoder *encoder,
                              const LinearHall *hall)
{
    assert(record == &axis.record && encoder != NULL);
}
void Param_Upload(InterfaceParam *param, const MotorConfiguration *configuration)
{
    assert(param == &axis.record.motor && configuration == &axis.configuration);
}
void Param_StoreLinearHall(InterfaceParam *param, const LinearHall *hall)
{
    assert(param == &axis.record.motor && hall != NULL);
}
BspResult BspFlash_Write(const void *data, size_t length)
{
    assert(output_off && calibration_publication_pending);
    assert(data == &axis.record && length == sizeof(axis.record));
    return BSP_OK;
}

MotorCalibrationType MotorLifecycle_GetCalibrationType(const MotorLifecycle *instance)
{
    (void)instance;
    return simulated_state == MOTOR_STATE_CALIBRATION ? simulated_calibration_type
                                                      : MOTOR_CALIBRATION_NONE;
}

void MotorLifecycle_BeginOuter(MotorLifecycle *life, MotorOuterWork *work)
{
    (void)life;
    work->allowed = false;
}
void MotorOuterLoop_SlowTick(MotorOuterLoop *outer,
                             const MotorConfiguration *config,
                             MotorControlState *motor,
                             PiController *pi,
                             EncoderState *encoder)
{
    (void)outer;
    (void)config;
    (void)motor;
    (void)pi;
    (void)encoder;
    assert(false);
}
void MotorLifecycle_CommitOuter(MotorLifecycle *life, const MotorOuterWork *work)
{
    (void)life;
    (void)work;
}

void Encoder_UpdateSlowEstimate(EncoderState *encoder)
{
    assert(encoder == &axis.rotor_encoder);
}

bool MotorLifecycle_IsCalibrationRunnable(const MotorLifecycle *instance)
{
    (void)instance;
    return simulated_state == MOTOR_STATE_CALIBRATION &&
           simulated_calibration_type != MOTOR_CALIBRATION_NONE;
}

bool MotorLifecycle_ReadPendingCalibrationPublication(const MotorLifecycle *instance,
                                                      uint32_t *calibration_epoch)
{
    (void)instance;
    if (!calibration_publication_pending)
    {
        return false;
    }
    *calibration_epoch = publication_epoch;
    return true;
}

void MotorLifecycle_CompleteCalibrationPublication(MotorLifecycle *instance,
                                                   uint32_t calibration_epoch,
                                                   bool publication_succeeded)
{
    (void)instance;
    assert(calibration_epoch == publication_epoch && output_off);
    assert(publication_succeeded == calibration_publication_succeeds);
    calibration_publication_pending = false;
    ++calibration_publication_calls;
}

MotorTaskResult EncoderCalibration_StepLut(EncoderCalibrationSession *instance,
                                           const EncoderCalibrationControl *control,
                                           EncoderCalibrationWorkspace *workspace)
{
    (void)instance;
    assert(control->configuration == &axis.configuration);
    assert(control->foc == &axis.foc && control->motor == &axis.motor);
    assert(control->encoder == &axis.rotor_encoder && control->observer == &axis.observer &&
           control->startup == &axis.startup && workspace != NULL);
    ++calibration_cycle_calls;
    return simulated_calibration_outcome;
}

MotorTaskResult EncoderCalibration_StepElecZero(EncoderCalibrationSession *instance,
                                                const EncoderCalibrationControl *control)
{
    (void)instance;
    assert(control->configuration == &axis.configuration);
    assert(control->foc == &axis.foc && control->motor == &axis.motor &&
           control->encoder == &axis.rotor_encoder);
    ++calibration_cycle_calls;
    return simulated_calibration_outcome;
}

bool Encoder_SetMechanicalZero(EncoderCalibration *calibration, EncoderState *encoder)
{
    assert(calibration == &axis.configuration.device.encoder && encoder == &axis.rotor_encoder);
    ++calibration_cycle_calls;
    return true;
}

bool EncoderCalibration_PublishLut(EncoderCalibrationSession *instance,
                                   EncoderCalibration *calibration,
                                   EncoderState *encoder)
{
    (void)instance;
    assert(output_off && calibration == &axis.configuration.device.encoder);
    assert(encoder == &axis.rotor_encoder);
    return calibration_publication_succeeds;
}
static bool simulated_defaults;
static bool simulated_save;
static unsigned velocity_restarts;
static unsigned critical_depth;
static bool update_hall_on_unlock;
static void inject_hall_after_snapshot(void);
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
    return critical_depth++;
}

void BspSystem_ExitCritical(uint32_t previous)
{
    assert(critical_depth == previous + 1U);
    critical_depth = previous;
}

void BspSystem_MemoryBarrier(void)
{
    inject_hall_after_snapshot();
}

static void load_parameters(MotorConfiguration *configuration)
{
    assert(configuration == &axis.configuration);
    /* 模拟已存储的不同d/q增益，确保初始化不会覆盖为编译默认值。 */
    MotorWindingThermal_Default(&axis.configuration.model.thermal);
    axis.configuration.application.current_limit_a = 6.0f;
    axis.configuration.model.id_kp = 2.0f;
    axis.configuration.model.id_ki = 3.0f;
    axis.configuration.model.iq_kp = 4.0f;
    axis.configuration.model.iq_ki = 5.0f;
    axis.configuration.application.speed_kp = 0.2f;
    axis.configuration.application.speed_ki = 0.3f;
    axis.configuration.application.axis_profile_valid = true;
    axis.configuration.device.phase_a_offset_counts = 2048.0f;
    axis.configuration.device.phase_b_offset_counts = 2048.0f;
    axis.configuration.device.phase_c_offset_counts = 2048.0f;
}

void MotorLifecycle_Init(MotorLifecycle *life, const MotorLifecycleBindings *bindings)
{
    assert(life == &axis.lifecycle);
    ++state_initializations;
    bindings->motor->control_mode = MOTOR_CONTROL_NONE;
    assert(!MotorCurrentOffsets_IsValid(bindings->configuration) &&
           isnan(bindings->foc->phase_current_a.a));
}
void Encoder_InitFeedback(EncoderCalibration *calibration,
                          const int32_t *pole_pairs,
                          const MotorAxisProfile *axis_profile,
                          const bool *axis_profile_valid,
                          EncoderState *encoder)
{
    assert(calibration == &axis.configuration.device.encoder);
    assert(pole_pairs == &axis.configuration.model.motor_pole_pairs);
    assert(axis_profile == &axis.configuration.application.axis_profile);
    assert(axis_profile_valid == &axis.configuration.application.axis_profile_valid);
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
void MotorOuterLoop_RequestReset(MotorOuterLoop *instance)
{
    (void)instance;
}

bool BspPwm_ArePhasesDisabled(void)
{
    return output_off;
}
static float bsp_offsets[3];
float BspAdc_ReadRawChannel(BspAdcChannel channel)
{
    if (channel == BSP_ADC_PHASE_CURRENT_C && enable_on_commit)
    {
        output_off = false;
    }
    if (channel == BSP_ADC_PHASE_CURRENT_C && mode_on_commit)
    {
        axis.motor.control_mode = MOTOR_CONTROL_CURRENT;
    }
    if (channel == BSP_ADC_LINEAR_HALL_SIN || channel == BSP_ADC_LINEAR_HALL_COS)
    {
        return channel == BSP_ADC_LINEAR_HALL_SIN ? hall_raw_sin : hall_raw_cos;
    }
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
    assert(output_off && critical_depth == 0U && !axis.current_offsets.complete);
    ++phase_offset_writes;
    memcpy(bsp_offsets, offsets, sizeof(bsp_offsets));
}
void BspAdc_SetBusCurrentOffset(float offset_counts)
{
    assert(output_off && critical_depth == 0U && !axis.current_offsets.complete &&
           phase_offset_writes == 1U);
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
#if BSP_ENABLED_LINEAR_HALL
    return test_ms;
#else
    return 0U;
#endif
}
float Encoder_GetMechPos(const EncoderState *encoder)
{
    return encoder->mechanical_position_rad;
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

MotorState MotorLifecycle_GetState(const MotorLifecycle *instance)
{
    (void)instance;
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
    if (axis.motor.fault_code != MOTOR_FAULT_NONE)
    {
        output_off = true;
    }
}
uint32_t MotorLifecycle_WorkEpoch(const MotorLifecycle *instance)
{
    (void)instance;
    record_fast_call('W');
    return 1U;
}
void MotorLifecycle_PostOutcome(MotorLifecycle *instance, MotorTaskResult outcome, uint32_t epoch)
{
    (void)instance;
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

static const MotorAxisHardware s_board_hardware = {
    .power = {.phases_disabled = BspPwm_ArePhasesDisabled, .set_duty = BspPwm_SetDuty},
    .write_record = BspFlash_Write};

/* STARTUP_CURRENT_OFFSET_PRODUCTION：脚本插入真实生产体。 */

static void sample_test_currents(void)
{
    MotorAxis_SampleFast(&axis);
    MotorAxis_ControlFast(&axis);
}

static void initialize(void)
{
    memset(&axis.motor, 0, sizeof(axis.motor));
    memset(&axis.foc, 0, sizeof(axis.foc));
    memset(&axis.speed_pi, 0, sizeof(axis.speed_pi));
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
    axis.foc.id_pi.integral_output = axis.foc.iq_pi.integral_output = 2.0f;
    axis.speed_pi.integral_output = 2.0f;
    MotorAxis_Init(&axis);
    assert(axis.foc.bus_voltage_filter_alpha == 0.05f);
    assert(axis.foc.bus_current_filter_alpha == 0.05f);
    assert(isnan(axis.foc.bus_current_a));
    assert(isnan(axis.foc.bus_current_filtered_a));
    assert(axis.motor.control_mode == MOTOR_CONTROL_NONE);
    assert(axis.foc.id_pi.proportional_gain == 2.0f && axis.foc.id_pi.integral_gain == 3.0f);
    assert(axis.foc.iq_pi.proportional_gain == 4.0f && axis.foc.iq_pi.integral_gain == 5.0f);
    assert(axis.speed_pi.proportional_gain == 0.2f && axis.speed_pi.integral_gain == 0.3f);
    assert(axis.foc.id_pi.sample_period_s == FOC_TS_S);
    assert(axis.foc.iq_pi.sample_period_s == FOC_TS_S &&
           axis.speed_pi.sample_period_s == SPEED_LOOP_TS_S);
    assert(axis.foc.id_pi.output_max == 0.0f && axis.foc.iq_pi.output_min == 0.0f);
    assert(axis.speed_pi.output_min == -1.0f && axis.speed_pi.output_max == 1.0f);
    assert(axis.foc.id_pi.integral_output == 0.0f && axis.foc.iq_pi.integral_output == 0.0f);
    assert(axis.speed_pi.integral_output == 0.0f);
    assert(!MotorCurrentOffsets_IsValid(&axis.configuration));
}

static void test_full_window(void)
{
    initialize();
    for (unsigned i = 0U; i < CURRENT_OFFSET_SAMPLE_COUNT; ++i)
    {
        samples[0] = (i & 1U) ? 8131U : 8130U;
        bus_sample_raw = BSP_ENABLED_BUS_CURRENT ? ((i & 1U) ? 2050.0f : 2049.0f) : NAN;
        assert(!axis.current_offsets.complete);
        MotorAxis_SampleFast(&axis);
        MotorAxis_ControlFast(&axis);
        if (i < CURRENT_OFFSET_SAMPLE_COUNT - 1U)
        {
            assert(phase_offset_writes == 0U && bus_offset_writes == 0U);
            assert(isnan(axis.foc.phase_current_a.a));
            assert(isnan(axis.foc.bus_current_a));
        }
        assert(output_off && axis.motor.fault_code == MOTOR_FAULT_NONE);
    }
    assert(axis.current_offsets.complete && !axis.current_offsets.failed);
    assert(phase_offset_writes == 1U && bus_offset_writes == (BSP_ENABLED_BUS_CURRENT ? 1U : 0U));
    assert(bus_raw_reads == (BSP_ENABLED_BUS_CURRENT ? CURRENT_OFFSET_SAMPLE_COUNT : 0U));
    assert(critical_depth == 0U);
    assert(MotorCurrentOffsets_IsValid(&axis.configuration));
    assert(axis.configuration.device.phase_a_offset_counts == 2032.625f);
    assert(axis.configuration.device.phase_b_offset_counts == 2051.25f);
    assert(axis.configuration.device.phase_c_offset_counts == 2036.5f);
    if (BSP_ENABLED_BUS_CURRENT)
    {
        assert(bsp_bus_offset == 2049.5f);
        assert(fabsf(axis.foc.bus_current_a - 0.05f) < 1e-6f);
    }
    else
    {
        assert(isnan(axis.foc.bus_current_a));
    }
    unsigned prior_raw_reads = current_reads + bus_raw_reads;
    uint32_t old_sum = axis.current_offsets.phase_sum_raw[0];
    for (unsigned i = 0U; i < 20U; ++i)
    {
        MotorAxis_SampleFast(&axis);
        MotorAxis_ControlFast(&axis);
    }
    assert(current_reads + bus_raw_reads == prior_raw_reads);
    assert(axis.current_offsets.phase_sum_raw[0] == old_sum);
    assert(phase_offset_writes == 1U && bus_offset_writes == (BSP_ENABLED_BUS_CURRENT ? 1U : 0U));

    /* 控制器复位只清PI，不能清除同一FOC对象中已发布的测量。 */
    FocState measured = axis.foc;
    axis.foc.id_pi.integral_output = 1.0f;
    axis.foc.iq_pi.integral_output = -1.0f;
    Foc_ResetCurrent(&axis.foc);
    measured.id_pi = axis.foc.id_pi;
    measured.iq_pi = axis.foc.iq_pi;
    assert(memcmp(&measured, &axis.foc, sizeof(measured)) == 0);
    assert(axis.foc.id_pi.integral_output == 0.0f);
    assert(axis.foc.iq_pi.integral_output == 0.0f);
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
        assert(axis.current_offsets.failed && axis.current_offsets.sample_count == 0U);
        assert(axis.motor.fault_code == MOTOR_FAULT_CURRENT_OFFSET);
        unsigned reads = current_reads + bus_raw_reads;
        inject_raw = false;
        sample_test_currents();
        assert(current_reads + bus_raw_reads == reads);
        assert(!axis.current_offsets.complete && phase_offset_writes == 0U &&
               bus_offset_writes == 0U);
        if (BSP_ENABLED_BUS_CURRENT)
        {
            initialize();
            bus_sample_raw = invalid_raw[i];
            sample_test_currents();
            assert(axis.current_offsets.failed && !axis.current_offsets.complete);
            assert(phase_offset_writes == 0U && bus_offset_writes == 0U);
        }
    }
    initialize();
    output_off = false;
    sample_test_currents();
    assert(axis.current_offsets.failed && current_reads == 0U && bus_raw_reads == 0U);
    assert(output_off && axis.motor.fault_code == MOTOR_FAULT_CURRENT_OFFSET);

    initialize();
    axis.motor.control_mode = MOTOR_CONTROL_CURRENT;
    sample_test_currents();
    assert(axis.current_offsets.failed && current_reads == 0U);

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
        assert(axis.current_offsets.failed && !axis.current_offsets.complete);
        assert(phase_offset_writes == 0U && bus_offset_writes == 0U);
        assert(critical_depth == 0U && isnan(axis.configuration.device.phase_a_offset_counts));
    }
    initialize();
    axis.motor.fault_code = MOTOR_FAULT_ENCODER;
    inject_raw = true;
    injected_raw = NAN;
    sample_test_currents();
    assert(axis.motor.fault_code == MOTOR_FAULT_ENCODER);
}

static void test_idle_only(void)
{
    for (MotorState state = MOTOR_STATE_SELF_TEST; state <= MOTOR_STATE_FAULT; ++state)
    {
        initialize();
        simulated_state = state;
        for (unsigned i = 0U; i < 10U; ++i)
        {
            MotorAxis_SampleFast(&axis);
            MotorAxis_ControlFast(&axis);
        }
        assert(current_reads == 0U && bus_raw_reads == 0U);
        assert(phase_offset_writes == 0U && bus_offset_writes == 0U);
        assert(!axis.current_offsets.complete && axis.current_offsets.sample_count == 0U);
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
        MotorAxis_SampleFast(&axis);
        MotorAxis_ControlFast(&axis);
    }
    assert(current_reads + bus_raw_reads == reads);
    assert(phase_offset_writes == 1U && bus_offset_writes == (BSP_ENABLED_BUS_CURRENT ? 1U : 0U));
}

static void test_voltage_limits_refresh_only(void)
{
    /* 母线有效、升降和失效均经真实FastTick；校正pending不执行电流控制。 */
    static const float voltages_v[] = {32.84f, 26.0f, 33.0f, 0.0f, -1.0f, NAN, INFINITY};
    initialize();
    axis.foc.id_pi =
        (PiController){7.0f, 8.0f, 0.123f, -3.0f, 4.0f, 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};
    axis.foc.iq_pi =
        (PiController){9.0f, 10.0f, 0.124f, -5.0f, 6.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f};
    axis.speed_pi.output_min = -0.25f;
    axis.speed_pi.output_max = 0.75f;
    axis.speed_pi.integral_output = 0.4f;
    PiController expected_speed = axis.speed_pi;
    for (unsigned i = 0U; i < sizeof(voltages_v) / sizeof(voltages_v[0]); ++i)
    {
        PiController expected_id = axis.foc.id_pi;
        PiController expected_iq = axis.foc.iq_pi;
        simulated_bus_voltage_v = voltages_v[i];
        float maximum_voltage_v = Foc_MaxVoltage(simulated_bus_voltage_v);
        expected_id.output_min = expected_iq.output_min = -maximum_voltage_v;
        expected_id.output_max = expected_iq.output_max = maximum_voltage_v;
        MotorAxis_SampleFast(&axis);
        MotorAxis_ControlFast(&axis);
        assert(memcmp(&axis.foc.id_pi, &expected_id, sizeof(expected_id)) == 0);
        assert(memcmp(&axis.foc.iq_pi, &expected_iq, sizeof(expected_iq)) == 0);
        assert(memcmp(&axis.speed_pi, &expected_speed, sizeof(expected_speed)) == 0);
        assert(control_calls == 0U && observer_calls == 0U && output_off);
        assert(axis.motor.fault_code == MOTOR_FAULT_NONE);
    }
}

/* B/A/V/R/O/Z/E/F/S/W/Q/C/M/P/T依次表示事务、ADC、电压、读回、观测器、
 * 测速重置、编码器完成、故障检查、关断检查、代际、许可、控制、维护、预载、发布。 */
static void expect_fast_tick_order(const char *expected)
{
    unsigned prior_reads = fast_frame_reads;
    fast_trace_length = 0U;
    trace_enabled = true;
    MotorAxis_SampleFast(&axis);
    MotorAxis_ControlFast(&axis);
    trace_enabled = false;
    assert(strcmp(fast_trace, expected) == 0);
    assert(fast_frame_reads == prior_reads + 1U);
}

static void prepare_fast_tick_order(MotorState state, MotorControlMode mode)
{
    initialize();
    axis.current_offsets.complete = true;
    axis.configuration.device.phase_a_offset_counts = 2048.0f;
    axis.configuration.device.phase_b_offset_counts = 2048.0f;
    axis.configuration.device.phase_c_offset_counts = 2048.0f;
    for (unsigned phase = 0U; phase < 3U; ++phase)
    {
        bsp_offsets[phase] = 2048.0f;
        samples[phase] = 8192U;
    }
    simulated_state = state;
    axis.motor.control_mode = mode;
}

static void test_fast_tick_order(void)
{
    initialize();
    expect_fast_tick_order("BAVEFSWTS");

    prepare_fast_tick_order(MOTOR_STATE_RUNNING, MOTOR_CONTROL_CURRENT);
    expect_fast_tick_order("BAVOEFSWCTS");
    prepare_fast_tick_order(MOTOR_STATE_RUNNING, MOTOR_CONTROL_SPEED);
    expect_fast_tick_order("BAVEFSWCTS");
    prepare_fast_tick_order(MOTOR_STATE_RUNNING, MOTOR_CONTROL_POSITION);
    expect_fast_tick_order("BAVEFSWCTS");
    prepare_fast_tick_order(MOTOR_STATE_IDLE, MOTOR_CONTROL_NONE);
    expect_fast_tick_order("BAVOEFSWPTS");

    /* 故障前后均保留采样收尾；执行中新故障必须由末尾的关断检查接住。 */
    prepare_fast_tick_order(MOTOR_STATE_RUNNING, MOTOR_CONTROL_CURRENT);
    output_off = false;
    axis.motor.fault_code = MOTOR_FAULT_ENCODER;
    expect_fast_tick_order("BAVOEFSWTS");
    assert(output_off && axis.motor.fault_code == MOTOR_FAULT_ENCODER);
    prepare_fast_tick_order(MOTOR_STATE_RUNNING, MOTOR_CONTROL_CURRENT);
    output_off = false;
    inject_control_fault = true;
    expect_fast_tick_order("BAVOEFSWCTS");
    assert(output_off && axis.motor.fault_code == MOTOR_FAULT_OVERCURRENT);
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
        assert(axis.current_offsets.complete == valid);
        assert(axis.current_offsets.failed != valid);
        assert(MotorCurrentOffsets_IsValid(&axis.configuration) == valid);
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
        assert(axis.current_offsets.complete == valid);
        assert(axis.current_offsets.failed != valid);
        assert(phase_offset_writes == (valid ? 1U : 0U));
        assert(bus_offset_writes == (valid ? 1U : 0U));
        assert(MotorCurrentOffsets_IsValid(&axis.configuration) == valid);
    }
    initialize();
    for (unsigned i = 0U; i < CURRENT_OFFSET_SAMPLE_COUNT - 1U; ++i)
    {
        sample_test_currents();
    }
    bus_sample_raw = NAN;
    sample_test_currents();
    assert(axis.current_offsets.failed && !axis.current_offsets.complete);
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
        OutputEncoder_Init(&axis.output_encoder, 1.0f);
        OutputEncoder_Locate(&axis.output_encoder, 0.0f, 0.0f);
        simulated_calibration_outcome = (MotorTaskResult){
            calibration_type == MOTOR_CALIBRATION_ENCODER_LINEARIZATION ? MOTOR_TASK_FINALIZE
                                                                        : MOTOR_TASK_COMPLETED,
            MOTOR_FAULT_NONE};
        MotorAxis_SampleFast(&axis);
        MotorAxis_ControlFast(&axis);
        assert(calibration_cycle_calls == 1U && control_calls == 0U);
    }
    prepare_fast_tick_order(MOTOR_STATE_CALIBRATION, MOTOR_CONTROL_NONE);
    simulated_calibration_type = MOTOR_CALIBRATION_ENCODER_LINEARIZATION;
    calibration_publication_pending = true;
    MotorCalibration_ProcessPublication(&axis.calibration);
    assert(calibration_publication_calls == 0U);
    MotorAxis_SampleFast(&axis);
    MotorCalibration_ProcessPublication(&axis.calibration);
    assert(calibration_publication_calls == 1U);
    MotorCalibration_ProcessPublication(&axis.calibration);
    assert(calibration_publication_calls == 1U);
    calibration_publication_pending = true;
    calibration_publication_succeeds = false;
    ++publication_epoch;
    MotorCalibration_ProcessPublication(&axis.calibration);
    MotorAxis_SampleFast(&axis);
    assert(calibration_publication_calls == 2U);
#if BSP_ENABLED_LINEAR_HALL
    /* Hall 随观测器闭环采集；启动、构表、停止和电零位对齐均不采样。 */
    static const MotorElectricalPreset electrical = {.gear_ratio = 8.0f};
    static const MotorAxisPreset pitch = {.joint = MOTOR_JOINT_PITCH, .electrical = &electrical};
    axis.configuration.preset = &pitch;
    OutputEncoder_Init(&axis.output_encoder, 8.0f);
    LinearHall_Init(&axis.linear_hall);
    simulated_state = MOTOR_STATE_CALIBRATION;
    simulated_phase = CS_NULL;
    (void)MotorCalibration_StepFast(&axis.calibration);
    assert(isinf(axis.calibration.hall_sweep.sin_min));
    hall_raw_sin = 0;
    hall_raw_cos = 4095;
    simulated_phase = CS_OBS_STARTUP;
    MotorAxis_SampleFast(&axis);
    MotorAxis_OuterTick(&axis);
    assert(isinf(axis.calibration.hall_sweep.sin_min));
    simulated_phase = CS_OBS_SAMPLE_CW;
    hall_raw_sin = 100;
    hall_raw_cos = 600;
    axis.rotor_encoder.directed_q15 = 1234;
    MotorAxis_SampleFast(&axis);
    update_hall_on_unlock = true;
    MotorAxis_OuterTick(&axis);
    assert(!update_hall_on_unlock);
    assert(axis.calibration.output_calibration.samples[0].motor_counts == 1234);
    assert(axis.calibration.output_calibration.samples[0].sin_counts == 100 &&
           axis.calibration.output_calibration.samples[0].cos_counts == 600);
    hall_raw_sin = 3500;
    hall_raw_cos = 3600;
    MotorAxis_SampleFast(&axis);
    MotorAxis_OuterTick(&axis);
    assert(axis.calibration.hall_sweep.sin_min == 100 &&
           axis.calibration.hall_sweep.cos_max == 3600);
    /* 中止后新会话不得带入旧极值。 */
    simulated_state = MOTOR_STATE_IDLE;
    hall_raw_sin = 0;
    hall_raw_cos = 4095;
    MotorAxis_OuterTick(&axis);
    assert(axis.calibration.hall_sweep.sin_min == 100 &&
           axis.calibration.hall_sweep.cos_max == 3600);
    simulated_state = MOTOR_STATE_CALIBRATION;
    simulated_phase = CS_NULL;
    (void)MotorCalibration_StepFast(&axis.calibration);
    assert(isinf(axis.calibration.hall_sweep.sin_min));
    simulated_phase = CS_OBS_SAMPLE_CW;
    hall_raw_sin = 200;
    hall_raw_cos = 700;
    MotorAxis_SampleFast(&axis);
    MotorAxis_OuterTick(&axis);
    hall_raw_sin = 3400;
    hall_raw_cos = 3500;
    MotorAxis_SampleFast(&axis);
    MotorAxis_OuterTick(&axis);
    hall_raw_sin = 0;
    hall_raw_cos = 4095;
    const EncoderCalibrationPhase excluded[] = {
        CS_OBS_BUILD_LUT, CS_OBS_STOP_DECEL, CS_OBS_STOP_CURRENT, CS_LUT_PUBLISH_PENDING};
    for (unsigned i = 0; i < sizeof(excluded) / sizeof(excluded[0]); ++i)
    {
        simulated_phase = excluded[i];
        MotorAxis_OuterTick(&axis);
    }
    calibration_publication_pending = calibration_publication_succeeds = true;
    MotorCalibration_ProcessPublication(&axis.calibration);
    test_ms = 201U;
    MotorCalibration_ProcessPublication(&axis.calibration);
    MotorAxis_SampleFast(&axis);
    assert(simulated_calibration_type == MOTOR_CALIBRATION_JOINT_ELECTRICAL_ZERO);
    assert(axis.calibration.hall_sweep.sin_min == 200);
    MotorAxis_OuterTick(&axis);
    simulated_calibration_outcome.result = MOTOR_TASK_COMPLETED;
    assert(MotorCalibration_StepFast(&axis.calibration).result == MOTOR_TASK_COMPLETED);
    axis.calibration.output_calibration.count = OUTPUT_ENCODER_CAL_SAMPLES;
    memset(&axis.configuration.device.encoder, 0, sizeof(axis.configuration.device.encoder));
    for (unsigned i = 0; i < OUTPUT_ENCODER_CAL_SAMPLES; ++i)
    {
        int32_t counts = (int32_t)(i * 8.0f * 65536.0f * 1.01f / (OUTPUT_ENCODER_CAL_SAMPLES - 1U));
        float h = (float)counts * (_2PI / (65536.0f * 8.0f));
        axis.calibration.output_calibration.samples[i] =
            (OutputEncoderCalibSample){counts,
                                       (uint16_t)(1800.0f - 1600.0f * sinf(h)),
                                       (uint16_t)(2100.0f + 1400.0f * cosf(h))};
    }
    calibration_publication_pending = true;
    MotorCalibration_ProcessPublication(&axis.calibration);
    assert(calibration_publication_calls == 3 && axis.linear_hall.calibration_success);
    assert(axis.linear_hall.sin_off_counts == 1800 && axis.linear_hall.sin_amp == 1600);
    assert(axis.linear_hall.cos_off_counts == 2100 && axis.linear_hall.cos_amp == 1400);
    axis.configuration.preset = NULL;
#endif
    puts("PASS calibration dispatch: linearization/electrical/mechanical, publication");
}

static void test_output_encoder_startup(void)
{
    simulated_state = MOTOR_STATE_IDLE;
    axis.configuration.preset = NULL;
    axis.configuration.device.encoder.calib_flag = ENC_CALIB_LINEARIZED;
    axis.rotor_encoder.has_valid_sample = true;
    axis.linear_hall.calibration_success = true;
    axis.linear_hall.val.sin_cal = -sinf(0.45f);
    axis.linear_hall.val.cos_cal = cosf(0.45f);
    axis.linear_hall.val.angle_rad = 0.45f;
    const float ratios[] = {1.0f, 8.0f};
    for (unsigned i = 0U; i < sizeof(ratios) / sizeof(ratios[0]); ++i)
    {
        OutputEncoder_Init(&axis.output_encoder, ratios[i]);
        axis.output_encoder.params.status = OUTPUT_ENCODER_CAL_OK;
        axis.current_offsets.complete = false;
        axis.rotor_encoder.linearized_q15 = 0U;
        MotorAxis_OuterTick(&axis);
        assert(!OutputEncoder_IsReady(&axis.output_encoder));
        /* 真实角接近半圈、Hall 有小误差时，启动零码不能提前决定圈段。 */
        axis.rotor_encoder.linearized_q15 = 32760U;
        axis.current_offsets.complete = true;
        MotorAxis_OuterTick(&axis);
        assert(OutputEncoder_IsReady(&axis.output_encoder));
        float expected = 32760.0f * (_2PI / 65536.0f) / ratios[i];
        assert(fabsf(axis.output_encoder.initial_position_rad - expected) < 1e-5f);
        assert(axis.output_encoder.motor_delta_q15 == 0);
        axis.rotor_encoder.linearized_q15 += 100U;
        MotorAxis_OuterTick(&axis);
        assert(axis.output_encoder.motor_delta_q15 == 100);
    }
    puts("PASS output encoder waits for startup sampling before locating direct/geared axes");
}

int main(void)
{
    /* Watch类型与对象布局不随零偏计算的所属目录迁移而变化。 */
    assert(sizeof(CurrentOffsetCalibration) == 24U + sizeof(axis.current_offsets.phases_disabled));
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
    MotorAxis_SampleFast(&axis);
    MotorAxis_ControlFast(&axis);
    MotorAxis_SampleFast(&axis);
    MotorAxis_ControlFast(&axis);
    assert(velocity_restarts == 0U);
    simulated_save = false;
    MotorAxis_SampleFast(&axis);
    MotorAxis_ControlFast(&axis);
    assert(velocity_restarts == 0U);
    initialize();
    simulated_state = MOTOR_STATE_RUNNING;
    MotorAxis_SampleFast(&axis);
    MotorAxis_ControlFast(&axis);
    assert(control_calls == 0U && observer_calls == 0U && progress_calls == 1U);
    initialize();
    axis.foc.bus_voltage_filtered_v = 30.0f;
    Foc_ConfigureControllers(&axis.configuration, &axis.foc, &axis.speed_pi);
    Foc_UpdateCurrentLimits(&axis.foc);
    assert(axis.foc.id_pi.output_max == Foc_MaxVoltage(30.0f));
    assert(axis.foc.iq_pi.output_min == -Foc_MaxVoltage(30.0f));
    assert(state_initializations >= 7U);
    /* 无效通道仍发布NaN测量，不由采样搬运函数伪造故障或有效电流。 */
    axis.current_offsets.complete = true;
    axis.configuration.device.phase_a_offset_counts = 2048.0f;
    axis.configuration.device.phase_b_offset_counts = 2048.0f;
    axis.configuration.device.phase_c_offset_counts = 2048.0f;
    frame_available = false;
    output_off = false;
    unsigned prior_reads = fast_frame_reads;
    expect_fast_tick_order("BAVOEFSWPTS");
    assert(fast_frame_reads == prior_reads + 1U && !output_off);
    assert(axis.motor.fault_code == MOTOR_FAULT_NONE);
    assert(isnan(axis.foc.phase_current_a.a) && isnan(axis.foc.phase_current_a.b) &&
           isnan(axis.foc.phase_current_a.c));
    test_calibration_dispatch_and_publication();
    test_output_encoder_startup();
    puts("PASS FastTick order: channel ADC, SPI overlap, core dispatch and two fault stops");
    puts("PASS IDLE-only shared current-offset window, phase/bus scale, atomic publish and failure "
         "freeze");
    puts("PASS startup offset/PI initialization and sampled voltage limits preserve PI state");
    return 0;
}

void MotorLifecycle_BeginFastTick(MotorLifecycle *instance)
{
    (void)instance;
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

static void inject_hall_after_snapshot(void)
{
#if BSP_ENABLED_LINEAR_HALL
    if (update_hall_on_unlock)
    {
        /* 复制完成立即抢占，消费者仍须使用同一个旧三元组。 */
        update_hall_on_unlock = false;
        hall_raw_sin = 333;
        hall_raw_cos = 444;
        axis.rotor_encoder.directed_q15 = 2345;
        MotorAxis_SampleFast(&axis);
    }
#else
    (void)update_hall_on_unlock;
#endif
}
