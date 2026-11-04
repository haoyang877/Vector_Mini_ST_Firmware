/* 五态运行集成：生产状态/请求/维护代码，硬件和叶任务使用有界替身。 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "motor_lifecycle.h"
#include "motor_control.h"
#include "application_config.h"
#include "motor_state_machine.h"
#include "motor_fault.h"
#include "motor_sensing.h"
#include "bsp_pwm.h"
#include "bsp_system.h"
#include "control_math.h"
#include "encoder_calibration.h"
#include "motor_axis.h"
#include "app_protocol.h"
#include "../../../firmware/communication/yg_protocol.c"
#include "../../../firmware/communication/yg_protocol_boot.c"

static MotorConfiguration configuration;
static MotorControlState motor;
static FocState foc;
static PiController pi;
static EncoderState encoder;
static SensorlessStartupState startup;
static bool phases_disabled, start_fails, stop_fails, outer_ready;
static unsigned critical_depth, critical_calls, fault_on_enter;
static unsigned calibration_cancel_count;
static bool calibration_resources_released;

void EncoderCalibration_Cancel(void)
{
    ++calibration_cancel_count;
    calibration_resources_released = true;
}

bool EncoderCalibration_IsIdle(void)
{
    return calibration_resources_released;
}

uint32_t BspSystem_EnterCritical(void)
{
    ++critical_calls;
    if (fault_on_enter == critical_calls)
    {
        motor.fault_code = MOTOR_FAULT_OVERCURRENT;
        phases_disabled = true;
    }
    return critical_depth++;
}
void BspSystem_ExitCritical(uint32_t saved)
{
    assert(critical_depth == saved + 1U);
    critical_depth = saved;
}
uint32_t BspSystem_EnterControlCritical(void)
{
    return BspSystem_EnterCritical();
}
void BspSystem_ExitControlCritical(uint32_t saved)
{
    BspSystem_ExitCritical(saved);
}
void BspSystem_MemoryBarrier(void)
{
}
bool BspSystem_CompareExchange32(volatile uint32_t *address, uint32_t expected, uint32_t desired)
{
    if (*address != expected)
    {
        return false;
    }
    *address = desired;
    return true;
}
void BspPwm_Enable(void)
{
    if (!start_fails)
    {
        phases_disabled = false;
    }
}
void BspPwm_Disable(void)
{
    if (!stop_fails)
    {
        phases_disabled = true;
    }
}
bool BspPwm_ArePhasesDisabled(void)
{
    return phases_disabled;
}
bool Encoder_IsOnline(const EncoderState *state)
{
    return state->has_valid_sample;
}
uint8_t Encoder_GetCalibrationFlag(const EncoderCalibration *calibration)
{
    return calibration->calib_flag;
}
float Encoder_GetMechPos(const EncoderState *state)
{
    return state->mechanical_position_rad;
}
float Encoder_GetMechVel(const EncoderState *state)
{
    return state->mechanical_velocity_rad_s;
}
bool McuTemperature_IsFresh(void)
{
    return true;
}
bool MotorCurrentOffsets_IsValid(const MotorConfiguration *config)
{
    return config->device.phase_a_offset_counts == 2048.0f;
}
bool ParameterValue_ConfigurationValid(const MotorConfiguration *config)
{
    return config->application.axis_profile_valid && config->application.current_limit_a > 0.0f;
}

float Math_Abs(float value)
{
    return fabsf(value);
}
float Foc_MaxVoltage(float voltage)
{
    return voltage * 0.5f;
}
void Foc_ResetCurrent(FocState *state)
{
    state->current_dq_a.d = 0.0f;
    state->current_dq_a.q = 0.0f;
}
void PiController_Reset(PiController *controller)
{
    controller->integral_output = 0.0f;
}
void SensorlessStartup_Reset(SensorlessStartupState *state)
{
    (void)state;
}
bool MotorOuterLoop_IsReady(void)
{
    return outer_ready;
}
void MotorOuterLoop_RequestReset(void)
{
    outer_ready = false;
}

static bool stop_during_outer, fault_during_outer;
void MotorOuterLoop_SlowTick(const MotorConfiguration *config,
                             MotorControlState *calculated,
                             PiController *controller,
                             EncoderState *feedback)
{
    (void)config;
    (void)controller;
    (void)feedback;
    outer_ready = true;
    calculated->iq_target_a = 0.5f;
    if (stop_during_outer)
    {
        stop_during_outer = false;
        MotorLifecycle_RequestRemoteStop();
    }
    if (fault_during_outer)
    {
        fault_during_outer = false;
        motor.fault_code = MOTOR_FAULT_OVERCURRENT;
        phases_disabled = true;
    }
}
static void tick(void)
{
    MotorLifecycle_PostOutcome((MotorTaskResult){MOTOR_TASK_RUNNING, MOTOR_FAULT_NONE},
                               MotorLifecycle_WorkEpoch());
    MotorLifecycle_SupervisorTick();
    MotorLifecycle_BeginFastTick();
    assert(critical_depth == 0U);
}

static void reset(void)
{
    memset(&configuration, 0, sizeof(configuration));
    memset(&motor, 0, sizeof(motor));
    memset(&foc, 0, sizeof(foc));
    memset(&encoder, 0, sizeof(encoder));
    configuration.application.axis_profile_valid = true;
    configuration.configuration_confirmed = true;
    configuration.application.current_limit_a = 6.0f;
    configuration.application.speed_limit_rad_s = 20.0f;
    configuration.model.motor_pole_pairs = 7U;
    configuration.device.phase_a_offset_counts = 2048.0f;
    configuration.device.encoder.calib_flag = ENC_CALIB_ALL;
    encoder.has_valid_sample = true;
    foc.bus_voltage_v = foc.bus_voltage_filtered_v = 30.0f;
    foc.temperature_c = 25.0f;
    phases_disabled = true;
    start_fails = stop_fails = outer_ready = false;
    critical_calls = fault_on_enter = 0U;
    calibration_cancel_count = 0U;
    calibration_resources_released = true;
    MotorLifecycle_Init(&configuration, &motor, &foc, &pi, &encoder, &startup);
    assert(MotorLifecycle_GetState() == MOTOR_STATE_IDLE);
    tick();
}

static void test_five_states(void)
{
    for (unsigned state = 0; state <= MOTOR_STATE_FAULT; ++state)
    {
        assert(MotorStateMachine_Step((MotorState)state, MOTOR_STATE_EVENT_FAULT) ==
               MOTOR_STATE_FAULT);
        for (unsigned event = 0; event <= MOTOR_STATE_EVENT_CLEAR; ++event)
        {
            assert(MotorStateMachine_Step((MotorState)state, (MotorStateEvent)event) <=
                   MOTOR_STATE_FAULT);
        }
    }
    assert(MotorStateMachine_Step(MOTOR_STATE_IDLE, MOTOR_STATE_EVENT_RUN) == MOTOR_STATE_RUNNING);
    assert(MotorStateMachine_Step(MOTOR_STATE_IDLE, MOTOR_STATE_EVENT_CALIBRATE) ==
           MOTOR_STATE_CALIBRATION);
    assert(MotorStateMachine_Step(MOTOR_STATE_RUNNING, MOTOR_STATE_EVENT_STOP) == MOTOR_STATE_IDLE);
    assert(MotorStateMachine_Step(MOTOR_STATE_CALIBRATION, MOTOR_STATE_EVENT_COMPLETE) ==
           MOTOR_STATE_IDLE);
    assert(MotorStateMachine_Step(MOTOR_STATE_FAULT, MOTOR_STATE_EVENT_STOP) == MOTOR_STATE_FAULT);
    assert(MotorStateMachine_Step(MOTOR_STATE_FAULT, MOTOR_STATE_EVENT_CLEAR) == MOTOR_STATE_IDLE);
    assert(MotorStateMachine_Step(MOTOR_STATE_SELF_TEST, MOTOR_STATE_EVENT_RUN) ==
           MOTOR_STATE_SELF_TEST);
    assert(MotorStateMachine_Step((MotorState)99, MOTOR_STATE_EVENT_RUN) == MOTOR_STATE_FAULT);
    assert(MotorStateMachine_Step(MOTOR_STATE_IDLE, (MotorStateEvent)99) == MOTOR_STATE_FAULT);
}
static void test_control_and_stop(void)
{
    reset();
    assert(MotorLifecycle_RequestEnable(MOTOR_CONTROL_CURRENT) == MOTOR_CONTROL_ACCEPTED);
    assert(MotorLifecycle_GetState() == MOTOR_STATE_IDLE && phases_disabled);
    tick();
    assert(MotorLifecycle_GetState() == MOTOR_STATE_RUNNING && !phases_disabled);
    uint32_t epoch = MotorLifecycle_WorkEpoch();
    MotorLifecycle_RequestRemoteStop();
    tick();
    assert(MotorLifecycle_GetState() == MOTOR_STATE_IDLE && phases_disabled);
    MotorLifecycle_PostOutcome((MotorTaskResult){MOTOR_TASK_FAILED, MOTOR_FAULT_ENCODER}, epoch);
    tick();
    assert(motor.fault_code == MOTOR_FAULT_NONE);
    assert(MotorLifecycle_RequestEnable(MOTOR_CONTROL_SPEED) == MOTOR_CONTROL_ACCEPTED);
    stop_during_outer = true;
    tick();
    assert(phases_disabled && motor.iq_target_a == 0.0f);
}
static void test_bus_current_snapshot(void)
{
    reset();
    MotorStatus sample = MotorLifecycle_ReadStatus();
    assert(sample.state == MOTOR_STATE_IDLE && sample.fault == MOTOR_FAULT_NONE &&
           sample.mode == MOTOR_CONTROL_NONE);
}

static void test_faults_and_safety(void)
{
    reset();
    assert(MotorLifecycle_RequestEnable(MOTOR_CONTROL_CURRENT) == MOTOR_CONTROL_ACCEPTED);
    start_fails = true;
    tick();
    tick();
    assert(MotorLifecycle_GetState() == MOTOR_STATE_FAULT && phases_disabled);
    start_fails = false;
    assert(MotorLifecycle_RequestClear() == MOTOR_CONTROL_ACCEPTED);
    assert(MotorLifecycle_GetState() == MOTOR_STATE_IDLE &&
           motor.control_mode == MOTOR_CONTROL_NONE);
    assert(phases_disabled);

    reset();
    assert(MotorLifecycle_RequestEnable(MOTOR_CONTROL_CURRENT) == MOTOR_CONTROL_ACCEPTED);
    tick();
    stop_fails = true;
    MotorLifecycle_RequestRemoteStop();
    tick();
    assert(MotorLifecycle_GetState() == MOTOR_STATE_FAULT);
    assert(MotorLifecycle_RequestClear() != MOTOR_CONTROL_ACCEPTED);

    reset();
    configuration.device.phase_a_offset_counts = 0.0f;
    tick();
    assert(MotorLifecycle_RequestEnable(MOTOR_CONTROL_CURRENT) != MOTOR_CONTROL_ACCEPTED);
    reset();
    assert(MotorLifecycle_RequestEnable(MOTOR_CONTROL_CURRENT) == MOTOR_CONTROL_ACCEPTED);
    foc.bus_voltage_filtered_v = NAN;
    tick();
    assert(phases_disabled && MotorLifecycle_GetState() == MOTOR_STATE_FAULT);

    reset();
    assert(MotorLifecycle_RequestEnable(MOTOR_CONTROL_CURRENT) == MOTOR_CONTROL_ACCEPTED);
    tick();
    MotorLifecycle_PostOutcome((MotorTaskResult){MOTOR_TASK_FAILED, MOTOR_FAULT_OVERCURRENT},
                               MotorLifecycle_WorkEpoch());
    assert(phases_disabled);
    tick();
    assert(MotorLifecycle_GetState() == MOTOR_STATE_FAULT);
}

static void test_commit_preemption_and_stop(void)
{
    reset();
    assert(MotorLifecycle_RequestEnable(MOTOR_CONTROL_SPEED) == MOTOR_CONTROL_ACCEPTED);
    tick();
    fault_during_outer = true;
    tick();
    assert(phases_disabled && motor.iq_target_a == 0.0f);
    reset();
    assert(MotorLifecycle_RequestEnable(MOTOR_CONTROL_CURRENT) == MOTOR_CONTROL_ACCEPTED);
    tick();
    MotorLifecycle_PostOutcome((MotorTaskResult){MOTOR_TASK_STOP_REQUESTED, MOTOR_FAULT_NONE},
                               MotorLifecycle_WorkEpoch());
    tick();
    assert(phases_disabled && MotorLifecycle_GetState() == MOTOR_STATE_IDLE);
    assert(MotorLifecycle_LockStop());
    tick();
    assert(MotorLifecycle_IsStopped());
    assert(MotorLifecycle_RequestCalibration(MOTOR_CALIBRATION_MECHANICAL_ZERO) ==
           MOTOR_CONTROL_BUSY);
    assert(MotorLifecycle_RequestEnable(MOTOR_CONTROL_CURRENT) == MOTOR_CONTROL_BUSY);
    assert(MotorLifecycle_RequestEnable(MOTOR_CONTROL_CURRENT) != MOTOR_CONTROL_ACCEPTED);
}

static void test_calibration_completes_to_idle(void)
{
    reset();
    assert(MotorLifecycle_RequestCalibration(MOTOR_CALIBRATION_NONE) == MOTOR_CONTROL_INVALID);
    assert(MotorLifecycle_RequestCalibration((MotorCalibrationType)99) == MOTOR_CONTROL_INVALID);
    assert(MotorLifecycle_RequestCalibration(MOTOR_CALIBRATION_MECHANICAL_ZERO) ==
           MOTOR_CONTROL_ACCEPTED);
    uint32_t calibration_epoch = MotorLifecycle_WorkEpoch();
    assert(MotorLifecycle_GetState() == MOTOR_STATE_CALIBRATION && phases_disabled);
    assert(MotorLifecycle_IsCalibrationRunnable());
    assert(MotorLifecycle_RequestEnable(MOTOR_CONTROL_CURRENT) != MOTOR_CONTROL_ACCEPTED);
    assert(MotorLifecycle_RequestEnable(MOTOR_CONTROL_CURRENT) != MOTOR_CONTROL_ACCEPTED);
    MotorLifecycle_PostOutcome((MotorTaskResult){MOTOR_TASK_COMPLETED, MOTOR_FAULT_NONE},
                               calibration_epoch);
    assert(!MotorLifecycle_IsCalibrationRunnable());
    tick();
    assert(MotorLifecycle_GetState() == MOTOR_STATE_IDLE && phases_disabled);
    assert(MotorLifecycle_GetCalibrationType() == MOTOR_CALIBRATION_NONE);
    assert(MotorLifecycle_WorkEpoch() != calibration_epoch);
    assert(calibration_cancel_count == 1U);

    reset();
    configuration.device.encoder.calib_flag = ENC_CALIB_LINEARIZED;
    assert(MotorLifecycle_RequestCalibration(MOTOR_CALIBRATION_ELECTRICAL_ZERO) ==
           MOTOR_CONTROL_ACCEPTED);
    assert(phases_disabled && !MotorLifecycle_IsCalibrationRunnable());
    MotorLifecycle_BeginFastTick();
    assert(!phases_disabled && MotorLifecycle_GetState() == MOTOR_STATE_CALIBRATION);
    calibration_epoch = MotorLifecycle_WorkEpoch();
    MotorLifecycle_PostOutcome((MotorTaskResult){MOTOR_TASK_COMPLETED, MOTOR_FAULT_NONE},
                               calibration_epoch);
    assert(phases_disabled);
    tick();
    assert(MotorLifecycle_GetState() == MOTOR_STATE_IDLE);
}

static void test_calibration_stop_fault_and_late_result(void)
{
    reset();
    encoder.has_valid_sample = false;
    assert(MotorLifecycle_RequestCalibration(MOTOR_CALIBRATION_ENCODER_LINEARIZATION) ==
           MOTOR_CONTROL_DENIED);
    assert(MotorLifecycle_RequestCalibration(MOTOR_CALIBRATION_ELECTRICAL_ZERO) ==
           MOTOR_CONTROL_DENIED);
    assert(MotorLifecycle_RequestCalibration(MOTOR_CALIBRATION_MECHANICAL_ZERO) ==
           MOTOR_CONTROL_DENIED);
    assert(MotorLifecycle_GetState() == MOTOR_STATE_IDLE && phases_disabled);

    reset();
    configuration.device.encoder.calib_flag = 0U;
    assert(MotorLifecycle_RequestCalibration(MOTOR_CALIBRATION_ELECTRICAL_ZERO) ==
           MOTOR_CONTROL_DENIED);
    assert(MotorLifecycle_RequestCalibration(MOTOR_CALIBRATION_ENCODER_LINEARIZATION) ==
           MOTOR_CONTROL_ACCEPTED);
    uint32_t cancelled_epoch = MotorLifecycle_WorkEpoch();
    MotorLifecycle_BeginFastTick();
    assert(MotorLifecycle_RequestRemoteStop());
    tick();
    MotorLifecycle_PostOutcome((MotorTaskResult){MOTOR_TASK_COMPLETED, MOTOR_FAULT_NONE},
                               cancelled_epoch);
    tick();
    assert(phases_disabled && MotorLifecycle_GetState() == MOTOR_STATE_IDLE);
    assert(MotorLifecycle_GetCalibrationType() == MOTOR_CALIBRATION_NONE);

    reset();
    assert(MotorLifecycle_RequestCalibration(MOTOR_CALIBRATION_ELECTRICAL_ZERO) ==
           MOTOR_CONTROL_ACCEPTED);
    MotorLifecycle_BeginFastTick();
    MotorLifecycle_PostOutcome((MotorTaskResult){MOTOR_TASK_FAILED, MOTOR_FAULT_ENCODER},
                               MotorLifecycle_WorkEpoch());
    assert(phases_disabled);
    tick();
    assert(MotorLifecycle_GetState() == MOTOR_STATE_FAULT);
    assert(MotorLifecycle_RequestClear() == MOTOR_CONTROL_ACCEPTED);
    assert(MotorLifecycle_GetState() == MOTOR_STATE_IDLE);

    reset();
    assert(MotorLifecycle_RequestCalibration(MOTOR_CALIBRATION_ELECTRICAL_ZERO) ==
           MOTOR_CONTROL_ACCEPTED);
    MotorLifecycle_BeginFastTick();
    stop_fails = true;
    MotorLifecycle_PostOutcome((MotorTaskResult){MOTOR_TASK_COMPLETED, MOTOR_FAULT_NONE},
                               MotorLifecycle_WorkEpoch());
    assert(motor.fault_code == MOTOR_FAULT_CONTROL_OVERRUN);
    tick();
    assert(MotorLifecycle_GetState() == MOTOR_STATE_FAULT);
}

static void test_linearization_waits_for_publication(void)
{
    reset();
    assert(MotorLifecycle_RequestCalibration(MOTOR_CALIBRATION_ENCODER_LINEARIZATION) ==
           MOTOR_CONTROL_ACCEPTED);
    MotorLifecycle_BeginFastTick();
    uint32_t calibration_epoch = MotorLifecycle_WorkEpoch();
    MotorLifecycle_PostOutcome((MotorTaskResult){MOTOR_TASK_FINALIZE, MOTOR_FAULT_NONE},
                               calibration_epoch);
    tick();
    assert(phases_disabled && MotorLifecycle_GetState() == MOTOR_STATE_CALIBRATION);
    assert(!MotorLifecycle_IsCalibrationRunnable());
    uint32_t publication_epoch;
    uint32_t interrupt_mask = BspSystem_EnterCritical();
    assert(MotorLifecycle_ReadPendingCalibrationPublication(&publication_epoch));
    assert(publication_epoch == calibration_epoch);
    MotorLifecycle_CompleteCalibrationPublication(calibration_epoch + 1U, true);
    assert(MotorLifecycle_ReadPendingCalibrationPublication(&publication_epoch));
    MotorLifecycle_CompleteCalibrationPublication(calibration_epoch, true);
    BspSystem_ExitCritical(interrupt_mask);
    tick();
    assert(MotorLifecycle_GetState() == MOTOR_STATE_IDLE);

    reset();
    assert(MotorLifecycle_RequestCalibration(MOTOR_CALIBRATION_ENCODER_LINEARIZATION) ==
           MOTOR_CONTROL_ACCEPTED);
    MotorLifecycle_BeginFastTick();
    calibration_epoch = MotorLifecycle_WorkEpoch();
    MotorLifecycle_PostOutcome((MotorTaskResult){MOTOR_TASK_FINALIZE, MOTOR_FAULT_NONE},
                               calibration_epoch);
    interrupt_mask = BspSystem_EnterCritical();
    MotorLifecycle_CompleteCalibrationPublication(calibration_epoch, false);
    BspSystem_ExitCritical(interrupt_mask);
    tick();
    assert(MotorLifecycle_GetState() == MOTOR_STATE_FAULT && phases_disabled);

    reset();
    assert(MotorLifecycle_RequestCalibration(MOTOR_CALIBRATION_ENCODER_LINEARIZATION) ==
           MOTOR_CONTROL_ACCEPTED);
    MotorLifecycle_BeginFastTick();
    calibration_epoch = MotorLifecycle_WorkEpoch();
    MotorLifecycle_PostOutcome((MotorTaskResult){MOTOR_TASK_FINALIZE, MOTOR_FAULT_NONE},
                               calibration_epoch);
    assert(MotorLifecycle_RequestRemoteStop());
    interrupt_mask = BspSystem_EnterCritical();
    assert(!MotorLifecycle_ReadPendingCalibrationPublication(&publication_epoch));
    MotorLifecycle_CompleteCalibrationPublication(calibration_epoch, true);
    BspSystem_ExitCritical(interrupt_mask);
    tick();
    assert(MotorLifecycle_GetState() == MOTOR_STATE_IDLE);
}

static void test_calibration_communication_to_idle(void)
{
    assert(Protocol_Init(1U));
    const uint16_t kinds[] = {PROTOCOL_JOB_KIND_ENCODER_LUT,
                              PROTOCOL_JOB_KIND_ELECTRICAL_ZERO,
                              PROTOCOL_JOB_KIND_MECHANICAL_ZERO};
    const MotorCalibrationType types[] = {MOTOR_CALIBRATION_ENCODER_LINEARIZATION,
                                          MOTOR_CALIBRATION_ELECTRICAL_ZERO,
                                          MOTOR_CALIBRATION_MECHANICAL_ZERO};
    for (unsigned index = 0U; index < 3U; ++index)
    {
        reset();
        uint8_t payload[4] = {0};
        YgProtocol_WriteU16Le(payload, kinds[index]);
        ProtocolExchange exchange = {.request = {.version = YG_PROTOCOL_VERSION,
                                                 .flags = YG_PROTOCOL_FLAGS_ACK_REQUEST,
                                                 .source_node = PROTOCOL_MASTER_ID,
                                                 .destination_node = 1U,
                                                 .message_type = PROTOCOL_START_JOB,
                                                 .sequence = (uint16_t)(index + 1U),
                                                 .payload_length = sizeof(payload),
                                                 .payload = payload}};
        assert(handle_command(&exchange) == YG_PROTOCOL_OK);
        assert(YgProtocol_ReadU16Le(exchange.reply.data) == 1U);
        assert(YgProtocol_ReadU32Le(exchange.reply.data + 4U) != 0U);
        assert(MotorLifecycle_GetState() == MOTOR_STATE_CALIBRATION);
        assert(MotorLifecycle_GetCalibrationType() == types[index]);
        assert(phases_disabled);

        /* 同一运行会话不能被第二个通信请求覆盖。 */
        assert(handle_command(&exchange) == YG_PROTOCOL_OK);
        assert(YgProtocol_ReadU16Le(exchange.reply.data) != 1U);
        MotorLifecycle_BeginFastTick();
        assert(phases_disabled == (types[index] == MOTOR_CALIBRATION_MECHANICAL_ZERO));
        assert(MotorLifecycle_IsCalibrationRunnable());
        uint32_t calibration_epoch = MotorLifecycle_WorkEpoch();

        if (types[index] == MOTOR_CALIBRATION_ENCODER_LINEARIZATION)
        {
            MotorLifecycle_PostOutcome((MotorTaskResult){MOTOR_TASK_FINALIZE, MOTOR_FAULT_NONE},
                                       calibration_epoch);
            tick();
            assert(MotorLifecycle_GetState() == MOTOR_STATE_CALIBRATION && phases_disabled);
            uint32_t publication_epoch;
            uint32_t interrupt_mask = BspSystem_EnterCritical();
            assert(MotorLifecycle_ReadPendingCalibrationPublication(&publication_epoch));
            MotorLifecycle_CompleteCalibrationPublication(publication_epoch, true);
            BspSystem_ExitCritical(interrupt_mask);
        }
        else
        {
            MotorLifecycle_PostOutcome((MotorTaskResult){MOTOR_TASK_COMPLETED, MOTOR_FAULT_NONE},
                                       calibration_epoch);
        }
        tick();
        assert(MotorLifecycle_GetState() == MOTOR_STATE_IDLE && phases_disabled);
        assert(MotorLifecycle_GetCalibrationType() == MOTOR_CALIBRATION_NONE);
    }
    puts(
        "PASS calibration communication: three wire kinds -> real lifecycle -> completion -> IDLE");
}

static void test_live_targets(void)
{
    reset();
    MotorTarget target = {.mode = MOTOR_CONTROL_CURRENT, .iq_a = 0.25f};
    assert(MotorLifecycle_SetTarget(&target) == MOTOR_CONTROL_DENIED);
    assert(MotorLifecycle_RequestEnable(MOTOR_CONTROL_CURRENT) == MOTOR_CONTROL_ACCEPTED);
    tick();
    assert(MotorLifecycle_SetTarget(&target) == MOTOR_CONTROL_ACCEPTED);
    assert(motor.iq_target_a == 0.25f && motor.id_target_a == 0.0f);
    target.iq_a = 6.1f;
    assert(MotorLifecycle_SetTarget(&target) == MOTOR_CONTROL_OUT_OF_RANGE);
    assert(motor.iq_target_a == 0.25f);
    target.iq_a = NAN;
    assert(MotorLifecycle_SetTarget(&target) == MOTOR_CONTROL_INVALID);
    target.iq_a = 0.0f;
    target.mode = MOTOR_CONTROL_SPEED;
    assert(MotorLifecycle_SetTarget(&target) == MOTOR_CONTROL_DENIED);
    target.mode = MOTOR_CONTROL_CURRENT;
    target.speed_rad_s = 1.0f;
    assert(MotorLifecycle_SetTarget(&target) == MOTOR_CONTROL_INVALID);
    target.speed_rad_s = 0.0f;
    MotorLifecycle_RequestRemoteStop();
    assert(MotorLifecycle_SetTarget(&target) == MOTOR_CONTROL_DENIED);
    tick();
    assert(MotorLifecycle_RequestEnable(MOTOR_CONTROL_SPEED) == MOTOR_CONTROL_ACCEPTED);
    tick();
    target.mode = MOTOR_CONTROL_SPEED;
    target.speed_rad_s = -1.0f;
    assert(MotorLifecycle_SetTarget(&target) == MOTOR_CONTROL_ACCEPTED);
    assert(motor.speed_target_rad_s == -1.0f);
    target.speed_rad_s = 20.1f;
    assert(MotorLifecycle_SetTarget(&target) == MOTOR_CONTROL_OUT_OF_RANGE);
    motor.fault_code = MOTOR_FAULT_OVERCURRENT;
    assert(MotorLifecycle_SetTarget(&target) == MOTOR_CONTROL_FAULT);
    const MotorControlMode modes[] = {MOTOR_CONTROL_POSITION,
                                      MOTOR_CONTROL_VQ,
                                      MOTOR_CONTROL_OPEN_VOLTAGE,
                                      MOTOR_CONTROL_OPEN_CURRENT};
    for (unsigned i = 0; i < sizeof(modes) / sizeof(modes[0]); ++i)
    {
        reset();
        assert(MotorLifecycle_RequestEnable(modes[i]) == MOTOR_CONTROL_ACCEPTED);
        tick();
        target = (MotorTarget){.mode = (uint16_t)modes[i]};
        if (modes[i] == MOTOR_CONTROL_POSITION)
        {
            target.position_rad = 0.1f;
        }
        else if (modes[i] == MOTOR_CONTROL_OPEN_CURRENT)
        {
            target.id_a = 1.0f;
            target.iq_a = -1.0f;
            target.frequency_hz = -20.0f;
        }
        else
        {
            target.vq_v = 1.0f;
            target.vd_v = modes[i] == MOTOR_CONTROL_OPEN_VOLTAGE ? 1.0f : 0.0f;
        }
        assert(MotorLifecycle_SetTarget(&target) == MOTOR_CONTROL_ACCEPTED);
        if (modes[i] == MOTOR_CONTROL_POSITION)
        {
            configuration.application.axis_profile.magic = MOTOR_AXIS_PROFILE_MAGIC;
            configuration.application.axis_profile.minimum_position_rad = -1.0f;
            configuration.application.axis_profile.maximum_position_rad = 1.0f;
            target.position_rad = 2.0f;
        }
        else if (modes[i] == MOTOR_CONTROL_OPEN_CURRENT)
        {
            target.frequency_hz = 201.0f;
        }
        else
        {
            target.vq_v = 100.0f;
        }
        assert(MotorLifecycle_SetTarget(&target) == MOTOR_CONTROL_OUT_OF_RANGE);
    }
    reset();
    foc.bus_current_filtered_a = 0.125f;
    encoder.mechanical_position_rad = 1.5f;
    encoder.mechanical_velocity_continuous_rad_s = -2.25f;
    MotorStatus status = MotorLifecycle_ReadStatus();
    assert(status.measurements_valid && status.position_rad == 1.5f &&
           status.speed_rad_s == -2.25f && status.bus_current_a == 0.125f);
    assert(isnan(status.iq_reference_a) && isnan(status.iq_a) && isnan(status.vq_v));
}

int main(void)
{
    test_live_targets();
    test_five_states();
    test_control_and_stop();
    test_bus_current_snapshot();
    test_faults_and_safety();
    test_commit_preemption_and_stop();
    test_calibration_completes_to_idle();
    test_calibration_stop_fault_and_late_result();
    test_linearization_waits_for_publication();
    test_calibration_communication_to_idle();
    puts("PASS core lifecycle and calibration: completion, publication, stale epochs, stop, fault "
         "shutdown, recovery and preemption");
    return 0;
}
