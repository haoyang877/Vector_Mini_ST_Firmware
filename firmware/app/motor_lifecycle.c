#include "motor_lifecycle.h"

#include <math.h>
#include <string.h>
#include "application_config.h"
#include "axis_config.h"
#include "bsp_system.h"
#include "encoder_calibration.h"
#include "motor_control.h"
#include "motor_fault.h"
#include "motor_sensing.h"
#include "parameter_values.h"

static void apply_state_event(MotorLifecycle *life, MotorStateEvent event)
{
    life->state = MotorStateMachine_NextState(life->state, event);
}

void MotorLifecycle_Init(MotorLifecycle *life, const MotorLifecycleBindings *bindings)
{
    memset(life, 0, sizeof(*life));
    life->drive = *bindings;
    life->state = MOTOR_STATE_IDLE;
    life->epoch = 2U;
    bindings->motor->control_mode = MOTOR_CONTROL_NONE;
}

MotorState MotorLifecycle_GetState(const MotorLifecycle *life)
{
    return life->state;
}

uint32_t MotorLifecycle_WorkEpoch(const MotorLifecycle *life)
{
    return life->epoch;
}

/* 启动和标定准入都读取本轴的实测输入。启相前由 ADC 再次复核。 */
static bool measurements_valid(MotorLifecycle *life)
{
    return life->fast_samples_fresh && MotorCurrentOffsets_IsValid(life->drive.configuration) &&
           isfinite(life->drive.foc->bus_voltage_v) &&
           isfinite(life->drive.foc->bus_voltage_filtered_v) &&
           isfinite(life->drive.foc->phase_current_a.a) &&
           isfinite(life->drive.foc->phase_current_a.b) &&
           isfinite(life->drive.foc->phase_current_a.c) &&
           (!BSP_REQUIRED_MCU_TEMPERATURE || (McuTemperature_IsFresh(life->drive.temperature) &&
                                              isfinite(life->drive.foc->temperature_c)));
}

static bool bus_voltage_in_enable_range(float bus_voltage_v)
{
    /* 两端均为有限阈值；NaN 与无穷也无法同时满足闭区间比较。 */
    return bus_voltage_v >= BUS_VOLTAGE_ENABLE_MIN_V && bus_voltage_v <= BUS_VOLTAGE_ENABLE_MAX_V;
}

static bool power_supply_valid(const MotorLifecycle *life)
{
    const FocState *foc = life->drive.foc;
    return bus_voltage_in_enable_range(foc->bus_voltage_filtered_v) &&
           foc->bus_voltage_v < BUS_VOLTAGE_HARD_OVERVOLTAGE_V;
}

static bool control_inputs_valid(MotorLifecycle *life, MotorControlMode mode)
{
    bool sensorless = mode == MOTOR_CONTROL_CURRENT && life->drive.motor->uses_sensorless_feedback;
    return measurements_valid(life) && life->drive.configuration->application.axis_profile_valid &&
           life->drive.configuration->configuration_confirmed && power_supply_valid(life) &&
           (sensorless || Encoder_IsOnline(life->drive.rotor_encoder)) &&
           (sensorless || MotorControlMode_IsOpenLoop(mode) ||
            (Encoder_GetCalibrationFlag(&life->drive.configuration->device.encoder) &
             ENC_CALIB_ALL) == ENC_CALIB_ALL);
}

static bool calibration_requires_power(MotorCalibrationType calibration_type)
{
    return calibration_type == MOTOR_CALIBRATION_ENCODER_LINEARIZATION ||
           calibration_type == MOTOR_CALIBRATION_ELECTRICAL_ZERO ||
           calibration_type == MOTOR_CALIBRATION_JOINT_ELECTRICAL_ZERO;
}

static bool calibration_inputs_valid(MotorLifecycle *life, MotorCalibrationType kind)
{
    if (!measurements_valid(life) || !life->drive.configuration->application.axis_profile_valid ||
        !Encoder_IsOnline(life->drive.rotor_encoder))
    {
        return false;
    }
    if (!calibration_requires_power(kind))
    {
        return true;
    }
    const MotorConfiguration *config = life->drive.configuration;
    if (kind == MOTOR_CALIBRATION_ENCODER_LINEARIZATION && config->preset != NULL)
    {
        const SensorlessStartupConfig *startup = config->preset->startup;
        float budget =
            fminf(config->model.calibration_current_a, config->application.current_limit_a);
        if (budget < startup->align_current_a ||
            budget * budget < startup->startup_iq_a * startup->startup_iq_a +
                                  startup->startup_id_a * startup->startup_id_a)
        {
            return false;
        }
    }
    return config->configuration_confirmed && power_supply_valid(life) &&
           ((kind != MOTOR_CALIBRATION_ELECTRICAL_ZERO &&
             kind != MOTOR_CALIBRATION_JOINT_ELECTRICAL_ZERO) ||
            (Encoder_GetCalibrationFlag(&life->drive.configuration->device.encoder) &
             ENC_CALIB_LINEARIZED) != 0U);
}

static bool advance_session_epoch(MotorLifecycle *life)
{
    if (life->epoch == UINT32_MAX)
    {
        life->fault_locked = true;
        MotorFault_Report(life->drive.motor, MOTOR_FAULT_CONTROL_OVERRUN);
        return false;
    }
    ++life->epoch;
    return true;
}

static void reset_control_state(MotorLifecycle *life)
{
    MotorControlState *motor = life->drive.motor;
    motor->id_target_a = 0.0f;
    motor->iq_target_a = 0.0f;
    motor->vq_target_v = 0.0f;
    motor->vd_target_v = 0.0f;
    motor->open_loop_frequency_hz = 0.0f;
    motor->open_loop_angle_rad = 0.0f;
    motor->speed_target_rad_s = 0.0f;
    motor->speed_ramped_rad_s = 0.0f;
    motor->position_reference_rad = 0.0f;
    motor->position_target_reached = false;
    motor->position_velocity_filtered_rad_s = 0.0f;
    life->drive.foc->current_dq_a.d = 0.0f;
    life->drive.foc->current_dq_a.q = 0.0f;
    Foc_ResetCurrent(life->drive.foc);
    PiController_Reset(life->drive.speed_pi);
    SensorlessStartup_Reset(life->drive.sensorless_startup);
    MotorOuterLoop_RequestReset(life->drive.outer);
}

static void disable_phases_on_fault(MotorLifecycle *life)
{
    if (life->drive.motor->fault_code != MOTOR_FAULT_NONE && !life->drive.power->phases_disabled())
    {
        life->drive.power->disable();
    }
}

/* IDLE → RUNNING：前台登记，外环发布许可，ADC 确认实际启相。 */
static void process_pending_stop(MotorLifecycle *life);
static void stop_on_unsafe_conditions(MotorLifecycle *life, bool configuration_valid);
static MotorControlResult accept_start_request(MotorLifecycle *life, MotorControlMode mode)
{
    MotorControlState *motor = life->drive.motor;
    if (motor->fault_code != MOTOR_FAULT_NONE)
    {
        return MOTOR_CONTROL_FAULT;
    }
    if (life->start_pending || life->starts_locked)
    {
        return MOTOR_CONTROL_BUSY;
    }
    if (life->state != MOTOR_STATE_IDLE || life->stop_pending ||
        motor->control_mode != MOTOR_CONTROL_NONE || motor->fault_code != MOTOR_FAULT_NONE ||
        !life->drive.power->phases_disabled() || motor->uses_sensorless_feedback ||
        !ParameterValue_ConfigurationValid(life->drive.configuration) ||
        !control_inputs_valid(life, mode))
    {
        return MOTOR_CONTROL_DENIED;
    }
    float start_position_rad = 0.0f;
    if (mode == MOTOR_CONTROL_POSITION)
    {
        start_position_rad = Encoder_GetMechPos(life->drive.rotor_encoder);
        const MotorConfiguration *config = life->drive.configuration;
        if (!MotorAxisProfile_AllowsPosition(&config->application.axis_profile,
                                             config->application.axis_profile_valid,
                                             start_position_rad,
                                             start_position_rad))
        {
            return motor->fault_code == MOTOR_FAULT_NONE ? MOTOR_CONTROL_DENIED
                                                         : MOTOR_CONTROL_FAULT;
        }
    }
    if (!advance_session_epoch(life))
    {
        return MOTOR_CONTROL_FAULT;
    }
    reset_control_state(life);
    if (mode == MOTOR_CONTROL_POSITION)
    {
        motor->position_target_rad = start_position_rad;
        motor->position_reference_rad = start_position_rad;
    }
    motor->control_mode = mode;
    life->start_pending = true;
    return MOTOR_CONTROL_ACCEPTED;
}

MotorControlResult MotorLifecycle_RequestEnable(MotorLifecycle *life, MotorControlMode mode)
{
    if (!MotorControlMode_IsValid(mode))
    {
        return MOTOR_CONTROL_INVALID;
    }
    uint32_t mask = BspSystem_EnterControlCritical();
    MotorControlResult result = accept_start_request(life, mode);
    BspSystem_ExitControlCritical(mask);
    return result;
}

void MotorLifecycle_CommitOuter(MotorLifecycle *life, const MotorOuterWork *work)
{
    if (work->owner != life)
    {
        return;
    }
    bool configuration_valid = ParameterValue_ConfigurationValid(life->drive.configuration);
    uint32_t mask = BspSystem_EnterControlCritical();
    process_pending_stop(life);
    bool same_session =
        work->epoch == life->epoch && work->motor.control_mode == life->drive.motor->control_mode;
    if (work->allowed && same_session && work->motor.fault_code != MOTOR_FAULT_NONE)
    {
        MotorFault_Report(life->drive.motor, (MotorFaultCode)work->motor.fault_code);
    }
    stop_on_unsafe_conditions(life, configuration_valid);
    bool uses_outer_loop = life->drive.motor->control_mode == MOTOR_CONTROL_SPEED ||
                           life->drive.motor->control_mode == MOTOR_CONTROL_POSITION;
    bool outer_ready = !uses_outer_loop || MotorOuterLoop_IsReady(life->drive.outer);
    bool can_commit = work->allowed && same_session &&
                      life->drive.motor->fault_code == MOTOR_FAULT_NONE && !life->stop_pending &&
                      !life->starts_locked;
    if (can_commit && outer_ready)
    {
        if (uses_outer_loop)
        {
            MotorControlState *motor = life->drive.motor;
            motor->id_target_a = work->motor.id_target_a;
            motor->iq_target_a = work->motor.iq_target_a;
            motor->uses_speed_ramp = work->motor.uses_speed_ramp;
            motor->speed_ramped_rad_s = work->motor.speed_ramped_rad_s;
            motor->position_reference_rad = work->motor.position_reference_rad;
            motor->position_target_reached = work->motor.position_target_reached;
            motor->position_velocity_filtered_rad_s = work->motor.position_velocity_filtered_rad_s;
            motor->position_trajectory_speed_rad_s = work->motor.position_trajectory_speed_rad_s;
            motor->position_trajectory_acceleration_rad_s2 =
                work->motor.position_trajectory_acceleration_rad_s2;
            *life->drive.speed_pi = work->speed_pi;
        }
        /* ADC 可随时确认已有许可。只给仍待启动的轴登记一次。 */
        if (life->phase_enable_epoch == 0U && life->state == MOTOR_STATE_IDLE &&
            life->start_pending && control_inputs_valid(life, life->drive.motor->control_mode))
        {
            BspSystem_MemoryBarrier();
            life->phase_enable_epoch = work->epoch;
        }
    }
    else if (!can_commit && work->allowed)
    {
        MotorOuterLoop_RequestReset(life->drive.outer);
    }
    ++life->outer_ticks;
    BspSystem_ExitControlCritical(mask);
}

static bool can_enable_phases(MotorLifecycle *life, uint32_t epoch)
{
    if (epoch != life->epoch || life->starts_locked)
    {
        return false;
    }
    switch (life->state)
    {
    case MOTOR_STATE_IDLE:
    {
        MotorControlMode mode = life->drive.motor->control_mode;
        bool uses_outer = mode == MOTOR_CONTROL_SPEED || mode == MOTOR_CONTROL_POSITION;
        return life->start_pending && control_inputs_valid(life, mode) &&
               (!uses_outer || MotorOuterLoop_IsReady(life->drive.outer));
    }
    case MOTOR_STATE_CALIBRATION:
        return calibration_requires_power(life->calibration_type) &&
               life->calibration_handoff == MOTOR_CALIBRATION_NO_HANDOFF &&
               calibration_inputs_valid(life, life->calibration_type);
    default:
        return false;
    }
}

void MotorLifecycle_BeginFastTick(MotorLifecycle *life)
{
    if (life->drive.motor->control_mode == MOTOR_CONTROL_NONE &&
        isfinite(life->drive.foc->bus_voltage_v) &&
        life->drive.foc->bus_voltage_v >= BUS_VOLTAGE_HARD_OVERVOLTAGE_V)
    {
        MotorFault_Report(life->drive.motor, MOTOR_FAULT_OVERVOLTAGE);
    }
    if (!life->drive.configuration->application.axis_profile_valid &&
        (!life->drive.power->phases_disabled() ||
         life->drive.motor->control_mode != MOTOR_CONTROL_NONE))
    {
        MotorFault_Report(life->drive.motor, MOTOR_FAULT_INVALID_PARAMETERS);
    }
    disable_phases_on_fault(life);
    if (life->stop_pending || life->drive.motor->fault_code != MOTOR_FAULT_NONE)
    {
        life->phase_enable_epoch = 0U;
        if (!life->drive.power->phases_disabled())
        {
            life->drive.power->disable();
        }
        return;
    }
    uint32_t epoch = life->phase_enable_epoch;
    if (epoch == 0U)
    {
        return;
    }
    life->phase_enable_epoch = 0U;
    if (can_enable_phases(life, epoch))
    {
        life->drive.power->enable();
        if (!life->drive.power->phases_disabled())
        {
            if (life->state == MOTOR_STATE_IDLE)
            {
                apply_state_event(life, MOTOR_STATE_EVENT_RUN);
                life->start_pending = false;
            }
            return;
        }
    }
    MotorFault_Report(life->drive.motor, MOTOR_FAULT_CONTROL_OVERRUN);
    disable_phases_on_fault(life);
}

/* STOP 先撤销启相许可。慢拍关相并结束会话，已有故障优先回到 FAULT。 */
static void mark_stop_pending(MotorLifecycle *life)
{
    life->stop_pending = true;
    life->phase_enable_epoch = 0U;
    life->start_pending = false;
    BspSystem_MemoryBarrier();
    life->drive.motor->control_mode = MOTOR_CONTROL_NONE;
}

void MotorLifecycle_RequestStop(MotorLifecycle *life)
{
    uint32_t mask = BspSystem_EnterControlCritical();
    mark_stop_pending(life);
    BspSystem_ExitControlCritical(mask);
}

static void stop_motor(MotorLifecycle *life, MotorStateEvent event)
{
    bool calibration_was_active = life->calibration_type != MOTOR_CALIBRATION_NONE;
    life->phase_enable_epoch = 0U;
    life->start_pending = false;
    life->calibration_type = MOTOR_CALIBRATION_NONE;
    life->calibration_handoff = MOTOR_CALIBRATION_NO_HANDOFF;
    BspSystem_MemoryBarrier();
    life->drive.power->disable();
    if (calibration_was_active)
    {
        EncoderCalibration_Cancel(life->drive.calibration);
    }
    life->drive.motor->control_mode = MOTOR_CONTROL_NONE;
    reset_control_state(life);
    (void)advance_session_epoch(life);
    if (!life->drive.power->phases_disabled())
    {
        life->fault_locked = true;
        MotorFault_Report(life->drive.motor, MOTOR_FAULT_CONTROL_OVERRUN);
    }
    if (life->drive.motor->fault_code != MOTOR_FAULT_NONE)
    {
        event = MOTOR_STATE_EVENT_FAULT;
    }
    apply_state_event(life, event);
}

static void process_pending_stop(MotorLifecycle *life)
{
    if (life->stop_pending)
    {
        stop_motor(life, MOTOR_STATE_EVENT_STOP);
        life->stop_pending = false;
    }
}

bool MotorLifecycle_ArePhasesDisabled(const MotorLifecycle *life)
{
    return life->drive.power->phases_disabled();
}

bool MotorLifecycle_IsStopped(const MotorLifecycle *life)
{
    return (life->state == MOTOR_STATE_IDLE || life->state == MOTOR_STATE_FAULT) &&
           !life->stop_pending && !life->start_pending &&
           life->drive.motor->control_mode == MOTOR_CONTROL_NONE &&
           life->drive.power->phases_disabled();
}

bool MotorLifecycle_LockStop(MotorLifecycle *life)
{
    uint32_t mask = BspSystem_EnterControlCritical();
    bool accepted = !life->fault_locked;
    if (accepted)
    {
        life->starts_locked = true;
        mark_stop_pending(life);
    }
    BspSystem_ExitControlCritical(mask);
    return accepted;
}

/* IDLE → CALIBRATION：完成算法、关相、发布和保存后，监督拍才结束标定。 */
static MotorControlResult accept_calibration_request(MotorLifecycle *life,
                                                     MotorCalibrationType kind)
{
    if (life->drive.motor->fault_code != MOTOR_FAULT_NONE)
    {
        return MOTOR_CONTROL_FAULT;
    }
    if (life->start_pending || life->stop_pending || life->starts_locked ||
        !EncoderCalibration_IsIdle(life->drive.calibration))
    {
        return MOTOR_CONTROL_BUSY;
    }
    if (life->state != MOTOR_STATE_IDLE || !life->drive.power->phases_disabled() ||
        !ParameterValue_ConfigurationValid(life->drive.configuration) ||
        !calibration_inputs_valid(life, kind))
    {
        return MOTOR_CONTROL_DENIED;
    }
    if (!advance_session_epoch(life))
    {
        return MOTOR_CONTROL_FAULT;
    }
    reset_control_state(life);
    life->drive.motor->control_mode = MOTOR_CONTROL_NONE;
    life->calibration_type = kind;
    apply_state_event(life, MOTOR_STATE_EVENT_CALIBRATE);
    if (calibration_requires_power(kind))
    {
        BspSystem_MemoryBarrier();
        life->phase_enable_epoch = life->epoch;
    }
    return MOTOR_CONTROL_ACCEPTED;
}

MotorControlResult MotorLifecycle_RequestCalibration(MotorLifecycle *life,
                                                     MotorCalibrationType calibration_type)
{
    if (calibration_type <= MOTOR_CALIBRATION_NONE ||
        calibration_type > MOTOR_CALIBRATION_MECHANICAL_ZERO)
    {
        return MOTOR_CONTROL_INVALID;
    }
    uint32_t mask = BspSystem_EnterControlCritical();
    MotorControlResult result = accept_calibration_request(life, calibration_type);
    BspSystem_ExitControlCritical(mask);
    return result;
}

MotorCalibrationType MotorLifecycle_GetCalibrationType(const MotorLifecycle *life)
{
    return life->state == MOTOR_STATE_CALIBRATION ? life->calibration_type : MOTOR_CALIBRATION_NONE;
}

bool MotorLifecycle_IsCalibrationRunnable(const MotorLifecycle *life)
{
    if (life->state != MOTOR_STATE_CALIBRATION ||
        life->calibration_type == MOTOR_CALIBRATION_NONE || life->stop_pending ||
        life->drive.motor->fault_code != MOTOR_FAULT_NONE ||
        life->calibration_handoff != MOTOR_CALIBRATION_NO_HANDOFF || life->starts_locked)
    {
        return false;
    }
    return calibration_requires_power(life->calibration_type)
               ? !life->drive.power->phases_disabled()
               : life->drive.power->phases_disabled();
}

/* ADC 已完成标定算法。只有实际关相后，才把发布资格交给前台。 */
static void disable_phases_for_publication(MotorLifecycle *life)
{
    life->phase_enable_epoch = 0U;
    life->drive.power->disable();
    BspSystem_MemoryBarrier();
    if (life->drive.power->phases_disabled())
    {
        life->calibration_handoff = MOTOR_CALIBRATION_AWAITING_PUBLICATION;
    }
    else
    {
        MotorFault_Report(life->drive.motor, MOTOR_FAULT_CONTROL_OVERRUN);
    }
}

void MotorLifecycle_PostOutcome(MotorLifecycle *life, MotorTaskResult outcome, uint32_t epoch)
{
    ++life->fast_sample_ticks;
    if (epoch != life->epoch || life->stop_pending)
    {
        disable_phases_on_fault(life);
        return;
    }
    switch (outcome.result)
    {
    case MOTOR_TASK_FAILED:
        MotorFault_Report(life->drive.motor,
                          outcome.error == MOTOR_FAULT_NONE ? MOTOR_FAULT_CONTROL_OVERRUN
                                                            : outcome.error);
        break;
    case MOTOR_TASK_STOP_REQUESTED:
        mark_stop_pending(life);
        break;
    case MOTOR_TASK_COMPLETED:
    case MOTOR_TASK_FINALIZE:
        if (life->state == MOTOR_STATE_CALIBRATION &&
            life->calibration_type != MOTOR_CALIBRATION_NONE &&
            (outcome.result == MOTOR_TASK_COMPLETED ||
             life->calibration_type == MOTOR_CALIBRATION_ENCODER_LINEARIZATION))
        {
            disable_phases_for_publication(life);
        }
        break;
    default:
        break;
    }
    disable_phases_on_fault(life);
}

bool MotorLifecycle_ReadPendingCalibrationPublication(const MotorLifecycle *life,
                                                      uint32_t *calibration_epoch)
{
    if (life->calibration_handoff != MOTOR_CALIBRATION_AWAITING_PUBLICATION ||
        life->state != MOTOR_STATE_CALIBRATION || life->stop_pending ||
        life->drive.motor->fault_code != MOTOR_FAULT_NONE || !life->drive.power->phases_disabled())
    {
        return false;
    }
    *calibration_epoch = life->epoch;
    return true;
}

void MotorLifecycle_ContinueElectricalZero(MotorLifecycle *life)
{
    life->calibration_type = MOTOR_CALIBRATION_JOINT_ELECTRICAL_ZERO;
    life->calibration_handoff = MOTOR_CALIBRATION_NO_HANDOFF;
    BspSystem_MemoryBarrier();
    life->phase_enable_epoch = life->epoch;
}

void MotorLifecycle_CompleteCalibrationPublication(MotorLifecycle *life,
                                                   uint32_t calibration_epoch,
                                                   bool publication_succeeded)
{
    uint32_t current_calibration_epoch;
    if (!MotorLifecycle_ReadPendingCalibrationPublication(life, &current_calibration_epoch) ||
        current_calibration_epoch != calibration_epoch)
    {
        return;
    }
    if (publication_succeeded)
    {
        BspSystem_MemoryBarrier();
        life->calibration_handoff = MOTOR_CALIBRATION_AWAITING_COMPLETION;
    }
    else
    {
        life->calibration_handoff = MOTOR_CALIBRATION_NO_HANDOFF;
        MotorFault_Report(life->drive.motor, MOTOR_FAULT_ENCODER);
        disable_phases_on_fault(life);
    }
}

static void complete_calibration_after_save(MotorLifecycle *life)
{
    if (life->state != MOTOR_STATE_CALIBRATION ||
        life->calibration_handoff != MOTOR_CALIBRATION_AWAITING_COMPLETION)
    {
        return;
    }
    /* 等待收尾时快环已经关相且不能再执行标定，监督拍独占状态迁移。 */
    if (!life->stop_pending && life->drive.motor->fault_code == MOTOR_FAULT_NONE)
    {
        stop_motor(life, MOTOR_STATE_EVENT_COMPLETE);
    }
}

/* FAULT → IDLE：显式清错，源已恢复且输出关闭；失败仍锁存故障。 */
static void enter_fault(MotorLifecycle *life, MotorFaultCode fault)
{
    MotorFault_Report(life->drive.motor,
                      fault == MOTOR_FAULT_NONE ? MOTOR_FAULT_CONTROL_OVERRUN : fault);
    if (life->state != MOTOR_STATE_FAULT)
    {
        stop_motor(life, MOTOR_STATE_EVENT_FAULT);
    }
    else if (!life->drive.power->phases_disabled())
    {
        life->drive.power->disable();
    }
}

static bool try_clear_recovered_fault(MotorLifecycle *life)
{
    MotorFaultCode fault = (MotorFaultCode)life->drive.motor->fault_code;
    if (life->fault_locked || !life->drive.power->phases_disabled() || !measurements_valid(life) ||
        !EncoderCalibration_IsIdle(life->drive.calibration) ||
        !ParameterValue_ConfigurationValid(life->drive.configuration))
    {
        return false;
    }
    if (fault != MOTOR_FAULT_NONE &&
        !MotorFault_SourceRecovered(life->drive.configuration,
                                    &life->recovery,
                                    life->drive.motor,
                                    life->drive.foc,
                                    life->drive.rotor_encoder,
                                    fault,
                                    life->fast_samples_fresh,
                                    true,
                                    McuTemperature_IsFresh(life->drive.temperature)))
    {
        return false;
    }
    /* 外层已挡住监督拍；先准备空闲状态，最后才释放快环的故障门禁。 */
    life->drive.motor->control_mode = MOTOR_CONTROL_NONE;
    apply_state_event(life, MOTOR_STATE_EVENT_CLEAR);
    if (!BspSystem_CompareExchange32(&life->drive.motor->fault_code, fault, MOTOR_FAULT_NONE))
    {
        apply_state_event(life, MOTOR_STATE_EVENT_FAULT);
        return false;
    }
    return true;
}

MotorControlResult MotorLifecycle_RequestClear(MotorLifecycle *life)
{
    uint32_t mask = BspSystem_EnterControlCritical();
    MotorControlResult result = MOTOR_CONTROL_DENIED;
    if (life->start_pending || life->stop_pending || life->starts_locked)
    {
        result = MOTOR_CONTROL_BUSY;
    }
    else if ((life->state == MOTOR_STATE_IDLE &&
              life->drive.motor->fault_code == MOTOR_FAULT_NONE) ||
             (life->state == MOTOR_STATE_FAULT && try_clear_recovered_fault(life)))
    {
        result = MOTOR_CONTROL_ACCEPTED;
    }
    BspSystem_ExitControlCritical(mask);
    return result;
}

/* 监督拍处理 STOP、标定完成和故障，Loader 仅读取健康进展。 */
static void stop_on_unsafe_conditions(MotorLifecycle *life, bool configuration_valid)
{
    if (life->drive.motor->fault_code != MOTOR_FAULT_NONE)
    {
        enter_fault(life, (MotorFaultCode)life->drive.motor->fault_code);
    }
    else if ((life->state == MOTOR_STATE_RUNNING || life->start_pending) &&
             (!configuration_valid ||
              !control_inputs_valid(life, life->drive.motor->control_mode) ||
              (life->state == MOTOR_STATE_RUNNING && life->drive.power->phases_disabled())))
    {
        enter_fault(life, MOTOR_FAULT_CONTROL_OVERRUN);
    }
    else if (life->state == MOTOR_STATE_CALIBRATION &&
             life->calibration_handoff == MOTOR_CALIBRATION_NO_HANDOFF &&
             (!configuration_valid || !calibration_inputs_valid(life, life->calibration_type) ||
              (calibration_requires_power(life->calibration_type)
                   ? life->phase_enable_epoch == 0U && life->drive.power->phases_disabled()
                   : !life->drive.power->phases_disabled())))
    {
        enter_fault(life, MOTOR_FAULT_CONTROL_OVERRUN);
    }
}

void MotorLifecycle_SupervisorTick(MotorLifecycle *life)
{
    uint32_t sample_ticks = life->fast_sample_ticks;
    life->fast_samples_fresh = sample_ticks != life->observed_fast_sample_ticks;
    life->observed_fast_sample_ticks = sample_ticks;
    process_pending_stop(life);
    MotorFault_UpdateRecovery(
        &life->recovery, life->drive.foc, (MotorFaultCode)life->drive.motor->fault_code);
    bool configuration_valid = ParameterValue_ConfigurationValid(life->drive.configuration);
    uint32_t mask = BspSystem_EnterControlCritical();
    complete_calibration_after_save(life);
    stop_on_unsafe_conditions(life, configuration_valid);
    ++life->supervisor_ticks;
    BspSystem_ExitControlCritical(mask);
}

bool MotorLifecycle_ReadBootHealth(MotorLifecycle *life, uint32_t *progress)
{
    if (progress == NULL)
    {
        return false;
    }
    uint32_t mask = BspSystem_EnterControlCritical();
    *progress = life->supervisor_ticks;
    bool healthy = measurements_valid(life) && !life->fault_locked;
    BspSystem_ExitControlCritical(mask);
    return healthy;
}

/* 已公开的生命周期查询入口，新代码直接使用独立状态机。 */
MotorState MotorLifecycle_NextState(MotorState state, MotorStateEvent event)
{
    return MotorStateMachine_NextState(state, event);
}
