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

/* AT32F456 的链接脚本将这个纯函数单独放入快环 SRAM。 */
__attribute__((noinline)) MotorState MotorStateMachine_NextState(MotorState state,
                                                                 MotorStateEvent event)
{
    /* 故障事件优先于任何状态的正常出口。无法识别的状态或事件也进入故障。 */
    if ((unsigned)state > MOTOR_STATE_FAULT)
    {
        return MOTOR_STATE_FAULT;
    }
    switch (event)
    {
    case MOTOR_STATE_EVENT_RUN:
        return state == MOTOR_STATE_IDLE ? MOTOR_STATE_RUNNING : state;
    case MOTOR_STATE_EVENT_CALIBRATE:
        return state == MOTOR_STATE_IDLE ? MOTOR_STATE_CALIBRATION : state;
    case MOTOR_STATE_EVENT_STOP:
    case MOTOR_STATE_EVENT_COMPLETE:
        return state == MOTOR_STATE_RUNNING || state == MOTOR_STATE_CALIBRATION ? MOTOR_STATE_IDLE
                                                                                : state;
    case MOTOR_STATE_EVENT_CLEAR:
        return state == MOTOR_STATE_FAULT ? MOTOR_STATE_IDLE : state;
    case MOTOR_STATE_EVENT_FAULT:
    default:
        return MOTOR_STATE_FAULT;
    }
}

static bool try_enable_permitted_phases(MotorLifecycle *life);
static bool calibration_requires_power(MotorCalibrationType calibration_type);
static bool can_clear_latched_fault(MotorLifecycle *life, MotorFaultCode fault);
static void disable_phases_and_end_session(MotorLifecycle *life);

/* 前台、ADC 和慢拍在原调用点提交事件。状态及转换所需的功率动作在此决定。 */
static bool transition_motor_state(MotorLifecycle *life, MotorStateEvent event)
{
    switch (event)
    {
    case MOTOR_STATE_EVENT_RUN:
        if (!try_enable_permitted_phases(life))
        {
            return false;
        }
        if (life->state == MOTOR_STATE_IDLE)
        {
            life->state = MotorStateMachine_NextState(life->state, event);
            life->start_pending = false;
        }
        return true;

    case MOTOR_STATE_EVENT_CALIBRATE:
        life->state = MotorStateMachine_NextState(life->state, event);
        if (calibration_requires_power(life->calibration_type))
        {
            BspSystem_MemoryBarrier();
            life->phase_enable_epoch = life->epoch;
        }
        return true;

    case MOTOR_STATE_EVENT_CLEAR:
    {
        MotorFaultCode fault = (MotorFaultCode)life->drive.motor->fault_code;
        if (life->state != MOTOR_STATE_FAULT || !can_clear_latched_fault(life, fault))
        {
            return false;
        }
        /* 先准备空闲状态，再原子释放快环故障门禁；遇到新故障仍保持锁存。 */
        life->drive.motor->control_mode = MOTOR_CONTROL_NONE;
        life->state = MotorStateMachine_NextState(life->state, event);
        if (!BspSystem_CompareExchange32(&life->drive.motor->fault_code, fault, MOTOR_FAULT_NONE))
        {
            life->state = MOTOR_STATE_FAULT;
            return false;
        }
        return true;
    }

    case MOTOR_STATE_EVENT_COMPLETE:
        if (life->state != MOTOR_STATE_CALIBRATION ||
            life->calibration_handoff != MOTOR_CALIBRATION_AWAITING_COMPLETION ||
            life->stop_pending || life->drive.motor->fault_code != MOTOR_FAULT_NONE)
        {
            return false;
        }
        break;
    case MOTOR_STATE_EVENT_STOP:
        if (!life->stop_pending)
        {
            return false;
        }
        break;
    case MOTOR_STATE_EVENT_FAULT:
        if (life->state == MOTOR_STATE_FAULT)
        {
            if (!life->drive.power->phases_disabled())
            {
                life->drive.power->disable();
            }
            return true;
        }
        break;
    default:
        return false;
    }

    /* STOP、标定完成及首次故障都结束会话，关相失败时故障优先。 */
    disable_phases_and_end_session(life);
    life->state = MotorStateMachine_NextState(
        life->state,
        life->drive.motor->fault_code == MOTOR_FAULT_NONE ? event : MOTOR_STATE_EVENT_FAULT);
    if (event == MOTOR_STATE_EVENT_STOP)
    {
        life->stop_pending = false;
    }
    return true;
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
static bool has_valid_axis_samples(MotorLifecycle *life)
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

static bool bus_voltage_within_operating_limits(const MotorLifecycle *life)
{
    const FocState *foc = life->drive.foc;
    return bus_voltage_in_enable_range(foc->bus_voltage_filtered_v) &&
           foc->bus_voltage_v < BUS_VOLTAGE_HARD_OVERVOLTAGE_V;
}

static bool inputs_allow_control_mode(MotorLifecycle *life, MotorControlMode mode)
{
    bool sensorless = mode == MOTOR_CONTROL_CURRENT && life->drive.motor->uses_sensorless_feedback;
    return has_valid_axis_samples(life) &&
           life->drive.configuration->application.axis_profile_valid &&
           life->drive.configuration->configuration_confirmed &&
           bus_voltage_within_operating_limits(life) &&
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

static bool inputs_allow_calibration(MotorLifecycle *life, MotorCalibrationType kind)
{
    if (!has_valid_axis_samples(life) ||
        !life->drive.configuration->application.axis_profile_valid ||
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
    return config->configuration_confirmed && bus_voltage_within_operating_limits(life) &&
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

static void reset_targets_and_controllers(MotorLifecycle *life)
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
        !inputs_allow_control_mode(life, mode))
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
    reset_targets_and_controllers(life);
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
    (void)transition_motor_state(life, MOTOR_STATE_EVENT_STOP);
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
            life->start_pending && inputs_allow_control_mode(life, life->drive.motor->control_mode))
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

static bool try_enable_permitted_phases(MotorLifecycle *life)
{
    uint32_t epoch = life->phase_enable_epoch;
    if (epoch == 0U)
    {
        return false;
    }
    life->phase_enable_epoch = 0U;
    bool permitted = false;
    if (epoch == life->epoch && !life->starts_locked)
    {
        if (life->state == MOTOR_STATE_IDLE && life->start_pending)
        {
            MotorControlMode mode = life->drive.motor->control_mode;
            bool uses_outer = mode == MOTOR_CONTROL_SPEED || mode == MOTOR_CONTROL_POSITION;
            permitted = inputs_allow_control_mode(life, mode) &&
                        (!uses_outer || MotorOuterLoop_IsReady(life->drive.outer));
        }
        else if (life->state == MOTOR_STATE_CALIBRATION)
        {
            permitted = calibration_requires_power(life->calibration_type) &&
                        life->calibration_handoff == MOTOR_CALIBRATION_NO_HANDOFF &&
                        inputs_allow_calibration(life, life->calibration_type);
        }
    }
    if (permitted)
    {
        life->drive.power->enable();
        if (!life->drive.power->phases_disabled())
        {
            return true;
        }
    }
    MotorFault_Report(life->drive.motor, MOTOR_FAULT_CONTROL_OVERRUN);
    disable_phases_on_fault(life);
    return false;
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
    if (life->phase_enable_epoch != 0U)
    {
        (void)transition_motor_state(life, MOTOR_STATE_EVENT_RUN);
    }
}

/* STOP 先撤销启相许可。慢拍关相并结束会话，已有故障优先回到 FAULT。 */
static void revoke_enable_and_request_stop(MotorLifecycle *life)
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
    revoke_enable_and_request_stop(life);
    BspSystem_ExitControlCritical(mask);
}

static void disable_phases_and_end_session(MotorLifecycle *life)
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
    reset_targets_and_controllers(life);
    (void)advance_session_epoch(life);
    if (!life->drive.power->phases_disabled())
    {
        life->fault_locked = true;
        MotorFault_Report(life->drive.motor, MOTOR_FAULT_CONTROL_OVERRUN);
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
        revoke_enable_and_request_stop(life);
    }
    BspSystem_ExitControlCritical(mask);
    return accepted;
}

/* IDLE → CALIBRATION：完成算法、关相、发布和保存后，监督拍才结束标定。 */
MotorControlResult MotorLifecycle_RequestCalibration(MotorLifecycle *life,
                                                     MotorCalibrationType calibration_type)
{
    if (calibration_type <= MOTOR_CALIBRATION_NONE ||
        calibration_type > MOTOR_CALIBRATION_MECHANICAL_ZERO)
    {
        return MOTOR_CONTROL_INVALID;
    }
    uint32_t mask = BspSystem_EnterControlCritical();
    MotorControlResult result = MOTOR_CONTROL_DENIED;
    if (life->drive.motor->fault_code != MOTOR_FAULT_NONE)
    {
        result = MOTOR_CONTROL_FAULT;
    }
    else if (life->start_pending || life->stop_pending || life->starts_locked ||
             !EncoderCalibration_IsIdle(life->drive.calibration))
    {
        result = MOTOR_CONTROL_BUSY;
    }
    else if (life->state == MOTOR_STATE_IDLE && life->drive.power->phases_disabled() &&
             ParameterValue_ConfigurationValid(life->drive.configuration) &&
             inputs_allow_calibration(life, calibration_type))
    {
        result = MOTOR_CONTROL_FAULT;
        if (advance_session_epoch(life))
        {
            reset_targets_and_controllers(life);
            life->drive.motor->control_mode = MOTOR_CONTROL_NONE;
            life->calibration_type = calibration_type;
            (void)transition_motor_state(life, MOTOR_STATE_EVENT_CALIBRATE);
            result = MOTOR_CONTROL_ACCEPTED;
        }
    }
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
        revoke_enable_and_request_stop(life);
        break;
    case MOTOR_TASK_COMPLETED:
    case MOTOR_TASK_FINALIZE:
        if (life->state == MOTOR_STATE_CALIBRATION &&
            life->calibration_type != MOTOR_CALIBRATION_NONE &&
            (outcome.result == MOTOR_TASK_COMPLETED ||
             life->calibration_type == MOTOR_CALIBRATION_ENCODER_LINEARIZATION))
        {
            /* 关相读回成功后，前台才可保存标定。 */
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

static bool can_clear_latched_fault(MotorLifecycle *life, MotorFaultCode fault)
{
    if (life->fault_locked || !life->drive.power->phases_disabled() ||
        !has_valid_axis_samples(life) || !EncoderCalibration_IsIdle(life->drive.calibration) ||
        !ParameterValue_ConfigurationValid(life->drive.configuration))
    {
        return false;
    }
    return fault == MOTOR_FAULT_NONE ||
           MotorFault_SourceRecovered(life->drive.configuration,
                                      &life->recovery,
                                      life->drive.motor,
                                      life->drive.foc,
                                      life->drive.rotor_encoder,
                                      fault,
                                      life->fast_samples_fresh,
                                      true,
                                      McuTemperature_IsFresh(life->drive.temperature));
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
             transition_motor_state(life, MOTOR_STATE_EVENT_CLEAR))
    {
        result = MOTOR_CONTROL_ACCEPTED;
    }
    BspSystem_ExitControlCritical(mask);
    return result;
}

/* 监督拍处理 STOP、标定完成和故障，Loader 仅读取健康进展。 */
static void stop_on_unsafe_conditions(MotorLifecycle *life, bool configuration_valid)
{
    MotorFaultCode fault = (MotorFaultCode)life->drive.motor->fault_code;
    if (fault == MOTOR_FAULT_NONE)
    {
        if (life->state == MOTOR_STATE_RUNNING || life->start_pending)
        {
            if (configuration_valid &&
                inputs_allow_control_mode(life, life->drive.motor->control_mode) &&
                (life->state != MOTOR_STATE_RUNNING || !life->drive.power->phases_disabled()))
            {
                return;
            }
        }
        else if (life->state == MOTOR_STATE_CALIBRATION &&
                 life->calibration_handoff == MOTOR_CALIBRATION_NO_HANDOFF)
        {
            bool needs_power = calibration_requires_power(life->calibration_type);
            if (configuration_valid && inputs_allow_calibration(life, life->calibration_type) &&
                (needs_power
                     ? life->phase_enable_epoch != 0U || !life->drive.power->phases_disabled()
                     : life->drive.power->phases_disabled()))
            {
                return;
            }
        }
        else
        {
            return;
        }
        fault = MOTOR_FAULT_CONTROL_OVERRUN;
    }
    MotorFault_Report(life->drive.motor, fault);
    (void)transition_motor_state(life, MOTOR_STATE_EVENT_FAULT);
}

void MotorLifecycle_SupervisorTick(MotorLifecycle *life)
{
    uint32_t sample_ticks = life->fast_sample_ticks;
    life->fast_samples_fresh = sample_ticks != life->observed_fast_sample_ticks;
    life->observed_fast_sample_ticks = sample_ticks;
    (void)transition_motor_state(life, MOTOR_STATE_EVENT_STOP);
    MotorFault_UpdateRecovery(
        &life->recovery, life->drive.foc, (MotorFaultCode)life->drive.motor->fault_code);
    bool configuration_valid = ParameterValue_ConfigurationValid(life->drive.configuration);
    uint32_t mask = BspSystem_EnterControlCritical();
    (void)transition_motor_state(life, MOTOR_STATE_EVENT_COMPLETE);
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
    bool healthy = has_valid_axis_samples(life) && !life->fault_locked;
    BspSystem_ExitControlCritical(mask);
    return healthy;
}

/* 兼容已有调用方的纯迁移查询。 */
MotorState MotorLifecycle_NextState(MotorState state, MotorStateEvent event)
{
    return MotorStateMachine_NextState(state, event);
}
