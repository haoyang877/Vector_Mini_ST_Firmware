#include "motor_state_machine.h"

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
#include "yg_protocol_common.h"

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

static bool try_enable_permitted_phases(MotorStateMachine *machine);
static bool calibration_requires_power(MotorCalibrationType calibration_type);
static bool has_valid_axis_samples(MotorStateMachine *machine);
static void disable_phases_and_end_session(MotorStateMachine *machine);

/* 前台、ADC 和慢拍在原调用点提交事件。状态及转换所需的功率动作在此决定。 */
static bool transition_motor_state(MotorStateMachine *machine, MotorStateEvent event)
{
    switch (event)
    {
    case MOTOR_STATE_EVENT_RUN:
        if (!try_enable_permitted_phases(machine))
        {
            return false;
        }
        if (machine->state == MOTOR_STATE_IDLE)
        {
            machine->state = MotorStateMachine_NextState(machine->state, event);
            machine->start_pending = false;
        }
        return true;

    case MOTOR_STATE_EVENT_CALIBRATE:
        machine->state = MotorStateMachine_NextState(machine->state, event);
        if (calibration_requires_power(machine->calibration_type))
        {
            BspSystem_MemoryBarrier();
            machine->phase_enable_epoch = machine->epoch;
        }
        return true;

    case MOTOR_STATE_EVENT_CLEAR:
    {
        MotorFaultCode fault = (MotorFaultCode)machine->drive.motor->fault_code;
        if (machine->state != MOTOR_STATE_FAULT || machine->fault_locked ||
            !machine->drive.power->phases_disabled() || !has_valid_axis_samples(machine) ||
            !EncoderCalibration_IsIdle(machine->drive.calibration) ||
            !ParameterValue_ConfigurationValid(machine->drive.configuration))
        {
            return false;
        }
        if (fault != MOTOR_FAULT_NONE &&
            !MotorFault_SourceRecovered(machine->drive.configuration,
                                        &machine->recovery,
                                        machine->drive.motor,
                                        machine->drive.foc,
                                        machine->drive.rotor_encoder,
                                        fault,
                                        machine->fast_samples_fresh,
                                        true,
                                        McuTemperature_IsFresh(machine->drive.temperature)))
        {
            return false;
        }
        /* 先准备空闲状态，再原子释放快环故障门禁；遇到新故障仍保持锁存。 */
        machine->drive.motor->control_mode = MOTOR_CONTROL_NONE;
        machine->state = MotorStateMachine_NextState(machine->state, event);
        if (!BspSystem_CompareExchange32(
                &machine->drive.motor->fault_code, fault, MOTOR_FAULT_NONE))
        {
            machine->state = MOTOR_STATE_FAULT;
            return false;
        }
        return true;
    }

    case MOTOR_STATE_EVENT_COMPLETE:
        if (machine->state != MOTOR_STATE_CALIBRATION || machine->stop_pending ||
            machine->drive.motor->fault_code != MOTOR_FAULT_NONE)
        {
            return false;
        }
        break;
    case MOTOR_STATE_EVENT_STOP:
        break;
    case MOTOR_STATE_EVENT_FAULT:
        if (machine->state == MOTOR_STATE_FAULT)
        {
            if (!machine->drive.power->phases_disabled())
            {
                machine->drive.power->disable();
            }
            return true;
        }
        break;
    default:
        return false;
    }

    bool calibration_was_active = machine->calibration_type != MOTOR_CALIBRATION_NONE;
    /* STOP、标定完成及首次故障都结束会话，关相失败时故障优先。 */
    disable_phases_and_end_session(machine);
    machine->state = MotorStateMachine_NextState(
        machine->state,
        machine->drive.motor->fault_code == MOTOR_FAULT_NONE ? event : MOTOR_STATE_EVENT_FAULT);
    if (calibration_was_active)
    {
        MotorCalibrationResult result =
            machine->drive.motor->fault_code != MOTOR_FAULT_NONE ? MOTOR_CALIBRATION_RESULT_FAILED
            : event == MOTOR_STATE_EVENT_COMPLETE ? MOTOR_CALIBRATION_RESULT_COMPLETED
                                                  : MOTOR_CALIBRATION_RESULT_CANCELLED;
        machine->calibration_failure = (MotorFaultCode)machine->drive.motor->fault_code;
        BspSystem_MemoryBarrier();
        machine->calibration_result = result;
    }
    if (event == MOTOR_STATE_EVENT_STOP)
    {
        machine->stop_pending = false;
    }
    return true;
}

void MotorStateMachine_Init(MotorStateMachine *machine, const MotorStateMachineBindings *bindings)
{
    memset(machine, 0, sizeof(*machine));
    machine->drive = *bindings;
    machine->state = MOTOR_STATE_IDLE;
    machine->epoch = 2U;
    bindings->motor->control_mode = MOTOR_CONTROL_NONE;
}

/* 启动和标定准入都读取本轴的实测输入。启相前由 ADC 再次复核。 */
static bool has_valid_axis_samples(MotorStateMachine *machine)
{
    return machine->fast_samples_fresh &&
           MotorCurrentOffsets_IsValid(machine->drive.configuration) &&
           isfinite(machine->drive.foc->bus_voltage_v) &&
           isfinite(machine->drive.foc->bus_voltage_filtered_v) &&
           isfinite(machine->drive.foc->phase_current_a.a) &&
           isfinite(machine->drive.foc->phase_current_a.b) &&
           isfinite(machine->drive.foc->phase_current_a.c) &&
           (!BSP_REQUIRED_MCU_TEMPERATURE || (McuTemperature_IsFresh(machine->drive.temperature) &&
                                              isfinite(machine->drive.foc->temperature_c)));
}

static bool bus_voltage_within_operating_limits(const FocState *foc)
{
    float filtered_v = foc->bus_voltage_filtered_v;
    return filtered_v >= BUS_VOLTAGE_ENABLE_MIN_V && filtered_v <= BUS_VOLTAGE_ENABLE_MAX_V &&
           foc->bus_voltage_v < BUS_VOLTAGE_HARD_OVERVOLTAGE_V;
}

static bool inputs_allow_control_mode(MotorStateMachine *machine, MotorControlMode mode)
{
    bool sensorless =
        mode == MOTOR_CONTROL_CURRENT && machine->drive.motor->uses_sensorless_feedback;
    return has_valid_axis_samples(machine) &&
           machine->drive.configuration->application.axis_profile_valid &&
           machine->drive.configuration->configuration_confirmed &&
           bus_voltage_within_operating_limits(machine->drive.foc) &&
           (sensorless || Encoder_IsOnline(machine->drive.rotor_encoder)) &&
           (sensorless || MotorControlMode_IsOpenLoop(mode) ||
            (Encoder_GetCalibrationFlag(&machine->drive.configuration->device.encoder) &
             ENC_CALIB_ALL) == ENC_CALIB_ALL);
}

static bool calibration_requires_power(MotorCalibrationType calibration_type)
{
    return calibration_type == MOTOR_CALIBRATION_ENCODER_LINEARIZATION ||
           calibration_type == MOTOR_CALIBRATION_ELECTRICAL_ZERO ||
           calibration_type == MOTOR_CALIBRATION_COMBINED_ELECTRICAL_ZERO;
}

static bool inputs_allow_calibration(MotorStateMachine *machine, MotorCalibrationType kind)
{
    if (!has_valid_axis_samples(machine) ||
        !machine->drive.configuration->application.axis_profile_valid ||
        !Encoder_IsOnline(machine->drive.rotor_encoder))
    {
        return false;
    }
    if (!calibration_requires_power(kind))
    {
        return true;
    }
    const MotorConfiguration *config = machine->drive.configuration;
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
    return config->configuration_confirmed &&
           bus_voltage_within_operating_limits(machine->drive.foc) &&
           ((kind != MOTOR_CALIBRATION_ELECTRICAL_ZERO &&
             kind != MOTOR_CALIBRATION_COMBINED_ELECTRICAL_ZERO) ||
            (Encoder_GetCalibrationFlag(&machine->drive.configuration->device.encoder) &
             ENC_CALIB_LINEARIZED) != 0U);
}

static bool advance_session_epoch(MotorStateMachine *machine)
{
    if (machine->epoch == UINT32_MAX)
    {
        machine->fault_locked = true;
        MotorFault_Report(machine->drive.motor, MOTOR_FAULT_CONTROL_OVERRUN);
        return false;
    }
    ++machine->epoch;
    return true;
}

static void reset_targets_and_controllers(MotorStateMachine *machine)
{
    MotorControlState *motor = machine->drive.motor;
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
    machine->drive.foc->current_dq_a.d = 0.0f;
    machine->drive.foc->current_dq_a.q = 0.0f;
    Foc_ResetCurrent(machine->drive.foc);
    PiController_Reset(machine->drive.speed_pi);
    SensorlessStartup_Reset(machine->drive.sensorless_startup);
    MotorOuterLoop_RequestReset(machine->drive.outer);
}

static void disable_phases_on_fault(MotorStateMachine *machine)
{
    if (machine->drive.motor->fault_code != MOTOR_FAULT_NONE &&
        !machine->drive.power->phases_disabled())
    {
        machine->drive.power->disable();
    }
}

/* IDLE → RUNNING：前台登记，外环发布许可，ADC 确认实际启相。 */
static void stop_on_unsafe_conditions(MotorStateMachine *machine, bool configuration_valid);
MotorControlResult MotorStateMachine_RequestStart(MotorStateMachine *machine, MotorControlMode mode)
{
    if (!MotorControlMode_IsValid(mode))
    {
        return MOTOR_CONTROL_INVALID;
    }
    uint32_t mask = BspSystem_EnterControlCritical();
    MotorControlState *motor = machine->drive.motor;
    const MotorConfiguration *config = machine->drive.configuration;
    MotorControlResult result = MOTOR_CONTROL_DENIED;
    if (motor->fault_code != MOTOR_FAULT_NONE)
    {
        result = MOTOR_CONTROL_FAULT;
    }
    else if (machine->parameters_updating || machine->start_pending || machine->starts_locked)
    {
        result = MOTOR_CONTROL_BUSY;
    }
    else if (machine->state == MOTOR_STATE_IDLE && !machine->stop_pending &&
             motor->control_mode == MOTOR_CONTROL_NONE && machine->drive.power->phases_disabled() &&
             !motor->uses_sensorless_feedback && ParameterValue_ConfigurationValid(config) &&
             inputs_allow_control_mode(machine, mode))
    {
        float position_rad = mode == MOTOR_CONTROL_POSITION
                                 ? Encoder_GetMechPos(machine->drive.rotor_encoder)
                                 : 0.0f;
        if (mode == MOTOR_CONTROL_POSITION &&
            !MotorAxisProfile_AllowsPosition(&config->application.axis_profile,
                                             config->application.axis_profile_valid,
                                             position_rad,
                                             position_rad))
        {
            result =
                motor->fault_code == MOTOR_FAULT_NONE ? MOTOR_CONTROL_DENIED : MOTOR_CONTROL_FAULT;
        }
        else if (!advance_session_epoch(machine))
        {
            result = MOTOR_CONTROL_FAULT;
        }
        else
        {
            reset_targets_and_controllers(machine);
            if (mode == MOTOR_CONTROL_POSITION)
            {
                motor->position_target_rad = position_rad;
                motor->position_reference_rad = position_rad;
            }
            motor->control_mode = mode;
            machine->start_pending = true;
            result = MOTOR_CONTROL_ACCEPTED;
        }
    }
    BspSystem_ExitControlCritical(mask);
    return result;
}

/* 先检查该模式的全部字段和限值，拒绝时保留原目标。 */
static bool dq_magnitude_within_limit(float d, float q, float limit)
{
    return isfinite(limit) && limit > 0.0f && d * d + q * q <= limit * limit;
}

static MotorControlResult validate_and_set_mode_target(MotorStateMachine *machine,
                                                       const MotorTarget *target)
{
    const MotorStateMachineBindings *drive = &machine->drive;
    const MotorConfiguration *configuration = drive->configuration;
    MotorControlState *motor = drive->motor;

    switch ((MotorControlMode)target->mode)
    {
    case MOTOR_CONTROL_CURRENT:
    {
        if (target->position_rad != 0.0f || target->speed_rad_s != 0.0f || target->vd_v != 0.0f ||
            target->vq_v != 0.0f || target->frequency_hz != 0.0f || target->id_a != 0.0f)
        {
            return MOTOR_CONTROL_INVALID;
        }
        float limit_a = configuration->application.current_limit_a;
        if (motor->thermal_limit_active)
        {
            limit_a = fminf(limit_a, motor->thermal_current_limit_a);
        }
        if (!dq_magnitude_within_limit(target->id_a, target->iq_a, limit_a))
        {
            return MOTOR_CONTROL_OUT_OF_RANGE;
        }
        motor->id_target_a = target->id_a;
        motor->iq_target_a = target->iq_a;
        break;
    }
    case MOTOR_CONTROL_SPEED:
    {
        if (target->position_rad != 0.0f || target->id_a != 0.0f || target->iq_a != 0.0f ||
            target->vd_v != 0.0f || target->vq_v != 0.0f || target->frequency_hz != 0.0f)
        {
            return MOTOR_CONTROL_INVALID;
        }
        float limit_rad_s = configuration->application.speed_limit_rad_s;
        if (!isfinite(limit_rad_s) || fabsf(target->speed_rad_s) > limit_rad_s)
        {
            return MOTOR_CONTROL_OUT_OF_RANGE;
        }
        motor->speed_target_rad_s = target->speed_rad_s;
        return MOTOR_CONTROL_ACCEPTED;
    }
    case MOTOR_CONTROL_POSITION:
        if (target->speed_rad_s != 0.0f || target->id_a != 0.0f || target->iq_a != 0.0f ||
            target->vd_v != 0.0f || target->vq_v != 0.0f || target->frequency_hz != 0.0f)
        {
            return MOTOR_CONTROL_INVALID;
        }
        if (!MotorAxisProfile_AllowsPosition(&configuration->application.axis_profile,
                                             configuration->application.axis_profile_valid,
                                             Encoder_GetMechPos(drive->rotor_encoder),
                                             target->position_rad))
        {
            return MOTOR_CONTROL_OUT_OF_RANGE;
        }
        motor->position_target_rad = target->position_rad;
        return MOTOR_CONTROL_ACCEPTED;
    case MOTOR_CONTROL_VQ:
        if (target->position_rad != 0.0f || target->speed_rad_s != 0.0f || target->id_a != 0.0f ||
            target->iq_a != 0.0f || target->vd_v != 0.0f || target->frequency_hz != 0.0f)
        {
            return MOTOR_CONTROL_INVALID;
        }
        if (!dq_magnitude_within_limit(
                target->vd_v, target->vq_v, Foc_MaxVoltage(drive->foc->bus_voltage_filtered_v)))
        {
            return MOTOR_CONTROL_OUT_OF_RANGE;
        }
        motor->vd_target_v = target->vd_v;
        motor->vq_target_v = target->vq_v;
        break;
    case MOTOR_CONTROL_OPEN_VOLTAGE:
        if (target->position_rad != 0.0f || target->speed_rad_s != 0.0f || target->id_a != 0.0f ||
            target->iq_a != 0.0f)
        {
            return MOTOR_CONTROL_INVALID;
        }
        if (!(fabsf(target->frequency_hz) <=
              MotorConfiguration_OpenLoopLimitHz(configuration,
                                                 MOTOR_OPEN_LOOP_MAX_ELECTRICAL_HZ)) ||
            !dq_magnitude_within_limit(
                target->vd_v, target->vq_v, Foc_MaxVoltage(drive->foc->bus_voltage_filtered_v)))
        {
            return MOTOR_CONTROL_OUT_OF_RANGE;
        }
        motor->vd_target_v = target->vd_v;
        motor->vq_target_v = target->vq_v;
        motor->open_loop_frequency_hz = target->frequency_hz;
        break;
    case MOTOR_CONTROL_OPEN_CURRENT:
    {
        if (target->position_rad != 0.0f || target->speed_rad_s != 0.0f || target->vd_v != 0.0f ||
            target->vq_v != 0.0f)
        {
            return MOTOR_CONTROL_INVALID;
        }
        float limit_a = configuration->application.current_limit_a;
        if (motor->thermal_limit_active)
        {
            limit_a = fminf(limit_a, motor->thermal_current_limit_a);
        }
        if (!(fabsf(target->frequency_hz) <=
              MotorConfiguration_OpenLoopLimitHz(configuration,
                                                 MOTOR_OPEN_LOOP_MAX_ELECTRICAL_HZ)) ||
            !dq_magnitude_within_limit(target->id_a, target->iq_a, limit_a))
        {
            return MOTOR_CONTROL_OUT_OF_RANGE;
        }
        motor->id_target_a = target->id_a;
        motor->iq_target_a = target->iq_a;
        motor->open_loop_frequency_hz = target->frequency_hz;
        break;
    }
    default:
        return MOTOR_CONTROL_INVALID;
    }

    return MOTOR_CONTROL_ACCEPTED;
}

MotorControlResult MotorStateMachine_SetTarget(MotorStateMachine *machine,
                                               const MotorTarget *target)
{
    if (target == NULL || !MotorControlMode_IsValid((MotorControlMode)target->mode) ||
        !isfinite(target->position_rad) || !isfinite(target->speed_rad_s) ||
        !isfinite(target->iq_a) || !isfinite(target->vq_v) || !isfinite(target->frequency_hz) ||
        !isfinite(target->id_a) || !isfinite(target->vd_v))
    {
        return MOTOR_CONTROL_INVALID;
    }

    uint32_t mask = BspSystem_EnterControlCritical();
    MotorControlResult result = MOTOR_CONTROL_DENIED;
    if (machine->drive.motor->fault_code != MOTOR_FAULT_NONE)
    {
        result = MOTOR_CONTROL_FAULT;
    }
    else if (machine->state == MOTOR_STATE_RUNNING && !machine->stop_pending &&
             !machine->starts_locked && target->mode == machine->drive.motor->control_mode)
    {
        result = validate_and_set_mode_target(machine, target);
    }
    BspSystem_ExitControlCritical(mask);
    return result;
}

void MotorStateMachine_ReadOuterInputs(MotorStateMachine *machine, MotorOuterWork *work)
{
    uint32_t mask = BspSystem_EnterControlCritical();
    work->owner = machine;
    work->allowed = false;
    for (unsigned attempt = 0U; attempt < 2U; ++attempt)
    {
        uint32_t fast = machine->fast_sample_ticks;
        BspSystem_MemoryBarrier();
        work->epoch = machine->epoch;
        work->motor = *machine->drive.motor;
        work->speed_pi = *machine->drive.speed_pi;
        BspSystem_MemoryBarrier();
        if (fast == machine->fast_sample_ticks)
        {
            work->allowed = (machine->state == MOTOR_STATE_RUNNING || machine->start_pending) &&
                            work->motor.control_mode != MOTOR_CONTROL_NONE &&
                            work->motor.fault_code == MOTOR_FAULT_NONE && !machine->stop_pending &&
                            !machine->starts_locked;
            break;
        }
    }
    BspSystem_ExitControlCritical(mask);
}

/* 空闲且没有启动请求时，监督逻辑无需扫描整套电机参数。 */
static bool active_configuration_valid(const MotorStateMachine *machine)
{
    return (!machine->start_pending && machine->state != MOTOR_STATE_RUNNING &&
            machine->state != MOTOR_STATE_CALIBRATION) ||
           ParameterValue_ConfigurationValid(machine->drive.configuration);
}

void MotorStateMachine_CommitOuterResult(MotorStateMachine *machine, const MotorOuterWork *work)
{
    if (work->owner != machine)
    {
        return;
    }
    bool configuration_valid = machine->parameters_updating || active_configuration_valid(machine);
    uint32_t mask = BspSystem_EnterControlCritical();
    if (machine->stop_pending)
    {
        (void)transition_motor_state(machine, MOTOR_STATE_EVENT_STOP);
    }
    bool current_work = work->allowed && work->epoch == machine->epoch &&
                        work->motor.control_mode == machine->drive.motor->control_mode;
    if (current_work && work->motor.fault_code != MOTOR_FAULT_NONE)
    {
        MotorFault_Report(machine->drive.motor, (MotorFaultCode)work->motor.fault_code);
    }
    stop_on_unsafe_conditions(machine, configuration_valid);
    bool uses_outer_loop = machine->drive.motor->control_mode == MOTOR_CONTROL_SPEED ||
                           machine->drive.motor->control_mode == MOTOR_CONTROL_POSITION;
    bool outer_ready = !uses_outer_loop || MotorOuterLoop_IsReady(machine->drive.outer);
    bool can_commit = current_work && machine->drive.motor->fault_code == MOTOR_FAULT_NONE &&
                      !machine->stop_pending && !machine->starts_locked;
    if (can_commit && outer_ready)
    {
        if (uses_outer_loop)
        {
            MotorControlState *motor = machine->drive.motor;
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
            *machine->drive.speed_pi = work->speed_pi;
        }
        /* ADC 可随时确认已有许可。只给仍待启动的轴登记一次。 */
        if (machine->phase_enable_epoch == 0U && machine->state == MOTOR_STATE_IDLE &&
            machine->start_pending)
        {
            BspSystem_MemoryBarrier();
            machine->phase_enable_epoch = work->epoch;
        }
    }
    else if (!can_commit && work->allowed)
    {
        MotorOuterLoop_RequestReset(machine->drive.outer);
    }
    ++machine->outer_ticks;
    BspSystem_ExitControlCritical(mask);
}

static bool try_enable_permitted_phases(MotorStateMachine *machine)
{
    uint32_t epoch = machine->phase_enable_epoch;
    machine->phase_enable_epoch = 0U;
    bool permitted = false;
    if (epoch == machine->epoch && !machine->starts_locked && !machine->parameters_updating)
    {
        if (machine->state == MOTOR_STATE_IDLE && machine->start_pending)
        {
            MotorControlMode mode = machine->drive.motor->control_mode;
            bool uses_outer = mode == MOTOR_CONTROL_SPEED || mode == MOTOR_CONTROL_POSITION;
            permitted = inputs_allow_control_mode(machine, mode) &&
                        (!uses_outer || MotorOuterLoop_IsReady(machine->drive.outer));
        }
        else if (machine->state == MOTOR_STATE_CALIBRATION)
        {
            permitted = calibration_requires_power(machine->calibration_type) &&
                        machine->calibration_handoff == MOTOR_CALIBRATION_NO_HANDOFF &&
                        inputs_allow_calibration(machine, machine->calibration_type);
        }
    }
    if (permitted)
    {
        machine->drive.power->enable();
        if (!machine->drive.power->phases_disabled())
        {
            return true;
        }
    }
    MotorFault_Report(machine->drive.motor, MOTOR_FAULT_CONTROL_OVERRUN);
    disable_phases_on_fault(machine);
    return false;
}

void MotorStateMachine_BeginFastTick(MotorStateMachine *machine)
{
    if (machine->drive.motor->control_mode == MOTOR_CONTROL_NONE &&
        isfinite(machine->drive.foc->bus_voltage_v) &&
        machine->drive.foc->bus_voltage_v >= BUS_VOLTAGE_HARD_OVERVOLTAGE_V)
    {
        MotorFault_Report(machine->drive.motor, MOTOR_FAULT_OVERVOLTAGE);
    }
    if (!machine->drive.configuration->application.axis_profile_valid &&
        (!machine->drive.power->phases_disabled() ||
         machine->drive.motor->control_mode != MOTOR_CONTROL_NONE))
    {
        MotorFault_Report(machine->drive.motor, MOTOR_FAULT_INVALID_PARAMETERS);
    }
    MotorFaultCode fault = (MotorFaultCode)machine->drive.motor->fault_code;
    if (machine->stop_pending || fault != MOTOR_FAULT_NONE)
    {
        const MotorPowerOutput *power = machine->drive.power;
        bool outputs_active = !power->phases_disabled();
        if (outputs_active && fault != MOTOR_FAULT_NONE)
        {
            power->disable();
            outputs_active = !power->phases_disabled();
        }
        machine->phase_enable_epoch = 0U;
        if (outputs_active)
        {
            power->disable();
        }
        return;
    }
    if (machine->phase_enable_epoch != 0U)
    {
        (void)transition_motor_state(machine, MOTOR_STATE_EVENT_RUN);
    }
}

/* STOP 先撤销启相许可。慢拍关相并结束会话，已有故障优先回到 FAULT。 */
static void revoke_enable_and_request_stop(MotorStateMachine *machine)
{
    machine->stop_pending = true;
    machine->phase_enable_epoch = 0U;
    machine->start_pending = false;
    BspSystem_MemoryBarrier();
    machine->drive.motor->control_mode = MOTOR_CONTROL_NONE;
}

void MotorStateMachine_RequestStop(MotorStateMachine *machine)
{
    uint32_t mask = BspSystem_EnterControlCritical();
    revoke_enable_and_request_stop(machine);
    BspSystem_ExitControlCritical(mask);
}

static void disable_phases_and_end_session(MotorStateMachine *machine)
{
    machine->motion_lease_active = false;
    bool calibration_was_active = machine->calibration_type != MOTOR_CALIBRATION_NONE;
    machine->phase_enable_epoch = 0U;
    machine->start_pending = false;
    machine->calibration_type = MOTOR_CALIBRATION_NONE;
    machine->calibration_handoff = MOTOR_CALIBRATION_NO_HANDOFF;
    BspSystem_MemoryBarrier();
    machine->drive.power->disable();
    if (calibration_was_active)
    {
        EncoderCalibration_Cancel(machine->drive.calibration);
    }
    machine->drive.motor->control_mode = MOTOR_CONTROL_NONE;
    reset_targets_and_controllers(machine);
    (void)advance_session_epoch(machine);
    if (!machine->drive.power->phases_disabled())
    {
        machine->fault_locked = true;
        MotorFault_Report(machine->drive.motor, MOTOR_FAULT_CONTROL_OVERRUN);
    }
}

bool MotorStateMachine_ArePhasesDisabled(const MotorStateMachine *machine)
{
    return machine->drive.power->phases_disabled();
}

bool MotorStateMachine_IsStopped(const MotorStateMachine *machine)
{
    return (machine->state == MOTOR_STATE_IDLE || machine->state == MOTOR_STATE_FAULT) &&
           !machine->stop_pending && !machine->start_pending &&
           machine->drive.motor->control_mode == MOTOR_CONTROL_NONE &&
           machine->drive.power->phases_disabled();
}

bool MotorStateMachine_LockStartsAndRequestStop(MotorStateMachine *machine)
{
    uint32_t mask = BspSystem_EnterControlCritical();
    bool accepted = !machine->fault_locked;
    if (accepted)
    {
        machine->starts_locked = true;
        revoke_enable_and_request_stop(machine);
    }
    BspSystem_ExitControlCritical(mask);
    return accepted;
}

/* IDLE → CALIBRATION：完成算法、关相、发布和保存后，监督拍才结束标定。 */
MotorControlResult MotorStateMachine_RequestCalibration(MotorStateMachine *machine,
                                                        MotorCalibrationType calibration_type)
{
    if (machine->drive.parameter_revision != NULL &&
        *machine->drive.parameter_revision == UINT32_MAX)
    {
        return MOTOR_CONTROL_OUT_OF_RANGE;
    }
    if (calibration_type <= MOTOR_CALIBRATION_NONE ||
        calibration_type > MOTOR_CALIBRATION_MECHANICAL_ZERO)
    {
        return MOTOR_CONTROL_INVALID;
    }
    uint32_t mask = BspSystem_EnterControlCritical();
    MotorControlResult result = MOTOR_CONTROL_DENIED;
    if (machine->drive.motor->fault_code != MOTOR_FAULT_NONE)
    {
        result = MOTOR_CONTROL_FAULT;
    }
    else if (machine->parameters_updating || machine->start_pending || machine->stop_pending ||
             machine->starts_locked || !EncoderCalibration_IsIdle(machine->drive.calibration))
    {
        result = MOTOR_CONTROL_BUSY;
    }
    else if (machine->state == MOTOR_STATE_IDLE && machine->drive.power->phases_disabled() &&
             ParameterValue_ConfigurationValid(machine->drive.configuration) &&
             inputs_allow_calibration(machine, calibration_type))
    {
        result = MOTOR_CONTROL_FAULT;
        if (advance_session_epoch(machine))
        {
            reset_targets_and_controllers(machine);
            machine->drive.motor->control_mode = MOTOR_CONTROL_NONE;
            machine->calibration_result_epoch = machine->epoch;
            machine->calibration_failure = MOTOR_FAULT_NONE;
            machine->calibration_result = MOTOR_CALIBRATION_RESULT_RUNNING;
            machine->calibration_type = calibration_type;
            (void)transition_motor_state(machine, MOTOR_STATE_EVENT_CALIBRATE);
            result = MOTOR_CONTROL_ACCEPTED;
        }
    }
    BspSystem_ExitControlCritical(mask);
    return result;
}

MotorCalibrationType MotorStateMachine_GetCalibrationType(const MotorStateMachine *machine)
{
    return machine->state == MOTOR_STATE_CALIBRATION ? machine->calibration_type
                                                     : MOTOR_CALIBRATION_NONE;
}

bool MotorStateMachine_IsCalibrationRunnable(const MotorStateMachine *machine)
{
    if (machine->state != MOTOR_STATE_CALIBRATION ||
        machine->calibration_type == MOTOR_CALIBRATION_NONE || machine->stop_pending ||
        machine->drive.motor->fault_code != MOTOR_FAULT_NONE ||
        machine->calibration_handoff != MOTOR_CALIBRATION_NO_HANDOFF || machine->starts_locked)
    {
        return false;
    }
    return calibration_requires_power(machine->calibration_type)
               ? !machine->drive.power->phases_disabled()
               : machine->drive.power->phases_disabled();
}

void MotorStateMachine_FinishFastTick(MotorStateMachine *machine,
                                      MotorTaskResult outcome,
                                      uint32_t epoch)
{
    ++machine->fast_sample_ticks;
    if (epoch != machine->epoch || machine->stop_pending)
    {
        disable_phases_on_fault(machine);
        return;
    }
    switch (outcome.result)
    {
    case MOTOR_TASK_FAILED:
        MotorFault_Report(machine->drive.motor,
                          outcome.error == MOTOR_FAULT_NONE ? MOTOR_FAULT_CONTROL_OVERRUN
                                                            : outcome.error);
        break;
    case MOTOR_TASK_STOP_REQUESTED:
        revoke_enable_and_request_stop(machine);
        break;
    case MOTOR_TASK_COMPLETED:
    case MOTOR_TASK_FINALIZE:
        if (machine->state == MOTOR_STATE_CALIBRATION &&
            machine->calibration_type != MOTOR_CALIBRATION_NONE &&
            (outcome.result == MOTOR_TASK_COMPLETED ||
             machine->calibration_type == MOTOR_CALIBRATION_ENCODER_LINEARIZATION))
        {
            /* 关相读回成功后，前台才可保存标定。 */
            machine->phase_enable_epoch = 0U;
            machine->drive.power->disable();
            BspSystem_MemoryBarrier();
            if (machine->drive.power->phases_disabled())
            {
                machine->calibration_handoff = MOTOR_CALIBRATION_AWAITING_PUBLICATION;
            }
            else
            {
                MotorFault_Report(machine->drive.motor, MOTOR_FAULT_CONTROL_OVERRUN);
            }
        }
        break;
    default:
        break;
    }
    disable_phases_on_fault(machine);
}

bool MotorStateMachine_ReadPendingCalibrationPublication(const MotorStateMachine *machine,
                                                         uint32_t *calibration_epoch)
{
    if (machine->calibration_handoff != MOTOR_CALIBRATION_AWAITING_PUBLICATION ||
        machine->state != MOTOR_STATE_CALIBRATION || machine->stop_pending ||
        machine->drive.motor->fault_code != MOTOR_FAULT_NONE ||
        !machine->drive.power->phases_disabled())
    {
        return false;
    }
    *calibration_epoch = machine->epoch;
    return true;
}

void MotorStateMachine_ContinueElectricalZero(MotorStateMachine *machine)
{
    machine->calibration_type = MOTOR_CALIBRATION_COMBINED_ELECTRICAL_ZERO;
    machine->calibration_handoff = MOTOR_CALIBRATION_NO_HANDOFF;
    BspSystem_MemoryBarrier();
    machine->phase_enable_epoch = machine->epoch;
}

void MotorStateMachine_CompleteCalibrationPublication(MotorStateMachine *machine,
                                                      uint32_t calibration_epoch,
                                                      bool publication_succeeded)
{
    uint32_t current_calibration_epoch;
    if (!MotorStateMachine_ReadPendingCalibrationPublication(machine, &current_calibration_epoch) ||
        current_calibration_epoch != calibration_epoch)
    {
        return;
    }
    if (publication_succeeded)
    {
        machine->calibration_saved_epoch = calibration_epoch;
        BspSystem_MemoryBarrier();
        machine->calibration_handoff = MOTOR_CALIBRATION_AWAITING_COMPLETION;
    }
    else
    {
        machine->calibration_handoff = MOTOR_CALIBRATION_NO_HANDOFF;
        MotorFault_Report(machine->drive.motor, MOTOR_FAULT_ENCODER);
        disable_phases_on_fault(machine);
    }
}

MotorControlResult MotorStateMachine_RequestFaultClear(MotorStateMachine *machine)
{
    uint32_t mask = BspSystem_EnterControlCritical();
    MotorControlResult result = MOTOR_CONTROL_DENIED;
    if (machine->parameters_updating || machine->start_pending || machine->stop_pending ||
        machine->starts_locked)
    {
        result = MOTOR_CONTROL_BUSY;
    }
    else if ((machine->state == MOTOR_STATE_IDLE &&
              machine->drive.motor->fault_code == MOTOR_FAULT_NONE) ||
             transition_motor_state(machine, MOTOR_STATE_EVENT_CLEAR))
    {
        result = MOTOR_CONTROL_ACCEPTED;
    }
    BspSystem_ExitControlCritical(mask);
    return result;
}

/* 监督拍处理 STOP、标定完成和故障，Loader 仅读取健康进展。 */
static void stop_on_unsafe_conditions(MotorStateMachine *machine, bool configuration_valid)
{
    if (machine->drive.motor->fault_code == MOTOR_FAULT_NONE)
    {
        if (machine->state == MOTOR_STATE_RUNNING || machine->start_pending)
        {
            if (configuration_valid &&
                inputs_allow_control_mode(machine, machine->drive.motor->control_mode) &&
                (machine->state != MOTOR_STATE_RUNNING || !machine->drive.power->phases_disabled()))
            {
                return;
            }
        }
        else if (machine->state == MOTOR_STATE_CALIBRATION)
        {
            MotorCalibrationType kind = machine->calibration_type;
            bool power_valid =
                calibration_requires_power(kind)
                    ? machine->phase_enable_epoch != 0U || !machine->drive.power->phases_disabled()
                    : machine->drive.power->phases_disabled();
            bool running_valid =
                configuration_valid && inputs_allow_calibration(machine, kind) && power_valid;
            /* ADC 可在监督检查期间关相并发布交接；只对同一运行阶段锁存故障。 */
            if (running_valid || machine->calibration_handoff != MOTOR_CALIBRATION_NO_HANDOFF ||
                machine->calibration_type != kind)
            {
                return;
            }
        }
        else
        {
            return;
        }
        MotorFault_Report(machine->drive.motor, MOTOR_FAULT_CONTROL_OVERRUN);
    }
    (void)transition_motor_state(machine, MOTOR_STATE_EVENT_FAULT);
}

void MotorStateMachine_SupervisorTick(MotorStateMachine *machine)
{
    if (machine->motion_lease_active &&
        (uint32_t)(BspSystem_TimeMs() - machine->motion_renewed_ms) >= PROTOCOL_MOTION_LEASE_MS)
    {
        machine->motion_lease_active = false;
        MotorFault_Report(machine->drive.motor, MOTOR_FAULT_CAN_DISCONNECTED);
        MotorStateMachine_RequestStop(machine);
    }
    uint32_t sample_ticks = machine->fast_sample_ticks;
    machine->fast_samples_fresh = sample_ticks != machine->observed_fast_sample_ticks;
    machine->observed_fast_sample_ticks = sample_ticks;
    if (machine->stop_pending)
    {
        (void)transition_motor_state(machine, MOTOR_STATE_EVENT_STOP);
    }
    MotorFault_UpdateRecovery(
        &machine->recovery, machine->drive.foc, (MotorFaultCode)machine->drive.motor->fault_code);
    bool configuration_valid = machine->parameters_updating || active_configuration_valid(machine);
    uint32_t mask = BspSystem_EnterControlCritical();
    if (machine->calibration_handoff == MOTOR_CALIBRATION_AWAITING_COMPLETION)
    {
        (void)transition_motor_state(machine, MOTOR_STATE_EVENT_COMPLETE);
    }
    stop_on_unsafe_conditions(machine, configuration_valid);
    ++machine->supervisor_ticks;
    BspSystem_ExitControlCritical(mask);
}

bool MotorStateMachine_ReadBootHealth(MotorStateMachine *machine, uint32_t *progress)
{
    if (progress == NULL)
    {
        return false;
    }
    uint32_t mask = BspSystem_EnterControlCritical();
    *progress = machine->supervisor_ticks;
    bool healthy = has_valid_axis_samples(machine) && !machine->fault_locked;
    BspSystem_ExitControlCritical(mask);
    return healthy;
}

static MotorStatus build_motor_feedback_snapshot(const MotorStateMachine *machine)
{
    const MotorControlState *motor = machine->drive.motor;
    const FocState *foc = machine->drive.foc;
    const EncoderState *encoder = machine->drive.rotor_encoder;
    bool active =
        machine->state == MOTOR_STATE_RUNNING && motor->control_mode != MOTOR_CONTROL_NONE;
    bool current = active && motor->control_mode != MOTOR_CONTROL_OPEN_VOLTAGE &&
                   motor->control_mode != MOTOR_CONTROL_VQ;
    bool position = active && motor->control_mode == MOTOR_CONTROL_POSITION;
    bool speed = active && motor->control_mode == MOTOR_CONTROL_SPEED;
    MotorOutputTelemetry output = {0};
    uint32_t now_ms = BspSystem_TimeMs();
    uint32_t age_ms = UINT32_MAX;
    if (machine->drive.output != NULL)
    {
        output = *machine->drive.output;
        if (output.sample_counter != 0U)
        {
            age_ms = (uint32_t)(now_ms - output.sample_ms);
        }
        if (age_ms > 10U)
        {
            output.flags &= (uint16_t)~3U;
        }
    }
    const MotorTemperatureTelemetry *ntc = machine->drive.motor_temperature;
    float motor_temperature_c =
        ntc != NULL && ntc->valid && (uint32_t)(now_ms - ntc->sample_ms) <= 30U ? ntc->celsius
                                                                                : NAN;
    return (MotorStatus){
        .output = output,
        .output_age_ms = age_ms,
        .motor_temperature_c = motor_temperature_c,
        .motor_temperature_valid = isfinite(motor_temperature_c),
        .state = machine->state,
        .mode = (uint16_t)motor->control_mode,
        .fault = (uint16_t)motor->fault_code,
        .measurements_valid = machine->fast_samples_fresh,
        .position_reference_rad = position ? motor->position_reference_rad : NAN,
        .position_rad = encoder->has_valid_sample ? encoder->mechanical_position_rad : NAN,
        .speed_reference_rad_s = speed      ? motor->speed_target_rad_s
                                 : position ? motor->speed_ramped_rad_s
                                            : NAN,
        .speed_rad_s =
            encoder->has_valid_sample ? encoder->mechanical_velocity_continuous_rad_s : NAN,
        .bus_voltage_v = foc->bus_voltage_filtered_v,
        .bus_current_a = foc->bus_current_filtered_a,
        .iq_reference_a = current ? motor->iq_target_a : NAN,
        .iq_a = active ? foc->current_dq_a.q : NAN,
        .temperature_c =
            McuTemperature_IsFresh(machine->drive.temperature) ? foc->temperature_c : NAN,
        .vq_v = active ? foc->voltage_dq_v.q : NAN,
        .vd_v = active ? foc->voltage_dq_v.d : NAN,
    };
}

bool MotorStateMachine_ReadStatus(const MotorStateMachine *machine, MotorStatus *status)
{
    /* 每个计数只有对应 ISR 写。前台被抢占后重读一次，连续争用则报告本帧不可用。 */
    for (unsigned attempt = 0U; attempt < 2U; ++attempt)
    {
        uint32_t fast = machine->fast_sample_ticks;
        uint32_t outer = machine->outer_ticks;
        uint32_t supervisor = machine->supervisor_ticks;
        BspSystem_MemoryBarrier();
        MotorStatus sample = build_motor_feedback_snapshot(machine);
        BspSystem_MemoryBarrier();
        if (fast == machine->fast_sample_ticks && outer == machine->outer_ticks &&
            supervisor == machine->supervisor_ticks)
        {
            *status = sample;
            return true;
        }
    }
    return false;
}
