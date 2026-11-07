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
#include "motor_state_machine.h"
#include "parameter_values.h"

/* 拥有启停、标定、目标提交和功率安全；无目标邮箱或协议请求身份。 */
static bool state_inputs_valid(MotorLifecycle *life)
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

static void clear_running_data(MotorLifecycle *life)
{
    life->drive.motor->id_target_a = 0.0f;
    life->drive.motor->iq_target_a = 0.0f;
    life->drive.motor->vq_target_v = 0.0f;
    life->drive.motor->vd_target_v = 0.0f;
    life->drive.motor->open_loop_frequency_hz = 0.0f;
    life->drive.motor->open_loop_angle_rad = 0.0f;
    life->drive.motor->speed_target_rad_s = 0.0f;
    life->drive.motor->speed_ramped_rad_s = 0.0f;
    life->drive.motor->position_reference_rad = 0.0f;
    life->drive.motor->position_target_reached = false;
    life->drive.motor->position_velocity_filtered_rad_s = 0.0f;
    life->drive.foc->current_dq_a.d = 0.0f;
    life->drive.foc->current_dq_a.q = 0.0f;
    Foc_ResetCurrent(life->drive.foc);
    PiController_Reset(life->drive.speed_pi);
    SensorlessStartup_Reset(life->drive.sensorless_startup);
    MotorOuterLoop_RequestReset(life->drive.outer);
}

static bool state_vbus_in_enable_window(float bus_voltage_v)
{
    /* 两端均为有限阈值；NaN 与无穷也无法同时满足闭区间比较。 */
    return bus_voltage_v >= BUS_VOLTAGE_ENABLE_MIN_V && bus_voltage_v <= BUS_VOLTAGE_ENABLE_MAX_V;
}

static bool new_epoch(MotorLifecycle *life)
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

static void stop_motor(MotorLifecycle *life, MotorStateEvent event)
{
    bool calibration_was_active = life->calibration_type != MOTOR_CALIBRATION_NONE;
    life->power_enable_epoch = 0U;
    life->enable_pending = false;
    life->calibration_type = MOTOR_CALIBRATION_NONE;
    life->calibration_completion_pending = false;
    life->calibration_publication_pending = false;
    BspSystem_MemoryBarrier();
    life->drive.power->disable();
    if (calibration_was_active)
    {
        EncoderCalibration_Cancel(life->drive.calibration);
    }
    life->drive.motor->control_mode = MOTOR_CONTROL_NONE;
    clear_running_data(life);
    (void)new_epoch(life);
    if (!life->drive.power->phases_disabled())
    {
        life->fault_locked = true;
        MotorFault_Report(life->drive.motor, MOTOR_FAULT_CONTROL_OVERRUN);
    }
    life->state = MotorStateMachine_Step(
        life->state,
        life->drive.motor->fault_code == MOTOR_FAULT_NONE ? event : MOTOR_STATE_EVENT_FAULT);
}

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

static void fast_fault_stop(MotorLifecycle *life)
{
    if (life->drive.motor->fault_code != MOTOR_FAULT_NONE && !life->drive.power->phases_disabled())
    {
        life->drive.power->disable();
    }
}

static bool output_inputs_valid(MotorLifecycle *life)
{
    MotorControlMode mode = life->drive.motor->control_mode;
    bool sensorless = mode == MOTOR_CONTROL_CURRENT && life->drive.motor->uses_sensorless_feedback;
    return state_inputs_valid(life) && life->drive.configuration->application.axis_profile_valid &&
           life->drive.configuration->configuration_confirmed &&
           state_vbus_in_enable_window(life->drive.foc->bus_voltage_filtered_v) &&
           life->drive.foc->bus_voltage_v < BUS_VOLTAGE_HARD_OVERVOLTAGE_V &&
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

static bool calibration_inputs_valid(MotorLifecycle *life)
{
    if (!state_inputs_valid(life) || !life->drive.configuration->application.axis_profile_valid ||
        !Encoder_IsOnline(life->drive.rotor_encoder))
    {
        return false;
    }
    if (!calibration_requires_power(life->calibration_type))
    {
        return true;
    }
    const MotorConfiguration *config = life->drive.configuration;
    if (life->calibration_type == MOTOR_CALIBRATION_ENCODER_LINEARIZATION && config->preset != NULL)
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
           state_vbus_in_enable_window(life->drive.foc->bus_voltage_filtered_v) &&
           life->drive.foc->bus_voltage_v < BUS_VOLTAGE_HARD_OVERVOLTAGE_V &&
           ((life->calibration_type != MOTOR_CALIBRATION_ELECTRICAL_ZERO &&
             life->calibration_type != MOTOR_CALIBRATION_JOINT_ELECTRICAL_ZERO) ||
            (Encoder_GetCalibrationFlag(&life->drive.configuration->device.encoder) &
             ENC_CALIB_LINEARIZED) != 0U);
}

void MotorLifecycle_Init(MotorLifecycle *life, const MotorLifecycleBindings *bindings)
{
    memset(life, 0, sizeof(*life));
    life->drive = *bindings;
    life->state = MOTOR_STATE_IDLE;
    life->epoch = 2U;
    bindings->motor->control_mode = MOTOR_CONTROL_NONE;
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
    fast_fault_stop(life);
    if (life->stop_pending || life->drive.motor->fault_code != MOTOR_FAULT_NONE)
    {
        life->power_enable_epoch = 0U;
        if (!life->drive.power->phases_disabled())
        {
            life->drive.power->disable();
        }
        return;
    }
    uint32_t epoch = life->power_enable_epoch;
    if (epoch == 0U)
    {
        return;
    }
    life->power_enable_epoch = 0U;
    bool outer = life->drive.motor->control_mode == MOTOR_CONTROL_SPEED ||
                 life->drive.motor->control_mode == MOTOR_CONTROL_POSITION;
    bool starting_control = life->state == MOTOR_STATE_IDLE && life->enable_pending;
    bool starting_calibration = life->state == MOTOR_STATE_CALIBRATION &&
                                calibration_requires_power(life->calibration_type) &&
                                !life->calibration_completion_pending &&
                                !life->calibration_publication_pending;
    bool permitted = epoch == life->epoch && !life->starts_locked &&
                     ((starting_control && output_inputs_valid(life) &&
                       (!outer || MotorOuterLoop_IsReady(life->drive.outer))) ||
                      (starting_calibration && calibration_inputs_valid(life)));
    if (permitted)
    {
        life->drive.power->enable();
        permitted = !life->drive.power->phases_disabled();
    }
    if (!permitted)
    {
        MotorFault_Report(life->drive.motor, MOTOR_FAULT_CONTROL_OVERRUN);
        fast_fault_stop(life);
        return;
    }
    if (starting_control)
    {
        life->state = MotorStateMachine_Step(life->state, MOTOR_STATE_EVENT_RUN);
        life->enable_pending = false;
    }
}

void MotorLifecycle_PostOutcome(MotorLifecycle *life, MotorTaskResult outcome, uint32_t epoch)
{
    ++life->fast_sample_ticks;
    if (epoch == life->epoch && !life->stop_pending)
    {
        if (outcome.result == MOTOR_TASK_FAILED)
        {
            MotorFault_Report(life->drive.motor,
                              outcome.error == MOTOR_FAULT_NONE ? MOTOR_FAULT_CONTROL_OVERRUN
                                                                : outcome.error);
        }
        else if (outcome.result == MOTOR_TASK_STOP_REQUESTED)
        {
            life->stop_pending = true;
            life->power_enable_epoch = 0U;
            life->enable_pending = false;
            BspSystem_MemoryBarrier();
            life->drive.motor->control_mode = MOTOR_CONTROL_NONE;
        }
        else if (life->state == MOTOR_STATE_CALIBRATION &&
                 life->calibration_type != MOTOR_CALIBRATION_NONE &&
                 (outcome.result == MOTOR_TASK_COMPLETED ||
                  (outcome.result == MOTOR_TASK_FINALIZE &&
                   life->calibration_type == MOTOR_CALIBRATION_ENCODER_LINEARIZATION)))
        {
            life->power_enable_epoch = 0U;
            life->drive.power->disable();
            BspSystem_MemoryBarrier();
            if (!life->drive.power->phases_disabled())
            {
                MotorFault_Report(life->drive.motor, MOTOR_FAULT_CONTROL_OVERRUN);
            }
            else
            {
                /* 两类传动的标定结果均经停机前台保存后才报告完成。 */
                life->calibration_publication_pending = true;
            }
        }
    }
    fast_fault_stop(life);
}

bool MotorLifecycle_RequestRemoteStop(MotorLifecycle *life)
{
    uint32_t mask = BspSystem_EnterControlCritical();
    life->stop_pending = true;
    life->power_enable_epoch = 0U;
    life->enable_pending = false;
    BspSystem_MemoryBarrier();
    life->drive.motor->control_mode = MOTOR_CONTROL_NONE;
    BspSystem_ExitControlCritical(mask);
    return true;
}

MotorControlResult MotorLifecycle_RequestEnable(MotorLifecycle *life, MotorControlMode mode)
{
    if (!MotorControlMode_IsValid(mode))
    {
        return MOTOR_CONTROL_INVALID;
    }
    uint32_t mask = BspSystem_EnterControlCritical();
    MotorControlResult result = MOTOR_CONTROL_DENIED;
    if (life->drive.motor->fault_code != MOTOR_FAULT_NONE)
    {
        result = MOTOR_CONTROL_FAULT;
    }
    else if (life->enable_pending || life->starts_locked)
    {
        result = MOTOR_CONTROL_BUSY;
    }
    else if (life->state == MOTOR_STATE_IDLE && !life->stop_pending &&
             life->drive.motor->control_mode == MOTOR_CONTROL_NONE &&
             life->drive.motor->fault_code == MOTOR_FAULT_NONE &&
             life->drive.power->phases_disabled() && !life->drive.motor->uses_sensorless_feedback &&
             ParameterValue_ConfigurationValid(life->drive.configuration) &&
             state_inputs_valid(life) && life->drive.configuration->configuration_confirmed &&
             Encoder_IsOnline(life->drive.rotor_encoder) &&
             state_vbus_in_enable_window(life->drive.foc->bus_voltage_filtered_v) &&
             life->drive.foc->bus_voltage_v < BUS_VOLTAGE_HARD_OVERVOLTAGE_V &&
             (MotorControlMode_IsOpenLoop(mode) ||
              (Encoder_GetCalibrationFlag(&life->drive.configuration->device.encoder) &
               ENC_CALIB_ALL) == ENC_CALIB_ALL))
    {
        float position_rad =
            mode == MOTOR_CONTROL_POSITION ? Encoder_GetMechPos(life->drive.rotor_encoder) : 0.0f;
        bool position_valid = mode != MOTOR_CONTROL_POSITION ||
                              MotorAxisProfile_AllowsPosition(
                                  &life->drive.configuration->application.axis_profile,
                                  life->drive.configuration->application.axis_profile_valid,
                                  position_rad,
                                  position_rad);
        if (position_valid && new_epoch(life))
        {
            clear_running_data(life);
            if (mode == MOTOR_CONTROL_POSITION)
            {
                life->drive.motor->position_target_rad = life->drive.motor->position_reference_rad =
                    position_rad;
            }
            life->drive.motor->control_mode = mode;
            life->enable_pending = true;
            result = MOTOR_CONTROL_ACCEPTED;
        }
        else if (life->drive.motor->fault_code != MOTOR_FAULT_NONE)
        {
            result = MOTOR_CONTROL_FAULT;
        }
    }
    BspSystem_ExitControlCritical(mask);
    return result;
}

MotorControlResult MotorLifecycle_SetTarget(MotorLifecycle *life, const MotorTarget *target)
{
    if (target == NULL || !MotorControlMode_IsValid((MotorControlMode)target->mode) ||
        !isfinite(target->position_rad) || !isfinite(target->speed_rad_s) ||
        !isfinite(target->iq_a) || !isfinite(target->vq_v) || !isfinite(target->frequency_hz) ||
        !isfinite(target->id_a) || !isfinite(target->vd_v))
    {
        return MOTOR_CONTROL_INVALID;
    }
    uint32_t mask = BspSystem_EnterCritical();
    MotorControlResult result = MOTOR_CONTROL_DENIED;
    MotorControlState *motor = life->drive.motor;
    if (motor->fault_code != MOTOR_FAULT_NONE)
    {
        result = MOTOR_CONTROL_FAULT;
    }
    else if (life->state == MOTOR_STATE_RUNNING && !life->stop_pending && !life->starts_locked &&
             target->mode == motor->control_mode)
    {
        float current_limit = life->drive.configuration->application.current_limit_a;
        float voltage_limit = Foc_MaxVoltage(life->drive.foc->bus_voltage_filtered_v);
        bool current =
            target->mode == MOTOR_CONTROL_CURRENT || target->mode == MOTOR_CONTROL_OPEN_CURRENT;
        bool voltage =
            target->mode == MOTOR_CONTROL_VQ || target->mode == MOTOR_CONTROL_OPEN_VOLTAGE;
        bool speed = target->mode == MOTOR_CONTROL_SPEED;
        bool position = target->mode == MOTOR_CONTROL_POSITION;
        bool open = MotorControlMode_IsOpenLoop((MotorControlMode)target->mode);
        if (motor->thermal_limit_active)
        {
            current_limit = fminf(current_limit, motor->thermal_current_limit_a);
        }
        bool fields = (position || target->position_rad == 0.0f) &&
                      (speed || target->speed_rad_s == 0.0f) && (current || target->iq_a == 0.0f) &&
                      (voltage || target->vq_v == 0.0f) && (open || target->frequency_hz == 0.0f) &&
                      (target->mode == MOTOR_CONTROL_OPEN_CURRENT || target->id_a == 0.0f) &&
                      (target->mode == MOTOR_CONTROL_OPEN_VOLTAGE || target->vd_v == 0.0f);
        bool range =
            (!current || (isfinite(current_limit) && current_limit > 0.0f &&
                          target->id_a * target->id_a + target->iq_a * target->iq_a <=
                              current_limit * current_limit)) &&
            (!voltage || (isfinite(voltage_limit) && voltage_limit > 0.0f &&
                          target->vd_v * target->vd_v + target->vq_v * target->vq_v <=
                              voltage_limit * voltage_limit)) &&
            (!speed || (isfinite(life->drive.configuration->application.speed_limit_rad_s) &&
                        fabsf(target->speed_rad_s) <=
                            life->drive.configuration->application.speed_limit_rad_s)) &&
            (!position || MotorAxisProfile_AllowsPosition(
                              &life->drive.configuration->application.axis_profile,
                              life->drive.configuration->application.axis_profile_valid,
                              Encoder_GetMechPos(life->drive.rotor_encoder),
                              target->position_rad)) &&
            (!open || fabsf(target->frequency_hz) <=
                          MotorConfiguration_OpenLoopLimitHz(life->drive.configuration,
                                                             MOTOR_OPEN_LOOP_MAX_ELECTRICAL_HZ));
        result = !fields  ? MOTOR_CONTROL_INVALID
                 : !range ? MOTOR_CONTROL_OUT_OF_RANGE
                          : MOTOR_CONTROL_ACCEPTED;
        if (result == MOTOR_CONTROL_ACCEPTED)
        {
            if (current)
            {
                motor->id_target_a = target->id_a;
                motor->iq_target_a = target->iq_a;
            }
            if (voltage)
            {
                motor->vd_target_v = target->vd_v;
                motor->vq_target_v = target->vq_v;
            }
            if (open)
            {
                motor->open_loop_frequency_hz = target->frequency_hz;
            }
            if (speed)
            {
                motor->speed_target_rad_s = target->speed_rad_s;
            }
            if (position)
            {
                motor->position_target_rad = target->position_rad;
            }
        }
    }
    BspSystem_ExitCritical(mask);
    return result;
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
    MotorControlResult result = MOTOR_CONTROL_DENIED;
    if (life->drive.motor->fault_code != MOTOR_FAULT_NONE)
    {
        result = MOTOR_CONTROL_FAULT;
    }
    else if (life->enable_pending || life->stop_pending || life->starts_locked ||
             !EncoderCalibration_IsIdle(life->drive.calibration))
    {
        result = MOTOR_CONTROL_BUSY;
    }
    else if (life->state == MOTOR_STATE_IDLE && life->drive.power->phases_disabled() &&
             ParameterValue_ConfigurationValid(life->drive.configuration))
    {
        life->calibration_type = calibration_type;
        bool inputs_valid = calibration_inputs_valid(life);
        if (inputs_valid && new_epoch(life))
        {
            clear_running_data(life);
            life->drive.motor->control_mode = MOTOR_CONTROL_NONE;
            life->state = MotorStateMachine_Step(life->state, MOTOR_STATE_EVENT_CALIBRATE);
            if (calibration_requires_power(calibration_type))
            {
                BspSystem_MemoryBarrier();
                life->power_enable_epoch = life->epoch;
            }
            result = MOTOR_CONTROL_ACCEPTED;
        }
        else if (inputs_valid)
        {
            result = MOTOR_CONTROL_FAULT;
        }
        if (result != MOTOR_CONTROL_ACCEPTED)
        {
            life->calibration_type = MOTOR_CALIBRATION_NONE;
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
        life->drive.motor->fault_code != MOTOR_FAULT_NONE || life->calibration_completion_pending ||
        life->calibration_publication_pending || life->starts_locked)
    {
        return false;
    }
    return calibration_requires_power(life->calibration_type)
               ? !life->drive.power->phases_disabled()
               : life->drive.power->phases_disabled();
}

bool MotorLifecycle_ReadPendingCalibrationPublication(const MotorLifecycle *life,
                                                      uint32_t *calibration_epoch)
{
    if (!life->calibration_publication_pending || life->state != MOTOR_STATE_CALIBRATION ||
        life->stop_pending || life->drive.motor->fault_code != MOTOR_FAULT_NONE ||
        !life->drive.power->phases_disabled())
    {
        return false;
    }
    *calibration_epoch = life->epoch;
    return true;
}

void MotorLifecycle_ContinueElectricalZero(MotorLifecycle *life)
{
    life->calibration_type = MOTOR_CALIBRATION_JOINT_ELECTRICAL_ZERO;
    life->calibration_publication_pending = false;
    BspSystem_MemoryBarrier();
    life->power_enable_epoch = life->epoch;
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
    life->calibration_publication_pending = false;
    if (publication_succeeded)
    {
        BspSystem_MemoryBarrier();
        life->calibration_completion_pending = true;
    }
    else
    {
        MotorFault_Report(life->drive.motor, MOTOR_FAULT_ENCODER);
        fast_fault_stop(life);
    }
}

static void finish_calibration(MotorLifecycle *life)
{
    if (life->state != MOTOR_STATE_CALIBRATION || !life->calibration_completion_pending)
    {
        return;
    }
    uint32_t mask = BspSystem_EnterCritical();
    if (!life->stop_pending && life->drive.motor->fault_code == MOTOR_FAULT_NONE)
    {
        stop_motor(life, MOTOR_STATE_EVENT_COMPLETE);
    }
    BspSystem_ExitCritical(mask);
}

static bool clear_fault(MotorLifecycle *life)
{
    MotorFaultCode fault = (MotorFaultCode)life->drive.motor->fault_code;
    if (life->fault_locked || !life->drive.power->phases_disabled() || !state_inputs_valid(life) ||
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
    uint32_t mask = BspSystem_EnterCritical();
    bool cleared =
        BspSystem_CompareExchange32(&life->drive.motor->fault_code, fault, MOTOR_FAULT_NONE);
    if (cleared)
    {
        life->state = MotorStateMachine_Step(life->state, MOTOR_STATE_EVENT_CLEAR);
        life->drive.motor->control_mode = MOTOR_CONTROL_NONE;
    }
    BspSystem_ExitCritical(mask);
    return cleared;
}

MotorControlResult MotorLifecycle_RequestClear(MotorLifecycle *life)
{
    uint32_t mask = BspSystem_EnterControlCritical();
    MotorControlResult result = MOTOR_CONTROL_DENIED;
    if (life->enable_pending || life->stop_pending || life->starts_locked)
    {
        result = MOTOR_CONTROL_BUSY;
    }
    else if ((life->state == MOTOR_STATE_IDLE &&
              life->drive.motor->fault_code == MOTOR_FAULT_NONE) ||
             (life->state == MOTOR_STATE_FAULT && clear_fault(life)))
    {
        result = MOTOR_CONTROL_ACCEPTED;
    }
    BspSystem_ExitControlCritical(mask);
    return result;
}

static void check_running_conditions(MotorLifecycle *life, bool configuration_valid)
{
    if (life->drive.motor->fault_code != MOTOR_FAULT_NONE)
    {
        enter_fault(life, (MotorFaultCode)life->drive.motor->fault_code);
    }
    else if ((life->state == MOTOR_STATE_RUNNING || life->enable_pending) &&
             (!configuration_valid || !output_inputs_valid(life) ||
              (life->state == MOTOR_STATE_RUNNING && life->drive.power->phases_disabled())))
    {
        enter_fault(life, MOTOR_FAULT_CONTROL_OVERRUN);
    }
    else if (life->state == MOTOR_STATE_CALIBRATION && !life->calibration_completion_pending &&
             !life->calibration_publication_pending &&
             (!configuration_valid || !calibration_inputs_valid(life) ||
              (calibration_requires_power(life->calibration_type)
                   ? life->power_enable_epoch == 0U && life->drive.power->phases_disabled()
                   : !life->drive.power->phases_disabled())))
    {
        enter_fault(life, MOTOR_FAULT_CONTROL_OVERRUN);
    }
}

static void consume_stop(MotorLifecycle *life)
{
    if (life->stop_pending)
    {
        life->stop_pending = false;
        stop_motor(life, MOTOR_STATE_EVENT_STOP);
    }
}

void MotorLifecycle_SupervisorTick(MotorLifecycle *life)
{
    uint32_t sample_ticks = life->fast_sample_ticks;
    life->fast_samples_fresh = sample_ticks != life->observed_fast_sample_ticks;
    life->observed_fast_sample_ticks = sample_ticks;
    consume_stop(life);
    MotorFault_UpdateRecovery(
        &life->recovery, life->drive.foc, (MotorFaultCode)life->drive.motor->fault_code);
    bool configuration_valid = ParameterValue_ConfigurationValid(life->drive.configuration);
    uint32_t mask = BspSystem_EnterControlCritical();
    finish_calibration(life);
    check_running_conditions(life, configuration_valid);
    ++life->supervisor_ticks;
    BspSystem_ExitControlCritical(mask);
}

void MotorLifecycle_BeginOuter(MotorLifecycle *life, MotorOuterWork *work)
{
    uint32_t mask = BspSystem_EnterControlCritical();
    work->owner = life;
    work->epoch = life->epoch;
    work->motor = *life->drive.motor;
    work->speed_pi = *life->drive.speed_pi;
    work->allowed = (life->state == MOTOR_STATE_RUNNING || life->enable_pending) &&
                    work->motor.control_mode != MOTOR_CONTROL_NONE &&
                    work->motor.fault_code == MOTOR_FAULT_NONE && !life->stop_pending &&
                    !life->starts_locked;
    BspSystem_ExitControlCritical(mask);
}

void MotorLifecycle_CommitOuter(MotorLifecycle *life, const MotorOuterWork *work)
{
    if (work->owner != life)
    {
        return;
    }
    bool configuration_valid = ParameterValue_ConfigurationValid(life->drive.configuration);
    uint32_t mask = BspSystem_EnterControlCritical();
    consume_stop(life);
    bool same =
        work->epoch == life->epoch && work->motor.control_mode == life->drive.motor->control_mode;
    if (work->allowed && same && work->motor.fault_code != MOTOR_FAULT_NONE)
    {
        MotorFault_Report(life->drive.motor, (MotorFaultCode)work->motor.fault_code);
    }
    check_running_conditions(life, configuration_valid);
    bool outer = life->drive.motor->control_mode == MOTOR_CONTROL_SPEED ||
                 life->drive.motor->control_mode == MOTOR_CONTROL_POSITION;
    bool ready = !outer || MotorOuterLoop_IsReady(life->drive.outer);
    bool can_enable = ready && life->state == MOTOR_STATE_IDLE && life->enable_pending &&
                      output_inputs_valid(life);
    /* ADC 可抢占控制临界区。代际、STOP、故障的最终复核和写入必须在同一全屏蔽区。 */
    uint32_t commit = BspSystem_EnterCritical();
    bool valid = work->allowed && work->epoch == life->epoch &&
                 work->motor.control_mode == life->drive.motor->control_mode &&
                 life->drive.motor->fault_code == MOTOR_FAULT_NONE && !life->stop_pending &&
                 !life->starts_locked;
    if (valid && outer && ready)
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
    if (valid && can_enable)
    {
        BspSystem_MemoryBarrier();
        life->power_enable_epoch = work->epoch;
    }
    BspSystem_ExitCritical(commit);
    if (!valid && work->allowed)
    {
        MotorOuterLoop_RequestReset(life->drive.outer);
    }
    BspSystem_ExitControlCritical(mask);
}

uint32_t MotorLifecycle_WorkEpoch(const MotorLifecycle *life)
{
    return life->epoch;
}

MotorState MotorLifecycle_GetState(const MotorLifecycle *life)
{
    return life->state;
}

bool MotorLifecycle_ReadBootHealth(MotorLifecycle *life, uint32_t *progress)
{
    if (progress == NULL)
    {
        return false;
    }
    uint32_t mask = BspSystem_EnterControlCritical();
    *progress = life->supervisor_ticks;
    bool healthy = state_inputs_valid(life) && !life->fault_locked;
    BspSystem_ExitControlCritical(mask);
    return healthy;
}

bool MotorLifecycle_LockStop(MotorLifecycle *life)
{
    uint32_t mask = BspSystem_EnterControlCritical();
    bool accepted = !life->fault_locked;
    if (accepted)
    {
        life->starts_locked = true;
        (void)MotorLifecycle_RequestRemoteStop(life);
    }
    BspSystem_ExitControlCritical(mask);
    return accepted;
}

bool MotorLifecycle_ArePhasesDisabled(const MotorLifecycle *life)
{
    return life->drive.power->phases_disabled();
}

bool MotorLifecycle_IsStopped(const MotorLifecycle *life)
{
    return (life->state == MOTOR_STATE_IDLE || life->state == MOTOR_STATE_FAULT) &&
           !life->stop_pending && !life->enable_pending &&
           life->drive.motor->control_mode == MOTOR_CONTROL_NONE &&
           life->drive.power->phases_disabled();
}

MotorStatus MotorLifecycle_ReadStatus(const MotorLifecycle *life)
{
    uint32_t mask = BspSystem_EnterCritical();
    const MotorControlState *motor = life->drive.motor;
    const FocState *foc = life->drive.foc;
    const EncoderState *encoder = life->drive.rotor_encoder;
    bool active = life->state == MOTOR_STATE_RUNNING && motor->control_mode != MOTOR_CONTROL_NONE;
    bool current = active && motor->control_mode != MOTOR_CONTROL_OPEN_VOLTAGE &&
                   motor->control_mode != MOTOR_CONTROL_VQ;
    bool position = active && motor->control_mode == MOTOR_CONTROL_POSITION;
    bool speed = active && motor->control_mode == MOTOR_CONTROL_SPEED;
    MotorStatus status = {
        .state = life->state,
        .mode = (uint16_t)motor->control_mode,
        .fault = (uint16_t)motor->fault_code,
        .measurements_valid = life->fast_samples_fresh,
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
        .temperature_c = McuTemperature_IsFresh(life->drive.temperature) ? foc->temperature_c : NAN,
        .vq_v = active ? foc->voltage_dq_v.q : NAN,
        .vd_v = active ? foc->voltage_dq_v.d : NAN,
    };
    BspSystem_ExitCritical(mask);
    return status;
}

/* 五态纯迁移；自检仅占位，所有输出和请求资格由APP运行层处理。 */
MotorState MotorStateMachine_Step(MotorState state, MotorStateEvent event)
{
    if ((unsigned)state > MOTOR_STATE_FAULT || (unsigned)event > MOTOR_STATE_EVENT_CLEAR ||
        event == MOTOR_STATE_EVENT_FAULT)
    {
        return MOTOR_STATE_FAULT;
    }
    switch (state)
    {
    case MOTOR_STATE_IDLE:
        if (event == MOTOR_STATE_EVENT_RUN)
        {
            return MOTOR_STATE_RUNNING;
        }
        if (event == MOTOR_STATE_EVENT_CALIBRATE)
        {
            return MOTOR_STATE_CALIBRATION;
        }
        break;
    case MOTOR_STATE_SELF_TEST:
        break;
    case MOTOR_STATE_CALIBRATION:
    case MOTOR_STATE_RUNNING:
        if (event == MOTOR_STATE_EVENT_COMPLETE || event == MOTOR_STATE_EVENT_STOP)
        {
            return MOTOR_STATE_IDLE;
        }
        break;
    case MOTOR_STATE_FAULT:
        if (event == MOTOR_STATE_EVENT_CLEAR)
        {
            return MOTOR_STATE_IDLE;
        }
        break;
    default:
        return MOTOR_STATE_FAULT;
    }
    return state;
}
