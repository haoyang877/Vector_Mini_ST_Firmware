#include "motor_lifecycle.h"

#include <math.h>
#include <string.h>
#include "application_config.h"
#include "axis_config.h"
#include "bsp_pwm.h"
#include "bsp_system.h"
#include "encoder_calibration.h"
#include "motor_control.h"
#include "motor_fault.h"
#include "motor_sensing.h"
#include "motor_state_machine.h"
#include "parameter_values.h"

/* 拥有启停、标定、目标提交和功率安全；无目标邮箱或协议请求身份。 */
typedef struct
{
    MotorState state;
    const MotorConfiguration *configuration;
    MotorControlState *motor;
    FocState *foc;
    PiController *speed_pi;
    EncoderState *rotor_encoder;
    SensorlessStartupState *sensorless_startup;
    MotorFaultRecovery recovery;
    MotorCalibrationType calibration_type;
    uint32_t epoch, supervisor_epoch;
    volatile uint32_t power_enable_epoch, fast_sample_epoch;
    uint32_t observed_fast_sample_epoch;
    volatile bool stop_pending;
    volatile bool calibration_completion_pending, calibration_publication_pending;
    bool enable_pending, fault_locked, fast_samples_fresh, starts_locked;
} MotorLifecycleContext;

/* 保留只读Watch符号，外部不直接读写对象。 */
MotorLifecycleContext s_context;

static bool state_inputs_valid(void)
{
    return s_context.fast_samples_fresh && MotorCurrentOffsets_IsValid(s_context.configuration) &&
           isfinite(s_context.foc->bus_voltage_v) &&
           isfinite(s_context.foc->bus_voltage_filtered_v) &&
           isfinite(s_context.foc->phase_current_a.a) &&
           isfinite(s_context.foc->phase_current_a.b) &&
           isfinite(s_context.foc->phase_current_a.c) &&
           (!BSP_REQUIRED_MCU_TEMPERATURE ||
            (McuTemperature_IsFresh() && isfinite(s_context.foc->temperature_c)));
}

static void clear_running_data(void)
{
    s_context.motor->id_target_a = 0.0f;
    s_context.motor->iq_target_a = 0.0f;
    s_context.motor->vq_target_v = 0.0f;
    s_context.motor->vd_target_v = 0.0f;
    s_context.motor->open_loop_frequency_hz = 0.0f;
    s_context.motor->open_loop_angle_rad = 0.0f;
    s_context.motor->speed_target_rad_s = 0.0f;
    s_context.motor->speed_ramped_rad_s = 0.0f;
    s_context.motor->position_reference_rad = 0.0f;
    s_context.motor->position_target_reached = false;
    s_context.motor->position_velocity_filtered_rad_s = 0.0f;
    s_context.foc->current_dq_a.d = 0.0f;
    s_context.foc->current_dq_a.q = 0.0f;
    Foc_ResetCurrent(s_context.foc);
    PiController_Reset(s_context.speed_pi);
    SensorlessStartup_Reset(s_context.sensorless_startup);
    MotorOuterLoop_RequestReset();
}

static bool state_vbus_in_enable_window(void)
{
    /* 两端均为有限阈值；NaN 与无穷也无法同时满足闭区间比较。 */
    return s_context.foc->bus_voltage_filtered_v >= BUS_VOLTAGE_ENABLE_MIN_V &&
           s_context.foc->bus_voltage_filtered_v <= BUS_VOLTAGE_ENABLE_MAX_V;
}

static bool new_epoch(void)
{
    if (s_context.epoch == UINT32_MAX)
    {
        s_context.fault_locked = true;
        MotorFault_Report(s_context.motor, MOTOR_FAULT_CONTROL_OVERRUN);
        return false;
    }
    ++s_context.epoch;
    return true;
}

static void stop_motor(MotorStateEvent event)
{
    bool calibration_was_active = s_context.calibration_type != MOTOR_CALIBRATION_NONE;
    s_context.power_enable_epoch = 0U;
    s_context.enable_pending = false;
    s_context.calibration_type = MOTOR_CALIBRATION_NONE;
    s_context.calibration_completion_pending = false;
    s_context.calibration_publication_pending = false;
    BspSystem_MemoryBarrier();
    BspPwm_Disable();
    if (calibration_was_active)
    {
        EncoderCalibrationTask_Cancel();
    }
    s_context.motor->control_mode = MOTOR_CONTROL_NONE;
    clear_running_data();
    (void)new_epoch();
    if (!BspPwm_ArePhasesDisabled())
    {
        s_context.fault_locked = true;
        MotorFault_Report(s_context.motor, MOTOR_FAULT_CONTROL_OVERRUN);
    }
    s_context.state = MotorStateMachine_Step(
        s_context.state,
        s_context.motor->fault_code == MOTOR_FAULT_NONE ? event : MOTOR_STATE_EVENT_FAULT);
}

static void enter_fault(MotorFaultCode fault)
{
    MotorFault_Report(s_context.motor,
                      fault == MOTOR_FAULT_NONE ? MOTOR_FAULT_CONTROL_OVERRUN : fault);
    if (s_context.state != MOTOR_STATE_FAULT)
    {
        stop_motor(MOTOR_STATE_EVENT_FAULT);
    }
    else if (!BspPwm_ArePhasesDisabled())
    {
        BspPwm_Disable();
    }
}

static void fast_fault_stop(void)
{
    if (s_context.motor->fault_code != MOTOR_FAULT_NONE && !BspPwm_ArePhasesDisabled())
    {
        BspPwm_Disable();
    }
}

static bool output_inputs_valid(void)
{
    MotorControlMode mode = s_context.motor->control_mode;
    bool sensorless = mode == MOTOR_CONTROL_CURRENT && s_context.motor->uses_sensorless_feedback;
    return state_inputs_valid() && s_context.configuration->application.axis_profile_valid &&
           s_context.configuration->configuration_confirmed && state_vbus_in_enable_window() &&
           s_context.foc->bus_voltage_v < BUS_VOLTAGE_HARD_OVERVOLTAGE_V &&
           (sensorless || Encoder_IsOnline(s_context.rotor_encoder)) &&
           (sensorless || MotorControlMode_IsOpenLoop(mode) ||
            (Encoder_GetCalibrationFlag(&s_context.configuration->device.encoder) &
             ENC_CALIB_ALL) == ENC_CALIB_ALL);
}

static bool calibration_requires_power(MotorCalibrationType calibration_type)
{
    return calibration_type == MOTOR_CALIBRATION_ENCODER_LINEARIZATION ||
           calibration_type == MOTOR_CALIBRATION_ELECTRICAL_ZERO;
}

static bool calibration_inputs_valid(void)
{
    if (!state_inputs_valid() || !s_context.configuration->application.axis_profile_valid ||
        !Encoder_IsOnline(s_context.rotor_encoder))
    {
        return false;
    }
    if (!calibration_requires_power(s_context.calibration_type))
    {
        return true;
    }
    return s_context.configuration->configuration_confirmed && state_vbus_in_enable_window() &&
           s_context.foc->bus_voltage_v < BUS_VOLTAGE_HARD_OVERVOLTAGE_V &&
           (s_context.calibration_type != MOTOR_CALIBRATION_ELECTRICAL_ZERO ||
            (Encoder_GetCalibrationFlag(&s_context.configuration->device.encoder) &
             ENC_CALIB_LINEARIZED) != 0U);
}

void MotorLifecycle_Init(const MotorConfiguration *configuration,
                         MotorControlState *motor,
                         FocState *foc,
                         PiController *pi,
                         EncoderState *encoder,
                         SensorlessStartupState *startup)
{
    memset(&s_context, 0, sizeof(s_context));
    s_context.configuration = configuration;
    s_context.motor = motor;
    s_context.foc = foc;
    s_context.speed_pi = pi;
    s_context.rotor_encoder = encoder;
    s_context.sensorless_startup = startup;
    s_context.state = MOTOR_STATE_IDLE;
    s_context.epoch = 2U;
    motor->control_mode = MOTOR_CONTROL_NONE;
}

void MotorLifecycle_BeginFastTick(void)
{
    if (s_context.motor->control_mode == MOTOR_CONTROL_NONE &&
        isfinite(s_context.foc->bus_voltage_v) &&
        s_context.foc->bus_voltage_v >= BUS_VOLTAGE_HARD_OVERVOLTAGE_V)
    {
        MotorFault_Report(s_context.motor, MOTOR_FAULT_OVERVOLTAGE);
    }
    if (!s_context.configuration->application.axis_profile_valid &&
        (!BspPwm_ArePhasesDisabled() || s_context.motor->control_mode != MOTOR_CONTROL_NONE))
    {
        MotorFault_Report(s_context.motor, MOTOR_FAULT_INVALID_PARAMETERS);
    }
    fast_fault_stop();
    if (s_context.stop_pending || s_context.motor->fault_code != MOTOR_FAULT_NONE)
    {
        s_context.power_enable_epoch = 0U;
        if (!BspPwm_ArePhasesDisabled())
        {
            BspPwm_Disable();
        }
        return;
    }
    uint32_t epoch = s_context.power_enable_epoch;
    if (epoch == 0U)
    {
        return;
    }
    s_context.power_enable_epoch = 0U;
    bool outer = s_context.motor->control_mode == MOTOR_CONTROL_SPEED ||
                 s_context.motor->control_mode == MOTOR_CONTROL_POSITION;
    bool starting_control = s_context.state == MOTOR_STATE_IDLE && s_context.enable_pending;
    bool starting_calibration = s_context.state == MOTOR_STATE_CALIBRATION &&
                                calibration_requires_power(s_context.calibration_type) &&
                                !s_context.calibration_completion_pending &&
                                !s_context.calibration_publication_pending;
    bool permitted =
        epoch == s_context.epoch && !s_context.starts_locked &&
        ((starting_control && output_inputs_valid() && (!outer || MotorOuterLoop_IsReady())) ||
         (starting_calibration && calibration_inputs_valid()));
    if (permitted)
    {
        BspPwm_Enable();
        permitted = !BspPwm_ArePhasesDisabled();
    }
    if (!permitted)
    {
        MotorFault_Report(s_context.motor, MOTOR_FAULT_CONTROL_OVERRUN);
        fast_fault_stop();
        return;
    }
    if (starting_control)
    {
        s_context.state = MotorStateMachine_Step(s_context.state, MOTOR_STATE_EVENT_RUN);
        s_context.enable_pending = false;
    }
}

void MotorLifecycle_PostOutcome(MotorTaskResult outcome, uint32_t epoch)
{
    ++s_context.fast_sample_epoch;
    if (epoch == s_context.epoch && !s_context.stop_pending)
    {
        if (outcome.result == MOTOR_TASK_FAILED)
        {
            MotorFault_Report(s_context.motor,
                              outcome.error == MOTOR_FAULT_NONE ? MOTOR_FAULT_CONTROL_OVERRUN
                                                                : outcome.error);
        }
        else if (outcome.result == MOTOR_TASK_STOP_REQUESTED)
        {
            s_context.stop_pending = true;
            s_context.power_enable_epoch = 0U;
            s_context.enable_pending = false;
            BspSystem_MemoryBarrier();
            s_context.motor->control_mode = MOTOR_CONTROL_NONE;
        }
        else if (s_context.state == MOTOR_STATE_CALIBRATION &&
                 s_context.calibration_type != MOTOR_CALIBRATION_NONE &&
                 (outcome.result == MOTOR_TASK_COMPLETED ||
                  (outcome.result == MOTOR_TASK_FINALIZE &&
                   s_context.calibration_type == MOTOR_CALIBRATION_ENCODER_LINEARIZATION)))
        {
            s_context.power_enable_epoch = 0U;
            BspPwm_Disable();
            BspSystem_MemoryBarrier();
            if (!BspPwm_ArePhasesDisabled())
            {
                MotorFault_Report(s_context.motor, MOTOR_FAULT_CONTROL_OVERRUN);
            }
            else if (outcome.result == MOTOR_TASK_FINALIZE)
            {
                s_context.calibration_publication_pending = true;
            }
            else
            {
                s_context.calibration_completion_pending = true;
            }
        }
    }
    fast_fault_stop();
}

bool MotorLifecycle_RequestRemoteStop(void)
{
    uint32_t mask = BspSystem_EnterControlCritical();
    s_context.stop_pending = true;
    s_context.power_enable_epoch = 0U;
    s_context.enable_pending = false;
    BspSystem_MemoryBarrier();
    s_context.motor->control_mode = MOTOR_CONTROL_NONE;
    BspSystem_ExitControlCritical(mask);
    return true;
}

MotorControlResult MotorLifecycle_RequestEnable(MotorControlMode mode)
{
    if (!MotorControlMode_IsValid(mode))
    {
        return MOTOR_CONTROL_INVALID;
    }
    uint32_t mask = BspSystem_EnterControlCritical();
    MotorControlResult result = MOTOR_CONTROL_DENIED;
    if (s_context.motor->fault_code != MOTOR_FAULT_NONE)
    {
        result = MOTOR_CONTROL_FAULT;
    }
    else if (s_context.enable_pending || s_context.starts_locked)
    {
        result = MOTOR_CONTROL_BUSY;
    }
    else if (s_context.state == MOTOR_STATE_IDLE && !s_context.stop_pending &&
             s_context.motor->control_mode == MOTOR_CONTROL_NONE &&
             s_context.motor->fault_code == MOTOR_FAULT_NONE && BspPwm_ArePhasesDisabled() &&
             !s_context.motor->uses_sensorless_feedback &&
             ParameterValue_ConfigurationValid(s_context.configuration) && state_inputs_valid() &&
             s_context.configuration->configuration_confirmed &&
             Encoder_IsOnline(s_context.rotor_encoder) && state_vbus_in_enable_window() &&
             s_context.foc->bus_voltage_v < BUS_VOLTAGE_HARD_OVERVOLTAGE_V &&
             (MotorControlMode_IsOpenLoop(mode) ||
              (Encoder_GetCalibrationFlag(&s_context.configuration->device.encoder) &
               ENC_CALIB_ALL) == ENC_CALIB_ALL))
    {
        float position_rad =
            mode == MOTOR_CONTROL_POSITION ? Encoder_GetMechPos(s_context.rotor_encoder) : 0.0f;
        bool position_valid =
            mode != MOTOR_CONTROL_POSITION ||
            MotorAxisProfile_AllowsPosition(&s_context.configuration->application.axis_profile,
                                            s_context.configuration->application.axis_profile_valid,
                                            position_rad,
                                            position_rad);
        if (position_valid && new_epoch())
        {
            clear_running_data();
            if (mode == MOTOR_CONTROL_POSITION)
            {
                s_context.motor->position_target_rad = s_context.motor->position_reference_rad =
                    position_rad;
            }
            s_context.motor->control_mode = mode;
            s_context.enable_pending = true;
            result = MOTOR_CONTROL_ACCEPTED;
        }
        else if (s_context.motor->fault_code != MOTOR_FAULT_NONE)
        {
            result = MOTOR_CONTROL_FAULT;
        }
    }
    BspSystem_ExitControlCritical(mask);
    return result;
}

MotorControlResult MotorLifecycle_SetTarget(const MotorTarget *target)
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
    MotorControlState *motor = s_context.motor;
    if (motor->fault_code != MOTOR_FAULT_NONE)
    {
        result = MOTOR_CONTROL_FAULT;
    }
    else if (s_context.state == MOTOR_STATE_RUNNING && !s_context.stop_pending &&
             !s_context.starts_locked && target->mode == motor->control_mode)
    {
        float current_limit = s_context.configuration->application.current_limit_a;
        float voltage_limit = Foc_MaxVoltage(s_context.foc->bus_voltage_filtered_v);
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
        bool range = (!current || (isfinite(current_limit) && current_limit > 0.0f &&
                                   target->id_a * target->id_a + target->iq_a * target->iq_a <=
                                       current_limit * current_limit)) &&
                     (!voltage || (isfinite(voltage_limit) && voltage_limit > 0.0f &&
                                   target->vd_v * target->vd_v + target->vq_v * target->vq_v <=
                                       voltage_limit * voltage_limit)) &&
                     (!speed || (isfinite(s_context.configuration->application.speed_limit_rad_s) &&
                                 fabsf(target->speed_rad_s) <=
                                     s_context.configuration->application.speed_limit_rad_s)) &&
                     (!position || MotorAxisProfile_AllowsPosition(
                                       &s_context.configuration->application.axis_profile,
                                       s_context.configuration->application.axis_profile_valid,
                                       Encoder_GetMechPos(s_context.rotor_encoder),
                                       target->position_rad)) &&
                     (!open || fabsf(target->frequency_hz) <= MOTOR_OPEN_LOOP_MAX_ELECTRICAL_HZ);
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

MotorControlResult MotorLifecycle_RequestCalibration(MotorCalibrationType calibration_type)
{
    if (calibration_type <= MOTOR_CALIBRATION_NONE ||
        calibration_type > MOTOR_CALIBRATION_MECHANICAL_ZERO)
    {
        return MOTOR_CONTROL_INVALID;
    }
    uint32_t mask = BspSystem_EnterControlCritical();
    MotorControlResult result = MOTOR_CONTROL_DENIED;
    if (s_context.motor->fault_code != MOTOR_FAULT_NONE)
    {
        result = MOTOR_CONTROL_FAULT;
    }
    else if (s_context.enable_pending || s_context.stop_pending || s_context.starts_locked ||
             !EncoderCalibrationTask_IsReleased())
    {
        result = MOTOR_CONTROL_BUSY;
    }
    else if (s_context.state == MOTOR_STATE_IDLE && BspPwm_ArePhasesDisabled() &&
             ParameterValue_ConfigurationValid(s_context.configuration))
    {
        s_context.calibration_type = calibration_type;
        bool inputs_valid = calibration_inputs_valid();
        if (inputs_valid && new_epoch())
        {
            clear_running_data();
            s_context.motor->control_mode = MOTOR_CONTROL_NONE;
            s_context.state = MotorStateMachine_Step(s_context.state, MOTOR_STATE_EVENT_CALIBRATE);
            if (calibration_requires_power(calibration_type))
            {
                BspSystem_MemoryBarrier();
                s_context.power_enable_epoch = s_context.epoch;
            }
            result = MOTOR_CONTROL_ACCEPTED;
        }
        else if (inputs_valid)
        {
            result = MOTOR_CONTROL_FAULT;
        }
        if (result != MOTOR_CONTROL_ACCEPTED)
        {
            s_context.calibration_type = MOTOR_CALIBRATION_NONE;
        }
    }
    BspSystem_ExitControlCritical(mask);
    return result;
}

MotorCalibrationType MotorLifecycle_GetCalibrationType(void)
{
    return s_context.state == MOTOR_STATE_CALIBRATION ? s_context.calibration_type
                                                      : MOTOR_CALIBRATION_NONE;
}

bool MotorLifecycle_IsCalibrationRunnable(void)
{
    if (s_context.state != MOTOR_STATE_CALIBRATION ||
        s_context.calibration_type == MOTOR_CALIBRATION_NONE || s_context.stop_pending ||
        s_context.motor->fault_code != MOTOR_FAULT_NONE ||
        s_context.calibration_completion_pending || s_context.calibration_publication_pending ||
        s_context.starts_locked)
    {
        return false;
    }
    return calibration_requires_power(s_context.calibration_type) ? !BspPwm_ArePhasesDisabled()
                                                                  : BspPwm_ArePhasesDisabled();
}

bool MotorLifecycle_ReadPendingCalibrationPublication(uint32_t *calibration_epoch)
{
    if (calibration_epoch == NULL || !s_context.calibration_publication_pending ||
        s_context.state != MOTOR_STATE_CALIBRATION || s_context.stop_pending ||
        s_context.motor->fault_code != MOTOR_FAULT_NONE || !BspPwm_ArePhasesDisabled())
    {
        return false;
    }
    *calibration_epoch = s_context.epoch;
    return true;
}

void MotorLifecycle_CompleteCalibrationPublication(uint32_t calibration_epoch,
                                                   bool publication_succeeded)
{
    uint32_t current_calibration_epoch;
    if (!MotorLifecycle_ReadPendingCalibrationPublication(&current_calibration_epoch) ||
        current_calibration_epoch != calibration_epoch)
    {
        return;
    }
    s_context.calibration_publication_pending = false;
    if (publication_succeeded)
    {
        BspSystem_MemoryBarrier();
        s_context.calibration_completion_pending = true;
    }
    else
    {
        MotorFault_Report(s_context.motor, MOTOR_FAULT_ENCODER);
        fast_fault_stop();
    }
}

static void finish_calibration(void)
{
    if (s_context.state != MOTOR_STATE_CALIBRATION || !s_context.calibration_completion_pending)
    {
        return;
    }
    uint32_t mask = BspSystem_EnterCritical();
    if (!s_context.stop_pending && s_context.motor->fault_code == MOTOR_FAULT_NONE)
    {
        stop_motor(MOTOR_STATE_EVENT_COMPLETE);
    }
    BspSystem_ExitCritical(mask);
}

static bool clear_fault(void)
{
    MotorFaultCode fault = (MotorFaultCode)s_context.motor->fault_code;
    if (s_context.fault_locked || !BspPwm_ArePhasesDisabled() || !state_inputs_valid() ||
        !EncoderCalibrationTask_IsReleased() ||
        !ParameterValue_ConfigurationValid(s_context.configuration))
    {
        return false;
    }
    if (fault != MOTOR_FAULT_NONE && !MotorFault_SourceRecovered(s_context.configuration,
                                                                 &s_context.recovery,
                                                                 s_context.motor,
                                                                 s_context.foc,
                                                                 s_context.rotor_encoder,
                                                                 fault,
                                                                 s_context.fast_samples_fresh,
                                                                 true,
                                                                 McuTemperature_IsFresh()))
    {
        return false;
    }
    uint32_t mask = BspSystem_EnterCritical();
    bool cleared =
        BspSystem_CompareExchange32(&s_context.motor->fault_code, fault, MOTOR_FAULT_NONE);
    if (cleared)
    {
        s_context.state = MotorStateMachine_Step(s_context.state, MOTOR_STATE_EVENT_CLEAR);
        s_context.motor->control_mode = MOTOR_CONTROL_NONE;
    }
    BspSystem_ExitCritical(mask);
    return cleared;
}

MotorControlResult MotorLifecycle_RequestClear(void)
{
    uint32_t mask = BspSystem_EnterControlCritical();
    MotorControlResult result = MOTOR_CONTROL_DENIED;
    if (s_context.enable_pending || s_context.stop_pending || s_context.starts_locked)
    {
        result = MOTOR_CONTROL_BUSY;
    }
    else if ((s_context.state == MOTOR_STATE_IDLE &&
              s_context.motor->fault_code == MOTOR_FAULT_NONE) ||
             (s_context.state == MOTOR_STATE_FAULT && clear_fault()))
    {
        result = MOTOR_CONTROL_ACCEPTED;
    }
    BspSystem_ExitControlCritical(mask);
    return result;
}

static void check_running_conditions(bool configuration_valid)
{
    if (s_context.motor->fault_code != MOTOR_FAULT_NONE)
    {
        enter_fault((MotorFaultCode)s_context.motor->fault_code);
    }
    else if ((s_context.state == MOTOR_STATE_RUNNING || s_context.enable_pending) &&
             (!configuration_valid || !output_inputs_valid() ||
              (s_context.state == MOTOR_STATE_RUNNING && BspPwm_ArePhasesDisabled())))
    {
        enter_fault(MOTOR_FAULT_CONTROL_OVERRUN);
    }
    else if (s_context.state == MOTOR_STATE_CALIBRATION &&
             !s_context.calibration_completion_pending &&
             !s_context.calibration_publication_pending &&
             (!configuration_valid || !calibration_inputs_valid() ||
              (calibration_requires_power(s_context.calibration_type)
                   ? s_context.power_enable_epoch == 0U && BspPwm_ArePhasesDisabled()
                   : !BspPwm_ArePhasesDisabled())))
    {
        enter_fault(MOTOR_FAULT_CONTROL_OVERRUN);
    }
}

static void consume_stop(void)
{
    if (s_context.stop_pending)
    {
        s_context.stop_pending = false;
        stop_motor(MOTOR_STATE_EVENT_STOP);
    }
}

void MotorLifecycle_SupervisorTick(void)
{
    uint32_t sample_epoch = s_context.fast_sample_epoch;
    s_context.fast_samples_fresh = sample_epoch != s_context.observed_fast_sample_epoch;
    s_context.observed_fast_sample_epoch = sample_epoch;
    consume_stop();
    MotorFault_UpdateRecovery(
        &s_context.recovery, s_context.foc, (MotorFaultCode)s_context.motor->fault_code);
    bool configuration_valid = ParameterValue_ConfigurationValid(s_context.configuration);
    uint32_t mask = BspSystem_EnterControlCritical();
    finish_calibration();
    check_running_conditions(configuration_valid);
    uint32_t epoch = s_context.epoch;
    MotorControlState calculated = *s_context.motor;
    PiController speed_pi = *s_context.speed_pi;
    bool allowed = (s_context.state == MOTOR_STATE_RUNNING || s_context.enable_pending) &&
                   calculated.control_mode != MOTOR_CONTROL_NONE &&
                   calculated.fault_code == MOTOR_FAULT_NONE && !s_context.stop_pending &&
                   !s_context.starts_locked;
    BspSystem_ExitControlCritical(mask);
    if (allowed)
    {
        MotorOuterLoop_SlowTick(
            s_context.configuration, &calculated, &speed_pi, s_context.rotor_encoder);
    }
    mask = BspSystem_EnterControlCritical();
    bool same =
        epoch == s_context.epoch && calculated.control_mode == s_context.motor->control_mode;
    if (allowed && same && calculated.fault_code != MOTOR_FAULT_NONE)
    {
        MotorFault_Report(s_context.motor, (MotorFaultCode)calculated.fault_code);
    }
    consume_stop();
    check_running_conditions(same ? configuration_valid
                                  : ParameterValue_ConfigurationValid(s_context.configuration));
    bool valid = allowed && epoch == s_context.epoch && same && !s_context.stop_pending &&
                 !s_context.starts_locked;
    bool outer = s_context.motor->control_mode == MOTOR_CONTROL_SPEED ||
                 s_context.motor->control_mode == MOTOR_CONTROL_POSITION;
    /* ADC仍可抢占监督；只在最后发布目标与PI副本时短暂全屏蔽。 */
    uint32_t commit = BspSystem_EnterCritical();
    valid = valid && s_context.motor->fault_code == MOTOR_FAULT_NONE;
    if (valid && outer && MotorOuterLoop_IsReady())
    {
        s_context.motor->id_target_a = calculated.id_target_a;
        s_context.motor->iq_target_a = calculated.iq_target_a;
        s_context.motor->uses_speed_ramp = calculated.uses_speed_ramp;
        s_context.motor->speed_ramped_rad_s = calculated.speed_ramped_rad_s;
        s_context.motor->position_reference_rad = calculated.position_reference_rad;
        s_context.motor->position_target_reached = calculated.position_target_reached;
        s_context.motor->position_velocity_filtered_rad_s =
            calculated.position_velocity_filtered_rad_s;
        s_context.motor->position_trajectory_speed_rad_s =
            calculated.position_trajectory_speed_rad_s;
        *s_context.speed_pi = speed_pi;
    }
    ++s_context.supervisor_epoch;
    BspSystem_ExitCritical(commit);
    if (valid && (!outer || MotorOuterLoop_IsReady()) && s_context.state == MOTOR_STATE_IDLE &&
        s_context.enable_pending && output_inputs_valid())
    {
        BspSystem_MemoryBarrier();
        s_context.power_enable_epoch = s_context.epoch;
    }
    BspSystem_ExitControlCritical(mask);
    if (!valid && allowed)
    {
        MotorOuterLoop_RequestReset();
    }
}

uint32_t MotorLifecycle_WorkEpoch(void)
{
    return s_context.epoch;
}

MotorState MotorLifecycle_GetState(void)
{
    return s_context.state;
}

bool MotorLifecycle_ReadBootHealth(uint32_t *progress)
{
    if (progress == NULL)
    {
        return false;
    }
    uint32_t mask = BspSystem_EnterControlCritical();
    *progress = s_context.supervisor_epoch;
    bool healthy = state_inputs_valid() && !s_context.fault_locked;
    BspSystem_ExitControlCritical(mask);
    return healthy;
}

bool MotorLifecycle_LockStop(void)
{
    uint32_t mask = BspSystem_EnterControlCritical();
    bool accepted = !s_context.fault_locked;
    if (accepted)
    {
        s_context.starts_locked = true;
        (void)MotorLifecycle_RequestRemoteStop();
    }
    BspSystem_ExitControlCritical(mask);
    return accepted;
}

bool MotorLifecycle_IsStopped(void)
{
    return (s_context.state == MOTOR_STATE_IDLE || s_context.state == MOTOR_STATE_FAULT) &&
           !s_context.stop_pending && !s_context.enable_pending &&
           s_context.motor->control_mode == MOTOR_CONTROL_NONE && BspPwm_ArePhasesDisabled();
}

MotorStatus MotorLifecycle_ReadStatus(void)
{
    uint32_t mask = BspSystem_EnterCritical();
    const MotorControlState *motor = s_context.motor;
    const FocState *foc = s_context.foc;
    const EncoderState *encoder = s_context.rotor_encoder;
    bool active =
        s_context.state == MOTOR_STATE_RUNNING && motor->control_mode != MOTOR_CONTROL_NONE;
    bool current = active && motor->control_mode != MOTOR_CONTROL_OPEN_VOLTAGE &&
                   motor->control_mode != MOTOR_CONTROL_VQ;
    bool position = active && motor->control_mode == MOTOR_CONTROL_POSITION;
    bool speed = active && motor->control_mode == MOTOR_CONTROL_SPEED;
    MotorStatus status = {
        .state = s_context.state,
        .mode = (uint16_t)motor->control_mode,
        .fault = (uint16_t)motor->fault_code,
        .measurements_valid = s_context.fast_samples_fresh,
        .position_reference_rad = position ? motor->position_target_rad : NAN,
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
        .temperature_c = McuTemperature_IsFresh() ? foc->temperature_c : NAN,
        .vq_v = active ? foc->voltage_dq_v.q : NAN,
        .vd_v = active ? foc->voltage_dq_v.d : NAN,
    };
    BspSystem_ExitCritical(mask);
    return status;
}
