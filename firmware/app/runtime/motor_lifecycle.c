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
    uint32_t epoch, supervisor_ticks;
    volatile uint32_t power_enable_epoch, fast_sample_ticks;
    uint32_t observed_fast_sample_ticks;
    volatile bool stop_pending;
    volatile bool calibration_completion_pending, calibration_publication_pending;
    bool enable_pending, fault_locked, fast_samples_fresh, starts_locked;
} MotorLifecycleContext;

/* 保留只读Watch符号，外部不直接读写对象。 */
MotorLifecycleContext g_motor_context;

static bool state_inputs_valid(void)
{
    return g_motor_context.fast_samples_fresh &&
           MotorCurrentOffsets_IsValid(g_motor_context.configuration) &&
           isfinite(g_motor_context.foc->bus_voltage_v) &&
           isfinite(g_motor_context.foc->bus_voltage_filtered_v) &&
           isfinite(g_motor_context.foc->phase_current_a.a) &&
           isfinite(g_motor_context.foc->phase_current_a.b) &&
           isfinite(g_motor_context.foc->phase_current_a.c) &&
           (!BSP_REQUIRED_MCU_TEMPERATURE ||
            (McuTemperature_IsFresh() && isfinite(g_motor_context.foc->temperature_c)));
}

static void clear_running_data(void)
{
    g_motor_context.motor->id_target_a = 0.0f;
    g_motor_context.motor->iq_target_a = 0.0f;
    g_motor_context.motor->vq_target_v = 0.0f;
    g_motor_context.motor->vd_target_v = 0.0f;
    g_motor_context.motor->open_loop_frequency_hz = 0.0f;
    g_motor_context.motor->open_loop_angle_rad = 0.0f;
    g_motor_context.motor->speed_target_rad_s = 0.0f;
    g_motor_context.motor->speed_ramped_rad_s = 0.0f;
    g_motor_context.motor->position_reference_rad = 0.0f;
    g_motor_context.motor->position_target_reached = false;
    g_motor_context.motor->position_velocity_filtered_rad_s = 0.0f;
    g_motor_context.foc->current_dq_a.d = 0.0f;
    g_motor_context.foc->current_dq_a.q = 0.0f;
    Foc_ResetCurrent(g_motor_context.foc);
    PiController_Reset(g_motor_context.speed_pi);
    SensorlessStartup_Reset(g_motor_context.sensorless_startup);
    MotorOuterLoop_RequestReset();
}

static bool state_vbus_in_enable_window(void)
{
    /* 两端均为有限阈值；NaN 与无穷也无法同时满足闭区间比较。 */
    return g_motor_context.foc->bus_voltage_filtered_v >= BUS_VOLTAGE_ENABLE_MIN_V &&
           g_motor_context.foc->bus_voltage_filtered_v <= BUS_VOLTAGE_ENABLE_MAX_V;
}

static bool new_epoch(void)
{
    if (g_motor_context.epoch == UINT32_MAX)
    {
        g_motor_context.fault_locked = true;
        MotorFault_Report(g_motor_context.motor, MOTOR_FAULT_CONTROL_OVERRUN);
        return false;
    }
    ++g_motor_context.epoch;
    return true;
}

static void stop_motor(MotorStateEvent event)
{
    bool calibration_was_active = g_motor_context.calibration_type != MOTOR_CALIBRATION_NONE;
    g_motor_context.power_enable_epoch = 0U;
    g_motor_context.enable_pending = false;
    g_motor_context.calibration_type = MOTOR_CALIBRATION_NONE;
    g_motor_context.calibration_completion_pending = false;
    g_motor_context.calibration_publication_pending = false;
    BspSystem_MemoryBarrier();
    BspPwm_Disable();
    if (calibration_was_active)
    {
        EncoderCalibration_Cancel();
    }
    g_motor_context.motor->control_mode = MOTOR_CONTROL_NONE;
    clear_running_data();
    (void)new_epoch();
    if (!BspPwm_ArePhasesDisabled())
    {
        g_motor_context.fault_locked = true;
        MotorFault_Report(g_motor_context.motor, MOTOR_FAULT_CONTROL_OVERRUN);
    }
    g_motor_context.state = MotorStateMachine_Step(
        g_motor_context.state,
        g_motor_context.motor->fault_code == MOTOR_FAULT_NONE ? event : MOTOR_STATE_EVENT_FAULT);
}

static void enter_fault(MotorFaultCode fault)
{
    MotorFault_Report(g_motor_context.motor,
                      fault == MOTOR_FAULT_NONE ? MOTOR_FAULT_CONTROL_OVERRUN : fault);
    if (g_motor_context.state != MOTOR_STATE_FAULT)
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
    if (g_motor_context.motor->fault_code != MOTOR_FAULT_NONE && !BspPwm_ArePhasesDisabled())
    {
        BspPwm_Disable();
    }
}

static bool output_inputs_valid(void)
{
    MotorControlMode mode = g_motor_context.motor->control_mode;
    bool sensorless =
        mode == MOTOR_CONTROL_CURRENT && g_motor_context.motor->uses_sensorless_feedback;
    return state_inputs_valid() && g_motor_context.configuration->application.axis_profile_valid &&
           g_motor_context.configuration->configuration_confirmed &&
           state_vbus_in_enable_window() &&
           g_motor_context.foc->bus_voltage_v < BUS_VOLTAGE_HARD_OVERVOLTAGE_V &&
           (sensorless || Encoder_IsOnline(g_motor_context.rotor_encoder)) &&
           (sensorless || MotorControlMode_IsOpenLoop(mode) ||
            (Encoder_GetCalibrationFlag(&g_motor_context.configuration->device.encoder) &
             ENC_CALIB_ALL) == ENC_CALIB_ALL);
}

static bool calibration_requires_power(MotorCalibrationType calibration_type)
{
    return calibration_type == MOTOR_CALIBRATION_ENCODER_LINEARIZATION ||
           calibration_type == MOTOR_CALIBRATION_ELECTRICAL_ZERO ||
           calibration_type == MOTOR_CALIBRATION_LINEAR_HALL;
}

static bool calibration_inputs_valid(void)
{
    if (!state_inputs_valid() || !g_motor_context.configuration->application.axis_profile_valid ||
        !Encoder_IsOnline(g_motor_context.rotor_encoder))
    {
        return false;
    }
    if (!calibration_requires_power(g_motor_context.calibration_type))
    {
        return true;
    }
    const MotorConfiguration *config = g_motor_context.configuration;
    if (g_motor_context.calibration_type == MOTOR_CALIBRATION_ENCODER_LINEARIZATION &&
        config->preset != NULL)
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
    return config->configuration_confirmed && state_vbus_in_enable_window() &&
           g_motor_context.foc->bus_voltage_v < BUS_VOLTAGE_HARD_OVERVOLTAGE_V &&
           ((g_motor_context.calibration_type != MOTOR_CALIBRATION_ELECTRICAL_ZERO &&
             g_motor_context.calibration_type != MOTOR_CALIBRATION_LINEAR_HALL) ||
            (Encoder_GetCalibrationFlag(&g_motor_context.configuration->device.encoder) &
             ENC_CALIB_LINEARIZED) != 0U);
}

void MotorLifecycle_Init(const MotorConfiguration *configuration,
                         MotorControlState *motor,
                         FocState *foc,
                         PiController *pi,
                         EncoderState *encoder,
                         SensorlessStartupState *startup)
{
    memset(&g_motor_context, 0, sizeof(g_motor_context));
    g_motor_context.configuration = configuration;
    g_motor_context.motor = motor;
    g_motor_context.foc = foc;
    g_motor_context.speed_pi = pi;
    g_motor_context.rotor_encoder = encoder;
    g_motor_context.sensorless_startup = startup;
    g_motor_context.state = MOTOR_STATE_IDLE;
    g_motor_context.epoch = 2U;
    motor->control_mode = MOTOR_CONTROL_NONE;
}

void MotorLifecycle_BeginFastTick(void)
{
    if (g_motor_context.motor->control_mode == MOTOR_CONTROL_NONE &&
        isfinite(g_motor_context.foc->bus_voltage_v) &&
        g_motor_context.foc->bus_voltage_v >= BUS_VOLTAGE_HARD_OVERVOLTAGE_V)
    {
        MotorFault_Report(g_motor_context.motor, MOTOR_FAULT_OVERVOLTAGE);
    }
    if (!g_motor_context.configuration->application.axis_profile_valid &&
        (!BspPwm_ArePhasesDisabled() || g_motor_context.motor->control_mode != MOTOR_CONTROL_NONE))
    {
        MotorFault_Report(g_motor_context.motor, MOTOR_FAULT_INVALID_PARAMETERS);
    }
    fast_fault_stop();
    if (g_motor_context.stop_pending || g_motor_context.motor->fault_code != MOTOR_FAULT_NONE)
    {
        g_motor_context.power_enable_epoch = 0U;
        if (!BspPwm_ArePhasesDisabled())
        {
            BspPwm_Disable();
        }
        return;
    }
    uint32_t epoch = g_motor_context.power_enable_epoch;
    if (epoch == 0U)
    {
        return;
    }
    g_motor_context.power_enable_epoch = 0U;
    bool outer = g_motor_context.motor->control_mode == MOTOR_CONTROL_SPEED ||
                 g_motor_context.motor->control_mode == MOTOR_CONTROL_POSITION;
    bool starting_control =
        g_motor_context.state == MOTOR_STATE_IDLE && g_motor_context.enable_pending;
    bool starting_calibration = g_motor_context.state == MOTOR_STATE_CALIBRATION &&
                                calibration_requires_power(g_motor_context.calibration_type) &&
                                !g_motor_context.calibration_completion_pending &&
                                !g_motor_context.calibration_publication_pending;
    bool permitted =
        epoch == g_motor_context.epoch && !g_motor_context.starts_locked &&
        ((starting_control && output_inputs_valid() && (!outer || MotorOuterLoop_IsReady())) ||
         (starting_calibration && calibration_inputs_valid()));
    if (permitted)
    {
        BspPwm_Enable();
        permitted = !BspPwm_ArePhasesDisabled();
    }
    if (!permitted)
    {
        MotorFault_Report(g_motor_context.motor, MOTOR_FAULT_CONTROL_OVERRUN);
        fast_fault_stop();
        return;
    }
    if (starting_control)
    {
        g_motor_context.state =
            MotorStateMachine_Step(g_motor_context.state, MOTOR_STATE_EVENT_RUN);
        g_motor_context.enable_pending = false;
    }
}

void MotorLifecycle_PostOutcome(MotorTaskResult outcome, uint32_t epoch)
{
    ++g_motor_context.fast_sample_ticks;
    if (epoch == g_motor_context.epoch && !g_motor_context.stop_pending)
    {
        if (outcome.result == MOTOR_TASK_FAILED)
        {
            MotorFault_Report(g_motor_context.motor,
                              outcome.error == MOTOR_FAULT_NONE ? MOTOR_FAULT_CONTROL_OVERRUN
                                                                : outcome.error);
        }
        else if (outcome.result == MOTOR_TASK_STOP_REQUESTED)
        {
            g_motor_context.stop_pending = true;
            g_motor_context.power_enable_epoch = 0U;
            g_motor_context.enable_pending = false;
            BspSystem_MemoryBarrier();
            g_motor_context.motor->control_mode = MOTOR_CONTROL_NONE;
        }
        else if (g_motor_context.state == MOTOR_STATE_CALIBRATION &&
                 g_motor_context.calibration_type != MOTOR_CALIBRATION_NONE &&
                 (outcome.result == MOTOR_TASK_COMPLETED ||
                  (outcome.result == MOTOR_TASK_FINALIZE &&
                   g_motor_context.calibration_type == MOTOR_CALIBRATION_ENCODER_LINEARIZATION)))
        {
            g_motor_context.power_enable_epoch = 0U;
            BspPwm_Disable();
            BspSystem_MemoryBarrier();
            if (!BspPwm_ArePhasesDisabled())
            {
                MotorFault_Report(g_motor_context.motor, MOTOR_FAULT_CONTROL_OVERRUN);
            }
            else if (outcome.result == MOTOR_TASK_FINALIZE ||
                     (BSP_ENABLED_LINEAR_HALL &&
                      MotorAxisPreset_HasLinearHall(g_motor_context.configuration->preset)))
            {
                g_motor_context.calibration_publication_pending = true;
            }
            else
            {
                g_motor_context.calibration_completion_pending = true;
            }
        }
    }
    fast_fault_stop();
}

bool MotorLifecycle_RequestRemoteStop(void)
{
    uint32_t mask = BspSystem_EnterControlCritical();
    g_motor_context.stop_pending = true;
    g_motor_context.power_enable_epoch = 0U;
    g_motor_context.enable_pending = false;
    BspSystem_MemoryBarrier();
    g_motor_context.motor->control_mode = MOTOR_CONTROL_NONE;
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
    if (g_motor_context.motor->fault_code != MOTOR_FAULT_NONE)
    {
        result = MOTOR_CONTROL_FAULT;
    }
    else if (g_motor_context.enable_pending || g_motor_context.starts_locked)
    {
        result = MOTOR_CONTROL_BUSY;
    }
    else if (g_motor_context.state == MOTOR_STATE_IDLE && !g_motor_context.stop_pending &&
             g_motor_context.motor->control_mode == MOTOR_CONTROL_NONE &&
             g_motor_context.motor->fault_code == MOTOR_FAULT_NONE && BspPwm_ArePhasesDisabled() &&
             !g_motor_context.motor->uses_sensorless_feedback &&
             ParameterValue_ConfigurationValid(g_motor_context.configuration) &&
             state_inputs_valid() && g_motor_context.configuration->configuration_confirmed &&
             Encoder_IsOnline(g_motor_context.rotor_encoder) && state_vbus_in_enable_window() &&
             g_motor_context.foc->bus_voltage_v < BUS_VOLTAGE_HARD_OVERVOLTAGE_V &&
             (MotorControlMode_IsOpenLoop(mode) ||
              (Encoder_GetCalibrationFlag(&g_motor_context.configuration->device.encoder) &
               ENC_CALIB_ALL) == ENC_CALIB_ALL))
    {
        float position_rad = mode == MOTOR_CONTROL_POSITION
                                 ? Encoder_GetMechPos(g_motor_context.rotor_encoder)
                                 : 0.0f;
        bool position_valid = mode != MOTOR_CONTROL_POSITION ||
                              MotorAxisProfile_AllowsPosition(
                                  &g_motor_context.configuration->application.axis_profile,
                                  g_motor_context.configuration->application.axis_profile_valid,
                                  position_rad,
                                  position_rad);
        if (position_valid && new_epoch())
        {
            clear_running_data();
            if (mode == MOTOR_CONTROL_POSITION)
            {
                g_motor_context.motor->position_target_rad =
                    g_motor_context.motor->position_reference_rad = position_rad;
            }
            g_motor_context.motor->control_mode = mode;
            g_motor_context.enable_pending = true;
            result = MOTOR_CONTROL_ACCEPTED;
        }
        else if (g_motor_context.motor->fault_code != MOTOR_FAULT_NONE)
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
    MotorControlState *motor = g_motor_context.motor;
    if (motor->fault_code != MOTOR_FAULT_NONE)
    {
        result = MOTOR_CONTROL_FAULT;
    }
    else if (g_motor_context.state == MOTOR_STATE_RUNNING && !g_motor_context.stop_pending &&
             !g_motor_context.starts_locked && target->mode == motor->control_mode)
    {
        float current_limit = g_motor_context.configuration->application.current_limit_a;
        float voltage_limit = Foc_MaxVoltage(g_motor_context.foc->bus_voltage_filtered_v);
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
            (!speed || (isfinite(g_motor_context.configuration->application.speed_limit_rad_s) &&
                        fabsf(target->speed_rad_s) <=
                            g_motor_context.configuration->application.speed_limit_rad_s)) &&
            (!position || MotorAxisProfile_AllowsPosition(
                              &g_motor_context.configuration->application.axis_profile,
                              g_motor_context.configuration->application.axis_profile_valid,
                              Encoder_GetMechPos(g_motor_context.rotor_encoder),
                              target->position_rad)) &&
            (!open || fabsf(target->frequency_hz) <=
                          MotorConfiguration_OpenLoopLimitHz(g_motor_context.configuration,
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

MotorControlResult MotorLifecycle_RequestCalibration(MotorCalibrationType calibration_type)
{
    if (calibration_type <= MOTOR_CALIBRATION_NONE ||
        calibration_type > MOTOR_CALIBRATION_MECHANICAL_ZERO)
    {
        return MOTOR_CONTROL_INVALID;
    }
    uint32_t mask = BspSystem_EnterControlCritical();
    MotorControlResult result = MOTOR_CONTROL_DENIED;
    if (g_motor_context.motor->fault_code != MOTOR_FAULT_NONE)
    {
        result = MOTOR_CONTROL_FAULT;
    }
    else if (g_motor_context.enable_pending || g_motor_context.stop_pending ||
             g_motor_context.starts_locked || !EncoderCalibration_IsIdle())
    {
        result = MOTOR_CONTROL_BUSY;
    }
    else if (g_motor_context.state == MOTOR_STATE_IDLE && BspPwm_ArePhasesDisabled() &&
             ParameterValue_ConfigurationValid(g_motor_context.configuration))
    {
        g_motor_context.calibration_type = calibration_type;
        bool inputs_valid = calibration_inputs_valid();
        if (inputs_valid && new_epoch())
        {
            clear_running_data();
            g_motor_context.motor->control_mode = MOTOR_CONTROL_NONE;
            g_motor_context.state =
                MotorStateMachine_Step(g_motor_context.state, MOTOR_STATE_EVENT_CALIBRATE);
            if (calibration_requires_power(calibration_type))
            {
                BspSystem_MemoryBarrier();
                g_motor_context.power_enable_epoch = g_motor_context.epoch;
            }
            result = MOTOR_CONTROL_ACCEPTED;
        }
        else if (inputs_valid)
        {
            result = MOTOR_CONTROL_FAULT;
        }
        if (result != MOTOR_CONTROL_ACCEPTED)
        {
            g_motor_context.calibration_type = MOTOR_CALIBRATION_NONE;
        }
    }
    BspSystem_ExitControlCritical(mask);
    return result;
}

MotorCalibrationType MotorLifecycle_GetCalibrationType(void)
{
    return g_motor_context.state == MOTOR_STATE_CALIBRATION ? g_motor_context.calibration_type
                                                            : MOTOR_CALIBRATION_NONE;
}

bool MotorLifecycle_IsCalibrationRunnable(void)
{
    if (g_motor_context.state != MOTOR_STATE_CALIBRATION ||
        g_motor_context.calibration_type == MOTOR_CALIBRATION_NONE ||
        g_motor_context.stop_pending || g_motor_context.motor->fault_code != MOTOR_FAULT_NONE ||
        g_motor_context.calibration_completion_pending ||
        g_motor_context.calibration_publication_pending || g_motor_context.starts_locked)
    {
        return false;
    }
    return calibration_requires_power(g_motor_context.calibration_type)
               ? !BspPwm_ArePhasesDisabled()
               : BspPwm_ArePhasesDisabled();
}

bool MotorLifecycle_ReadPendingCalibrationPublication(uint32_t *calibration_epoch)
{
    if (calibration_epoch == NULL || !g_motor_context.calibration_publication_pending ||
        g_motor_context.state != MOTOR_STATE_CALIBRATION || g_motor_context.stop_pending ||
        g_motor_context.motor->fault_code != MOTOR_FAULT_NONE || !BspPwm_ArePhasesDisabled())
    {
        return false;
    }
    *calibration_epoch = g_motor_context.epoch;
    return true;
}

bool MotorLifecycle_ContinueLinearHall(uint32_t epoch)
{
    uint32_t pending;
    if (!MotorLifecycle_ReadPendingCalibrationPublication(&pending) || epoch != pending ||
        g_motor_context.starts_locked ||
        g_motor_context.calibration_type != MOTOR_CALIBRATION_ENCODER_LINEARIZATION ||
        !MotorAxisPreset_HasLinearHall(g_motor_context.configuration->preset))
    {
        return false;
    }
    g_motor_context.calibration_type = MOTOR_CALIBRATION_LINEAR_HALL;
    g_motor_context.calibration_publication_pending = false;
    BspSystem_MemoryBarrier();
    g_motor_context.power_enable_epoch = epoch;
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
    g_motor_context.calibration_publication_pending = false;
    if (publication_succeeded)
    {
        BspSystem_MemoryBarrier();
        g_motor_context.calibration_completion_pending = true;
    }
    else
    {
        MotorFault_Report(g_motor_context.motor, MOTOR_FAULT_ENCODER);
        fast_fault_stop();
    }
}

static void finish_calibration(void)
{
    if (g_motor_context.state != MOTOR_STATE_CALIBRATION ||
        !g_motor_context.calibration_completion_pending)
    {
        return;
    }
    uint32_t mask = BspSystem_EnterCritical();
    if (!g_motor_context.stop_pending && g_motor_context.motor->fault_code == MOTOR_FAULT_NONE)
    {
        stop_motor(MOTOR_STATE_EVENT_COMPLETE);
    }
    BspSystem_ExitCritical(mask);
}

static bool clear_fault(void)
{
    MotorFaultCode fault = (MotorFaultCode)g_motor_context.motor->fault_code;
    if (g_motor_context.fault_locked || !BspPwm_ArePhasesDisabled() || !state_inputs_valid() ||
        !EncoderCalibration_IsIdle() ||
        !ParameterValue_ConfigurationValid(g_motor_context.configuration))
    {
        return false;
    }
    if (fault != MOTOR_FAULT_NONE && !MotorFault_SourceRecovered(g_motor_context.configuration,
                                                                 &g_motor_context.recovery,
                                                                 g_motor_context.motor,
                                                                 g_motor_context.foc,
                                                                 g_motor_context.rotor_encoder,
                                                                 fault,
                                                                 g_motor_context.fast_samples_fresh,
                                                                 true,
                                                                 McuTemperature_IsFresh()))
    {
        return false;
    }
    uint32_t mask = BspSystem_EnterCritical();
    bool cleared =
        BspSystem_CompareExchange32(&g_motor_context.motor->fault_code, fault, MOTOR_FAULT_NONE);
    if (cleared)
    {
        g_motor_context.state =
            MotorStateMachine_Step(g_motor_context.state, MOTOR_STATE_EVENT_CLEAR);
        g_motor_context.motor->control_mode = MOTOR_CONTROL_NONE;
    }
    BspSystem_ExitCritical(mask);
    return cleared;
}

MotorControlResult MotorLifecycle_RequestClear(void)
{
    uint32_t mask = BspSystem_EnterControlCritical();
    MotorControlResult result = MOTOR_CONTROL_DENIED;
    if (g_motor_context.enable_pending || g_motor_context.stop_pending ||
        g_motor_context.starts_locked)
    {
        result = MOTOR_CONTROL_BUSY;
    }
    else if ((g_motor_context.state == MOTOR_STATE_IDLE &&
              g_motor_context.motor->fault_code == MOTOR_FAULT_NONE) ||
             (g_motor_context.state == MOTOR_STATE_FAULT && clear_fault()))
    {
        result = MOTOR_CONTROL_ACCEPTED;
    }
    BspSystem_ExitControlCritical(mask);
    return result;
}

static void check_running_conditions(bool configuration_valid)
{
    if (g_motor_context.motor->fault_code != MOTOR_FAULT_NONE)
    {
        enter_fault((MotorFaultCode)g_motor_context.motor->fault_code);
    }
    else if ((g_motor_context.state == MOTOR_STATE_RUNNING || g_motor_context.enable_pending) &&
             (!configuration_valid || !output_inputs_valid() ||
              (g_motor_context.state == MOTOR_STATE_RUNNING && BspPwm_ArePhasesDisabled())))
    {
        enter_fault(MOTOR_FAULT_CONTROL_OVERRUN);
    }
    else if (g_motor_context.state == MOTOR_STATE_CALIBRATION &&
             !g_motor_context.calibration_completion_pending &&
             !g_motor_context.calibration_publication_pending &&
             (!configuration_valid || !calibration_inputs_valid() ||
              (calibration_requires_power(g_motor_context.calibration_type)
                   ? g_motor_context.power_enable_epoch == 0U && BspPwm_ArePhasesDisabled()
                   : !BspPwm_ArePhasesDisabled())))
    {
        enter_fault(MOTOR_FAULT_CONTROL_OVERRUN);
    }
}

static void consume_stop(void)
{
    if (g_motor_context.stop_pending)
    {
        g_motor_context.stop_pending = false;
        stop_motor(MOTOR_STATE_EVENT_STOP);
    }
}

void MotorLifecycle_SupervisorTick(void)
{
    uint32_t sample_ticks = g_motor_context.fast_sample_ticks;
    g_motor_context.fast_samples_fresh = sample_ticks != g_motor_context.observed_fast_sample_ticks;
    g_motor_context.observed_fast_sample_ticks = sample_ticks;
    consume_stop();
    MotorFault_UpdateRecovery(&g_motor_context.recovery,
                              g_motor_context.foc,
                              (MotorFaultCode)g_motor_context.motor->fault_code);
    bool configuration_valid = ParameterValue_ConfigurationValid(g_motor_context.configuration);
    uint32_t mask = BspSystem_EnterControlCritical();
    finish_calibration();
    check_running_conditions(configuration_valid);
    ++g_motor_context.supervisor_ticks;
    BspSystem_ExitControlCritical(mask);
}

void MotorLifecycle_OuterTick(void)
{
    bool configuration_valid = ParameterValue_ConfigurationValid(g_motor_context.configuration);
    uint32_t mask = BspSystem_EnterControlCritical();
    uint32_t epoch = g_motor_context.epoch;
    MotorControlState calculated = *g_motor_context.motor;
    PiController speed_pi = *g_motor_context.speed_pi;
    bool allowed =
        (g_motor_context.state == MOTOR_STATE_RUNNING || g_motor_context.enable_pending) &&
        calculated.control_mode != MOTOR_CONTROL_NONE &&
        calculated.fault_code == MOTOR_FAULT_NONE && !g_motor_context.stop_pending &&
        !g_motor_context.starts_locked;
    BspSystem_ExitControlCritical(mask);
    if (allowed)
    {
        MotorOuterLoop_SlowTick(
            g_motor_context.configuration, &calculated, &speed_pi, g_motor_context.rotor_encoder);
    }
    mask = BspSystem_EnterControlCritical();
    bool same = epoch == g_motor_context.epoch &&
                calculated.control_mode == g_motor_context.motor->control_mode;
    if (allowed && same && calculated.fault_code != MOTOR_FAULT_NONE)
    {
        MotorFault_Report(g_motor_context.motor, (MotorFaultCode)calculated.fault_code);
    }
    consume_stop();
    check_running_conditions(
        same ? configuration_valid
             : ParameterValue_ConfigurationValid(g_motor_context.configuration));
    bool valid = allowed && epoch == g_motor_context.epoch && same &&
                 !g_motor_context.stop_pending && !g_motor_context.starts_locked;
    bool outer = g_motor_context.motor->control_mode == MOTOR_CONTROL_SPEED ||
                 g_motor_context.motor->control_mode == MOTOR_CONTROL_POSITION;
    /* ADC仍可抢占监督；只在最后发布目标与PI副本时短暂全屏蔽。 */
    uint32_t commit = BspSystem_EnterCritical();
    valid = valid && g_motor_context.motor->fault_code == MOTOR_FAULT_NONE;
    if (valid && outer && MotorOuterLoop_IsReady())
    {
        g_motor_context.motor->id_target_a = calculated.id_target_a;
        g_motor_context.motor->iq_target_a = calculated.iq_target_a;
        g_motor_context.motor->uses_speed_ramp = calculated.uses_speed_ramp;
        g_motor_context.motor->speed_ramped_rad_s = calculated.speed_ramped_rad_s;
        g_motor_context.motor->position_reference_rad = calculated.position_reference_rad;
        g_motor_context.motor->position_target_reached = calculated.position_target_reached;
        g_motor_context.motor->position_velocity_filtered_rad_s =
            calculated.position_velocity_filtered_rad_s;
        g_motor_context.motor->position_trajectory_speed_rad_s =
            calculated.position_trajectory_speed_rad_s;
        g_motor_context.motor->position_trajectory_acceleration_rad_s2 =
            calculated.position_trajectory_acceleration_rad_s2;
        *g_motor_context.speed_pi = speed_pi;
    }
    BspSystem_ExitCritical(commit);
    if (valid && (!outer || MotorOuterLoop_IsReady()) &&
        g_motor_context.state == MOTOR_STATE_IDLE && g_motor_context.enable_pending &&
        output_inputs_valid())
    {
        BspSystem_MemoryBarrier();
        g_motor_context.power_enable_epoch = g_motor_context.epoch;
    }
    BspSystem_ExitControlCritical(mask);
    if (!valid && allowed)
    {
        MotorOuterLoop_RequestReset();
    }
}

uint32_t MotorLifecycle_WorkEpoch(void)
{
    return g_motor_context.epoch;
}

MotorState MotorLifecycle_GetState(void)
{
    return g_motor_context.state;
}

bool MotorLifecycle_ReadBootHealth(uint32_t *progress)
{
    if (progress == NULL)
    {
        return false;
    }
    uint32_t mask = BspSystem_EnterControlCritical();
    *progress = g_motor_context.supervisor_ticks;
    bool healthy = state_inputs_valid() && !g_motor_context.fault_locked;
    BspSystem_ExitControlCritical(mask);
    return healthy;
}

bool MotorLifecycle_LockStop(void)
{
    uint32_t mask = BspSystem_EnterControlCritical();
    bool accepted = !g_motor_context.fault_locked;
    if (accepted)
    {
        g_motor_context.starts_locked = true;
        (void)MotorLifecycle_RequestRemoteStop();
    }
    BspSystem_ExitControlCritical(mask);
    return accepted;
}

bool MotorLifecycle_IsStopped(void)
{
    return (g_motor_context.state == MOTOR_STATE_IDLE ||
            g_motor_context.state == MOTOR_STATE_FAULT) &&
           !g_motor_context.stop_pending && !g_motor_context.enable_pending &&
           g_motor_context.motor->control_mode == MOTOR_CONTROL_NONE && BspPwm_ArePhasesDisabled();
}

MotorStatus MotorLifecycle_ReadStatus(void)
{
    uint32_t mask = BspSystem_EnterCritical();
    const MotorControlState *motor = g_motor_context.motor;
    const FocState *foc = g_motor_context.foc;
    const EncoderState *encoder = g_motor_context.rotor_encoder;
    bool active =
        g_motor_context.state == MOTOR_STATE_RUNNING && motor->control_mode != MOTOR_CONTROL_NONE;
    bool current = active && motor->control_mode != MOTOR_CONTROL_OPEN_VOLTAGE &&
                   motor->control_mode != MOTOR_CONTROL_VQ;
    bool position = active && motor->control_mode == MOTOR_CONTROL_POSITION;
    bool speed = active && motor->control_mode == MOTOR_CONTROL_SPEED;
    MotorStatus status = {
        .state = g_motor_context.state,
        .mode = (uint16_t)motor->control_mode,
        .fault = (uint16_t)motor->fault_code,
        .measurements_valid = g_motor_context.fast_samples_fresh,
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
        .temperature_c = McuTemperature_IsFresh() ? foc->temperature_c : NAN,
        .vq_v = active ? foc->voltage_dq_v.q : NAN,
        .vd_v = active ? foc->voltage_dq_v.d : NAN,
    };
    BspSystem_ExitCritical(mask);
    return status;
}
