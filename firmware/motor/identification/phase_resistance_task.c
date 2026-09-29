/* 相电阻辨识适配器：把核心模块的 id_a/iq_a/电角度命令交给 FOC_Current() 执行，
 * 并负责停机时的 PWM 释放与电流环复位；PWM 输出统一走 motor_hw 契约。 */
#include "phase_resistance_task.h"

#include <math.h>

#include "control_config.h"
#include "motor_hw.h"
#include "bus_voltage_profile.h"

#define PHASE_RESISTANCE_VBUS_MIN BUS_VOLTAGE_UNDERVOLTAGE_V
#define PHASE_RESISTANCE_VBUS_MAX BUS_VOLTAGE_OVERVOLTAGE_V

typedef struct
{
    PhaseResistanceContext core;
    PhaseResistanceModeStatus status;
    float applied_mod_d;
    float applied_mod_q;
    PhaseResistanceModeTelemetry telemetry;
    bool applied_voltage_valid;
    bool telemetry_valid;
    bool started;
} PhaseResistanceTaskContext;

static PhaseResistanceTaskContext s_phase_resistance_mode_context;

static float phase_resistance_mode_min(float first, float second)
{
    return first < second ? first : second;
}

/* PWM 释放与电流环复位由适配器负责，核心模块不做。 */
static void phase_resistance_mode_stop_output(FocState *foc, MotorControlState *motor)
{
    motor->id_target_a = 0.0f;
    motor->iq_target_a = 0.0f;
    FOC_CurrentController_Reset(foc);
    motor_hw_pwm_force_high_sides();
    s_phase_resistance_mode_context.applied_mod_d = 0.0f;
    s_phase_resistance_mode_context.applied_mod_q = 0.0f;
    s_phase_resistance_mode_context.applied_voltage_valid = false;
    s_phase_resistance_mode_context.telemetry_valid = false;
}

static void phase_resistance_mode_clear_result(MotorControlState *motor)
{
    uint8_t index;

    for (index = 0U; index < PHASE_RESISTANCE_VECTOR_COUNT; index++)
    {
        motor->phase_resistance_vector_ohm[index] = 0.0f;
    }
    motor->phase_resistance_a_ohm = 0.0f;
    motor->phase_resistance_b_ohm = 0.0f;
    motor->phase_resistance_c_ohm = 0.0f;
    motor->phase_resistance_spread_pct = 0.0f;
    motor->phase_resistance_valid = false;
    motor->phase_resistance_warning = false;
    motor->phase_resistance_balanced = false;
}

static float phase_resistance_mode_limit_current(const MotorControlState *motor,
                                                 float requested_current,
                                                 float configured_maximum_a)
{
    float maximum_current = configured_maximum_a;

    maximum_current =
        phase_resistance_mode_min(maximum_current, motor->application.current_limit_a);
    return phase_resistance_mode_min(requested_current, maximum_current);
}

static bool phase_resistance_mode_start(MotorControlState *motor,
                                        const PhaseResistanceConfig *configured)
{
    if (configured == NULL)
    {
        return false;
    }
    PhaseResistanceConfig config = *configured;
    config.test_current_low = phase_resistance_mode_limit_current(
        motor, configured->test_current_low, configured->maximum_test_current);
    config.test_current_high = phase_resistance_mode_limit_current(
        motor, configured->test_current_high, configured->maximum_test_current);
    config.maximum_test_current = phase_resistance_mode_limit_current(
        motor, configured->maximum_test_current, configured->maximum_test_current);

    PhaseResistance_Init(&s_phase_resistance_mode_context.core);
    s_phase_resistance_mode_context.applied_mod_d = 0.0f;
    s_phase_resistance_mode_context.applied_mod_q = 0.0f;
    s_phase_resistance_mode_context.applied_voltage_valid = false;
    s_phase_resistance_mode_context.telemetry_valid = false;
    return PhaseResistance_Start(&s_phase_resistance_mode_context.core, &config);
}

static void phase_resistance_mode_copy_result(MotorControlState *motor)
{
    PhaseResistanceResult result;
    uint8_t index;

    if (!PhaseResistance_GetResult(&s_phase_resistance_mode_context.core, &result))
    {
        return;
    }

    for (index = 0U; index < PHASE_RESISTANCE_VECTOR_COUNT; index++)
    {
        motor->phase_resistance_vector_ohm[index] = result.vector_resistance[index];
    }
    motor->phase_resistance_a_ohm = result.phase_resistance_a_ohm;
    motor->phase_resistance_b_ohm = result.phase_resistance_b_ohm;
    motor->phase_resistance_c_ohm = result.phase_resistance_c_ohm;
    motor->phase_resistance_spread_pct = result.spread_pct;
    motor->phase_resistance_valid = result.valid;
    motor->phase_resistance_warning = result.warning;
    motor->phase_resistance_balanced = result.balanced;
}

static PhaseResistanceModeStatus phase_resistance_mode_fail(FocState *foc,
                                                            MotorControlState *motor,
                                                            PhaseResistanceModeStatus status)
{
    phase_resistance_mode_stop_output(foc, motor);
    phase_resistance_mode_clear_result(motor);
    PhaseResistance_Cancel(&s_phase_resistance_mode_context.core);
    s_phase_resistance_mode_context.status = status;
    return status;
}

static PhaseResistanceModeStatus
phase_resistance_mode_map_core_status(PhaseResistanceCoreStatus status)
{
    if (status == PHASE_RESISTANCE_CORE_DONE)
    {
        return PHASE_RESISTANCE_MODE_DONE;
    }
    if (status == PHASE_RESISTANCE_CORE_SETTLE_TIMEOUT)
    {
        return PHASE_RESISTANCE_MODE_SETTLE_TIMEOUT;
    }
    return PHASE_RESISTANCE_MODE_INVALID_RESULT;
}

static PhaseResistanceModeStatus
phase_resistance_step(FocState *foc, MotorControlState *motor, const PhaseResistanceConfig *config)
{
    PhaseResistanceCommand command;
    PhaseResistanceSample sample;
    PhaseResistanceCoreStatus core_status;
    PhaseResistanceModeStatus mode_status;

    if (foc == NULL || motor == NULL)
    {
        return PHASE_RESISTANCE_MODE_INVALID_RESULT;
    }
    if (s_phase_resistance_mode_context.status != PHASE_RESISTANCE_MODE_RUNNING)
    {
        phase_resistance_mode_stop_output(foc, motor);
        return s_phase_resistance_mode_context.status;
    }
    if (foc->bus_voltage_filtered_v <= PHASE_RESISTANCE_VBUS_MIN)
    {
        return phase_resistance_mode_fail(foc, motor, PHASE_RESISTANCE_MODE_UNDER_VOLTAGE);
    }
    if (foc->bus_voltage_filtered_v >= PHASE_RESISTANCE_VBUS_MAX)
    {
        return phase_resistance_mode_fail(foc, motor, PHASE_RESISTANCE_MODE_OVER_VOLTAGE);
    }

    if (!s_phase_resistance_mode_context.started)
    {
        phase_resistance_mode_clear_result(motor);
        if (!phase_resistance_mode_start(motor, config))
        {
            return phase_resistance_mode_fail(foc, motor, PHASE_RESISTANCE_MODE_INVALID_RESULT);
        }
        s_phase_resistance_mode_context.started = true;
        FOC_CurrentController_Reset(foc);
        return PHASE_RESISTANCE_MODE_RUNNING;
    }

    PhaseResistance_GetCommand(&s_phase_resistance_mode_context.core, &command);
    if (!command.inject_current)
    {
        if (command.reset_current_controller)
        {
            phase_resistance_mode_stop_output(foc, motor);
        }
        core_status = PhaseResistance_InputSample(&s_phase_resistance_mode_context.core, NULL);
    }
    else
    {
        float applied_mod_d = s_phase_resistance_mode_context.applied_mod_d;
        float applied_mod_q = s_phase_resistance_mode_context.applied_mod_q;
        float current_magnitude;

        motor->id_target_a = command.id_ref;
        motor->iq_target_a = command.iq_ref;
        FOC_Current(foc, motor, command.electrical_angle, 0.0f);

        sample.id = foc->id_a;
        sample.iq = foc->iq_a;
        sample.id_filt = foc->id_filtered_a;
        sample.iq_filt = foc->iq_filtered_a;
        if (s_phase_resistance_mode_context.applied_voltage_valid)
        {
            sample.vd = applied_mod_d * foc->bus_voltage_v / 1.5f;
            sample.vq = applied_mod_q * foc->bus_voltage_v / 1.5f;
        }
        else
        {
            sample.vd = 0.0f;
            sample.vq = 0.0f;
        }
        current_magnitude = sqrtf(sample.id * sample.id + sample.iq * sample.iq);
        s_phase_resistance_mode_context.telemetry.electrical_angle = command.electrical_angle;
        s_phase_resistance_mode_context.telemetry.id_ref = command.id_ref;
        s_phase_resistance_mode_context.telemetry.id = sample.id;
        s_phase_resistance_mode_context.telemetry.iq = sample.iq;
        s_phase_resistance_mode_context.telemetry.vd = sample.vd;
        s_phase_resistance_mode_context.telemetry.vq = sample.vq;
        s_phase_resistance_mode_context.telemetry.current_magnitude = current_magnitude;
        s_phase_resistance_mode_context.telemetry.parallel_voltage =
            current_magnitude > 0.0f
                ? (sample.vd * sample.id + sample.vq * sample.iq) / current_magnitude
                : 0.0f;
        s_phase_resistance_mode_context.telemetry.vbus = foc->bus_voltage_v;
        s_phase_resistance_mode_context.telemetry_valid =
            s_phase_resistance_mode_context.applied_voltage_valid;

        s_phase_resistance_mode_context.applied_mod_d = foc->mod_d;
        s_phase_resistance_mode_context.applied_mod_q = foc->mod_q;
        s_phase_resistance_mode_context.applied_voltage_valid = true;
        core_status = PhaseResistance_InputSample(&s_phase_resistance_mode_context.core, &sample);
    }

    if (core_status == PHASE_RESISTANCE_CORE_RUNNING)
    {
        return PHASE_RESISTANCE_MODE_RUNNING;
    }

    mode_status = phase_resistance_mode_map_core_status(core_status);
    if (mode_status == PHASE_RESISTANCE_MODE_DONE)
    {
        phase_resistance_mode_stop_output(foc, motor);
        phase_resistance_mode_copy_result(motor);
        s_phase_resistance_mode_context.status = mode_status;
        return mode_status;
    }
    return phase_resistance_mode_fail(foc, motor, mode_status);
}

bool PhaseResistanceTask_GetTelemetry(PhaseResistanceModeTelemetry *telemetry)
{
    if (telemetry == NULL || !s_phase_resistance_mode_context.started ||
        !s_phase_resistance_mode_context.telemetry_valid)
    {
        return false;
    }

    *telemetry = s_phase_resistance_mode_context.telemetry;
    return true;
}

void PhaseResistanceTask_Cancel(FocState *foc, MotorControlState *motor)
{
    if (foc != NULL && motor != NULL &&
        (s_phase_resistance_mode_context.started ||
         s_phase_resistance_mode_context.status != PHASE_RESISTANCE_MODE_RUNNING))
    {
        phase_resistance_mode_stop_output(foc, motor);
    }

    PhaseResistance_Cancel(&s_phase_resistance_mode_context.core);
    s_phase_resistance_mode_context.started = false;
    s_phase_resistance_mode_context.status = PHASE_RESISTANCE_MODE_RUNNING;
}

bool PhaseResistanceTask_IsReleased(void)
{
    return !s_phase_resistance_mode_context.started;
}

MotorTaskResult PhaseResistanceTask_Run(FocState *foc,
                                        MotorControlState *motor,
                                        const PhaseResistanceConfig *config)
{
    PhaseResistanceModeStatus status = phase_resistance_step(foc, motor, config);
    if (status != PHASE_RESISTANCE_MODE_RUNNING)
    {
        /* 终态自行归还执行状态，不依赖下一禁用周期的外部取消。 */
        s_phase_resistance_mode_context.started = false;
    }
    switch (status)
    {
    case PHASE_RESISTANCE_MODE_DONE:
        return (MotorTaskResult){MOTOR_TASK_COMPLETED, MOTOR_FAULT_NONE};
    case PHASE_RESISTANCE_MODE_SETTLE_TIMEOUT:
        return (MotorTaskResult){MOTOR_TASK_FAILED, MOTOR_FAULT_PHASE_RESISTANCE};
    case PHASE_RESISTANCE_MODE_INVALID_RESULT:
        return (MotorTaskResult){MOTOR_TASK_FAILED, MOTOR_FAULT_INVALID_PARAMETERS};
    case PHASE_RESISTANCE_MODE_UNDER_VOLTAGE:
        return (MotorTaskResult){MOTOR_TASK_FAILED, MOTOR_FAULT_UNDERVOLTAGE};
    case PHASE_RESISTANCE_MODE_OVER_VOLTAGE:
        return (MotorTaskResult){MOTOR_TASK_FAILED, MOTOR_FAULT_OVERVOLTAGE};
    default:
        return (MotorTaskResult){MOTOR_TASK_RUNNING, MOTOR_FAULT_NONE};
    }
}
