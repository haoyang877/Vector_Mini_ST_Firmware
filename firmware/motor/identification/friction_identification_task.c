#include "friction_identification_task.h"

#include <math.h>
#include <string.h>

#include "foc_errhandle.h"
#include "foc_speed.h"
#include "control_config.h"
#include "utils.h"
#include "bus_voltage_profile.h"

static FrictionIdentification s_identification;
static uint32_t s_speed_loop_divider;
static bool s_session_active;

static void stop_output(MotorControlState *motor, PiController *speed_controller)
{
    if (motor == NULL || speed_controller == NULL)
    {
        return;
    }
    motor->speed_target_rad_s = 0.0f;
    motor->speed_ramped_rad_s = 0.0f;
    motor->id_target_a = 0.0f;
    motor->iq_target_a = 0.0f;
    PiController_Reset(speed_controller);
}

static bool runtime_config_is_valid(const MotorControlState *motor,
                                    const FrictionIdentificationTaskConfig *config)
{
    if (config == NULL || config->core.speed_point_count == 0U ||
        config->core.speed_point_count > FRICTION_IDENTIFICATION_MAX_SPEED_POINTS ||
        !isfinite(config->current_ratio_max) || config->current_ratio_max <= 0.0f ||
        config->current_ratio_max > 1.0f)
    {
        return false;
    }
    const float maximum_speed =
        config->core.speed_points_rad_s[config->core.speed_point_count - 1U];

    return motor != NULL && isfinite(motor->current_limit_a) && motor->current_limit_a > 0.0f &&
           isfinite(motor->speed_limit_rad_s) && motor->speed_limit_rad_s >= maximum_speed &&
           isfinite(motor->speed_kp) && motor->speed_kp > 0.0f && isfinite(motor->speed_ki) &&
           motor->speed_ki >= 0.0f;
}

static void fail(MotorControlState *motor,
                 PiController *speed_controller,
                 FrictionIdentificationReason reason,
                 bool report_module_error)
{
    FrictionIdentification_Fail(&s_identification, reason);
    s_session_active = false;
    stop_output(motor, speed_controller);
    if (report_module_error)
    {
        MotorFault_Report(motor, MOTOR_FAULT_FRICTION_IDENTIFICATION);
    }
}

void FrictionIdentificationTask_Init(const FrictionIdentificationTaskConfig *config)
{
    (void)FrictionIdentification_Init(&s_identification, config != NULL ? &config->core : NULL);
    s_speed_loop_divider = 0U;
    s_session_active = false;
}

static bool start_identification(MotorControlState *motor,
                                 EncoderState *encoder,
                                 PiController *speed_controller,
                                 const FrictionIdentificationTaskConfig *config)
{
    FrictionIdentificationTask_Init(config);
    if (encoder == NULL || speed_controller == NULL || !runtime_config_is_valid(motor, config) ||
        !Encoder_IsOnline(encoder) ||
        (Encoder_GetCalibFlag(encoder) & ENC_CALIB_ALL) != ENC_CALIB_ALL ||
        !FrictionIdentification_Start(&s_identification))
    {
        FrictionIdentification_Fail(&s_identification, FRICTION_IDENT_REASON_INVALID_CONFIG);
        return false;
    }
    stop_output(motor, speed_controller);
    motor->speed_target_rad_s = FrictionIdentification_GetTargetSpeed(&s_identification);
    s_session_active = true;
    return true;
}

void FrictionIdentificationTask_Abort(MotorControlState *motor, PiController *speed_controller)
{
    FrictionIdentification_Abort(&s_identification);
    stop_output(motor, speed_controller);
    s_session_active = false;
}

MotorTaskResult FrictionIdentificationTask_Task(FocState *foc,
                                                MotorControlState *motor,
                                                PiController *speed_controller,
                                                EncoderState *encoder,
                                                const FrictionIdentificationTaskConfig *config)
{
    FrictionIdentificationInput input;
    FrictionIdentificationState state;
    MotorTaskResult outcome = {MOTOR_TASK_RUNNING, MOTOR_FAULT_NONE};

    if (foc == NULL || speed_controller == NULL || encoder == NULL ||
        !runtime_config_is_valid(motor, config))
    {
        fail(motor, speed_controller, FRICTION_IDENT_REASON_INVALID_CONFIG, true);
        return (MotorTaskResult){MOTOR_TASK_FAILED, MOTOR_FAULT_FRICTION_IDENTIFICATION};
    }
    if (!Encoder_IsOnline(encoder))
    {
        MotorFault_Report(motor, MOTOR_FAULT_ENCODER);
        fail(motor, speed_controller, FRICTION_IDENT_REASON_SAFETY_FAULT, false);
        return (MotorTaskResult){MOTOR_TASK_FAILED, motor->fault_code};
    }
    if ((Encoder_GetCalibFlag(encoder) & ENC_CALIB_ALL) != ENC_CALIB_ALL)
    {
        MotorFault_Report(motor, MOTOR_FAULT_ENCODER_NOT_CALIBRATED);
        fail(motor, speed_controller, FRICTION_IDENT_REASON_SAFETY_FAULT, false);
        return (MotorTaskResult){MOTOR_TASK_FAILED, motor->fault_code};
    }
    /* 本层持有辨识工作状态，只有已受理的维护调度才能进入。 */
    if (!s_session_active && !start_identification(motor, encoder, speed_controller, config))
    {
        MotorFault_Report(motor, MOTOR_FAULT_FRICTION_IDENTIFICATION);
        return (MotorTaskResult){MOTOR_TASK_FAILED, MOTOR_FAULT_FRICTION_IDENTIFICATION};
    }

    if (foc->bus_voltage_filtered_v >= BUS_VOLTAGE_OVERVOLTAGE_V)
    {
        MotorFault_Report(motor, MOTOR_FAULT_OVERVOLTAGE);
    }
    else if (foc->bus_voltage_filtered_v <= BUS_VOLTAGE_UNDERVOLTAGE_V)
    {
        MotorFault_Report(motor, MOTOR_FAULT_UNDERVOLTAGE);
    }
    else if (foc->temperature_c >= 100.0f)
    {
        MotorFault_Report(motor, MOTOR_FAULT_OVERTEMPERATURE);
    }
    if (motor->fault_code != MOTOR_FAULT_NONE)
    {
        fail(motor, speed_controller, FRICTION_IDENT_REASON_SAFETY_FAULT, false);
        return (MotorTaskResult){MOTOR_TASK_FAILED, motor->fault_code};
    }

    SpeedMode_Run(foc,
                  motor,
                  speed_controller,
                  Encoder_GetElePhase(encoder),
                  Encoder_GetEleVel(encoder),
                  Encoder_GetMecVel(encoder));
    if (++s_speed_loop_divider < SPEED_LOOP_DIVIDER)
    {
        return outcome;
    }
    s_speed_loop_divider = 0U;

    input.measured_speed_rad_s = Encoder_GetMecVel(encoder);
    input.ramped_speed_reference_rad_s = motor->speed_ramped_rad_s;
    input.iq_a = foc->iq_filtered_a;
    input.mechanical_position_rad = Encoder_GetMecPos(encoder);
    input.current_saturated =
        Math_Abs(motor->iq_target_a) >= motor->current_limit_a * config->current_ratio_max;
    FrictionIdentification_Update(&s_identification, &input);
    motor->speed_target_rad_s = FrictionIdentification_GetTargetSpeed(&s_identification);

    state = FrictionIdentification_GetState(&s_identification);
    if (state == FRICTION_IDENT_FAILED)
    {
        s_session_active = false;
        stop_output(motor, speed_controller);
        if (motor->fault_code == MOTOR_FAULT_NONE)
        {
            MotorFault_Report(motor, MOTOR_FAULT_FRICTION_IDENTIFICATION);
        }
        outcome = (MotorTaskResult){MOTOR_TASK_FAILED, motor->fault_code};
    }
    else if (state == FRICTION_IDENT_COMPLETE)
    {
        s_session_active = false;
        stop_output(motor, speed_controller);
        /* 停机迁移与功率级关断由运行状态机执行。 */
        outcome.result = MOTOR_TASK_COMPLETED;
    }

    return outcome;
}

bool FrictionIdentificationTask_ApplyCandidate(MotorControlState *motor)
{
    const FrictionIdentificationResult *result =
        FrictionIdentification_GetResult(&s_identification);

    if (motor == NULL || result == NULL || !result->valid ||
        FrictionIdentification_GetState(&s_identification) != FRICTION_IDENT_COMPLETE ||
        motor->control_mode != MOTOR_CONTROL_NONE)
    {
        return false;
    }

    motor->friction_coulomb_pos_a = result->coulomb_pos_a;
    motor->friction_coulomb_neg_a = result->coulomb_neg_a;
    motor->friction_viscous_pos_a_per_rad_s = result->viscous_pos_a_per_rad_s;
    motor->friction_viscous_neg_a_per_rad_s = result->viscous_neg_a_per_rad_s;
    motor->friction_model_valid = true;
    return true;
}

bool FrictionIdentificationTask_IsReleased(void)
{
    return !s_session_active;
}
