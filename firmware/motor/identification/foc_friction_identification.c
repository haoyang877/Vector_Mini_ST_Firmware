#include "foc_friction_identification.h"

#include <math.h>
#include <string.h>

#include "foc_errhandle.h"
#include "foc_speed.h"
#include "control_config.h"
#include "utils.h"
#include "bus_voltage_profile.h"

static FrictionIdentification_TypeDef identification;
static uint32_t speed_loop_divider;
static bool session_started;

static void StopOutput(MotorControl_TypeDef *motor, PI_Controller_TypeDef *speed_controller)
{
    if (motor == NULL || speed_controller == NULL)
    {
        return;
    }
    motor->speedRef = 0.0f;
    motor->speedShadow = 0.0f;
    motor->idRef = 0.0f;
    motor->iqRef = 0.0f;
    PI_Controller_Reset(speed_controller);
}

static bool RuntimeConfigIsValid(const MotorControl_TypeDef *motor,
                                 const FocFrictionIdentificationConfig *config)
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

    return motor != NULL && isfinite(motor->current_limit) && motor->current_limit > 0.0f &&
           isfinite(motor->speed_limit) && motor->speed_limit >= maximum_speed &&
           isfinite(motor->speed_Kp) && motor->speed_Kp > 0.0f && isfinite(motor->speed_Ki) &&
           motor->speed_Ki >= 0.0f;
}

static void Fail(MotorControl_TypeDef *motor,
                 PI_Controller_TypeDef *speed_controller,
                 FrictionIdentificationReason_TypeDef reason,
                 bool report_module_error)
{
    FrictionIdentification_Fail(&identification, reason);
    session_started = false;
    StopOutput(motor, speed_controller);
    if (report_module_error)
    {
        MotorFault_Report(motor, FrictionIdentification_Error);
    }
}

void FocFrictionIdentification_Init(const FocFrictionIdentificationConfig *config)
{
    (void)FrictionIdentification_Init(&identification, config != NULL ? &config->core : NULL);
    speed_loop_divider = 0U;
    session_started = false;
}

static bool start_identification(MotorControl_TypeDef *motor,
                                 Encoder_TypeDef *encoder,
                                 PI_Controller_TypeDef *speed_controller,
                                 const FocFrictionIdentificationConfig *config)
{
    FocFrictionIdentification_Init(config);
    if (encoder == NULL || speed_controller == NULL || !RuntimeConfigIsValid(motor, config) ||
        !Encoder_IsOnline(encoder) ||
        (Encoder_GetCalibFlag(encoder) & ENC_CALIB_ALL) != ENC_CALIB_ALL ||
        !FrictionIdentification_Start(&identification))
    {
        FrictionIdentification_Fail(&identification, FRICTION_IDENT_REASON_INVALID_CONFIG);
        return false;
    }
    StopOutput(motor, speed_controller);
    motor->speedRef = FrictionIdentification_GetTargetSpeed(&identification);
    session_started = true;
    return true;
}

void FocFrictionIdentification_Abort(MotorControl_TypeDef *motor,
                                     PI_Controller_TypeDef *speed_controller)
{
    FrictionIdentification_Abort(&identification);
    StopOutput(motor, speed_controller);
    session_started = false;
}

MotorTaskResult FocFrictionIdentification_Task(FOC_TypeDef *foc,
                                               MotorControl_TypeDef *motor,
                                               PI_Controller_TypeDef *speed_controller,
                                               Encoder_TypeDef *encoder,
                                               const FocFrictionIdentificationConfig *config)
{
    FrictionIdentificationInput_TypeDef input;
    FrictionIdentificationState_TypeDef state;
    MotorTaskResult outcome = {MOTOR_TASK_RUNNING, No_Error};

    if (foc == NULL || speed_controller == NULL || encoder == NULL ||
        !RuntimeConfigIsValid(motor, config))
    {
        Fail(motor, speed_controller, FRICTION_IDENT_REASON_INVALID_CONFIG, true);
        return (MotorTaskResult){MOTOR_TASK_FAILED, FrictionIdentification_Error};
    }
    if (!Encoder_IsOnline(encoder))
    {
        MotorFault_Report(motor, Encoder_Error);
        Fail(motor, speed_controller, FRICTION_IDENT_REASON_SAFETY_FAULT, false);
        return (MotorTaskResult){MOTOR_TASK_FAILED, motor->ErrorNow};
    }
    if ((Encoder_GetCalibFlag(encoder) & ENC_CALIB_ALL) != ENC_CALIB_ALL)
    {
        MotorFault_Report(motor, Encoder_NotCalibrated);
        Fail(motor, speed_controller, FRICTION_IDENT_REASON_SAFETY_FAULT, false);
        return (MotorTaskResult){MOTOR_TASK_FAILED, motor->ErrorNow};
    }
    /* 本层持有辨识工作状态，只有已受理的维护调度才能进入。 */
    if (!session_started && !start_identification(motor, encoder, speed_controller, config))
    {
        MotorFault_Report(motor, FrictionIdentification_Error);
        return (MotorTaskResult){MOTOR_TASK_FAILED, FrictionIdentification_Error};
    }

    if (foc->Vbus_filt >= BUS_VOLTAGE_OVERVOLTAGE_V)
    {
        MotorFault_Report(motor, Over_Voltage);
    }
    else if (foc->Vbus_filt <= BUS_VOLTAGE_UNDERVOLTAGE_V)
    {
        MotorFault_Report(motor, Under_Voltage);
    }
    else if (foc->temp >= 100.0f)
    {
        MotorFault_Report(motor, High_Temprature);
    }
    if (motor->ErrorNow != No_Error)
    {
        Fail(motor, speed_controller, FRICTION_IDENT_REASON_SAFETY_FAULT, false);
        return (MotorTaskResult){MOTOR_TASK_FAILED, motor->ErrorNow};
    }

    SpeedMode_Run(foc,
                  motor,
                  speed_controller,
                  Encoder_GetElePhase(encoder),
                  Encoder_GetEleVel(encoder),
                  Encoder_GetMecVel(encoder));
    if (++speed_loop_divider < SPEED_LOOP_DIVIDER)
    {
        return outcome;
    }
    speed_loop_divider = 0U;

    input.measured_speed_rad_s = Encoder_GetMecVel(encoder);
    input.ramped_speed_reference_rad_s = motor->speedShadow;
    input.iq_a = foc->Iq_filt;
    input.mechanical_position_rad = Encoder_GetMecPos(encoder);
    input.current_saturated =
        fast_abs(motor->iqRef) >= motor->current_limit * config->current_ratio_max;
    FrictionIdentification_Update(&identification, &input);
    motor->speedRef = FrictionIdentification_GetTargetSpeed(&identification);

    state = FrictionIdentification_GetState(&identification);
    if (state == FRICTION_IDENT_FAILED)
    {
        session_started = false;
        StopOutput(motor, speed_controller);
        if (motor->ErrorNow == No_Error)
        {
            MotorFault_Report(motor, FrictionIdentification_Error);
        }
        outcome = (MotorTaskResult){MOTOR_TASK_FAILED, motor->ErrorNow};
    }
    else if (state == FRICTION_IDENT_COMPLETE)
    {
        session_started = false;
        StopOutput(motor, speed_controller);
        /* 停机迁移与功率级关断由运行状态机执行。 */
        outcome.result = MOTOR_TASK_COMPLETED;
    }

    return outcome;
}

bool FocFrictionIdentification_ApplyCandidate(MotorControl_TypeDef *motor)
{
    const FrictionIdentificationResult_TypeDef *result =
        FrictionIdentification_GetResult(&identification);

    if (motor == NULL || result == NULL || !result->valid ||
        FrictionIdentification_GetState(&identification) != FRICTION_IDENT_COMPLETE ||
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

bool FocFrictionIdentification_IsReleased(void)
{
    return !session_started;
}
