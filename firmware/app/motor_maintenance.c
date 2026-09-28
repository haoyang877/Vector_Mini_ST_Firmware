#include "motor_maintenance.h"

#include <math.h>
#include "bus_voltage_profile.h"
#include "foc_cogging_calibration.h"
#include "foc_encoder_calibration.h"
#include "foc_friction_identification.h"
#include "foc_phase_resistance.h"
#include "foc_param.h"
#include "motor_execution.h"

/* 维护调度只调用实际功能；生命周期、输出许可、保存衔接由唯一主状态机拥有。 */
ErrorNow_TypeDef MotorMaintenance_Admit(MotorMaintenanceKind kind,
                                        const MotorControl_TypeDef *motor,
                                        const FOC_TypeDef *foc,
                                        const Encoder_TypeDef *encoder)
{
    if ((unsigned)kind >= MOTOR_MAINTENANCE_NONE || !motor->axis_profile_valid)
    {
        return MotorParam_Error;
    }
    bool torque_task =
        kind == MOTOR_MAINTENANCE_ENCODER_LUT || kind == MOTOR_MAINTENANCE_ELECTRICAL_ZERO ||
        kind == MOTOR_MAINTENANCE_COGGING || kind == MOTOR_MAINTENANCE_PHASE_RESISTANCE ||
        kind == MOTOR_MAINTENANCE_FRICTION;
    if (torque_task)
    {
        if (!isfinite(foc->Vbus) || !isfinite(foc->Vbus_filt) ||
            foc->Vbus >= BUS_VOLTAGE_HARD_OVERVOLTAGE_V ||
            foc->Vbus_filt > BUS_VOLTAGE_ENABLE_MAX_V)
        {
            return Over_Voltage;
        }
        if (foc->Vbus_filt < BUS_VOLTAGE_ENABLE_MIN_V)
        {
            return Under_Voltage;
        }
    }
    if (kind == MOTOR_MAINTENANCE_COGGING && !FocCogging_CanStart(motor, encoder))
    {
        return CoggingCalibration_Error;
    }
    if (kind == MOTOR_MAINTENANCE_ELECTRICAL_ZERO &&
        (Encoder_GetCalibFlag(encoder) & ENC_CALIB_LINEARIZED) == 0U)
    {
        return Encoder_NotCalibrated;
    }
    if (kind != MOTOR_MAINTENANCE_SAVE && kind != MOTOR_MAINTENANCE_DEFAULTS &&
        kind != MOTOR_MAINTENANCE_PHASE_RESISTANCE && !Encoder_IsOnline(encoder))
    {
        return Encoder_Error;
    }
    if (kind == MOTOR_MAINTENANCE_FRICTION &&
        (Encoder_GetCalibFlag(encoder) & ENC_CALIB_ALL) != ENC_CALIB_ALL)
    {
        return Encoder_NotCalibrated;
    }
    return No_Error;
}

MotorTaskResult MotorMaintenance_Step(MotorMaintenanceKind kind,
                                      FOC_TypeDef *foc,
                                      MotorControl_TypeDef *motor,
                                      PI_Controller_TypeDef *pi,
                                      Encoder_TypeDef *encoder,
                                      Fluxobserver_TypeDef *observer,
                                      SensorlessStartup_TypeDef *startup)
{
    switch (kind)
    {
    case MOTOR_MAINTENANCE_ENCODER_LUT:
        return Task_Calib_EncoderObserver(foc, motor, pi, encoder, observer, startup);
    case MOTOR_MAINTENANCE_ELECTRICAL_ZERO:
        return Task_Calib_EleAngelOffset(foc, motor, encoder);
    case MOTOR_MAINTENANCE_COGGING:
        return FocCogging_Task(foc, motor, pi, encoder);
    case MOTOR_MAINTENANCE_PHASE_RESISTANCE:
        return PhaseResistanceMode_Run(foc, motor);
    case MOTOR_MAINTENANCE_FRICTION:
        return FocFrictionIdentification_Task(foc, motor, pi, encoder);
    case MOTOR_MAINTENANCE_MECHANICAL_ZERO:
        if (!Encoder_SetMechanicalZero(encoder))
        {
            return (MotorTaskResult){MOTOR_TASK_FAILED, Encoder_Error};
        }
        Task_Position_Mode_Reset();
        return (MotorTaskResult){MOTOR_TASK_COMPLETED, No_Error};
    case MOTOR_MAINTENANCE_DEFAULTS:
        Param_Return_Default(motor, encoder);
        return (MotorTaskResult){MOTOR_TASK_COMPLETED, No_Error};
    case MOTOR_MAINTENANCE_SAVE:
        return (MotorTaskResult){MOTOR_TASK_RUNNING, No_Error};
    default:
        return (MotorTaskResult){MOTOR_TASK_FAILED, MotorParam_Error};
    }
}

void MotorMaintenance_Cancel(MotorMaintenanceKind kind,
                             FOC_TypeDef *foc,
                             MotorControl_TypeDef *motor,
                             PI_Controller_TypeDef *pi)
{
    switch (kind)
    {
    case MOTOR_MAINTENANCE_ENCODER_LUT:
    case MOTOR_MAINTENANCE_ELECTRICAL_ZERO:
        FocEncoderCalibration_Cancel();
        break;
    case MOTOR_MAINTENANCE_COGGING:
        FocCogging_Abort();
        break;
    case MOTOR_MAINTENANCE_PHASE_RESISTANCE:
        PhaseResistanceMode_Cancel(foc, motor);
        break;
    case MOTOR_MAINTENANCE_FRICTION:
        FocFrictionIdentification_Abort(motor, pi);
        break;
    default:
        break;
    }
}

bool MotorMaintenance_IsReleased(void)
{
    return FocEncoderCalibration_IsReleased() && FocCogging_IsReleased() &&
           FocFrictionIdentification_IsReleased() && PhaseResistanceMode_IsReleased();
}
