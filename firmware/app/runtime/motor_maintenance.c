#include "motor_maintenance.h"

#include <math.h>
#include "bus_voltage_profile.h"
#include "encoder_calibration_task.h"
#include "foc_param.h"
#include "motor_execution.h"
#include "parameter_value_service.h"
#include "maintenance_workspace.h"
#include "motor_thermal_control.h"
#include "power_stage_hw.h"

/* 维护调度只调用实际功能；生命周期、输出许可、保存衔接由唯一主状态机拥有。 */
MotorFaultCode MotorMaintenance_Admit(const MotorConfiguration *configuration,
                                      MotorMaintenanceKind kind,
                                      const FocState *foc,
                                      const EncoderState *encoder)
{
    if ((kind != MOTOR_MAINTENANCE_ENCODER_LUT && kind != MOTOR_MAINTENANCE_ELECTRICAL_ZERO &&
         kind != MOTOR_MAINTENANCE_MECHANICAL_ZERO && kind != MOTOR_MAINTENANCE_SAVE &&
         kind != MOTOR_MAINTENANCE_DEFAULTS) ||
        !configuration->application.axis_profile_valid)
    {
        return MOTOR_FAULT_INVALID_PARAMETERS;
    }
    bool torque_task =
        kind == MOTOR_MAINTENANCE_ENCODER_LUT || kind == MOTOR_MAINTENANCE_ELECTRICAL_ZERO;
    if (torque_task)
    {
        if (!configuration->configuration_confirmed ||
            !ParameterValue_ConfigurationValid(configuration))
        {
            return MOTOR_FAULT_INVALID_PARAMETERS;
        }
        if (!isfinite(foc->bus_voltage_v) || !isfinite(foc->bus_voltage_filtered_v) ||
            foc->bus_voltage_v >= BUS_VOLTAGE_HARD_OVERVOLTAGE_V ||
            foc->bus_voltage_filtered_v > BUS_VOLTAGE_ENABLE_MAX_V)
        {
            return MOTOR_FAULT_OVERVOLTAGE;
        }
        if (foc->bus_voltage_filtered_v < BUS_VOLTAGE_ENABLE_MIN_V)
        {
            return MOTOR_FAULT_UNDERVOLTAGE;
        }
    }
    if (kind == MOTOR_MAINTENANCE_ELECTRICAL_ZERO &&
        (Encoder_GetCalibFlag(&configuration->device.encoder) & ENC_CALIB_LINEARIZED) == 0U)
    {
        return MOTOR_FAULT_ENCODER_NOT_CALIBRATED;
    }
    if (kind != MOTOR_MAINTENANCE_SAVE && kind != MOTOR_MAINTENANCE_DEFAULTS &&
        !Encoder_IsOnline(encoder))
    {
        return MOTOR_FAULT_ENCODER;
    }
    return MOTOR_FAULT_NONE;
}

MotorTaskResult MotorMaintenance_Step(MotorConfiguration *configuration,
                                      MotorMaintenanceKind kind,
                                      FocState *foc,
                                      MotorControlState *motor,
                                      PiController *pi,
                                      EncoderState *encoder,
                                      FluxObserverState *observer,
                                      SensorlessStartupState *startup)
{
    switch (kind)
    {
    case MOTOR_MAINTENANCE_ENCODER_LUT:
    {
        EncoderCalibrationWorkspace *workspace = MaintenanceWorkspace_BorrowEncoder();
        MotorTaskResult result = EncoderCalibrationTask_StepLut(configuration,
                                                                &configuration->device.encoder,
                                                                foc,
                                                                motor,
                                                                pi,
                                                                encoder,
                                                                observer,
                                                                startup,
                                                                workspace);
        if (EncoderCalibrationTask_IsReleased())
        {
            MaintenanceWorkspace_ReleaseEncoder();
        }
        return result;
    }
    case MOTOR_MAINTENANCE_ELECTRICAL_ZERO:
        return EncoderCalibrationTask_StepElectricalZero(
            configuration, &configuration->device.encoder, foc, motor, encoder);
    case MOTOR_MAINTENANCE_MECHANICAL_ZERO:
        if (!Encoder_SetMechanicalZero(&configuration->device.encoder, encoder))
        {
            return (MotorTaskResult){MOTOR_TASK_FAILED, MOTOR_FAULT_ENCODER};
        }
        MotorOuterLoop_RequestReset();
        return (MotorTaskResult){MOTOR_TASK_COMPLETED, MOTOR_FAULT_NONE};
    case MOTOR_MAINTENANCE_DEFAULTS:
    {
        /* 先同步热模型；失败不能先改活动参数，更不能以恢复默认清零热历史。 */
        MotorWindingThermalConfig thermal_defaults;
        MotorWindingThermal_Default(&thermal_defaults);
        if (!MotorThermalControl_Configure(&thermal_defaults, power_stage_hw_phases_disabled()))
        {
            return (MotorTaskResult){MOTOR_TASK_FAILED, MOTOR_FAULT_WINDING_THERMAL};
        }
        /* 运行期端点地址已冻结；恢复控制默认值不等于生产改址。 */
        uint8_t node_id = configuration->device.node_id;
        Param_Return_Default(configuration);
        motor->vq_target_v = 0.0f;
        configuration->device.node_id = node_id;
        return (MotorTaskResult){MOTOR_TASK_COMPLETED, MOTOR_FAULT_NONE};
    }
    case MOTOR_MAINTENANCE_SAVE:
        return (MotorTaskResult){MOTOR_TASK_RUNNING, MOTOR_FAULT_NONE};
    default:
        return (MotorTaskResult){MOTOR_TASK_FAILED, MOTOR_FAULT_INVALID_PARAMETERS};
    }
}

void MotorMaintenance_Cancel(MotorMaintenanceKind kind)
{
    if (kind == MOTOR_MAINTENANCE_ENCODER_LUT || kind == MOTOR_MAINTENANCE_ELECTRICAL_ZERO)
    {
        EncoderCalibrationTask_Cancel();
        if (EncoderCalibrationTask_IsReleased())
        {
            MaintenanceWorkspace_ReleaseEncoder();
        }
    }
}

bool MotorMaintenance_IsReleased(void)
{
    return MaintenanceWorkspace_IsReleased() && EncoderCalibrationTask_IsReleased();
}
