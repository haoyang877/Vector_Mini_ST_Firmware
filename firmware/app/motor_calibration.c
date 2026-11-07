#include "motor_calibration.h"
#include "bsp_system.h"
#include "motor_fault.h"
#include <math.h>

MotorTaskResult MotorCalibration_StepFast(MotorCalibration *cal)
{
    switch (MotorLifecycle_GetCalibrationType(cal->life))
    {
    case MOTOR_CALIBRATION_ENCODER_LINEARIZATION:
        if (EncoderCalibration_GetPhase(&cal->session) == CS_NULL)
        {
            cal->output->ready = false;
        }
#if BSP_ENABLED_LINEAR_HALL
        if (MotorAxisPreset_HasLinearHall(cal->drive.configuration->preset) &&
            EncoderCalibration_GetPhase(&cal->session) == CS_NULL)
        {
            /* 新会话覆盖上次中止的极值；只在观测器闭环采样阶段收集。 */
            LinearHall_CalibrationBegin(&cal->hall_sweep);
            OutputEncoder_CalibBegin(&cal->output_calibration, cal->output->params.gear_ratio);
            cal->output->ready = false;
        }
#endif
        return EncoderCalibration_StepLut(&cal->session, &cal->drive, &cal->workspace);
    case MOTOR_CALIBRATION_JOINT_ELECTRICAL_ZERO:
    case MOTOR_CALIBRATION_ELECTRICAL_ZERO:
        return EncoderCalibration_StepElecZero(&cal->session, &cal->drive);
    case MOTOR_CALIBRATION_MECHANICAL_ZERO:
        if (!OutputEncoder_IsReady(cal->output))
        {
            return (MotorTaskResult){MOTOR_TASK_FAILED, MOTOR_FAULT_ENCODER};
        }
        if (Encoder_SetMechanicalZero(&cal->drive.configuration->device.encoder,
                                      cal->drive.encoder))
        {
            MotorOuterLoop_RequestReset(cal->outer);
            return (MotorTaskResult){MOTOR_TASK_COMPLETED, MOTOR_FAULT_NONE};
        }
        return (MotorTaskResult){MOTOR_TASK_FAILED, MOTOR_FAULT_ENCODER};
    default:
        return (MotorTaskResult){MOTOR_TASK_FAILED, MOTOR_FAULT_INVALID_PARAMETERS};
    }
}

#if BSP_ENABLED_LINEAR_HALL
/* 在原标定会话内等待停稳；调用者持有提交临界区。 */
static void advance_joint_lut(MotorCalibration *cal, uint32_t epoch)
{
    uint32_t now = BspSystem_TimeMs();
    if (epoch != cal->coast_epoch)
    {
        cal->coast_epoch = epoch;
        cal->coast_start_ms = cal->stable_since_ms = now;
    }
    if (!(fabsf(Encoder_GetMechVelContinuous(cal->drive.encoder)) < 1.0f))
    {
        cal->stable_since_ms = now;
    }
    if (now - cal->stable_since_ms < 200U)
    {
        if (now - cal->coast_start_ms >= 6000U)
        {
            MotorLifecycle_CompleteCalibrationPublication(cal->life, epoch, false);
        }
        return;
    }
    if (EncoderCalibration_PublishLut(
            &cal->session, &cal->drive.configuration->device.encoder, cal->drive.encoder))
    {
        OutputEncoder_Init(cal->output, cal->output->params.gear_ratio);
        MotorLifecycle_ContinueElectricalZero(cal->life);
    }
    else
    {
        MotorLifecycle_CompleteCalibrationPublication(cal->life, epoch, false);
    }
}
#endif

void MotorCalibration_ProcessPublication(MotorCalibration *cal,
                                         OutputEncoderCalibrationProgress progress,
                                         void *progress_context)
{
#if !BSP_ENABLED_LINEAR_HALL
    (void)progress;
    (void)progress_context;
#endif
    uint32_t mask = BspSystem_EnterCritical();
    uint32_t epoch;
    if (!MotorLifecycle_ReadPendingCalibrationPublication(cal->life, &epoch))
    {
        BspSystem_ExitCritical(mask);
        return;
    }
    bool lut =
        MotorLifecycle_GetCalibrationType(cal->life) == MOTOR_CALIBRATION_ENCODER_LINEARIZATION;
#if BSP_ENABLED_LINEAR_HALL
    if (lut && MotorAxisPreset_HasLinearHall(cal->drive.configuration->preset))
    {
        advance_joint_lut(cal, epoch);
        BspSystem_ExitCritical(mask);
        return;
    }
#endif
    if (lut && !EncoderCalibration_PublishLut(
                   &cal->session, &cal->drive.configuration->device.encoder, cal->drive.encoder))
    {
        MotorLifecycle_CompleteCalibrationPublication(cal->life, epoch, false);
        BspSystem_ExitCritical(mask);
        return;
    }
    OutputEncoder output = *cal->output;
    MotorCalibrationType kind = MotorLifecycle_GetCalibrationType(cal->life);
    MotorFaultCode fault = MOTOR_FAULT_NONE;
    if (lut)
    {
        OutputEncoder_Init(&output, output.params.gear_ratio);
    }
    if (kind == MOTOR_CALIBRATION_MECHANICAL_ZERO)
    {
        OutputEncoder_Update(&output, cal->drive.encoder->linearized_q15);
        OutputEncoder_SetZero(&output);
    }
#if BSP_ENABLED_LINEAR_HALL
    LinearHall hall = *cal->hall;
    if (kind == MOTOR_CALIBRATION_JOINT_ELECTRICAL_ZERO &&
        !LinearHall_CalibrationFinish(&cal->hall_sweep, &hall))
    {
        fault = MOTOR_FAULT_LINEAR_HALL_CALIBRATION;
    }
#endif
    BspSystem_ExitCritical(mask);
#if BSP_ENABLED_LINEAR_HALL
    /* 建表及相位拟合允许控制中断抢占；提交前仍须复核原会话。 */
    if (kind == MOTOR_CALIBRATION_JOINT_ELECTRICAL_ZERO &&
        !OutputEncoder_CalibEnd(&output,
                                &cal->output_calibration,
                                &hall,
                                &cal->drive.configuration->device.encoder,
                                progress,
                                progress_context) &&
        fault == MOTOR_FAULT_NONE)
    {
        fault = MOTOR_FAULT_ENCODER;
    }
#endif
    Param_Upload(&cal->record->motor, cal->drive.configuration);
#if BSP_ENABLED_LINEAR_HALL
    if (MotorAxisPreset_HasLinearHall(cal->drive.configuration->preset))
    {
        Param_StoreLinearHall(&cal->record->motor, &hall);
    }
#endif
#if BSP_ENABLED_LINEAR_HALL
    Param_StoreOutputEncoder(cal->record, &output, &hall);
#else
    Param_StoreOutputEncoder(cal->record, &output, NULL);
#endif
    mask = BspSystem_EnterCritical();
    uint32_t pending_epoch;
    if (MotorLifecycle_ReadPendingCalibrationPublication(cal->life, &pending_epoch) &&
        pending_epoch == epoch)
    {
        *cal->output = output;
#if BSP_ENABLED_LINEAR_HALL
        *cal->hall = hall;
#endif
        BspResult saved = cal->write_record(cal->record, sizeof(*cal->record));
        if (saved != BSP_OK || fault != MOTOR_FAULT_NONE)
        {
            MotorFault_Report(cal->drive.motor,
                              saved != BSP_OK ? MOTOR_FAULT_CALIBRATION_STORAGE : fault);
        }
        else
        {
            MotorLifecycle_CompleteCalibrationPublication(cal->life, epoch, true);
        }
    }
    BspSystem_ExitCritical(mask);
}
