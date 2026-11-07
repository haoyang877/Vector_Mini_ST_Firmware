#include "motor_calibration.h"
#include "bsp_system.h"
#include "motor_fault.h"
#include <math.h>

MotorTaskResult MotorCalibration_StepFast(MotorCalibration *calibration)
{
    switch (MotorLifecycle_GetCalibrationType(calibration->lifecycle))
    {
    case MOTOR_CALIBRATION_ENCODER_LINEARIZATION:
        if (EncoderCalibration_GetPhase(&calibration->session) == CS_NULL)
        {
            calibration->output->ready = false;
#if BSP_ENABLED_LINEAR_HALL
            if (MotorAxisPreset_HasLinearHall(calibration->drive.configuration->preset))
            {
                LinearHall_CalibrationBegin(&calibration->hall_sweep);
                OutputEncoder_CalibBegin(&calibration->output_calibration,
                                         calibration->output->params.gear_ratio);
            }
#endif
        }
        return EncoderCalibration_StepLut(
            &calibration->session, &calibration->drive, &calibration->workspace);
    case MOTOR_CALIBRATION_JOINT_ELECTRICAL_ZERO:
    case MOTOR_CALIBRATION_ELECTRICAL_ZERO:
        return EncoderCalibration_StepElecZero(&calibration->session, &calibration->drive);
    case MOTOR_CALIBRATION_MECHANICAL_ZERO:
        if (!OutputEncoder_IsReady(calibration->output) ||
            !Encoder_SetMechanicalZero(&calibration->drive.configuration->device.encoder,
                                       calibration->drive.encoder))
        {
            return (MotorTaskResult){MOTOR_TASK_FAILED, MOTOR_FAULT_ENCODER};
        }
        MotorOuterLoop_RequestReset(calibration->outer);
        return (MotorTaskResult){MOTOR_TASK_COMPLETED, MOTOR_FAULT_NONE};
    default:
        return (MotorTaskResult){MOTOR_TASK_FAILED, MOTOR_FAULT_INVALID_PARAMETERS};
    }
}

#if BSP_ENABLED_LINEAR_HALL
/* 在原标定会话内等待停稳；调用者持有提交临界区。 */
static void publish_joint_lut_when_stopped(MotorCalibration *calibration, uint32_t epoch)
{
    uint32_t now_ms = BspSystem_TimeMs();
    if (epoch != calibration->coast_epoch)
    {
        calibration->coast_epoch = epoch;
        calibration->coast_start_ms = calibration->stable_since_ms = now_ms;
    }
    if (!(fabsf(Encoder_GetMechVelContinuous(calibration->drive.encoder)) < 1.0f))
    {
        calibration->stable_since_ms = now_ms;
    }
    if (now_ms - calibration->stable_since_ms < 200U)
    {
        if (now_ms - calibration->coast_start_ms >= 6000U)
        {
            MotorLifecycle_CompleteCalibrationPublication(calibration->lifecycle, epoch, false);
        }
        return;
    }
    if (EncoderCalibration_PublishLut(&calibration->session,
                                      &calibration->drive.configuration->device.encoder,
                                      calibration->drive.encoder))
    {
        OutputEncoder_Init(calibration->output, calibration->output->params.gear_ratio);
        MotorLifecycle_ContinueElectricalZero(calibration->lifecycle);
    }
    else
    {
        MotorLifecycle_CompleteCalibrationPublication(calibration->lifecycle, epoch, false);
    }
}
#endif

void MotorCalibration_ProcessPublication(MotorCalibration *calibration)
{
    uint32_t mask = BspSystem_EnterCritical();
    uint32_t epoch;
    if (!MotorLifecycle_ReadPendingCalibrationPublication(calibration->lifecycle, &epoch))
    {
        BspSystem_ExitCritical(mask);
        return;
    }
    MotorCalibrationType kind = MotorLifecycle_GetCalibrationType(calibration->lifecycle);
    bool publishing_lut = kind == MOTOR_CALIBRATION_ENCODER_LINEARIZATION;
#if BSP_ENABLED_LINEAR_HALL
    if (publishing_lut && MotorAxisPreset_HasLinearHall(calibration->drive.configuration->preset))
    {
        publish_joint_lut_when_stopped(calibration, epoch);
        BspSystem_ExitCritical(mask);
        return;
    }
#endif
    if (publishing_lut &&
        !EncoderCalibration_PublishLut(&calibration->session,
                                       &calibration->drive.configuration->device.encoder,
                                       calibration->drive.encoder))
    {
        MotorLifecycle_CompleteCalibrationPublication(calibration->lifecycle, epoch, false);
        BspSystem_ExitCritical(mask);
        return;
    }
    OutputEncoder output = *calibration->output;
    MotorFaultCode fault = MOTOR_FAULT_NONE;
    if (publishing_lut)
    {
        OutputEncoder_Init(&output, output.params.gear_ratio);
    }
    if (kind == MOTOR_CALIBRATION_MECHANICAL_ZERO)
    {
        OutputEncoder_Update(&output, calibration->drive.encoder->linearized_q15);
        OutputEncoder_SetZero(&output);
    }
#if BSP_ENABLED_LINEAR_HALL
    LinearHall hall = *calibration->hall;
    if (kind == MOTOR_CALIBRATION_JOINT_ELECTRICAL_ZERO &&
        !LinearHall_CalibrationFinish(&calibration->hall_sweep, &hall))
    {
        fault = MOTOR_FAULT_LINEAR_HALL_CALIBRATION;
    }
#endif
    BspSystem_ExitCritical(mask);
#if BSP_ENABLED_LINEAR_HALL
    /* 建表及相位拟合允许控制中断抢占；提交前仍须复核原会话。 */
    if (kind == MOTOR_CALIBRATION_JOINT_ELECTRICAL_ZERO &&
        !OutputEncoder_CalibEnd(&output,
                                &calibration->output_calibration,
                                &hall,
                                &calibration->drive.configuration->device.encoder) &&
        fault == MOTOR_FAULT_NONE)
    {
        fault = MOTOR_FAULT_ENCODER;
    }
#endif
    Param_Upload(&calibration->record->motor, calibration->drive.configuration);
#if BSP_ENABLED_LINEAR_HALL
    if (MotorAxisPreset_HasLinearHall(calibration->drive.configuration->preset))
    {
        Param_StoreLinearHall(&calibration->record->motor, &hall);
    }
    Param_StoreOutputEncoder(calibration->record, &output, &hall);
#else
    Param_StoreOutputEncoder(calibration->record, &output, NULL);
#endif
    mask = BspSystem_EnterCritical();
    uint32_t pending_epoch;
    if (MotorLifecycle_ReadPendingCalibrationPublication(calibration->lifecycle, &pending_epoch) &&
        pending_epoch == epoch)
    {
        *calibration->output = output;
#if BSP_ENABLED_LINEAR_HALL
        *calibration->hall = hall;
#endif
        BspResult saved =
            calibration->write_record(calibration->record, sizeof(*calibration->record));
        if (saved != BSP_OK || fault != MOTOR_FAULT_NONE)
        {
            MotorFault_Report(calibration->drive.motor,
                              saved != BSP_OK ? MOTOR_FAULT_CALIBRATION_STORAGE : fault);
        }
        else
        {
            MotorLifecycle_CompleteCalibrationPublication(calibration->lifecycle, epoch, true);
        }
    }
    BspSystem_ExitCritical(mask);
}
