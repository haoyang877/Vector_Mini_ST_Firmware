#include "motor_calibration.h"
#include "bsp_system.h"
#include "motor_fault.h"
#include <math.h>

/* 低频使用的测量分支留在Flash，避免挤占正常控制快环SRAM。 */
__attribute__((noinline)) static MotorTaskResult step_resistance(MotorCalibration *calibration)
{
    PhaseResistance *test = &calibration->workspace.resistance;
    FocState *foc = calibration->drive.foc;
    float turns = (float)calibration->drive.encoder->linearized_q15 / 65536.0f;
    if (calibration->resistance_epoch != calibration->state_machine->epoch)
    {
        calibration->resistance_epoch = calibration->state_machine->epoch;
        PhaseResistance_Init(test,
                             calibration->drive.configuration->model.motor_phase_resistance_ohm,
                             calibration->drive.configuration->model.motor_d_inductance_h,
                             turns);
    }
    float currents[3] = {foc->phase_current_a.a, foc->phase_current_a.b, foc->phase_current_a.c};
    float duty[3];
    uint8_t mask =
        PhaseResistance_Step(test, currents, foc->bus_voltage_v, turns, 1.0f / FOC_FREQ, duty);
    const MotorPowerOutput *power = calibration->state_machine->drive.power;
    power->select_phases(mask);
    power->set_duty(duty[0], duty[1], duty[2]);
    return (MotorTaskResult){test->done ? MOTOR_TASK_COMPLETED : MOTOR_TASK_RUNNING,
                             MOTOR_FAULT_NONE};
}

MotorTaskResult MotorCalibration_StepFast(MotorCalibration *calibration)
{
    if (MotorStateMachine_GetCalibrationType(calibration->state_machine) ==
        MOTOR_CALIBRATION_PHASE_RESISTANCE)
    {
        return step_resistance(calibration);
    }
    const float motor_limit_rad_s =
        MOTOR_CALIBRATION_OUTPUT_SPEED_LIMIT_RAD_S * calibration->output->params.gear_ratio;
    const float measured_rad_s = Encoder_GetMechVel(calibration->drive.encoder);
    if (!isfinite(measured_rad_s))
    {
        return (MotorTaskResult){MOTOR_TASK_FAILED, MOTOR_FAULT_ENCODER};
    }
    if (fabsf(measured_rad_s) > motor_limit_rad_s)
    {
        return (MotorTaskResult){MOTOR_TASK_FAILED, MOTOR_FAULT_CALIBRATION_OVERSPEED};
    }
    switch (MotorStateMachine_GetCalibrationType(calibration->state_machine))
    {
    case MOTOR_CALIBRATION_ENCODER_LINEARIZATION:
        if (EncoderCalibration_GetPhase(&calibration->session) == CS_NULL)
        {
            calibration->output->ready = false;
#if BSP_ENABLED_LINEAR_HALL
            if (MotorAxisPreset_HasLinearHall(calibration->drive.configuration->preset))
            {
                LinearHall_CalibrationBegin(&calibration->hall_sweep);
                OutputEncoder_CalibBegin(&calibration->storage.output_calibration,
                                         calibration->output->params.gear_ratio);
            }
#endif
        }
        return EncoderCalibration_StepLut(
            &calibration->session, &calibration->drive, &calibration->workspace.encoder);
    case MOTOR_CALIBRATION_COMBINED_ELECTRICAL_ZERO:
    case MOTOR_CALIBRATION_ELECTRICAL_ZERO:
        return EncoderCalibration_StepElecZero(&calibration->session, &calibration->drive);
    case MOTOR_CALIBRATION_MECHANICAL_ZERO:
        if (!OutputEncoder_IsReady(calibration->output) ||
            !Encoder_IsOnline(calibration->drive.encoder))
        {
            return (MotorTaskResult){MOTOR_TASK_FAILED, MOTOR_FAULT_ENCODER};
        }
        /* 输出零位在关相发布时设置，不改变电机闭环坐标和控制器状态。 */
        return (MotorTaskResult){MOTOR_TASK_COMPLETED, MOTOR_FAULT_NONE};
    default:
        return (MotorTaskResult){MOTOR_TASK_FAILED, MOTOR_FAULT_INVALID_PARAMETERS};
    }
}

#if BSP_ENABLED_LINEAR_HALL
/* 前台只判断停稳和截止时间，LUT 交给所属轴的下一 ADC 拍安装。 */
static void request_lut_install(MotorCalibration *calibration, uint32_t epoch)
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
    if (now_ms - calibration->stable_since_ms >= 200U)
    {
        calibration->lut_request_epoch = epoch;
        return;
    }
    if (now_ms - calibration->coast_start_ms >= 6000U)
    {
        MotorStateMachine_CompleteCalibrationPublication(calibration->state_machine, epoch, false);
    }
}
#endif

void MotorCalibration_PublishLutFast(MotorCalibration *calibration)
{
    uint32_t requested = calibration->lut_request_epoch;
    calibration->lut_request_epoch = 0U;
    uint32_t epoch;
    /* 邮箱只承载 LUT 请求；同会话的阶段推进以安装成功为前提，无需再查类型。 */
    if (!MotorStateMachine_ReadPendingCalibrationPublication(calibration->state_machine, &epoch) ||
        epoch != requested || calibration->lut_published_epoch == epoch)
    {
        return;
    }
    if (!EncoderCalibration_PublishLut(&calibration->session,
                                       &calibration->drive.configuration->device.encoder,
                                       calibration->drive.encoder))
    {
        MotorStateMachine_CompleteCalibrationPublication(calibration->state_machine, epoch, false);
        return;
    }
    calibration->lut_published_epoch = epoch;
#if BSP_ENABLED_LINEAR_HALL
    if (MotorAxisPreset_HasLinearHall(calibration->drive.configuration->preset))
    {
        OutputEncoder_Init(calibration->output, calibration->output->params.gear_ratio);
        MotorStateMachine_ContinueElectricalZero(calibration->state_machine);
    }
#endif
}

void MotorCalibration_ProcessPublication(MotorCalibration *calibration)
{
    /* 此处只准备候选；完整会话及功率复核集中在 ADC 安装和 Flash 提交边界。 */
    if (calibration->state_machine->calibration_handoff != MOTOR_CALIBRATION_AWAITING_PUBLICATION)
    {
        return;
    }
    uint32_t mask = BspSystem_EnterControlCritical();
    uint32_t epoch = calibration->state_machine->epoch;
    MotorCalibrationType kind = MotorStateMachine_GetCalibrationType(calibration->state_machine);
    bool publishing_lut = kind == MOTOR_CALIBRATION_ENCODER_LINEARIZATION;
    if (publishing_lut && calibration->lut_published_epoch != epoch)
    {
#if BSP_ENABLED_LINEAR_HALL
        if (MotorAxisPreset_HasLinearHall(calibration->drive.configuration->preset))
        {
            if (calibration->lut_request_epoch != epoch)
            {
                request_lut_install(calibration, epoch);
            }
        }
        else
#endif
        {
            calibration->lut_request_epoch = epoch;
        }
        BspSystem_ExitControlCritical(mask);
        return;
    }
    OutputEncoder output = *calibration->output;
    if (publishing_lut)
    {
        OutputEncoder_Init(&output, output.params.gear_ratio);
    }
    else if (kind == MOTOR_CALIBRATION_MECHANICAL_ZERO)
    {
        OutputEncoder_Update(&output, calibration->drive.encoder->linearized_q15);
        OutputEncoder_SetZero(&output);
    }
#if BSP_ENABLED_LINEAR_HALL
    LinearHall *hall = &calibration->workspace.publication_hall;
    *hall = *calibration->hall;
#endif
    BspSystem_ExitControlCritical(mask);
    MotorFaultCode fault = MOTOR_FAULT_NONE;
#if BSP_ENABLED_LINEAR_HALL
    /* 候选结算允许控制中断抢占；即使 Hall 失败也结算输出轴，保留完整失败记录。 */
    if (kind == MOTOR_CALIBRATION_COMBINED_ELECTRICAL_ZERO)
    {
        bool hall_ok = LinearHall_CalibrationFinish(&calibration->hall_sweep, hall);
        bool output_ok = OutputEncoder_CalibEnd(&output,
                                                &calibration->storage.output_calibration,
                                                hall,
                                                &calibration->drive.configuration->device.encoder);
        if (!hall_ok)
        {
            fault = MOTOR_FAULT_LINEAR_HALL_CALIBRATION;
        }
        else if (!output_ok)
        {
            fault = MOTOR_FAULT_ENCODER;
        }
    }
#endif
    Param_Upload(&calibration->record->motor, calibration->drive.configuration);
#if BSP_ENABLED_LINEAR_HALL
    if (MotorAxisPreset_HasLinearHall(calibration->drive.configuration->preset))
    {
        Param_StoreLinearHall(&calibration->record->motor, hall);
    }
    Param_StoreOutputEncoder(calibration->record, &output, hall);
#else
    Param_StoreOutputEncoder(calibration->record, &output, NULL);
#endif
    mask = BspSystem_EnterCritical();
    uint32_t pending_epoch;
    /* 计算期间可被抢占；联合标定可在同一 epoch 内换阶段，提交仍须核对类型。 */
    if (MotorStateMachine_ReadPendingCalibrationPublication(calibration->state_machine,
                                                            &pending_epoch) &&
        pending_epoch == epoch &&
        MotorStateMachine_GetCalibrationType(calibration->state_machine) == kind)
    {
        *calibration->output = output;
#if BSP_ENABLED_LINEAR_HALL
        *calibration->hall = *hall;
#endif
        if (calibration->write_record(calibration->record, sizeof(*calibration->record)) != BSP_OK)
        {
            fault = MOTOR_FAULT_CALIBRATION_STORAGE;
            if (calibration->parameters != NULL)
            {
                calibration->parameters->saved_revision = 0U;
                calibration->parameters->save_status = PARAMETER_PERSIST_FAILED;
            }
        }
        else
        {
            /* 保存失败状态/上次系数也会改写记录，持久效果必须如实登记。 */
            if (calibration->parameters != NULL)
            {
                if (calibration->parameters->revision != UINT32_MAX)
                {
                    ++calibration->parameters->revision;
                }
                calibration->parameters->saved_revision = calibration->parameters->revision;
                calibration->parameters->stored_record_crc =
                    calibration->record->motor.record_crc32;
                calibration->parameters->save_status = PARAMETER_OK;
            }
            MotorStateMachine_CompleteCalibrationPublication(
                calibration->state_machine, epoch, true);
        }
        if (fault != MOTOR_FAULT_NONE)
        {
            MotorFault_Report(calibration->drive.motor, fault);
        }
    }
    BspSystem_ExitCritical(mask);
}
