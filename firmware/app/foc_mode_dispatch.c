#include "foc_mode_dispatch.h"
#include <math.h>
#include "bus_voltage_profile.h"

#include "angle_feedback.h"
#include "foc_cogging_calibration.h"
#include "foc_encoder_calibration.h"
#include "foc_errhandle.h"
#include "foc_friction_identification.h"
#include "foc_param.h"
#include "foc_phase_resistance.h"
#include "foc_run.h"
#include "foc_run_state.h"
#include "fast_loop_profile.h"
#include "motor_hw.h"
#include "foc_sensorless_run.h"

/** 记录当前机械零位；成功后请求进入参数保存模式，失败时上报编码器故障。 */
static MotorWorkOutcome_TypeDef Task_SetMechanicalZero(Encoder_TypeDef *Encoder)
{
    MotorWorkOutcome_TypeDef outcome = {MOTOR_WORK_RUNNING, Motor_Disable, No_Error, false};

    if (!Encoder_SetMechanicalZero(Encoder))
    {
        outcome.result = MOTOR_WORK_FAULT;
        outcome.error = Encoder_Error;
        return outcome;
    }

    Task_Position_Mode_Reset();
    outcome.result = MOTOR_WORK_SWITCH_MODE;
    outcome.next_mode = Save_Param;
    return outcome;
}

MotorWorkOutcome_TypeDef FocMode_Dispatch(FOC_TypeDef *FOC,
                                          MotorControl_TypeDef *MotorControl,
                                          PI_Controller_TypeDef *PI_Speed,
                                          Encoder_TypeDef *OnBoard_Encoder,
                                          Fluxobserver_TypeDef *Fluxobserver,
                                          SensorlessStartup_TypeDef *SensorlessStartup)
{
    MotorWorkOutcome_TypeDef outcome = {MOTOR_WORK_RUNNING, Motor_Disable, No_Error, false};

    /* 模式切换或故障停机时释放编码器标定工作区，避免占用参数保存空间。 */
    if (MotorControl->ModeNow != Calib_EncoderObserver &&
        MotorControl->ModeNow != Calib_EleAngelOffset)
    {
        FocEncoderCalibration_Cancel();
    }

    /* 所有维护 worker 先受理再执行，参数修改和标定不能抢在生命周期准入前。 */
    switch (MotorControl->ModeNow)
    {
    case Calib_Anticogging:
    case Calib_Friction:
    case Calib_PhaseResistance:
    case Calib_EncoderObserver:
    case Calib_EleAngelOffset:
    case Set_ZeroPosition:
    case Default_Param:
        if (!FocRunState_IsMaintenanceActive(MotorControl->ModeNow))
        {
            return outcome;
        }
        break;
    default:
        break;
    }

    switch (MotorControl->ModeNow)
    {
    case Motor_Disable:
        PhaseResistanceMode_Cancel(FOC, MotorControl);
        /* 输出保持关闭。下一次使能前预置等占空比零矢量，
         * 避免首个电流采样窗口从 100% 预载跳变。 */
        motor_hw_pwm_set_duty(0.5f, 0.5f, 0.5f);
        break;

    case Current_Mode:
        Task_Current_Mode(FOC, MotorControl, OnBoard_Encoder, Fluxobserver);
        if (FocCogging_TakeTorqueTrip())
        {
            /* 台架保护跳闸：经结果协议停机，停相与模式回退由运行状态机执行。 */
            outcome.result = MOTOR_WORK_SWITCH_MODE;
            outcome.next_mode = Motor_Disable;
            outcome.power_off = true;
        }
        break;

    case Speed_Mode:
        FOC_Current(FOC,
                    MotorControl,
                    Encoder_GetElePhase(OnBoard_Encoder),
                    Encoder_GetEleVel(OnBoard_Encoder));
        break;

    case Position_Mode:
        FAST_PROFILE_BEGIN(FAST_PROFILE_POSITION_WITH_CURRENT);
        FOC_Current(FOC,
                    MotorControl,
                    Encoder_GetElePhase(OnBoard_Encoder),
                    Encoder_GetEleVel(OnBoard_Encoder));
        FAST_PROFILE_END(FAST_PROFILE_POSITION_WITH_CURRENT);
        break;

    case Calib_Anticogging:
        outcome = FocCogging_Task(FOC, MotorControl, PI_Speed, OnBoard_Encoder);
        break;

    case Calib_Friction:
        outcome = FocFrictionIdentification_Task(FOC, MotorControl, PI_Speed, OnBoard_Encoder);
        break;

    case Calib_PhaseResistance:
    {
        PhaseResistanceModeStatus_TypeDef status = PhaseResistanceMode_Run(FOC, MotorControl);

        if (status == PHASE_RESISTANCE_MODE_DONE)
        {
            outcome.result = MOTOR_WORK_STOP;
        }
        else if (status == PHASE_RESISTANCE_MODE_SETTLE_TIMEOUT)
        {
            outcome.result = MOTOR_WORK_FAULT;
            outcome.error = Large_Phase_Resistance;
        }
        else if (status == PHASE_RESISTANCE_MODE_INVALID_RESULT)
        {
            outcome.result = MOTOR_WORK_FAULT;
            outcome.error = MotorParam_Error;
        }
        else if (status == PHASE_RESISTANCE_MODE_UNDER_VOLTAGE)
        {
            outcome.result = MOTOR_WORK_FAULT;
            outcome.error = Under_Voltage;
        }
        else if (status == PHASE_RESISTANCE_MODE_OVER_VOLTAGE)
        {
            outcome.result = MOTOR_WORK_FAULT;
            outcome.error = Over_Voltage;
        }
        break;
    }

    case Calib_EncoderObserver:
        outcome = Task_Calib_EncoderObserver(
            FOC, MotorControl, PI_Speed, OnBoard_Encoder, Fluxobserver, SensorlessStartup);
        break;

    case Calib_EleAngelOffset:
        outcome = Task_Calib_EleAngelOffset(FOC, MotorControl, OnBoard_Encoder);
        break;

    case Vq_Mode:
        Task_Vq_Mode(FOC, MotorControl, OnBoard_Encoder);
        break;

    case Set_ZeroPosition:
        outcome = Task_SetMechanicalZero(OnBoard_Encoder);
        break;

    case Default_Param:
        /* 恢复默认参数不再隐式清除故障：清除统一走恢复矩阵（阶段 D）。 */
        Param_Return_Default(MotorControl, OnBoard_Encoder);
        outcome.result = MOTOR_WORK_SWITCH_MODE;
        outcome.next_mode = Save_Param;
        break;

    case Clear_Error:
        /* ModeNow==Clear_Error 为清除请求标记，由运行状态机按恢复矩阵处理。 */
        break;
    default:
        break;
    }

    return outcome;
}

bool FocMode_Admit(MotorControl_TypeDef *MotorControl,
                   const FOC_TypeDef *FOC,
                   const Encoder_TypeDef *OnBoard_Encoder,
                   ModeNow_TypeDef mode_set)
{
    /* 只允许当前实现的模式；删除的编号与未知值不能经内部调用绕过准入。 */
    switch (mode_set)
    {
    case Motor_Disable:
    case Current_Mode:
    case Speed_Mode:
    case Position_Mode:
    case Calib_Anticogging:
    case Set_ZeroPosition:
    case Default_Param:
    case Save_Param:
    case Clear_Error:
    case Calib_EncoderObserver:
    case Vq_Mode:
    case Calib_EleAngelOffset:
    case Calib_PhaseResistance:
    case Calib_Friction:
        break;
    default:
        return false;
    }
    /* 启相前拒绝电压不安全的请求，运行期间电压故障仍由监督路径锁存。 */
    if (MotorControl->ModeNow == Motor_Disable &&
        (mode_set == Current_Mode || mode_set == Speed_Mode || mode_set == Position_Mode ||
         mode_set == Calib_PhaseResistance || mode_set == Calib_EncoderOffset ||
         mode_set == Calib_EncoderObserver || mode_set == Calib_EleAngelOffset ||
         mode_set == Vq_Mode || mode_set == Calib_Anticogging || mode_set == Calib_Friction))
    {
        if (!isfinite(FOC->Vbus) || !isfinite(FOC->Vbus_filt) ||
            FOC->Vbus >= BUS_VOLTAGE_HARD_OVERVOLTAGE_V ||
            FOC->Vbus_filt > BUS_VOLTAGE_ENABLE_MAX_V)
        {
            if (MotorControl->ErrorNow == No_Error)
            {
                MotorFault_Report(MotorControl, Over_Voltage);
            }
            return false;
        }
        if (FOC->Vbus_filt < BUS_VOLTAGE_ENABLE_MIN_V)
        {
            if (MotorControl->ErrorNow == No_Error)
            {
                MotorFault_Report(MotorControl, Under_Voltage);
            }
            return false;
        }
    }
    /* 未知或未配置轴禁止输出转矩，诊断与显式存储请求仍可使用。 */
    if (!MotorControl->axis_profile_valid && mode_set != Motor_Disable && mode_set != Clear_Error &&
        mode_set != Save_Param)
    {
        MotorFault_Report(MotorControl, MotorParam_Error);
        return false;
    }
    if (mode_set == Position_Mode &&
        !MotorAxisProfile_AllowsPosition(&MotorControl->axis_profile,
                                         MotorControl->axis_profile_valid,
                                         Encoder_GetMecPos(OnBoard_Encoder),
                                         Encoder_GetMecPos(OnBoard_Encoder)))
    {
        MotorFault_Report(MotorControl, MotorParam_Error);
        return false;
    }
    if (mode_set == Calib_Anticogging && !FocCogging_CanStart(MotorControl, OnBoard_Encoder))
    {
        MotorFault_Report(MotorControl, CoggingCalibration_Error);
        return false;
    }
    if (mode_set == Calib_EleAngelOffset &&
        (Encoder_GetCalibFlag(OnBoard_Encoder) & ENC_CALIB_LINEARIZED) == 0U)
    {
        MotorFault_Report(MotorControl, Encoder_NotCalibrated);
        return false;
    }
    /* 使用编码器反馈的模式必须以有效传感器帧启动。 */
    if ((mode_set == Position_Mode || mode_set == Vq_Mode ||
         ((mode_set == Current_Mode || mode_set == Speed_Mode) &&
          MotorControl->isUseSensorless == false) ||
         mode_set == Calib_EncoderOffset || mode_set == Calib_EncoderObserver ||
         mode_set == Calib_EleAngelOffset || mode_set == Calib_Anticogging ||
         mode_set == Calib_Friction || mode_set == Set_ZeroPosition) &&
        !Encoder_IsOnline(OnBoard_Encoder))
    {
        MotorFault_Report(MotorControl, Encoder_Error);
        return false;
    }

    /* 编码器闭环要求必要标定已经完成。 */
    if (mode_set == Position_Mode || mode_set == Vq_Mode || mode_set == Calib_Anticogging ||
        mode_set == Calib_Friction ||
        ((mode_set == Current_Mode || mode_set == Speed_Mode) &&
         MotorControl->isUseSensorless == false))
    {
        if ((Encoder_GetCalibFlag(OnBoard_Encoder) & ENC_CALIB_ALL) != ENC_CALIB_ALL)
        {
            MotorFault_Report(MotorControl, Encoder_NotCalibrated);
            return false;
        }
    }

    if (MotorControl->ModeNow == Motor_Disable && MotorControl->ErrorNow == No_Error)
    {
        /* 位置模式以当前位置开始，禁止沿用历史目标。 */
        if (mode_set == Position_Mode)
        {
            float current_position = Encoder_GetMecPos(OnBoard_Encoder);

            if (!isfinite(current_position))
            {
                MotorFault_Report(MotorControl, Encoder_Error);
                return false;
            }
            MotorControl->posRef = current_position;
            MotorControl->posShadow = current_position;
            MotorControl->speedShadow = 0.0f;
            MotorControl->isReachTargetPos = false;
        }
        MotorControl->ModeNow = mode_set;
        return true;
    }

    return false;
}
