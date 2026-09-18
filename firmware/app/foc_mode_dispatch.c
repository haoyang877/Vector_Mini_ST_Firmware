#include "foc_mode_dispatch.h"

#include "common_inc.h"
#include "foc_cogging_calibration.h"
#include "foc_friction_identification.h"
#include "foc_phase_resistance.h"
#include "fast_loop_profile.h"
#include "motor_state.h"

/** 记录当前机械零位并切换到参数保存模式；编码器失败时置故障。 */
static void Task_SetMechanicalZero(MotorControl_TypeDef *MotorControl, Encoder_TypeDef *Encoder)
{
    if (!Encoder_SetMechanicalZero(Encoder))
    {
        Set_ErrorNow(Encoder_Error);
        return;
    }

    Task_Position_Mode_Reset();
    Set_ModeNow(Save_Param);
}

void FocMode_Dispatch(void)
{
    /* 退出齿槽转矩标定模式时立即释放补偿状态，避免残留力矩指令。 */
    if (ModeLast == Calib_Anticogging && MotorControl.ModeNow != Calib_Anticogging)
    {
        FocCogging_Abort();
    }

    switch (MotorControl.ModeNow)
    {
    case Motor_Disable:
        PhaseResistanceMode_Cancel(&FOC, &MotorControl);
        /* 输出保持关闭。下一次使能前预置等占空比零矢量，
         * 避免首个电流采样窗口从 100% 预载跳变；沿用现有 PWM 适配器。 */
        Set_A_Duty(0.5f);
        Set_B_Duty(0.5f);
        Set_C_Duty(0.5f);
        break;

    case Current_Mode:
        Task_Current_Mode(&FOC, &MotorControl, &OnBoard_Encoder, &Fluxobserver);
        break;

    case Speed_Mode:
        FOC_Current(&FOC,
                    &MotorControl,
                    Encoder_GetElePhase(&OnBoard_Encoder),
                    Encoder_GetEleVel(&OnBoard_Encoder));
        break;

    case Sensorless_Speed_Mode:
        Task_Sensorless_Speed_Mode(&FOC,
                                   &MotorControl,
                                   &PI_Speed,
                                   &Fluxobserver,
                                   &SensorlessStartup,
                                   &SensorlessStartup_DefaultConfig);
        break;

    case Position_Mode:
        FAST_PROFILE_BEGIN(FAST_PROFILE_POSITION_WITH_CURRENT);
        FOC_Current(&FOC,
                    &MotorControl,
                    Encoder_GetElePhase(&OnBoard_Encoder),
                    Encoder_GetEleVel(&OnBoard_Encoder));
        FAST_PROFILE_END(FAST_PROFILE_POSITION_WITH_CURRENT);
        break;

    case Position_Impedance_Mode:
        Task_Position_Impedance_Mode(&FOC, &MotorControl, &OnBoard_Encoder);
        break;

    case Calib_Anticogging:
        FocCogging_Task(&FOC, &MotorControl, &PI_Speed, &OnBoard_Encoder);
        break;

    case Calib_Friction:
        FocFrictionIdentification_Task(&FOC, &MotorControl, &PI_Speed, &OnBoard_Encoder);
        break;

    case Calib_Motor_R_L_Flux:
        Task_Calib_R_L_Flux(&FOC, &MotorControl);
        break;

    case Calib_PhaseResistance:
    {
        PhaseResistanceModeStatus_TypeDef status = PhaseResistanceMode_Run(&FOC, &MotorControl);

        if (status == PHASE_RESISTANCE_MODE_DONE)
        {
            Set_ModeNow(Motor_Disable);
        }
        else if (status == PHASE_RESISTANCE_MODE_SETTLE_TIMEOUT)
        {
            Set_ErrorNow(Large_Phase_Resistance);
        }
        else if (status == PHASE_RESISTANCE_MODE_INVALID_RESULT)
        {
            Set_ErrorNow(MotorParam_Error);
        }
        else if (status == PHASE_RESISTANCE_MODE_UNDER_VOLTAGE)
        {
            Set_ErrorNow(Under_Voltage);
        }
        else if (status == PHASE_RESISTANCE_MODE_OVER_VOLTAGE)
        {
            Set_ErrorNow(Over_Voltage);
        }
        break;
    }

    case Calib_EncoderOffset:
        Task_Calib_EncoderOffset(&FOC, &MotorControl, &OnBoard_Encoder, &Fluxobserver);
        break;

    case Calib_EncoderObserver:
        Task_Calib_EncoderObserver(
            &FOC, &MotorControl, &PI_Speed, &OnBoard_Encoder, &Fluxobserver, &SensorlessStartup);
        break;

    case Calib_EleAngelOffset:
        Task_Calib_EleAngelOffset(&FOC, &MotorControl, &OnBoard_Encoder);
        break;

    case Calib_CurrentOffset:
        Task_Calib_CurrentOffset(&FOC, &MotorControl);
        break;

    case Voltage_OpenLoop:
        Task_Voltage_Mode(&FOC, &MotorControl);
        break;

    case Vq_Mode:
        Task_Vq_Mode(&FOC, &MotorControl, &OnBoard_Encoder);
        break;

    case Set_ZeroPosition:
        Task_SetMechanicalZero(&MotorControl, &OnBoard_Encoder);
        break;

    case Default_Param:
        Param_Return_Default();

    /* 有意贯穿到 Clear_Error：恢复默认参数后清除故障。 */
    case Clear_Error:
        Set_ErrorNow(No_Error);
    default:
        break;
    }
}
