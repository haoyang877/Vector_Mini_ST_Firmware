#include "foc_run_state.h"

#include "common_inc.h"
#include "motor_state.h"

/* 需要编码器反馈的模式在坏帧超限后立即置编码器故障。 */
static bool Encoder_FeedbackRequired(const MotorControl_TypeDef *MotorControl)
{
    if (MotorControl->ModeNow == Current_Mode)
    {
        return !MotorControl->isUseSensorless;
    }

    return MotorControl->ModeNow == Speed_Mode || MotorControl->ModeNow == Position_Mode ||
           MotorControl->ModeNow == Position_Impedance_Mode || MotorControl->ModeNow == Vq_Mode ||
           MotorControl->ModeNow == Calib_EncoderOffset ||
           MotorControl->ModeNow == Calib_EncoderObserver ||
           MotorControl->ModeNow == Calib_EleAngelOffset ||
           MotorControl->ModeNow == Calib_Friction || MotorControl->ModeNow == Calib_Anticogging ||
           MotorControl->ModeNow == Set_ZeroPosition;
}

void FocRunState_CheckFastFaults(void)
{
    if (Encoder_FeedbackRequired(&MotorControl) &&
        OnBoard_Encoder.bad_frame_streak >= ENCODER_BAD_FRAME_OFFLINE_COUNT)
    {
        Set_ErrorNow(Encoder_Error);
    }

    /* 同时覆盖内部模式赋值，不只通信请求。 */
    if (!MotorControl.axis_profile_valid && MotorControl.ModeNow != Motor_Disable &&
        MotorControl.ModeNow != Save_Param && MotorControl.ModeNow != Clear_Error)
    {
        Set_ErrorNow(MotorParam_Error);
        Set_ModeNow(Motor_Disable);
    }
}

void FocRunState_HandleFaultIndication(void)
{
    /* 无故障：按当前模式刷新指示。 */
    if (MotorControl.ErrorNow == No_Error)
    {
        LED_SetState(0, (uint8_t)MotorControl.ModeNow);
    }
    /* 有故障：必要时强制停机并刷新故障指示。 */
    else
    {
        /* 参数写入 Flash 期间容错保留当前模式，不强制停机。 */
        if (MotorControl.ModeNow != Save_Param && MotorControl.ModeNow != Default_Param)
        {
            MotorControl.ModeNow = Motor_Disable;
        }

        LED_SetState(1, (uint8_t)MotorControl.ErrorNow);
    }
}

bool FocRunState_ManagePowerStage(void)
{
    static bool position_start_prepared;
    bool defer_position_power_start = false;

    if (ModeLast != Motor_Disable && MotorControl.ModeNow == Motor_Disable)
    {
        Clear_RunningData();
        Stop_PWM_Generate();
    }

    if (ModeLast == Motor_Disable && MotorControl.ModeNow != Motor_Disable &&
        MotorControl.ModeNow != Save_Param && MotorControl.ModeNow != Default_Param &&
        MotorControl.ModeNow != Clear_Error && MotorControl.ModeNow != Set_ZeroPosition &&
        MotorControl.ModeNow != Calib_Anticogging && MotorControl.axis_profile_valid)
    {
        /* 模式 3 首个周期在功率输出关闭时完成控制器校验/初始化，
         * 下一个快速周期再使能，避免叠加两项启动开销。 */
        if ((MotorControl.ModeNow == Position_Mode || MotorControl.ModeNow == Speed_Mode) &&
            (!position_start_prepared || !MotorOuterLoop_IsReady()))
        {
            position_start_prepared = true;
            defer_position_power_start = true;
        }
        else
        {
            Start_PWM_Generate();
        }
    }
    if (!defer_position_power_start)
    {
        position_start_prepared = false;
    }

    return defer_position_power_start;
}

void FocRunState_CommitModeAndError(bool defer_position_power_start)
{
    Detect_Mode_Error_Change();

    if (!defer_position_power_start)
    {
        ModeLast = MotorControl.ModeNow;
    }
    ErrorLast = MotorControl.ErrorNow;

    MotorControl.ModeNow_f = MotorControl.ModeNow;
    MotorControl.ErrorNow_f = MotorControl.ErrorNow;
}
