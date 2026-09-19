#include "foc_mode_dispatch.h"

#include "common_inc.h"
#include "foc_cogging_calibration.h"
#include "foc_friction_identification.h"
#include "foc_phase_resistance.h"
#include "fast_loop_profile.h"
#include "motor_hw.h"
#include "motor_state.h"

/** 记录当前机械零位；成功后请求进入参数保存模式，失败时上报编码器故障。 */
static MotorWorkOutcome_TypeDef Task_SetMechanicalZero(MotorControl_TypeDef *MotorControl,
                                                       Encoder_TypeDef *Encoder)
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

MotorWorkOutcome_TypeDef FocMode_Dispatch(void)
{
    MotorWorkOutcome_TypeDef outcome = {MOTOR_WORK_RUNNING, Motor_Disable, No_Error, false};

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
         * 避免首个电流采样窗口从 100% 预载跳变。 */
        motor_hw_pwm_set_duty(0.5f, 0.5f, 0.5f);
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
        outcome = FocFrictionIdentification_Task(&FOC, &MotorControl, &PI_Speed, &OnBoard_Encoder);
        break;

    case Calib_Motor_R_L_Flux:
        Task_Calib_R_L_Flux(&FOC, &MotorControl);
        break;

    case Calib_PhaseResistance:
    {
        PhaseResistanceModeStatus_TypeDef status = PhaseResistanceMode_Run(&FOC, &MotorControl);

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

    case Calib_EncoderOffset:
        Task_Calib_EncoderOffset(&FOC, &MotorControl, &OnBoard_Encoder, &Fluxobserver);
        break;

    case Calib_EncoderObserver:
        outcome = Task_Calib_EncoderObserver(
            &FOC, &MotorControl, &PI_Speed, &OnBoard_Encoder, &Fluxobserver, &SensorlessStartup);
        break;

    case Calib_EleAngelOffset:
        Task_Calib_EleAngelOffset(&FOC, &MotorControl, &OnBoard_Encoder);
        break;

    case Calib_CurrentOffset:
        outcome = Task_Calib_CurrentOffset(&FOC, &MotorControl);
        break;

    case Voltage_OpenLoop:
        Task_Voltage_Mode(&FOC, &MotorControl);
        break;

    case Vq_Mode:
        Task_Vq_Mode(&FOC, &MotorControl, &OnBoard_Encoder);
        break;

    case Set_ZeroPosition:
        outcome = Task_SetMechanicalZero(&MotorControl, &OnBoard_Encoder);
        break;

    case Default_Param:
        /* 恢复默认参数不再隐式清除故障：清除统一走恢复矩阵（阶段 D）。 */
        Param_Return_Default();
        break;

    case Clear_Error:
        /* ModeNow==Clear_Error 为清除请求标记，由运行状态机按恢复矩阵处理。 */
        break;
    default:
        break;
    }

    return outcome;
}
