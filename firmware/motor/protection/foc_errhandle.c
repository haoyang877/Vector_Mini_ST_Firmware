#include "foc_errhandle.h"

#include <math.h>
#include "foc_algorithm.h"
#include "foc_pid.h"
#include "angle_feedback.h"
#include "foc_sensorless.h"
#include "foc_sensorless_run.h"
#include "position_impedance.h"
#include "foc_friction_identification.h"
#include "foc_cogging_calibration.h"
#include "bus_voltage_profile.h"

extern MotorControl_TypeDef MotorControl;
extern FOC_TypeDef FOC;
extern PI_Controller_TypeDef PI_Speed;
extern ModeNow_TypeDef ModeLast;
extern ErrorNow_TypeDef ErrorLast;
extern Encoder_TypeDef OnBoard_Encoder;
extern SensorlessStartup_TypeDef SensorlessStartup;

bool is_Mode_Error_Change;

/**
    * @brief  读取当前电机模式
    * @retval 当前电机模式
 **/
ModeNow_TypeDef Get_ModeNow(void)
{
    return MotorControl.ModeNow;
}

/**
    * @brief  设置当前电机模式
    * @param  tModeNow: 目标模式
 **/
void Set_ModeNow(ModeNow_TypeDef tModeNow)
{
    MotorControl.ModeNow = tModeNow;
}

/**
    * @brief  登记当前电机故障
    * @param  tErrorNow: 故障编号
 **/
void Set_ErrorNow(ErrorNow_TypeDef tErrorNow)
{
    MotorControl.ErrorNow = tErrorNow;
}

/**
    * @brief  清理运行控制数据
 **/
void Clear_RunningData(void)
{
    if (ModeLast == Calib_Anticogging)
    {
        FocCogging_Abort();
    }
    if (ModeLast == Calib_Friction)
    {
        FocFrictionIdentification_Abort(&MotorControl, &PI_Speed);
    }
    MotorControl.idRef = 0.0f;
    MotorControl.iqRef = 0.0f;
    MotorControl.vqRef = 0.0f;
    MotorControl.speedRef = 0.0f;
    MotorControl.speedShadow = 0.0f;
    MotorControl.posShadow = 0.0f;
    MotorControl.isReachTargetPos = false;
    MotorControl.pos_vel_filtered = 0.0f;
    /* 位置控制器状态复位；外环邮箱失效由 app 故障/模式提交点统一处理。 */
    PositionImpedance_Reset();

    FOC.Id = 0.0f;
    FOC.Iq = 0.0f;
    FOC_CurrentController_Reset(&FOC);
    PI_Controller_Reset(&PI_Speed);
    SensorlessStartup_Reset(&SensorlessStartup);
}

/**
    * @brief  校验并登记电机模式请求
    *         电机本体参数由外部配置，编码器闭环要求线性化和电角度零位标定
    * @param  mode_set: 目标模式
    * @retval 受理模式返回 true
 **/
bool ModeSwitch_Handle(ModeNow_TypeDef mode_set)
{
    /* Mode 13/15（观测器 LUT、电角度零位）已重建；其余 FOC-Calibration 任务
     * （R_L_Flux/EncoderOffset/CurrentOffset）仍待重建，模式编号保留 ABI。 */
    if (mode_set == Calib_Motor_R_L_Flux || mode_set == Calib_EncoderOffset ||
        mode_set == Calib_CurrentOffset)
    {
        return false;
    }
    /* 启相前拒绝电压不安全的请求，运行期间电压故障仍由监督路径锁存。 */
    if (MotorControl.ModeNow == Motor_Disable &&
        (mode_set == Current_Mode || mode_set == Speed_Mode || mode_set == Position_Mode ||
         mode_set == Position_Impedance_Mode || mode_set == Calib_PhaseResistance ||
         mode_set == Calib_EncoderOffset || mode_set == Calib_EncoderObserver ||
         mode_set == Calib_EleAngelOffset || mode_set == Voltage_OpenLoop || mode_set == Vq_Mode ||
         mode_set == Sensorless_Speed_Mode || mode_set == Calib_Anticogging ||
         mode_set == Calib_Friction))
    {
        if (!isfinite(FOC.Vbus) || !isfinite(FOC.Vbus_filt) ||
            FOC.Vbus >= BUS_VOLTAGE_HARD_OVERVOLTAGE_V || FOC.Vbus_filt > BUS_VOLTAGE_ENABLE_MAX_V)
        {
            if (MotorControl.ErrorNow == No_Error)
            {
                Set_ErrorNow(Over_Voltage);
            }
            return false;
        }
        if (FOC.Vbus_filt < BUS_VOLTAGE_ENABLE_MIN_V)
        {
            if (MotorControl.ErrorNow == No_Error)
            {
                Set_ErrorNow(Under_Voltage);
            }
            return false;
        }
    }
    /* 未知或未配置轴禁止输出转矩，诊断与显式存储请求仍可使用。 */
    if (!MotorControl.axis_profile_valid && mode_set != Motor_Disable && mode_set != Clear_Error &&
        mode_set != Save_Param)
    {
        Set_ErrorNow(MotorParam_Error);
        return false;
    }
    if (mode_set == Position_Mode &&
        !MotorAxisProfile_AllowsPosition(&MotorControl.axis_profile,
                                         MotorControl.axis_profile_valid,
                                         Encoder_GetMecPos(&OnBoard_Encoder),
                                         Encoder_GetMecPos(&OnBoard_Encoder)))
    {
        Set_ErrorNow(MotorParam_Error);
        return false;
    }
    if (mode_set == Calib_Anticogging && !FocCogging_CanStart(&MotorControl, &OnBoard_Encoder))
    {
        Set_ErrorNow(CoggingCalibration_Error);
        return false;
    }
    /* 电机本体辨识（R/L/磁链）已不再使用。 */
    if (mode_set == Calib_Motor_R_L_Flux)
    {
        return false;
    }

    /* 使用编码器反馈的模式必须以有效传感器帧启动。 */
    if ((mode_set == Position_Mode || mode_set == Position_Impedance_Mode || mode_set == Vq_Mode ||
         ((mode_set == Current_Mode || mode_set == Speed_Mode) &&
          MotorControl.isUseSensorless == false) ||
         mode_set == Calib_EncoderOffset || mode_set == Calib_EncoderObserver ||
         mode_set == Calib_EleAngelOffset || mode_set == Calib_Anticogging ||
         mode_set == Calib_Friction || mode_set == Set_ZeroPosition) &&
        !Encoder_IsOnline(&OnBoard_Encoder))
    {
        Set_ErrorNow(Encoder_Error);
        return false;
    }

    if (mode_set == Sensorless_Speed_Mode)
    {
        if (MotorControl.motor_pole_pairs <= 0 || MotorControl.motor_phase_resistance <= 0.0f ||
            MotorControl.motor_d_inductance <= 0.0f || MotorControl.motor_q_inductance <= 0.0f ||
            MotorControl.motor_flux <= 0.0f || MotorControl.current_limit <= 0.0f)
        {
            Set_ErrorNow(MotorParam_Error);
            return false;
        }
    }

    /* 编码器闭环要求必要标定已经完成。 */
    if (mode_set == Position_Mode || mode_set == Position_Impedance_Mode || mode_set == Vq_Mode ||
        mode_set == Calib_Anticogging || mode_set == Calib_Friction ||
        ((mode_set == Current_Mode || mode_set == Speed_Mode) &&
         MotorControl.isUseSensorless == false))
    {
        if ((Encoder_GetCalibFlag(&OnBoard_Encoder) & ENC_CALIB_ALL) != ENC_CALIB_ALL)
        {
            Set_ErrorNow(Encoder_NotCalibrated);
            return false;
        }
    }

    if (MotorControl.ModeNow == Motor_Disable && MotorControl.ErrorNow == No_Error)
    {
        /* 位置模式以当前位置开始，禁止沿用历史目标。 */
        if (mode_set == Position_Mode || mode_set == Position_Impedance_Mode)
        {
            float current_position = Encoder_GetMecPos(&OnBoard_Encoder);

            if (!isfinite(current_position))
            {
                Set_ErrorNow(Encoder_Error);
                return false;
            }
            MotorControl.posRef = current_position;
            MotorControl.posShadow = current_position;
            MotorControl.speedShadow = 0.0f;
            MotorControl.isReachTargetPos = false;
            PositionImpedance_Reset();
        }
        MotorControl.ModeNow = mode_set;
        return true;
    }

    if ((MotorControl.ModeNow == Current_Mode || MotorControl.ModeNow == Speed_Mode ||
         MotorControl.ModeNow == Position_Mode || MotorControl.ModeNow == Position_Impedance_Mode ||
         MotorControl.ModeNow == Calib_Motor_R_L_Flux ||
         MotorControl.ModeNow == Calib_PhaseResistance ||
         MotorControl.ModeNow == Calib_EncoderOffset ||
         MotorControl.ModeNow == Calib_EncoderObserver ||
         MotorControl.ModeNow == Calib_EleAngelOffset ||
         MotorControl.ModeNow == Calib_CurrentOffset || MotorControl.ModeNow == Voltage_OpenLoop ||
         MotorControl.ModeNow == Vq_Mode || MotorControl.ModeNow == Sensorless_Speed_Mode ||
         MotorControl.ModeNow == Calib_Friction || MotorControl.ModeNow == Calib_Anticogging) &&
        MotorControl.ErrorNow == No_Error)
    {
        if (mode_set == Motor_Disable)
        {
            if (MotorControl.ModeNow == Calib_Anticogging)
            {
                /* ADC 可抢占请求处理：先登记 STOP，防止清理后被旧目标重新启动。 */
                MotorControl.ModeNow = mode_set;
                FocCogging_Abort();
                return true;
            }
            if (MotorControl.ModeNow == Calib_Friction)
            {
                FocFrictionIdentification_Abort(&MotorControl, &PI_Speed);
            }
            MotorControl.ModeNow = mode_set;
            return true;
        }
    }

    if (mode_set == Clear_Error)
    {
        if (MotorControl.ErrorNow != No_Error)
        {
            /* 清除权限收归运行状态机：这里只登记请求，由快速环按恢复矩阵准入。 */
            MotorControl.ModeNow = Clear_Error;
            return true;
        }
    }

    return false;
}

/**
    * @brief  检测模式或故障变化
 **/
void Detect_Mode_Error_Change(void)
{
    if (ModeLast != MotorControl.ModeNow || ErrorLast != MotorControl.ErrorNow)
    {
        is_Mode_Error_Change = true;
    }
}

/**
    * @brief  读取模式或故障变化标志
    * @retval 变化标志
 **/
bool Return_Mode_Error_Change(void)
{
    return is_Mode_Error_Change;
}

/**
    * @brief  清除模式或故障变化标志
 **/
void Clear_Mode_Error_Change(void)
{
    is_Mode_Error_Change = false;
}
