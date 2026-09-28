#include "foc_errhandle.h"
#include <stddef.h>
#include <math.h>
#include "foc_sensing.h"
#include "motor_sensing.h"
#include "bus_voltage_profile.h"
#include "utils.h"

/* 保留既有 2 kHz 监督恢复计数 2000 拍（1 s），温度恢复阈值 80 °C。 */
#define RUN_STATE_OC_RECOVER_TICKS 2000U
#define RUN_STATE_TEMP_RECOVER_C 80.0f

/* 报告只锁存故障；清除和模式变化由 APP 生命周期所有者处理。 */
void MotorFault_Report(MotorControl_TypeDef *motor, ErrorNow_TypeDef error)
{
    if (motor != NULL && error != No_Error &&
        (motor->ErrorNow == No_Error || motor->ErrorNow == CAN_DisConnect))
    {
        motor->ErrorNow = error;
    }
}

void MotorFault_UpdateRecovery(MotorFaultRecovery *recovery,
                               const FOC_TypeDef *FOC,
                               ErrorNow_TypeDef error)
{
    if (error != Over_Current)
    {
        recovery->overcurrent_ticks = 0U;
        return;
    }
    if (fast_abs(FOC->Ia) < MOTOR_SENSING_OVERCURRENT_TRIP_A * 0.9f &&
        fast_abs(FOC->Ib) < MOTOR_SENSING_OVERCURRENT_TRIP_A * 0.9f &&
        fast_abs(FOC->Ic) < MOTOR_SENSING_OVERCURRENT_TRIP_A * 0.9f)
    {
        if (recovery->overcurrent_ticks < RUN_STATE_OC_RECOVER_TICKS)
        {
            ++recovery->overcurrent_ticks;
        }
        return;
    }
    recovery->overcurrent_ticks = 0U;
}
bool MotorFault_SourceRecovered(const MotorFaultRecovery *recovery,
                                const MotorControl_TypeDef *MotorControl,
                                const FOC_TypeDef *FOC,
                                const Encoder_TypeDef *OnBoard_Encoder,
                                ErrorNow_TypeDef error,
                                bool samples_fresh,
                                bool maintenance_released,
                                bool temperature_fresh)
{
    switch (error)
    {
    case Encoder_Error:
        return samples_fresh && Encoder_IsOnline(OnBoard_Encoder) &&
               Encoder_GetBadFrameStreak(OnBoard_Encoder) == 0U;
    case TemperatureSensor_Error:
        return temperature_fresh && isfinite(FOC->temp);
    case High_Temprature:
        return temperature_fresh && FOC->temp < RUN_STATE_TEMP_RECOVER_C;
    case Over_Voltage:
    case Under_Voltage:
        return (isfinite(FOC->Vbus_filt) && FOC->Vbus_filt >= BUS_VOLTAGE_ENABLE_MIN_V &&
                FOC->Vbus_filt <= BUS_VOLTAGE_ENABLE_MAX_V);
    case Over_Current:
        return recovery->overcurrent_ticks >= RUN_STATE_OC_RECOVER_TICKS;
    case CurrentOffset_Error:
        return MotorCurrentOffsets_Valid(MotorControl);
    case PolePairs_Error:
        return MotorControl->motor_pole_pairs > 0;
    case MotorParam_Error:
        return MotorControl->axis_profile_valid && MotorControl->motor_pole_pairs > 0 &&
               isfinite(MotorControl->current_limit) && MotorControl->current_limit > 0.0f;
    case ControlOverrun_Error:
        return samples_fresh;
    case Large_Phase_Resistance:
    case Large_Phase_Inductance:
    case Encoder_NotCalibrated:
    case Sensorless_Error:
    case FrictionIdentification_Error:
    case CoggingCalibration_Error:
        /* 终止型辨识故障由显式 CLEAR 确认；资源已释放才允许重试，各模式标定准入不变。 */
        return MotorControl->axis_profile_valid && maintenance_released;
    default:
        return false;
    }
}
