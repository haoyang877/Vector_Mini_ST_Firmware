#include "foc_sensing.h"

#include <math.h>

#include "bus_voltage_profile.h"
#include "control_config.h"
#include "foc_errhandle.h"
#include "mcu_temperature.h"
#include "motor_sensing.h"
#include "utils.h"

/* 母线电压、相电流与 MCU 温度的判定、滤波与遥测。
 * 原始采样与板级换算在 platform/stm32g4/ports/motor/motor_sensing_stm32g4.c
 * 完成，换算常量由 platform/api/motor_sensing.h 拥有；本文件不访问寄存器。 */

#define OVERCURRENT_CONFIRM_CYCLES 5U

/* 零电流时 ADC 中点应落在 2048 附近；越界即认为零点偏置标定失效。 */
#define CURRENT_OFFSET_MIN_COUNTS 1948.0f
#define CURRENT_OFFSET_MAX_COUNTS 2148.0f

extern MotorControl_TypeDef MotorControl;
extern FOC_TypeDef FOC;
volatile McuTemperatureTelemetry McuTemperature;

/* 母线电压采样、滤波与过压/欠压确认计数。 */
void Vbus_Update(FOC_TypeDef *FOC, MotorControl_TypeDef *MotorControl)
{
    static uint32_t overvoltage_count, undervoltage_count, hard_overvoltage_count;

    FOC->Vbus = (float)motor_hw_vbus_sample_raw() * MOTOR_SENSING_ADC_SUM_TO_COUNTS *
                MOTOR_SENSING_VBUS_V_PER_COUNT;

    UTILS_LP_FAST(FOC->Vbus_filt, FOC->Vbus, 0.05f);

    if (MotorControl->ModeNow == Current_Mode || MotorControl->ModeNow == Speed_Mode ||
        MotorControl->ModeNow == Position_Mode ||
        MotorControl->ModeNow == Position_Impedance_Mode ||
        MotorControl->ModeNow == Calib_Motor_R_L_Flux ||
        MotorControl->ModeNow == Calib_PhaseResistance ||
        MotorControl->ModeNow == Calib_EncoderOffset ||
        MotorControl->ModeNow == Calib_EncoderObserver ||
        MotorControl->ModeNow == Calib_EleAngelOffset || MotorControl->ModeNow == Vq_Mode ||
        MotorControl->ModeNow == Voltage_OpenLoop ||
        MotorControl->ModeNow == Sensorless_Speed_Mode ||
        MotorControl->ModeNow == Calib_Anticogging || MotorControl->ModeNow == Calib_Friction)
    {
        /* 首个故障保持锁存；接近硬件电压上限时原始采样绕过低通，
         * 滤波阈值只用于拒绝短时跌落。 */
        if (MotorControl->ErrorNow != No_Error)
        {
            return;
        }
        if (!isfinite(FOC->Vbus) || !isfinite(FOC->Vbus_filt))
        {
            Set_ErrorNow(Over_Voltage);
            return;
        }
        if (FOC->Vbus >= BUS_VOLTAGE_HARD_OVERVOLTAGE_V)
        {
            if (hard_overvoltage_count < BUS_VOLTAGE_HARD_CONFIRM_CYCLES)
            {
                ++hard_overvoltage_count;
            }
            if (hard_overvoltage_count >= BUS_VOLTAGE_HARD_CONFIRM_CYCLES)
            {
                Set_ErrorNow(Over_Voltage);
                return;
            }
        }
        else
        {
            hard_overvoltage_count = 0U;
        }
        if (FOC->Vbus_filt >= BUS_VOLTAGE_OVERVOLTAGE_V)
        {
            if (overvoltage_count < (FOC_FREQ * BUS_VOLTAGE_OVERVOLTAGE_MS / 1000U))
            {
                ++overvoltage_count;
            }
            if (overvoltage_count >= (FOC_FREQ * BUS_VOLTAGE_OVERVOLTAGE_MS / 1000U))
            {
                Set_ErrorNow(Over_Voltage);
            }
        }
        else
        {
            overvoltage_count = 0U;
        }

        /* 欠压保护。 */
        if (FOC->Vbus_filt <= BUS_VOLTAGE_UNDERVOLTAGE_V)
        {
            if (undervoltage_count < (FOC_FREQ * BUS_VOLTAGE_UNDERVOLTAGE_MS / 1000U))
            {
                ++undervoltage_count;
            }
            if (undervoltage_count >= (FOC_FREQ * BUS_VOLTAGE_UNDERVOLTAGE_MS / 1000U))
            {
                Set_ErrorNow(Under_Voltage);
            }
        }
        else
        {
            undervoltage_count = 0U;
        }
    }
    else
    {
        overvoltage_count = 0U;
        undervoltage_count = 0U;
        hard_overvoltage_count = 0U;
    }
}

/* 三相电流换算、零点偏置有效性与过流确认计数。 */
void Current_Cal(FOC_TypeDef *FOC, MotorControl_TypeDef *MotorControl)
{
    static uint8_t overcurrent_count;

    if (!isfinite(MotorControl->A_Offset) || !isfinite(MotorControl->B_Offset) ||
        !isfinite(MotorControl->C_Offset) || MotorControl->A_Offset < CURRENT_OFFSET_MIN_COUNTS ||
        MotorControl->A_Offset > CURRENT_OFFSET_MAX_COUNTS ||
        MotorControl->B_Offset < CURRENT_OFFSET_MIN_COUNTS ||
        MotorControl->B_Offset > CURRENT_OFFSET_MAX_COUNTS ||
        MotorControl->C_Offset < CURRENT_OFFSET_MIN_COUNTS ||
        MotorControl->C_Offset > CURRENT_OFFSET_MAX_COUNTS)
    {
        Set_ErrorNow(CurrentOffset_Error);
    }
    else
    {
        FOC->Ia = -((float)motor_hw_current_sample_raw(MOTOR_HW_CURRENT_PHASE_A) *
                        MOTOR_SENSING_ADC_SUM_TO_COUNTS -
                    MotorControl->A_Offset) *
                  MOTOR_SENSING_CURRENT_A_PER_COUNT;
        FOC->Ib = -((float)motor_hw_current_sample_raw(MOTOR_HW_CURRENT_PHASE_B) *
                        MOTOR_SENSING_ADC_SUM_TO_COUNTS -
                    MotorControl->B_Offset) *
                  MOTOR_SENSING_CURRENT_A_PER_COUNT;
        FOC->Ic = -((float)motor_hw_current_sample_raw(MOTOR_HW_CURRENT_PHASE_C) *
                        MOTOR_SENSING_ADC_SUM_TO_COUNTS -
                    MotorControl->C_Offset) *
                  MOTOR_SENSING_CURRENT_A_PER_COUNT;
    }

    if (fast_abs(FOC->Ia) > MOTOR_SENSING_OVERCURRENT_TRIP_A ||
        fast_abs(FOC->Ib) > MOTOR_SENSING_OVERCURRENT_TRIP_A ||
        fast_abs(FOC->Ic) > MOTOR_SENSING_OVERCURRENT_TRIP_A)
    {
        if (overcurrent_count < OVERCURRENT_CONFIRM_CYCLES)
        {
            overcurrent_count++;
        }
        if (overcurrent_count >= OVERCURRENT_CONFIRM_CYCLES)
        {
            Set_ErrorNow(Over_Current);
        }
    }
    else
    {
        overcurrent_count = 0U;
    }
}

/* 内部温度轮询、结果发布、失效超时与过温跳闸。 */
void Temperature_Update(FOC_TypeDef *FOC)
{
    MotorHwTemperaturePoll_TypeDef poll;
    bool good = false;

    /* 软件触发的 ADC1 序列在两次 1 kHz 监督调用之间完成：端口在 JEOS 就绪时
     * 读取两个 rank、清标志并请求下一次序列。ADC2/PWM 电流采样与 20 kHz
     * 中断不受影响。 */
    motor_hw_temperature_poll(&poll);
    if (poll.sample_ready)
    {
        McuTemperature.raw_ts = poll.raw_ts;
        McuTemperature.raw_vref = poll.raw_vref;
        good = poll.conversion_ok;
        if (good)
        {
            FOC->temp = McuTemperature.valid ? FOC->temp + 0.02f * (poll.celsius - FOC->temp)
                                             : poll.celsius;
            McuTemperature.raw_celsius = poll.celsius;
            McuTemperature.vdda_mv = poll.vdda_mv;
            McuTemperature.valid = 1U;
            McuTemperature.missed_ms = 0U;
            ++McuTemperature.sample_count;
            /* 90°C 比后缀 6 器件的 105°C 结温上限低 15°C，为校准公差与测量
             * 误差留余量；不覆盖绕组/MOSFET 温度。 */
            if (poll.celsius >= MCU_TEMPERATURE_TRIP_C && MotorControl.ErrorNow == No_Error)
            {
                Set_ErrorNow(High_Temprature);
            }
        }
        else
        {
            McuTemperature.valid = 0U;
            FOC->temp = NAN;
        }
    }
    if (!good)
    {
        if (McuTemperature.missed_ms < MCU_TEMPERATURE_TIMEOUT_MS)
        {
            ++McuTemperature.missed_ms;
        }
        if (!McuTemperature.valid || McuTemperature.missed_ms >= MCU_TEMPERATURE_TIMEOUT_MS)
        {
            McuTemperature.valid = 0U;
            FOC->temp = NAN;
        }
        if (McuTemperature.missed_ms >= MCU_TEMPERATURE_TIMEOUT_MS &&
            MotorControl.ErrorNow == No_Error)
        {
            Set_ErrorNow(TemperatureSensor_Error);
        }
    }
}
