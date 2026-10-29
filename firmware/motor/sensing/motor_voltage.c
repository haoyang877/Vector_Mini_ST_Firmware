#include "application_config.h"
#include "motor_sensing.h"

#include <math.h>

#include "application_config.h"
#include "application_config.h"
#include "motor_fault.h"
#include "bsp_adc.h"
#include "control_math.h"

/* 母线电压采样、滤波与过压/欠压确认计数。 */
void MotorVoltage_Update(MotorMeasurement *measurement,
                         MotorControlState *motor_state,
                         bool output_expected,
                         const BspAdcFastSample *sample)
{
    static uint32_t overvoltage_count, undervoltage_count, hard_overvoltage_count;

    measurement->bus_voltage_v =
        sample != NULL && (sample->valid_mask & BSP_ADC_FAST_BUS_VOLTAGE_VALID) != 0U
            ? sample->bus_voltage_v
            : NAN;

    UTILS_LP_FAST(measurement->bus_voltage_filtered_v, measurement->bus_voltage_v, 0.05f);

    if (output_expected)
    {
        /* 首个故障保持锁存；接近硬件电压上限时原始采样绕过低通，
         * 滤波阈值只用于拒绝短时跌落。 */
        if (motor_state->fault_code != MOTOR_FAULT_NONE)
        {
            return;
        }
        if (!isfinite(measurement->bus_voltage_v) || !isfinite(measurement->bus_voltage_filtered_v))
        {
            MotorFault_Report(motor_state, MOTOR_FAULT_OVERVOLTAGE);
            return;
        }
        if (measurement->bus_voltage_v >= BUS_VOLTAGE_HARD_OVERVOLTAGE_V)
        {
            if (hard_overvoltage_count < BUS_VOLTAGE_HARD_CONFIRM_CYCLES)
            {
                ++hard_overvoltage_count;
            }
            if (hard_overvoltage_count >= BUS_VOLTAGE_HARD_CONFIRM_CYCLES)
            {
                MotorFault_Report(motor_state, MOTOR_FAULT_OVERVOLTAGE);
                return;
            }
        }
        else
        {
            hard_overvoltage_count = 0U;
        }
        if (measurement->bus_voltage_filtered_v >= BUS_VOLTAGE_OVERVOLTAGE_V)
        {
            if (overvoltage_count < (FOC_FREQ * BUS_VOLTAGE_OVERVOLTAGE_MS / 1000U))
            {
                ++overvoltage_count;
            }
            if (overvoltage_count >= (FOC_FREQ * BUS_VOLTAGE_OVERVOLTAGE_MS / 1000U))
            {
                MotorFault_Report(motor_state, MOTOR_FAULT_OVERVOLTAGE);
            }
        }
        else
        {
            overvoltage_count = 0U;
        }

        /* 欠压保护。 */
        if (measurement->bus_voltage_filtered_v <= BUS_VOLTAGE_UNDERVOLTAGE_V)
        {
            if (undervoltage_count < (FOC_FREQ * BUS_VOLTAGE_UNDERVOLTAGE_MS / 1000U))
            {
                ++undervoltage_count;
            }
            if (undervoltage_count >= (FOC_FREQ * BUS_VOLTAGE_UNDERVOLTAGE_MS / 1000U))
            {
                MotorFault_Report(motor_state, MOTOR_FAULT_UNDERVOLTAGE);
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
