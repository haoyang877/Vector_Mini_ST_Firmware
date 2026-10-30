#include "application_config.h"
#include "motor_sensing.h"

#include <math.h>

#include "application_config.h"
#include "motor_fault.h"
#include "application_config.h"
#include "bsp_adc.h"

/* 监督 ISR 更新、前台快照读取。 */
static volatile McuTemperatureTelemetry s_mcu_temperature;

/* 内部温度轮询、结果发布、失效超时与过温跳闸。
 * 板级可选策略只影响故障判定，绝不把缺失样本转换成有效温度。 */
void McuTemperature_Update(MotorMeasurement *measurement, MotorControlState *motor_state)
{
    struct
    {
        bool sample_ready, conversion_ok;
        uint32_t raw_ts, raw_vref;
        float celsius, vdda_mv;
    } poll = {0};
    bool good = false;

#if BSP_ENABLED_INTERNAL_ANALOG_SAMPLING
    poll.raw_ts = (uint32_t)BspAdc_ReadRawChannel(BSP_ADC_MCU_TEMPERATURE);
    poll.raw_vref = (uint32_t)BspAdc_ReadRawChannel(BSP_ADC_ANALOG_SUPPLY_VOLTAGE);
    poll.celsius = BspAdc_ReadChannel(BSP_ADC_MCU_TEMPERATURE);
    poll.vdda_mv = BspAdc_ReadChannel(BSP_ADC_ANALOG_SUPPLY_VOLTAGE) * 1000.0f;
    poll.sample_ready = true;
    poll.conversion_ok = isfinite(poll.celsius);
#endif
    if (poll.sample_ready)
    {
        s_mcu_temperature.raw_ts = poll.raw_ts;
        s_mcu_temperature.raw_vref = poll.raw_vref;
        good = poll.conversion_ok;
        if (good)
        {
            /* 0.01 @2 kHz 与原 1 kHz 的 0.02 等效，保持约 50 ms 时间常数。 */
            measurement->temperature_c =
                s_mcu_temperature.valid ? measurement->temperature_c +
                                              0.01f * (poll.celsius - measurement->temperature_c)
                                        : poll.celsius;
            s_mcu_temperature.raw_celsius = poll.celsius;
            s_mcu_temperature.vdda_mv = poll.vdda_mv;
            s_mcu_temperature.valid = 1U;
            s_mcu_temperature.missed_ticks = 0U;
            ++s_mcu_temperature.sample_count;
            /* 90°C 比后缀 6 器件的 105°C 结温上限低 15°C，为校准公差与测量
             * 误差留余量；不覆盖绕组/MOSFET 温度。 */
            if (BSP_REQUIRED_MCU_TEMPERATURE && poll.celsius >= MCU_TEMPERATURE_TRIP_C &&
                motor_state->fault_code == MOTOR_FAULT_NONE)
            {
                MotorFault_Report(motor_state, MOTOR_FAULT_OVERTEMPERATURE);
            }
        }
        else
        {
            s_mcu_temperature.valid = 0U;
            measurement->temperature_c = NAN;
        }
    }
    if (!good)
    {
        if (s_mcu_temperature.missed_ticks < MCU_TEMPERATURE_TIMEOUT_TICKS)
        {
            ++s_mcu_temperature.missed_ticks;
        }
        if (!s_mcu_temperature.valid ||
            s_mcu_temperature.missed_ticks >= MCU_TEMPERATURE_TIMEOUT_TICKS)
        {
            s_mcu_temperature.valid = 0U;
            measurement->temperature_c = NAN;
        }
        if (BSP_REQUIRED_MCU_TEMPERATURE &&
            s_mcu_temperature.missed_ticks >= MCU_TEMPERATURE_TIMEOUT_TICKS &&
            motor_state->fault_code == MOTOR_FAULT_NONE)
        {
            MotorFault_Report(motor_state, MOTOR_FAULT_TEMPERATURE_SENSOR);
        }
    }
}

bool McuTemperature_IsFresh(void)
{
    return s_mcu_temperature.valid != 0U &&
           s_mcu_temperature.missed_ticks < MCU_TEMPERATURE_TIMEOUT_TICKS;
}
