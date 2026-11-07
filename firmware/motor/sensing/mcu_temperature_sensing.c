#include "application_config.h"
#include "motor_sensing.h"

#include <math.h>

#include "motor_fault.h"
#include "bsp_adc.h"

/* 监督 ISR 更新、前台快照读取。 */

/* 内部温度轮询、结果发布、失效超时与过温跳闸。
 * 板级可选策略只影响故障判定，绝不把缺失样本转换成有效温度。 */
void McuTemperature_Update(volatile McuTemperatureTelemetry *temperature,
                           FocState *foc,
                           MotorControlState *motor_state)
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
        temperature->raw_ts = poll.raw_ts;
        temperature->raw_vref = poll.raw_vref;
        good = poll.conversion_ok;
        if (good)
        {
            /* 0.01 @2 kHz 与原 1 kHz 的 0.02 等效，保持约 50 ms 时间常数。 */
            foc->temperature_c =
                temperature->valid
                    ? foc->temperature_c + 0.01f * (poll.celsius - foc->temperature_c)
                    : poll.celsius;
            temperature->raw_celsius = poll.celsius;
            temperature->vdda_mv = poll.vdda_mv;
            temperature->valid = 1U;
            temperature->missed_ticks = 0U;
            ++temperature->sample_count;
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
            temperature->valid = 0U;
            foc->temperature_c = NAN;
        }
    }
    if (!good)
    {
        if (temperature->missed_ticks < MCU_TEMPERATURE_TIMEOUT_TICKS)
        {
            ++temperature->missed_ticks;
        }
        if (!temperature->valid || temperature->missed_ticks >= MCU_TEMPERATURE_TIMEOUT_TICKS)
        {
            temperature->valid = 0U;
            foc->temperature_c = NAN;
        }
        if (BSP_REQUIRED_MCU_TEMPERATURE &&
            temperature->missed_ticks >= MCU_TEMPERATURE_TIMEOUT_TICKS &&
            motor_state->fault_code == MOTOR_FAULT_NONE)
        {
            MotorFault_Report(motor_state, MOTOR_FAULT_TEMPERATURE_SENSOR);
        }
    }
}

bool McuTemperature_IsFresh(const volatile McuTemperatureTelemetry *temperature)
{
    return temperature->valid != 0U && temperature->missed_ticks < MCU_TEMPERATURE_TIMEOUT_TICKS;
}
