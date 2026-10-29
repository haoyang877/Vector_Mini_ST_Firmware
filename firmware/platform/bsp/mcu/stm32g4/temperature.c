#include "bsp_internal.h"
#include "bsp_config.h"

#include <math.h>
#include "stm32g4xx_ll_adc.h"

/* STM32出厂30/130°C、3.0V校准；VDDA不依赖温度原码或温度校准资格。 */
static void convert_sample(const AdcTemperatureSample *sample,
                           uint16_t cal30,
                           uint16_t cal130,
                           uint16_t vref_cal,
                           AdcTemperatureValue *value)
{
    *value = (AdcTemperatureValue){NAN, NAN, BSP_NOT_CALIBRATED, BSP_NOT_CALIBRATED};
    if (vref_cal == 0U || vref_cal >= 4095U)
    {
        return;
    }
    value->voltage_result = BSP_IO_ERROR;
    if (sample->reference_counts == 0U || sample->reference_counts >= 4095U)
    {
        return;
    }
    const float supply_mv = 3000.0f * (float)vref_cal / (float)sample->reference_counts;
    if (supply_mv < 1620.0f || supply_mv > 3600.0f)
    {
        return;
    }
    value->vdda_v = supply_mv / 1000.0f;
    value->voltage_result = BSP_OK;
    if (sample->mcu_temperature_counts == 0U || sample->mcu_temperature_counts >= 4095U ||
        cal30 == 0U || cal130 >= 4095U || cal130 <= cal30)
    {
        return;
    }
    const float normalized =
        (float)sample->mcu_temperature_counts * (float)vref_cal / (float)sample->reference_counts;
    const float temperature =
        30.0f + (normalized - (float)cal30) * 100.0f / (float)(cal130 - cal30);
    if (isfinite(temperature) && temperature >= -40.0f && temperature <= 150.0f)
    {
        value->celsius = temperature;
        value->temperature_result = BSP_OK;
    }
}

void McuAdc_ConvertTemperature(const AdcTemperatureSample *sample, AdcTemperatureValue *value)
{
    convert_sample(sample, *TEMPSENSOR_CAL1_ADDR, *TEMPSENSOR_CAL2_ADDR, *VREFINT_CAL_ADDR, value);
}
