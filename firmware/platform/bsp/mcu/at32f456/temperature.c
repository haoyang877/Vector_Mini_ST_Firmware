#include "bsp_internal.h"
#include "bsp_config.h"

#include <math.h>

/* ADC1普通序列的两路12位原码；VDDA和温度独立判定，未经实板校准不授予温度资格。 */
void McuAdc_ConvertTemperature(const AdcTemperatureSample *sample, AdcTemperatureValue *value)
{
    const float reference_mv = R40_INTERNAL_REFERENCE_MV;
    *value = (AdcTemperatureValue){NAN, NAN, BSP_NOT_CALIBRATED, BSP_NOT_CALIBRATED};
    if (!isfinite(reference_mv) || reference_mv < 1100.0f || reference_mv > 1300.0f)
    {
        return;
    }
    value->voltage_result = BSP_IO_ERROR;
    if (sample->reference_counts == 0U || sample->reference_counts >= R40_ADC_MAX_COUNTS)
    {
        return;
    }
    const float supply_mv =
        reference_mv * (float)R40_ADC_MAX_COUNTS / (float)sample->reference_counts;
    if (!isfinite(supply_mv) || supply_mv < 2400.0f || supply_mv > 3600.0f)
    {
        return;
    }
    value->vdda_v = supply_mv / 1000.0f;
    value->voltage_result = BSP_OK;
#if R40_TEMPERATURE_CALIBRATED
    const float calibration_mv = R40_TEMPERATURE_CAL_MV;
    const float calibration_c = R40_TEMPERATURE_CAL_C;
    const float slope_mv_per_c = R40_TEMPERATURE_SLOPE_MV_PER_C;
    if (!isfinite(calibration_mv) || calibration_mv < 100.0f || calibration_mv > 2000.0f ||
        !isfinite(calibration_c) || calibration_c < -40.0f || calibration_c > 105.0f ||
        !isfinite(slope_mv_per_c) || fabsf(slope_mv_per_c) < 1.0f ||
        fabsf(slope_mv_per_c) > 10.0f || sample->mcu_temperature_counts == 0U ||
        sample->mcu_temperature_counts >= R40_ADC_MAX_COUNTS)
    {
        return;
    }
    /* 保留原换算的运算顺序，避免借用VDDA引入额外舍入。 */
    const float measured_mv =
        (float)sample->mcu_temperature_counts * reference_mv / (float)sample->reference_counts;
    const float temperature = calibration_c + (measured_mv - calibration_mv) / slope_mv_per_c;
    if (isfinite(temperature) && temperature >= -40.0f && temperature <= 150.0f)
    {
        value->celsius = temperature;
        value->temperature_result = BSP_OK;
    }
#endif
}

/* ADC和上臂NTC共用3V3，按分压比换算，不依赖未经标定的内部参考源。
 * 越轨或无效配置返回传感器错误，不把异常原码换成有效温度。 */
BspResult McuAdc_MotorTemperature(uint16_t counts, float *celsius)
{
    const float r25 = R40_MOTOR_NTC_R25_OHM;
    const float beta = R40_MOTOR_NTC_BETA_K;
    const float pulldown = R40_MOTOR_NTC_PULLDOWN_OHM;
    if (!isfinite(r25) || r25 <= 0.0f || !isfinite(beta) || beta <= 0.0f || !isfinite(pulldown) ||
        pulldown <= 0.0f)
    {
        return BSP_SENSOR_FAULT;
    }
    if (counts > R40_ADC_MAX_COUNTS)
    {
        return BSP_SENSOR_FAULT;
    }
    if (counts <= R40_MOTOR_NTC_RAIL_MARGIN_COUNTS)
    {
        return BSP_SENSOR_FAULT;
    }
    if (counts >= R40_ADC_MAX_COUNTS - R40_MOTOR_NTC_RAIL_MARGIN_COUNTS)
    {
        return BSP_SENSOR_FAULT;
    }
    float resistance = pulldown * (float)(R40_ADC_MAX_COUNTS - counts) / (float)counts;
    float inverse_kelvin = 1.0f / 298.15f + logf(resistance / r25) / beta;
    if (!isfinite(resistance) || resistance <= 0.0f || !isfinite(inverse_kelvin) ||
        inverse_kelvin <= 0.0f)
    {
        return BSP_SENSOR_FAULT;
    }
    float temperature = 1.0f / inverse_kelvin - 273.15f;
    if (!isfinite(temperature) || temperature < R40_MOTOR_NTC_MIN_C ||
        temperature > R40_MOTOR_NTC_MAX_C)
    {
        return BSP_SENSOR_FAULT;
    }
    *celsius = temperature;
    return BSP_OK;
}
