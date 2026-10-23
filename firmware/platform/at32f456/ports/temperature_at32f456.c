#include "motor_sensing.h"

#include <math.h>
#include <stddef.h>
#include "r40_adc.h"
#include "../../config/boards/r40_analog.h"

/* ADC1普通序列的两路12位数据；绝不读取STM32校准地址，也不将NTC当作MCU温度。
 * 实板校准尚未提供时继续上报原始样本，明确让既有保护识别conversion_ok=false。 */
static bool s_seen;
static uint32_t s_last_sequence;

typedef struct
{
    float reference_mv, calibration_mv, calibration_c, slope_mv_per_c;
} R40TemperatureCalibration;

static const R40TemperatureCalibration s_calibration = {R40_INTERNAL_REFERENCE_MV,
                                                        R40_TEMPERATURE_CAL_MV,
                                                        R40_TEMPERATURE_CAL_C,
                                                        R40_TEMPERATURE_SLOPE_MV_PER_C};
static bool convert_sample(uint16_t raw_ts,
                           uint16_t raw_ref,
                           const R40TemperatureCalibration *calibration,
                           float *celsius,
                           float *vdda_mv)
{
    const float reference_mv = calibration->reference_mv;
    const float calibration_mv = calibration->calibration_mv;
    const float calibration_c = calibration->calibration_c;
    const float slope_mv_per_c = calibration->slope_mv_per_c;
    if (!isfinite(reference_mv) || reference_mv < 1100.0f || reference_mv > 1300.0f ||
        !isfinite(calibration_mv) || calibration_mv < 100.0f || calibration_mv > 2000.0f ||
        !isfinite(calibration_c) || calibration_c < -40.0f || calibration_c > 105.0f ||
        !isfinite(slope_mv_per_c) || fabsf(slope_mv_per_c) < 1.0f ||
        fabsf(slope_mv_per_c) > 10.0f || raw_ts == 0U || raw_ts >= R40_ADC_MAX_COUNTS ||
        raw_ref == 0U || raw_ref >= R40_ADC_MAX_COUNTS)
    {
        return false;
    }
    float supply = reference_mv * (float)R40_ADC_MAX_COUNTS / (float)raw_ref;
    float measured_mv = (float)raw_ts * reference_mv / (float)raw_ref;
    float temperature = calibration_c + (measured_mv - calibration_mv) / slope_mv_per_c;
    if (!isfinite(supply) || supply < 2400.0f || supply > 3600.0f || !isfinite(temperature) ||
        temperature < -40.0f || temperature > 150.0f)
    {
        return false;
    }
    *celsius = temperature;
    *vdda_mv = supply;
    return true;
}

static bool convert_temperature(uint16_t raw_ts, uint16_t raw_ref, float *celsius, float *vdda_mv)
{
    return R40_TEMPERATURE_CALIBRATED &&
           convert_sample(raw_ts, raw_ref, &s_calibration, celsius, vdda_mv);
}

void motor_hw_temperature_poll(MotorHwTemperaturePoll *result)
{
    if (result == NULL)
    {
        return;
    }
    result->sample_ready = false;
    result->conversion_ok = false;
    if (!R40_TEMPERATURE_SENSING_ENABLED)
    {
        return;
    }
    R40TemperatureSamples sample;
    if (!R40_AdcReadTemperature(&sample))
    {
        return;
    }
    if (s_seen && sample.sequence == s_last_sequence)
    {
        return;
    }
    s_seen = true;
    s_last_sequence = sample.sequence;
    result->sample_ready = true;
    result->raw_ts = sample.mcu_temperature_counts;
    result->raw_vref = sample.reference_counts;
    result->conversion_ok = convert_temperature(
        sample.mcu_temperature_counts, sample.reference_counts, &result->celsius, &result->vdda_mv);
}
