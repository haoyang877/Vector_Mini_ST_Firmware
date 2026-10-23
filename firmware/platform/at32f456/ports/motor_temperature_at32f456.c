#include "motor_temperature_at32f456.h"

#include <math.h>
#include <stddef.h>
#include "r40_adc.h"
#include "../../config/boards/r40_analog.h"

/* ADC和上臂NTC共用3V3，按分压比换算，不依赖未经标定的内部参考源。
 * 故障码用于私有诊断；近电源轨只能判疑似开短路，不证明具体接线故障。 */
static bool s_ntc_seen;
static R40MotorTemperatureSample s_last_ntc;

static R40MotorTemperatureStatus convert_ntc(uint16_t counts, float *ohm, float *celsius)
{
    const float r25 = R40_MOTOR_NTC_R25_OHM;
    const float beta = R40_MOTOR_NTC_BETA_K;
    const float pulldown = R40_MOTOR_NTC_PULLDOWN_OHM;
    if (!isfinite(r25) || r25 <= 0.0f || !isfinite(beta) || beta <= 0.0f || !isfinite(pulldown) ||
        pulldown <= 0.0f)
    {
        return R40_MOTOR_TEMPERATURE_BAD_CONFIG;
    }
    if (counts > R40_ADC_MAX_COUNTS)
    {
        return R40_MOTOR_TEMPERATURE_OUT_OF_RANGE;
    }
    if (counts <= R40_MOTOR_NTC_RAIL_MARGIN_COUNTS)
    {
        return R40_MOTOR_TEMPERATURE_OPEN;
    }
    if (counts >= R40_ADC_MAX_COUNTS - R40_MOTOR_NTC_RAIL_MARGIN_COUNTS)
    {
        return R40_MOTOR_TEMPERATURE_SHORT;
    }
    float resistance = pulldown * (float)(R40_ADC_MAX_COUNTS - counts) / (float)counts;
    float inverse_kelvin = 1.0f / 298.15f + logf(resistance / r25) / beta;
    if (!isfinite(resistance) || resistance <= 0.0f || !isfinite(inverse_kelvin) ||
        inverse_kelvin <= 0.0f)
    {
        return R40_MOTOR_TEMPERATURE_OUT_OF_RANGE;
    }
    *ohm = resistance;
    float temperature = 1.0f / inverse_kelvin - 273.15f;
    if (!isfinite(temperature) || temperature < R40_MOTOR_NTC_MIN_C ||
        temperature > R40_MOTOR_NTC_MAX_C)
    {
        return R40_MOTOR_TEMPERATURE_OUT_OF_RANGE;
    }
    *celsius = temperature;
    return R40_MOTOR_TEMPERATURE_VALID;
}

bool R40_MotorTemperaturePoll(R40MotorTemperatureSample *result)
{
    if (result == NULL)
    {
        return false;
    }
    *result = (R40MotorTemperatureSample){0U, 0U, R40_MOTOR_TEMPERATURE_UNAVAILABLE, NAN, NAN};
    if (!R40_MOTOR_NTC_ENABLED)
    {
        result->status = R40_MOTOR_TEMPERATURE_DISABLED;
        return false;
    }
    R40AuxSamples sample;
    if (!R40_AdcReadAux(&sample))
    {
        return false;
    }
    if (s_ntc_seen && sample.sequence == s_last_ntc.sequence)
    {
        *result = s_last_ntc;
        return false;
    }
    result->sequence = sample.sequence;
    result->raw_counts = sample.motor_temperature_counts;
    result->status = convert_ntc(result->raw_counts, &result->resistance_ohm, &result->celsius);
    s_last_ntc = *result;
    s_ntc_seen = true;
    return true;
}
