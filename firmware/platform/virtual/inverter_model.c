#include "inverter_model.h"
#include <math.h>
#include <stddef.h>

/* 当前板级比较极性的平均逆变器；去掉共模电压，不替代栅极驱动或死区分析。 */

bool InverterModel_Apply(const float duty[3], float bus_voltage_v, bool enabled, float voltage_v[3])
{
    if (duty == NULL || voltage_v == NULL || !isfinite(bus_voltage_v) || bus_voltage_v < 0.0f)
    {
        return false;
    }
    float mean = 0.0f;
    for (unsigned phase = 0U; phase < 3U; ++phase)
    {
        if (!isfinite(duty[phase]) || duty[phase] < 0.0f || duty[phase] > 1.0f)
        {
            return false;
        }
        mean += duty[phase] / 3.0f;
    }
    for (unsigned phase = 0U; phase < 3U; ++phase)
    {
        voltage_v[phase] = enabled ? bus_voltage_v * (mean - duty[phase]) : 0.0f;
    }
    return true;
}
