/* 母线物理量、可调低通和失效恢复的离线回归。 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "bsp_adc.h"
#include "foc.h"
#include "control_math.h"

static float voltage_sample_v, current_sample_a;
static ABCVect phase_sample_a = {1.0f, 2.0f, 3.0f};
static unsigned current_reads[BSP_ADC_BUS_CURRENT + 1];

float BspAdc_ReadChannel(BspAdcChannel channel)
{
    assert(channel >= BSP_ADC_PHASE_CURRENT_A && channel <= BSP_ADC_BUS_CURRENT);
    ++current_reads[channel];
    switch (channel)
    {
    case BSP_ADC_PHASE_CURRENT_A:
        return phase_sample_a.a;
    case BSP_ADC_PHASE_CURRENT_B:
        return phase_sample_a.b;
    case BSP_ADC_PHASE_CURRENT_C:
        return phase_sample_a.c;
    case BSP_ADC_BUS_VOLTAGE:
        return voltage_sample_v;
    default:
        return current_sample_a;
    }
}

/* BUS_MEASUREMENT_PRODUCTION */

int main(void)
{
    FocState measurement = {.bus_voltage_filter_alpha = 0.05f,
                            .bus_current_a = NAN,
                            .bus_current_filtered_a = NAN,
                            .bus_current_filter_alpha = 0.05f,
                            .phase_current_a = {1.0f, 2.0f, 3.0f},
                            .temperature_c = 25.0f};

    /* 默认电压响应保持原值，运行时更换系数立即生效。 */
    voltage_sample_v = 30.0f;
    MotorVoltage_Update(&measurement);
    assert(measurement.bus_voltage_v == 30.0f);
    assert(measurement.bus_voltage_filtered_v == 1.5f);
    measurement.bus_voltage_filter_alpha = 0.5f;
    MotorVoltage_Update(&measurement);
    assert(measurement.bus_voltage_filtered_v == 15.75f);
    measurement.bus_voltage_filter_alpha = 0.0f;
    voltage_sample_v = 20.0f;
    MotorVoltage_Update(&measurement);
    assert(measurement.bus_voltage_v == 20.0f && measurement.bus_voltage_filtered_v == 15.75f);
    measurement.bus_voltage_filter_alpha = 1.0f;
    MotorVoltage_Update(&measurement);
    assert(measurement.bus_voltage_filtered_v == 20.0f);
    measurement.bus_voltage_filter_alpha = 0.05f;
    voltage_sample_v = 32.5f;
    for (unsigned index = 0; index < 1000; ++index)
    {
        MotorVoltage_Update(&measurement);
    }
    assert(fabsf(measurement.bus_voltage_filtered_v - 32.5f) < 0.001f);
    float voltage_filtered_v = measurement.bus_voltage_filtered_v;

    /* 首次有效硬件电流建立基准，正负电流均参与同一低通。 */
    current_sample_a = 2.5f;
    MotorCurrent_Update(&measurement);
    assert(measurement.bus_current_a == 2.5f && measurement.bus_current_filtered_a == 2.5f);
    current_sample_a = -2.5f;
    MotorCurrent_Update(&measurement);
    assert(measurement.bus_current_a == -2.5f && measurement.bus_current_filtered_a == 2.25f);
    measurement.bus_current_filter_alpha = 0.5f;
    MotorCurrent_Update(&measurement);
    assert(measurement.bus_current_filtered_a == -0.125f);
    measurement.bus_current_filter_alpha = 0.0f;
    current_sample_a = 9.0f;
    MotorCurrent_Update(&measurement);
    assert(measurement.bus_current_a == 9.0f && measurement.bus_current_filtered_a == -0.125f);
    measurement.bus_current_filter_alpha = 1.0f;
    current_sample_a = -3.0f;
    MotorCurrent_Update(&measurement);
    assert(measurement.bus_current_filtered_a == -3.0f);

    /* 缺失值不伪装成零电流，恢复后不被上拍NAN污染。 */
    phase_sample_a = (ABCVect){-4.0f, 5.0f, 6.0f};
    current_sample_a = NAN;
    MotorCurrent_Update(&measurement);
    assert(isnan(measurement.bus_current_a) && isnan(measurement.bus_current_filtered_a));
    assert(measurement.phase_current_a.a == -4.0f && measurement.phase_current_a.b == 5.0f &&
           measurement.phase_current_a.c == 6.0f);
    phase_sample_a = (ABCVect){1.0f, 2.0f, 3.0f};
    current_sample_a = 4.0f;
    MotorCurrent_Update(&measurement);
    assert(measurement.bus_current_a == 4.0f && measurement.bus_current_filtered_a == 4.0f);
    current_sample_a = INFINITY;
    MotorCurrent_Update(&measurement);
    assert(isinf(measurement.bus_current_a) && isinf(measurement.bus_current_filtered_a));
    current_sample_a = 0.0f;
    MotorCurrent_Update(&measurement);
    assert(measurement.bus_current_a == 0.0f && measurement.bus_current_filtered_a == 0.0f);
    assert(measurement.bus_voltage_filtered_v == voltage_filtered_v);
    assert(measurement.phase_current_a.a == 1.0f && measurement.phase_current_a.b == 2.0f &&
           measurement.phase_current_a.c == 3.0f && measurement.temperature_c == 25.0f);

    FocState other = {.bus_current_filtered_a = 10.0f, .bus_current_filter_alpha = 0.5f};
    current_sample_a = 2.0f;
    MotorCurrent_Update(&other);
    assert(other.bus_current_a == 2.0f && other.bus_current_filtered_a == 6.0f);
    assert(measurement.bus_current_filtered_a == 0.0f);
    assert(current_reads[BSP_ADC_BUS_CURRENT] == 10U);
    assert(current_reads[BSP_ADC_PHASE_CURRENT_A] == current_reads[BSP_ADC_BUS_CURRENT]);
    assert(current_reads[BSP_ADC_PHASE_CURRENT_B] == current_reads[BSP_ADC_BUS_CURRENT]);
    assert(current_reads[BSP_ADC_PHASE_CURRENT_C] == current_reads[BSP_ADC_BUS_CURRENT]);
    voltage_sample_v = NAN;
    MotorVoltage_Update(&measurement);
    assert(isnan(measurement.bus_voltage_v) && isnan(measurement.bus_voltage_filtered_v));
    puts("PASS current sampling: four channels once per update, phase refresh with invalid bus, "
         "configurable filters, signed samples, missing/recovered input, isolation");
    return 0;
}
