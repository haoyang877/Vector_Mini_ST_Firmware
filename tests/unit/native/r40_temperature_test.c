/* 真实温度端口与未修改保护逻辑组合；只替换ADC快照，覆盖未校准失效和标定换算。 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "foc_sensing.h"
#include "mcu_temperature.h"
#include "../../../firmware/platform/at32f456/ports/temperature_at32f456.c"

static bool available = true;
static unsigned s_adc_reads;
static R40TemperatureSamples data;
static MotorControlState state;
static FocState foc;
volatile McuTemperatureTelemetry s_mcu_temperature;
static void MotorFault_Report(MotorControlState *motor, MotorFaultCode fault)
{
    motor->fault_code = fault;
}
#include "r40_temperature_production.inc"

bool R40_AdcReadTemperature(R40TemperatureSamples *out)
{
    ++s_adc_reads;
    if (!available)
    {
        return false;
    }
    *out = data;
    return true;
}

int main(void)
{
    MotorHwTemperaturePoll result = {
        .raw_ts = 12U, .raw_vref = 34U, .celsius = 56.0f, .vdda_mv = 78.0f};
    data = (R40TemperatureSamples){1600U, 1500U, 1U};
    motor_hw_temperature_poll(&result);
    if (!R40_TEMPERATURE_SENSING_ENABLED)
    {
        assert(!result.sample_ready && !result.conversion_ok && s_adc_reads == 0U);
        assert(result.raw_ts == 12U && result.raw_vref == 34U);
        assert(result.celsius == 56.0f && result.vdda_mv == 78.0f);
        assert(g_r40_mcu_temperature_diagnostic.status == R40_MCU_TEMPERATURE_DISABLED);
        assert(g_r40_mcu_temperature_diagnostic.protection_valid == 0U);
        assert(isnan(g_r40_mcu_temperature_diagnostic.temperature_c));
        puts("PASS disabled MCU temperature does not read ADC or publish valid protection data");
        return 0;
    }
    assert(result.sample_ready && result.raw_ts == 1600U && result.raw_vref == 1500U);
    assert(result.conversion_ok == EXPECT_CALIBRATED);
    assert(g_r40_mcu_temperature_diagnostic.begin == g_r40_mcu_temperature_diagnostic.end);
    assert(g_r40_mcu_temperature_diagnostic.begin != 0U &&
           (g_r40_mcu_temperature_diagnostic.begin & 1U) == 0U);
    assert(g_r40_mcu_temperature_diagnostic.sample_count == 1U);
    assert(g_r40_mcu_temperature_diagnostic.raw_ts == 1600U &&
           g_r40_mcu_temperature_diagnostic.raw_vref == 1500U);
    if (!R40_TEMPERATURE_CALIBRATED)
    {
        assert(g_r40_mcu_temperature_diagnostic.status == R40_MCU_TEMPERATURE_ESTIMATED);
        assert(fabsf(g_r40_mcu_temperature_diagnostic.temperature_c - 25.0f) < 0.001f);
        assert(g_r40_mcu_temperature_diagnostic.protection_valid == 0U);
        assert(result.celsius == 56.0f && result.vdda_mv == 78.0f);
    }
    else if (EXPECT_CALIBRATED)
    {
        assert(g_r40_mcu_temperature_diagnostic.status == R40_MCU_TEMPERATURE_CALIBRATED);
        assert(g_r40_mcu_temperature_diagnostic.protection_valid == 1U);
    }
    else
    {
        assert(g_r40_mcu_temperature_diagnostic.status == R40_MCU_TEMPERATURE_INVALID);
        assert(isnan(g_r40_mcu_temperature_diagnostic.temperature_c));
    }
    if (EXPECT_CALIBRATED)
    {
        assert(fabsf(result.celsius - 25.0f) < 0.001f && fabsf(result.vdda_mv - 3276.0f) < 0.01f);
    }
    motor_hw_temperature_poll(&result);
    assert(!result.sample_ready && !result.conversion_ok);
    assert(g_r40_mcu_temperature_diagnostic.sample_count == 1U);
    available = false;
    ++data.sequence;
    motor_hw_temperature_poll(&result);
    assert(!result.sample_ready);
    assert(g_r40_mcu_temperature_diagnostic.status == R40_MCU_TEMPERATURE_UNAVAILABLE);
    assert(g_r40_mcu_temperature_diagnostic.protection_valid == 0U);
    assert(isnan(g_r40_mcu_temperature_diagnostic.temperature_c));
    available = true;
    for (unsigned i = 0U; i < 200U; ++i)
    {
        ++data.sequence;
        Temperature_Update(&foc, &state);
    }
    if (EXPECT_CALIBRATED)
    {
        assert(s_mcu_temperature.valid && state.fault_code == MOTOR_FAULT_NONE);
        data.mcu_temperature_counts = 1970U;
        ++data.sequence;
        Temperature_Update(&foc, &state);
        assert(state.fault_code == MOTOR_FAULT_OVERTEMPERATURE);
        assert(s_mcu_temperature.raw_celsius > 90.0f);
        float celsius = 0.0f, supply = 0.0f;
        for (unsigned ref = 1400U; ref <= 1900U; ref += 100U)
        {
            unsigned ts = (unsigned)(ref * 1280U / 1200U);
            assert(convert_temperature((uint16_t)ts, (uint16_t)ref, &celsius, &supply));
            assert(fabsf(celsius - 25.0f) < 0.25f);
        }
        assert(!convert_temperature(0U, 1500U, &celsius, &supply));
        assert(!convert_temperature(4095U, 1500U, &celsius, &supply));
        assert(!convert_temperature(1600U, 0U, &celsius, &supply));
        assert(!convert_temperature(1600U, 4095U, &celsius, &supply));
        assert(!convert_temperature(1600U, 1000U, &celsius, &supply));
        assert(!convert_temperature(1600U, 3000U, &celsius, &supply));
        assert(!convert_temperature(2800U, 1500U, &celsius, &supply));
        assert(!convert_temperature(500U, 1500U, &celsius, &supply));
    }
    else
    {
        assert(!s_mcu_temperature.valid && isnan(foc.temperature_c));
        assert(state.fault_code == MOTOR_FAULT_TEMPERATURE_SENSOR);
        data.mcu_temperature_counts = 1970U;
        ++data.sequence;
        motor_hw_temperature_poll(&result);
        assert(!result.conversion_ok && result.celsius == 56.0f);
        if (!R40_TEMPERATURE_CALIBRATED)
        {
            assert(g_r40_mcu_temperature_diagnostic.temperature_c > 90.0f);
            assert(g_r40_mcu_temperature_diagnostic.protection_valid == 0U);
        }
    }
    /* 已显示温度后遇到无效ADC，不能保留可用状态或沿用上次温度。 */
    data.mcu_temperature_counts = 0U;
    ++data.sequence;
    motor_hw_temperature_poll(&result);
    assert(result.sample_ready && !result.conversion_ok);
    assert(g_r40_mcu_temperature_diagnostic.status == R40_MCU_TEMPERATURE_INVALID);
    assert(isnan(g_r40_mcu_temperature_diagnostic.temperature_c));
    data.mcu_temperature_counts = 1600U;
    data.sequence = UINT32_MAX;
    motor_hw_temperature_poll(&result);
    assert(result.sample_ready);
    data.sequence = 0U;
    motor_hw_temperature_poll(&result);
    assert(result.sample_ready);
    motor_hw_temperature_poll(NULL);
    puts("PASS AT32 temperature: unique snapshots, calibration validity, supply/range, wrap and "
         "diagnostic estimates isolated from unchanged protection");
    return 0;
}
