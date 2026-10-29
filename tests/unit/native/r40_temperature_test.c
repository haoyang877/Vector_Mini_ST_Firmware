/* 真实温度端口与未修改保护逻辑组合；只替换ADC快照，覆盖未校准失效和标定换算。 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "foc_sensing.h"
#include "mcu_temperature.h"
#include "../../../firmware/platform/bsp/mcu/at32f456/temperature.c"

#include "temperature_api_fixture.h"

static bool available = true;
static unsigned s_adc_reads;
static AdcTemperatureSample data;
#if BSP_ENABLED_MCU_TEMPERATURE
static MotorControlState state;
static FocState foc;
volatile McuTemperatureTelemetry s_mcu_temperature;
static void MotorFault_Report(MotorControlState *motor, MotorFaultCode fault)
{
    motor->fault_code = fault;
}
#include "r40_temperature_production.inc"
#endif

bool Adc_ReadTemperature(AdcTemperatureSample *out)
{
    ++s_adc_reads;
    if (!available)
    {
        return false;
    }
    *out = data;
    return true;
}

bool Adc_ReadAux(AdcAuxSample *out)
{
    (void)out;
    assert(false && "MCU temperature must not consume the motor NTC channel");
    return false;
}

static void TestTemperature_Poll(TestTemperaturePoll *result)
{
    if (result == NULL)
    {
        return;
    }
    result->sample_ready = result->conversion_ok = false;
#if BSP_ENABLED_INTERNAL_ANALOG_SAMPLING
    static bool seen;
    static uint32_t sequence;
    AdcTemperatureSample sample;
    if (!Adc_ReadTemperature(&sample) || (seen && sample.sequence == sequence))
    {
        return;
    }
    seen = true;
    sequence = sample.sequence;
    AdcTemperatureValue value;
    McuAdc_ConvertTemperature(&sample, &value);
    result->sample_ready = true;
    result->raw_ts = sample.mcu_temperature_counts;
    result->raw_vref = sample.reference_counts;
    result->conversion_ok = value.temperature_result == BSP_OK;
    if (result->conversion_ok)
    {
        result->celsius = value.celsius;
        result->vdda_mv = value.vdda_v * 1000.0f;
    }
    /* 温度资格失败不能影响同帧有效VDDA。 */
    if (R40_INTERNAL_REFERENCE_MV >= 1100.0f && R40_INTERNAL_REFERENCE_MV <= 1300.0f)
    {
        assert(value.voltage_result == BSP_OK);
        assert(fabsf(value.vdda_v - 3.276f) < 0.0001f);
    }
    else
    {
        assert(value.voltage_result == BSP_NOT_CALIBRATED && isnan(value.vdda_v));
    }
#endif
}
#if BSP_ENABLED_MCU_TEMPERATURE
static bool convert_temperature(uint16_t ts, uint16_t ref, float *c, float *mv)
{
    AdcTemperatureSample sample = {ts, ref, 0U, 0U};
    AdcTemperatureValue value;
    McuAdc_ConvertTemperature(&sample, &value);
    if (value.temperature_result != BSP_OK)
    {
        return false;
    }
    *c = value.celsius;
    *mv = value.vdda_v * 1000.0f;
    return true;
}
#endif

int main(void)
{
    TestTemperaturePoll result = {
        .raw_ts = 12U, .raw_vref = 34U, .celsius = 56.0f, .vdda_mv = 78.0f};
    AdcTemperatureSample invalid = {0U, 1500U, 0U, 0U};
    AdcTemperatureValue values;
    McuAdc_ConvertTemperature(&invalid, &values);
    assert(values.temperature_result == BSP_NOT_CALIBRATED && isnan(values.celsius));
    if (R40_INTERNAL_REFERENCE_MV >= 1100.0f && R40_INTERNAL_REFERENCE_MV <= 1300.0f)
    {
        assert(values.voltage_result == BSP_OK && fabsf(values.vdda_v - 3.276f) < 0.0001f);
        invalid.reference_counts = 0U;
        McuAdc_ConvertTemperature(&invalid, &values);
        assert(values.voltage_result == BSP_IO_ERROR && isnan(values.vdda_v));
    }
    data = (AdcTemperatureSample){1600U, 1500U, 1U, 0U};
    TestTemperature_Poll(&result);
    if (!BSP_ENABLED_MCU_TEMPERATURE)
    {
        assert(!result.sample_ready && !result.conversion_ok && s_adc_reads == 0U);
        assert(result.raw_ts == 12U && result.raw_vref == 34U);
        assert(result.celsius == 56.0f && result.vdda_mv == 78.0f);
        puts("PASS disabled MCU temperature does not read ADC or publish valid protection data");
        return 0;
    }
#if BSP_ENABLED_MCU_TEMPERATURE
    assert(result.sample_ready && result.raw_ts == 1600U && result.raw_vref == 1500U);
    assert(result.conversion_ok == EXPECT_CALIBRATED);
    if (!R40_TEMPERATURE_CALIBRATED)
    {
        assert(result.celsius == 56.0f && result.vdda_mv == 78.0f);
    }
    if (EXPECT_CALIBRATED)
    {
        assert(fabsf(result.celsius - 25.0f) < 0.001f && fabsf(result.vdda_mv - 3276.0f) < 0.01f);
    }
    TestTemperature_Poll(&result);
    assert(!result.sample_ready && !result.conversion_ok);
    available = false;
    ++data.sequence;
    TestTemperature_Poll(&result);
    assert(!result.sample_ready);
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
        TestTemperature_Poll(&result);
        assert(!result.conversion_ok && result.celsius == 56.0f);
    }
    /* 已显示温度后遇到无效ADC，不能保留可用状态或沿用上次温度。 */
    data.mcu_temperature_counts = 0U;
    ++data.sequence;
    TestTemperature_Poll(&result);
    assert(result.sample_ready && !result.conversion_ok);
    data.mcu_temperature_counts = 1600U;
    data.sequence = UINT32_MAX;
    TestTemperature_Poll(&result);
    assert(result.sample_ready);
    data.sequence = 0U;
    TestTemperature_Poll(&result);
    assert(result.sample_ready);
    TestTemperature_Poll(NULL);
    puts("PASS AT32 temperature: unique snapshots, calibration validity, supply/range, wrap and "
         "raw/calibrated values and unchanged protection");
#endif
    return 0;
}
