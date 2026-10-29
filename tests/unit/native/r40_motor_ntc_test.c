/* 验证实际NTC换算与公开ADC读数，测试观察状态不进入固件。 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "../../../firmware/platform/bsp/mcu/at32f456/adc.c"
#include "../../../firmware/platform/bsp/bsp_adc.c"
#include "../../../firmware/platform/bsp/mcu/at32f456/temperature.c"

int main(void)
{
    assert(Adc_Init() && Adc_Start());
    assert(sim_analog_pins == 0x9FU && !sim_adc_common.tempervintrv_state);
    assert(sim_adc[1].ordinary_length == 1U && sim_adc[1].ordinary_channels[0] == 7U);
    assert(sim_dma_config.buffer_size == 2U && sim_dma_enabled);
    float value = 123.0f;
#if EXPECT_BAD_NTC_CONFIG
    assert(McuAdc_MotorTemperature(372U, &value) == BSP_SENSOR_FAULT && value == 123.0f);
    puts("PASS invalid NTC configuration rejected");
    return 0;
#endif
    const uint16_t counts[] = {118U, 372U, 893U, 1804U, 2412U, 3014U};
    const float degrees[] = {0.0f, 25.0f, 50.0f, 80.0f, 100.0f, 125.0f};
    for (unsigned i = 0U; i < sizeof(counts) / sizeof(counts[0]); ++i)
    {
        assert(McuAdc_MotorTemperature(counts[i], &value) == BSP_OK);
        assert(fabsf(value - degrees[i]) < 0.2f);
    }
    const uint16_t invalid[] = {0U, 2U, 4093U, 4095U, 3U, 4000U, 4096U};
    for (unsigned i = 0U; i < sizeof(invalid) / sizeof(invalid[0]); ++i)
    {
        value = 123.0f;
        assert(McuAdc_MotorTemperature(invalid[i], &value) == BSP_SENSOR_FAULT);
        assert(value == 123.0f);
    }
    s_sampling.aux_dma[0] = 372U;
    sim_dma_flags = DMA1_HDT2_FLAG;
    DMA1_Channel2_IRQHandler();
    assert(BspAdc_UpdateAuxiliary() == BSP_OK);
    BspAdcValue sample;
    assert(BspAdc_ReadChannel(BSP_ADC_MOTOR_TEMPERATURE, &sample) == BSP_OK);
    assert(fabsf(sample.value - 25.0f) < 0.1f);
    sim_ms = 3U;
    assert(BspAdc_ReadChannel(BSP_ADC_MOTOR_TEMPERATURE, &sample) == BSP_STALE);
    puts("PASS real NTC curve, rails, invalid configuration, public reading and freshness");
    return 0;
}
