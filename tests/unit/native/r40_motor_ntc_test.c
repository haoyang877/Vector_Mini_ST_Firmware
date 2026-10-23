/* 执行真实ADC/NTC端口：分压方向、数值向量、新鲜度、独立开关与故障隔离。 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "../../../firmware/platform/at32f456/ports/motor_sensing_at32f456.c"
#include "../../../firmware/platform/at32f456/ports/temperature_at32f456.c"
#include "../../../firmware/platform/at32f456/ports/motor_temperature_at32f456.c"

static void publish_ntc(uint16_t counts, bool second)
{
    unsigned offset = second ? 2U : 0U;
    s_aux_dma[offset] = 123U;
    s_aux_dma[offset + 1U] = counts;
    sim_dma_flags = second ? DMA1_FDT2_FLAG : DMA1_HDT2_FLAG;
    DMA1_Channel2_IRQHandler();
}

int main(void)
{
    assert(R40_MOTOR_NTC_ENABLED == 1 && R40_TEMPERATURE_SENSING_ENABLED == 0);
    assert(R40_AdcInit() && R40_AdcStart());
    assert(sim_analog_pins == 0x9FU && !sim_adc_common.tempervintrv_state);
    assert(sim_adc[1].ordinary_length == 2U && sim_adc[1].ordinary_channels[1] == 7U);
    assert(sim_dma_config.buffer_size == 4U && sim_dma_enabled);
    assert(sim_adc[0].ordinary_edge == ADC_ORDINARY_TRIG_EDGE_NONE);
    assert(!sim_adc[0].dma_enabled && !sim_temperature_dma_enabled);
    assert(sim_temperature_dma_config.buffer_size == 0U && sim_temperature_irq_enables == 0U);
    R40MotorTemperatureSample sample;
    assert(!R40_MotorTemperaturePoll(NULL));
    assert(!R40_MotorTemperaturePoll(&sample));
    assert(sample.status == R40_MOTOR_TEMPERATURE_UNAVAILABLE && isnan(sample.celsius));
    publish_ntc(372U, false);
    assert(R40_MotorTemperaturePoll(&sample));
#if EXPECT_BAD_NTC_CONFIG
    assert(sample.status == R40_MOTOR_TEMPERATURE_BAD_CONFIG && isnan(sample.celsius));
    puts("PASS invalid NTC config cannot produce valid temperature");
    return 0;
#endif
    assert(sample.status == R40_MOTOR_TEMPERATURE_VALID && fabsf(sample.celsius - 25.0f) < 0.1f);
    assert(fabsf(sample.resistance_ohm - 10000.0f) < 10.0f && sample.sequence == 1U);
    assert(!R40_MotorTemperaturePoll(&sample) && sample.status == R40_MOTOR_TEMPERATURE_VALID);
    /* 由独立正向B方程取整的12位向量；不是实测或传感器精度声明。 */
    const uint16_t counts[] = {118U, 372U, 893U, 1804U, 2412U, 3014U};
    const float degrees[] = {0.0f, 25.0f, 50.0f, 80.0f, 100.0f, 125.0f};
    float previous = -100.0f;
    for (unsigned i = 0U; i < sizeof(counts) / sizeof(counts[0]); ++i)
    {
        publish_ntc(counts[i], (i & 1U) != 0U);
        assert(R40_MotorTemperaturePoll(&sample));
        assert(sample.raw_counts == counts[i] && sample.status == R40_MOTOR_TEMPERATURE_VALID);
        assert(fabsf(sample.celsius - degrees[i]) < 0.2f && sample.celsius > previous);
        previous = sample.celsius;
        R40AuxSamples aux;
        assert(R40_AdcReadAux(&aux) && aux.bus_current_counts == 123U);
    }
    const uint16_t invalid_counts[] = {0U, 2U, 4093U, 4095U, 3U, 4000U};
    const R40MotorTemperatureStatus invalid_states[] = {R40_MOTOR_TEMPERATURE_OPEN,
                                                        R40_MOTOR_TEMPERATURE_OPEN,
                                                        R40_MOTOR_TEMPERATURE_SHORT,
                                                        R40_MOTOR_TEMPERATURE_SHORT,
                                                        R40_MOTOR_TEMPERATURE_OUT_OF_RANGE,
                                                        R40_MOTOR_TEMPERATURE_OUT_OF_RANGE};
    for (unsigned i = 0U; i < sizeof(invalid_counts) / sizeof(invalid_counts[0]); ++i)
    {
        publish_ntc(invalid_counts[i], false);
        assert(R40_MotorTemperaturePoll(&sample) && sample.status == invalid_states[i]);
        assert(isnan(sample.celsius) && R40_AdcHealthy());
    }
    publish_ntc(372U, true);
    assert(R40_MotorTemperaturePoll(&sample) && sample.status == R40_MOTOR_TEMPERATURE_VALID);
    sim_ms = 2U;
    assert(!R40_MotorTemperaturePoll(&sample) && sample.status == R40_MOTOR_TEMPERATURE_VALID);
    sim_ms = 3U;
    assert(!R40_MotorTemperaturePoll(&sample));
    assert(sample.status == R40_MOTOR_TEMPERATURE_UNAVAILABLE && isnan(sample.celsius));
    /* 序列号和毫秒计时分别跨越UINT32_MAX，仍按新样本与2ms窗口处理。 */
    s_aux.sequence = UINT32_MAX;
    sim_ms = UINT32_MAX;
    publish_ntc(893U, false);
    assert(R40_MotorTemperaturePoll(&sample) && sample.sequence == 0U);
    sim_ms = 1U;
    assert(!R40_MotorTemperaturePoll(&sample) && sample.status == R40_MOTOR_TEMPERATURE_VALID);
    sim_ms = 2U;
    assert(!R40_MotorTemperaturePoll(&sample) &&
           sample.status == R40_MOTOR_TEMPERATURE_UNAVAILABLE);
    publish_ntc(372U, false);
    assert(R40_MotorTemperaturePoll(&sample));
    MotorHwTemperaturePoll mcu = {true, true, 12U, 34U, 56.0f, 78.0f};
    motor_hw_temperature_poll(&mcu);
    assert(!mcu.sample_ready && !mcu.conversion_ok && mcu.celsius == 56.0f);
    assert(!sim_temperature_dma_enabled && sim_temperature_irq_enables == 0U);
    sim_dma_flags = DMA1_DTERR2_FLAG;
    DMA1_Channel2_IRQHandler();
    assert(!R40_MotorTemperaturePoll(&sample) &&
           sample.status == R40_MOTOR_TEMPERATURE_UNAVAILABLE);
    assert(!R40_AdcHealthy() && sim_power_off && !sim_dma_enabled);
    puts("PASS motor NTC: nominal curve, polarity, rails, stale/fault, wrap; MCU temperature off");
    return 0;
}
