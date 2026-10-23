/* 直接执行ADC生产端口；只有硬件寄存器、DMA和中断事件由测试注入。 */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../../../firmware/platform/at32f456/ports/motor_sensing_at32f456.c"

extern unsigned sim_adc_trigger_ticks;
extern void (*sim_before_critical_enter)(void);
extern unsigned sim_critical_enters, sim_critical_exits;

static void complete_samples(void)
{
    const uint16_t conversions[4][4] = {
        {1, 2, 3, 4}, {1024, 1025, 1026, 1027}, {4095, 4095, 4095, 4095}, {2000, 2100, 2200, 2300}};
    assert(sim_adc[0].oversampled && sim_adc[0].ratio == ADC_OVERSAMPLE_RATIO_4 &&
           sim_adc[0].shift == ADC_OVERSAMPLE_SHIFT_0);
    for (unsigned rank = 0; rank < 4; ++rank)
    {
        sim_adc[0].result[rank] = 0;
        for (unsigned sample = 0; sample < 4; ++sample)
        {
            sim_adc[0].result[rank] += conversions[rank][sample];
        }
    }
    sim_adc[0].flags = ADC_PCCE_FLAG;
}

/* 模拟ADC恰在载波快照前抢占，采样序号与时间戳同时推进。 */
static void complete_pending_adc(void)
{
    complete_samples();
    assert(R40_AdcComplete());
}

int main(int argc, char **argv)
{
    unsigned scenario = argc > 1 ? (unsigned)strtoul(argv[1], NULL, 10) : 0U;
    if (scenario >= 1U && scenario <= 3U)
    {
        sim_adc_failure = scenario;
    }
    assert(!R40_AdcStart());
    bool initialized = R40_AdcInit();
    if (sim_adc_failure)
    {
        assert(!initialized && !sim_dma_enabled && sim_power_off && !R40_AdcHealthy());
        puts("PASS ADC ready/calibration failure remains disabled");
        return 0;
    }
    assert(initialized && !R40_AdcInit() && !sim_dma_enabled);
    assert(sim_analog_pins == (R40_OUTPUT_SINCOS_DIAGNOSTICS ? 0xFFU : 0x9FU));
    assert(sim_adc_common.div == ADC_HCLK_DIV_6 && sim_adc_common.tempervintrv_state);
    for (unsigned rank = 0; rank < 4; ++rank)
    {
        assert(sim_adc[0].channels[rank] == rank);
    }
    assert(sim_adc[1].ordinary_channels[0] == 4U && sim_adc[1].ordinary_channels[1] == 7U);
    assert(sim_adc[0].ordinary_channels[0] == 16U && sim_adc[0].ordinary_channels[1] == 17U);
    assert(sim_adc[0].ordinary_length == 2U && sim_adc[1].ordinary_length == R40_AUX_RANKS);
#if R40_OUTPUT_SINCOS_DIAGNOSTICS
    assert(sim_adc[1].ordinary_channels[2] == 5U && sim_adc[1].ordinary_channels[3] == 6U);
#endif
    assert(sim_dma_config.memory_base_addr == (uint32_t)(uintptr_t)s_aux_dma);
    assert(sim_dma_config.buffer_size == 2U * R40_AUX_RANKS && sim_dma_config.loop_mode_enable);
    assert(sim_temperature_dma_config.memory_base_addr == (uint32_t)(uintptr_t)s_temperature_dma);
    assert(sim_temperature_dma_config.buffer_size == 4U &&
           sim_temperature_dma_config.loop_mode_enable);
    assert(R40_AdcStart() && sim_dma_enabled && sim_temperature_dma_enabled && !R40_AdcStart());
    /* 低压台架占空比44%..56%；16次转换须与两侧开关沿各留至少2us。 */
    const float trigger_us = (float)sim_adc_trigger_ticks / 180.0f;
    const float conversion_us = 16.0f * (12.5f + 12.5f) / 30.0f;
    assert(trigger_us - 25.0f * 0.56f >= 2.0f);
    assert(50.0f - 25.0f * 0.56f - trigger_us - conversion_us >= 2.0f);
    assert(sim_adc[0].edge == ADC_PREEMPT_TRIG_EDGE_RISING &&
           sim_adc[1].ordinary_edge == ADC_ORDINARY_TRIG_EDGE_RISING &&
           sim_adc[0].ordinary_edge == ADC_ORDINARY_TRIG_EDGE_RISING);
    assert(!R40_AdcComplete());
    R40_AdcCarrierTick();
    complete_samples();
    if (scenario == 4U)
    {
        sim_adc[0].flags |= ADC_TCF_FLAG;
    }
    if (scenario == 7U)
    {
        sim_adc[0].result[2] = 16381U;
    }
    if (scenario == 13U)
    {
        sim_adc[0].flags |= ADC_OCCO_FLAG;
    }
    bool fresh = R40_AdcComplete();
    if (scenario == 4U || scenario == 7U || scenario == 13U)
    {
        assert(!fresh && !R40_AdcHealthy() && sim_stop_calls && !sim_dma_enabled);
        assert(motor_hw_vbus_sample_raw() == 0U);
        puts("PASS corrupt/failed ADC sequence cannot publish");
        return 0;
    }
    assert(fresh && motor_hw_current_sample_raw(MOTOR_HW_CURRENT_PHASE_A) == 10U);
    assert(motor_hw_current_sample_raw(MOTOR_HW_CURRENT_PHASE_B) == 4102U);
    assert(motor_hw_current_sample_raw(MOTOR_HW_CURRENT_PHASE_C) == 16380U);
    assert(motor_hw_current_sample_raw((MotorHwCurrentPhase)3) == 0U);
    assert(motor_hw_vbus_sample_raw() == 8600U && !R40_AdcComplete());
    if (scenario == 18U)
    {
        R40_AdcCarrierTick();
        unsigned entries = sim_critical_enters;
        sim_cycle_counter.CYCCNT += R40_CORE_HZ / FOC_FREQ;
        sim_before_critical_enter = complete_pending_adc;
        R40_AdcCarrierTick();
        assert(sim_before_critical_enter == NULL && sim_critical_enters == entries + 1U);
        assert(sim_critical_enters == sim_critical_exits && R40_AdcHealthy());
        sim_cycle_counter.CYCCNT += R40_CORE_HZ / FOC_FREQ;
        R40_AdcCarrierTick();
        assert(!R40_AdcHealthy() && g_r40_adc_fault_reason == 5U);
        assert(sim_critical_enters == sim_critical_exits && sim_stop_calls);
        puts("PASS atomic carrier snapshot accepts pending ADC and rejects real missing frame");
        return 0;
    }
    if (scenario == 17U)
    {
        /* 上次载波处理滞后于ADC，下次恢复正常顺序，且DWT跨32位回绕。 */
        sim_cycle_counter.CYCCNT = UINT32_MAX - 4000U;
        complete_samples();
        assert(R40_AdcComplete());
        R40_AdcCarrierTick();
        sim_cycle_counter.CYCCNT = 1000U;
        R40_AdcCarrierTick();
        assert(R40_AdcHealthy());
        sim_cycle_counter.CYCCNT = 6000U;
        R40_AdcCarrierTick();
        assert(!R40_AdcHealthy() && sim_stop_calls && g_r40_adc_fault_reason == 5U);
        puts("PASS reordered carrier/ADC IRQ and DWT wrap; real stale sample stops");
        return 0;
    }
    R40_AdcCarrierTick();
    if (scenario == 6U)
    {
        R40_AdcCarrierTick();
        assert(!R40_AdcHealthy() && sim_stop_calls && !sim_dma_enabled);
        puts("PASS missing next ADC frame latches shutdown");
        return 0;
    }
    R40AuxSamples out = {0};
    assert(!R40_AdcReadAux(&out));
    for (unsigned i = 0; i < 2U * R40_AUX_RANKS; ++i)
    {
        s_aux_dma[i] = 100U + i;
    }
    sim_dma_flags = DMA1_HDT2_FLAG;
    if (scenario == 5U)
    {
        sim_dma_flags |= DMA1_DTERR2_FLAG;
    }
    if (scenario == 8U)
    {
        sim_dma_flags |= DMA1_FDT2_FLAG;
    }
    if (scenario == 9U)
    {
        s_aux_dma[1] = 4096U;
    }
#if R40_OUTPUT_SINCOS_DIAGNOSTICS
    if (scenario == 15U || scenario == 16U)
    {
        s_aux_dma[R40_AUX_BASE_RANKS + scenario - 15U] = 4096U;
    }
#endif
    DMA1_Channel2_IRQHandler();
    if (scenario == 5U || scenario == 8U || scenario == 9U || scenario == 15U || scenario == 16U)
    {
        assert(!R40_AdcHealthy() && !R40_AdcReadAux(&out) && !sim_dma_enabled);
        puts("PASS DMA loss/error/corruption cannot expose torn data");
        return 0;
    }
    assert(R40_AdcReadAux(&out) && out.sequence == 1U && out.bus_current_counts == 100U);
    assert(out.motor_temperature_counts == 101U);
#if R40_OUTPUT_SINCOS_DIAGNOSTICS
    assert(g_r40_sincos_snapshot.begin == 2U && g_r40_sincos_snapshot.end == 2U);
    assert(g_r40_sincos_snapshot.sequence == 1U && g_r40_sincos_snapshot.pa5_counts == 102U &&
           g_r40_sincos_snapshot.pa6_counts == 103U);
#endif
    sim_dma_flags = DMA1_FDT2_FLAG;
    DMA1_Channel2_IRQHandler();
    assert(R40_AdcReadAux(&out) && out.sequence == 2U &&
           out.bus_current_counts == 100U + R40_AUX_RANKS);
    assert(out.motor_temperature_counts == 101U + R40_AUX_RANKS);
#if R40_OUTPUT_SINCOS_DIAGNOSTICS
#if defined(R40_KEIL_OPENLOOP) && R40_KEIL_OPENLOOP
    assert(g_r40_sincos_snapshot.sequence == 2U);
#else
    assert(g_r40_sincos_snapshot.sequence == 1U);
#endif
    for (unsigned i = 0; i < 39U; ++i)
    {
        sim_dma_flags = DMA1_FDT2_FLAG;
        DMA1_Channel2_IRQHandler();
    }
#if defined(R40_KEIL_OPENLOOP) && R40_KEIL_OPENLOOP
    assert(g_r40_sincos_snapshot.begin == 82U && g_r40_sincos_snapshot.end == 82U);
#else
    assert(g_r40_sincos_snapshot.begin == 4U && g_r40_sincos_snapshot.end == 4U);
#endif
    assert(g_r40_sincos_snapshot.sequence == 41U && g_r40_sincos_snapshot.pa5_counts == 106U &&
           g_r40_sincos_snapshot.pa6_counts == 107U);
#endif
    R40TemperatureSamples temperature = {0};
    assert(!R40_AdcReadTemperature(&temperature));
    s_temperature_dma[0] = 1200U;
    s_temperature_dma[1] = 1500U;
    s_temperature_dma[2] = 1300U;
    s_temperature_dma[3] = 1501U;
    sim_dma_flags = DMA1_HDT3_FLAG;
    if (scenario == 10U)
    {
        sim_dma_flags |= DMA1_DTERR3_FLAG;
    }
    if (scenario == 11U)
    {
        sim_dma_flags |= DMA1_FDT3_FLAG;
    }
    if (scenario == 12U)
    {
        s_temperature_dma[0] = 4096U;
    }
    if (scenario == 14U)
    {
        s_temperature.sequence = UINT32_MAX;
    }
    DMA1_Channel3_IRQHandler();
    if (scenario >= 10U && scenario <= 12U)
    {
        assert(!R40_AdcReadTemperature(&temperature) && !R40_AdcHealthy());
        assert(!sim_dma_enabled && !sim_temperature_dma_enabled && sim_power_off);
        puts("PASS ADC1 temperature DMA error/loss/range failure stops both streams");
        return 0;
    }
    assert(R40_AdcReadTemperature(&temperature) && temperature.mcu_temperature_counts == 1200U);
    assert(temperature.sequence == (scenario == 14U ? 0U : 1U));
    sim_dma_flags = DMA1_FDT3_FLAG;
    DMA1_Channel3_IRQHandler();
    assert(R40_AdcReadTemperature(&temperature) && temperature.mcu_temperature_counts == 1300U);
    sim_ms = 3U;
    R40AuxSamples previous = out;
    assert(!R40_AdcReadAux(&out) && memcmp(&out, &previous, sizeof(out)) == 0);
    assert(!R40_AdcReadAux(NULL));
    assert(!R40_AdcReadTemperature(&temperature) && !R40_AdcReadTemperature(NULL));
    const float a_per_count =
        R40_ADC_REFERENCE_V / R40_ADC_MAX_COUNTS / R40_CURRENT_GAIN / R40_PHASE_SHUNT_OHM;
    const float v_per_count = R40_ADC_REFERENCE_V / R40_ADC_MAX_COUNTS *
                              (R40_VBUS_TOP_KOHM + R40_VBUS_BOTTOM_KOHM) / R40_VBUS_BOTTOM_KOHM;
    assert(a_per_count > 0.0100f && a_per_count < 0.0101f);
    assert(v_per_count > 0.0252f && v_per_count < 0.0253f);
    puts("PASS real four-conversion sums, coherent DMA halves and board scale");
    return 0;
}
