/* 直接执行ADC生产端口；只有硬件寄存器、DMA和中断事件由测试注入。 */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../../../firmware/platform/bsp/mcu/at32f456/adc.c"
#include "../../../firmware/platform/bsp/bsp_adc.c"
#include "../../../firmware/platform/bsp/mcu/at32f456/temperature.c"

extern unsigned sim_adc_trigger_ticks;
extern void (*sim_before_critical_enter)(void);
extern unsigned sim_critical_enters, sim_critical_exits;

/* SDK编码映射为实际周期，预算取自生产初始化捕获值，而非另写一份目标配置。 */
static float sampling_cycles(unsigned sample_time)
{
    static const float s_cycles[] = {2.5f, 6.5f, 12.5f, 24.5f, 47.5f, 92.5f, 247.5f, 640.5f};
    assert(sample_time < sizeof(s_cycles) / sizeof(s_cycles[0]));
    return s_cycles[sample_time];
}

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
    sim_adc[1].flags |= ADC_PCCE_FLAG;
    sim_adc[1].result[0] = 2050U;
}

/* 模拟ADC恰在载波快照前抢占，采样序号与时间戳同时推进。 */
static void complete_pending_adc(void)
{
    complete_samples();
    assert(Adc_CompleteFast());
}

int main(int argc, char **argv)
{
    unsigned scenario = argc > 1 ? (unsigned)strtoul(argv[1], NULL, 10) : 0U;
    if (scenario >= 1U && scenario <= 3U)
    {
        sim_adc_failure = scenario;
    }
    assert(!Adc_Start());
    bool initialized = Adc_Init();
    if (sim_adc_failure)
    {
        assert(!initialized && !sim_dma_enabled && sim_power_off && !Adc_IsHealthy());
        puts("PASS ADC ready/calibration failure remains disabled");
        return 0;
    }
    assert(initialized && !Adc_Init() && !sim_dma_enabled);
    assert(sim_analog_pins == (BSP_ENABLED_OUTPUT_SINCOS ? 0xFFU : 0x9FU));
    assert(sim_adc_common.div == ADC_HCLK_DIV_3 && sim_adc_common.tempervintrv_state);
    for (unsigned rank = 0; rank < 4; ++rank)
    {
        assert(sim_adc[0].channels[rank] == rank);
    }
    assert(sim_adc[1].channels[0] == 4U && sim_adc[1].ordinary_channels[0] == 7U);
    assert(sim_adc[0].ordinary_channels[0] == 16U && sim_adc[0].ordinary_channels[1] == 17U);
    assert(sim_adc[0].ordinary_length == 2U && sim_adc[1].ordinary_length == R40_AUX_RANKS);
    assert(sim_dma_config.memory_base_addr == (uint32_t)(uintptr_t)s_sampling.aux_dma);
    assert(sim_dma_config.buffer_size == 2U * R40_AUX_RANKS && sim_dma_config.loop_mode_enable);
    assert(sim_temperature_dma_config.memory_base_addr ==
           (uint32_t)(uintptr_t)s_sampling.temperature_dma);
    assert(sim_temperature_dma_config.buffer_size == 4U &&
           sim_temperature_dma_config.loop_mode_enable);
    assert(Adc_Start() && sim_dma_enabled && sim_temperature_dma_enabled && !Adc_Start());
    if (scenario >= 19U && scenario <= 21U)
    {
        assert(!McuAdc_PauseForStorage());
        unsigned key = BspSystem_EnterCritical();
        sim_power_off = false;
        assert(!McuAdc_PauseForStorage());
        sim_power_off = true;
        sim_storage_isr = 18U;
        assert(!McuAdc_PauseForStorage());
        sim_storage_isr = 0U;
        sim_storage_stop_stuck = scenario == 20U;
        bool paused = McuAdc_PauseForStorage();
        if (scenario == 20U)
        {
            assert(!paused && !Adc_IsHealthy());
            puts("PASS storage stop timeout remains faulted");
            return 0;
        }
        assert(paused && !Adc_IsHealthy() && !sim_dma_enabled);
        assert(!McuAdc_PauseForStorage());
        assert(sim_adc[0].edge == ADC_PREEMPT_TRIG_EDGE_NONE);
        assert(sim_adc[1].ordinary_edge == ADC_ORDINARY_TRIG_EDGE_NONE);
        sim_dma_flags = DMA1_HDT2_FLAG | DMA1_FDT2_FLAG | DMA1_DTERR2_FLAG;
        sim_adc[0].flags = ADC_PCCE_FLAG | ADC_TCF_FLAG;
        sim_adc[1].flags = ADC_OCCO_FLAG;
        sim_storage_auto_samples = scenario != 21U;
        bool resumed = McuAdc_ResumeAfterStorage();
        if (scenario == 21U)
        {
            assert(!resumed && !Adc_IsHealthy());
            puts("PASS missing fresh auxiliary sample prevents storage recovery");
            return 0;
        }
        assert(resumed && Adc_IsHealthy() && sim_dma_enabled);
        assert(sim_dma_channel.dtcnt == 2U * R40_AUX_RANKS);
        assert(sim_cleared_irqs & (1U << DMA1_Channel2_IRQn));
        sim_storage_auto_samples = false;
        assert(sim_adc[0].flags == ADC_PCCE_FLAG && sim_dma_flags == DMA1_HDT2_FLAG);
        assert(Adc_CompleteFast());
        DMA1_Channel2_IRQHandler();
        assert(Adc_IsHealthy() && s_sampling.started && !s_sampling.storage_owned);
        assert(!McuAdc_ResumeAfterStorage());
        BspSystem_ExitCritical(key);
        puts("PASS storage recovery discards pending data and waits for fresh samples");
        return 0;
    }
    /* 实际分频及rank配置须保留建立时长，并让母线电流先完成；不冒充模拟精度实测。 */
    const float adc_mhz = (float)R40_CORE_HZ / 1000000.0f / ((unsigned)sim_adc_common.div + 2U);
    assert(adc_mhz == 60.0f);
    float conversion_us = 0.0f;
    for (unsigned rank = 0U; rank < 4U; ++rank)
    {
        const float sample_cycles = sampling_cycles(sim_adc[0].sample_times[rank]);
        assert(sample_cycles / adc_mhz >= (rank < 3U ? 0.2f : 0.4f));
        conversion_us += 4.0f * (sample_cycles + 12.5f) / adc_mhz;
    }
    assert(conversion_us > 7.46f && conversion_us < 7.48f);
    const float bus_sample_us = sampling_cycles(sim_adc[1].sample_times[0]) / adc_mhz;
    assert(bus_sample_us > 1.54f && bus_sample_us + 12.5f / adc_mhz < conversion_us);
    for (unsigned rank = 0U; rank < 2U; ++rank)
    {
        assert(sampling_cycles(sim_adc[0].ordinary_sample_times[rank]) / adc_mhz >= 5.1f);
    }
    assert(sampling_cycles(sim_adc[1].ordinary_sample_times[0]) / adc_mhz >= 10.0f);
    /* 低压台架占空比44%..56%；16次转换须与两侧开关沿各留至少2us。 */
    const float trigger_us = (float)sim_adc_trigger_ticks / 180.0f;
    assert(trigger_us - 25.0f * 0.56f >= 2.0f);
    assert(50.0f - 25.0f * 0.56f - trigger_us - conversion_us >= 2.0f);
    puts("PASS ADC 60MHz: phase 12.5/bus voltage 24.5, 12-bit/4x sums, 7.47us budget");
    assert(sim_adc[0].edge == ADC_PREEMPT_TRIG_EDGE_RISING &&
           sim_adc[1].ordinary_edge == ADC_ORDINARY_TRIG_EDGE_RISING &&
           sim_adc[0].ordinary_edge == ADC_ORDINARY_TRIG_EDGE_RISING);
    assert(!Adc_CompleteFast());
    sampling_carrier_tick();
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
    if (scenario == 22U)
    {
        sim_adc[1].flags &= ~ADC_PCCE_FLAG;
    }
    if (scenario == 23U)
    {
        sim_adc[1].result[0] = 4096U;
    }
    bool fresh = Adc_CompleteFast();
    if (scenario == 4U || scenario == 7U || scenario == 13U || scenario == 22U || scenario == 23U)
    {
        assert(!fresh && !Adc_IsHealthy() && sim_stop_calls && !sim_dma_enabled);
        assert(McuAdc_ReadRank(BSP_SAMPLE_VBUS_RANK) == 0U);
        if (scenario == 22U || scenario == 23U)
        {
            puts("PASS ADC2 missing/range-invalid bus current cannot publish a fast frame");
        }
        puts("PASS corrupt/failed ADC sequence cannot publish");
        return 0;
    }
    assert(fresh && McuAdc_ReadRank(BSP_SAMPLE_PHASE_A_RANK) == 10U);
    assert(McuAdc_ReadRank(BSP_SAMPLE_PHASE_B_RANK) == 4102U);
    assert(McuAdc_ReadRank(BSP_SAMPLE_PHASE_C_RANK) == 16380U);
    assert(McuAdc_ReadRank(99U) == 0U);
    assert(McuAdc_ReadRank(BSP_SAMPLE_VBUS_RANK) == 8600U && !Adc_CompleteFast());
    /* 执行真实PA4原码与2mΩ母线换算；未校零无有效物理量，不能借用相电流尺度。 */
    BspAdcRawCurrent raw_current;
    BspAdcFastSample physical;
    assert(BspAdc_ReadRawCurrent(&raw_current) == BSP_OK);
    assert((raw_current.valid_mask & BSP_ADC_FAST_BUS_CURRENT_VALID) != 0U &&
           raw_current.bus_current_counts == 2050.0f);
    assert(BspAdc_ReadFast(&physical) == BSP_OK &&
           (physical.valid_mask & BSP_ADC_FAST_BUS_CURRENT_VALID) == 0U &&
           isnan(physical.bus_current_a));
    assert(BspAdc_SetBusCurrentOffset(2048.0f) == BSP_OK);
    assert(BspAdc_ReadFast(&physical) == BSP_OK &&
           (physical.valid_mask & BSP_ADC_FAST_BUS_CURRENT_VALID) != 0U);
    const float expected_bus_current_a =
        2.0f * R40_ADC_REFERENCE_V / R40_ADC_MAX_COUNTS / R40_BUS_CURRENT_GAIN / R40_BUS_SHUNT_OHM;
    assert(fabsf(physical.bus_current_a - expected_bus_current_a) < 0.000001f);
    puts("PASS ADC2 PA4 bus current: actual raw sample, offset validity and physical conversion");
    if (scenario == 18U)
    {
        sampling_carrier_tick();
        unsigned entries = sim_critical_enters;
        sim_cycle_counter.CYCCNT += R40_CORE_HZ / FOC_FREQ;
        sim_before_critical_enter = complete_pending_adc;
        sampling_carrier_tick();
        assert(sim_before_critical_enter == NULL && sim_critical_enters > entries);
        assert(sim_critical_enters == sim_critical_exits && Adc_IsHealthy());
        sim_cycle_counter.CYCCNT += R40_CORE_HZ / FOC_FREQ;
        sampling_carrier_tick();
        assert(!Adc_IsHealthy());
        assert(sim_critical_enters == sim_critical_exits && sim_stop_calls);
        puts("PASS atomic carrier snapshot accepts pending ADC and rejects real missing frame");
        return 0;
    }
    if (scenario == 17U)
    {
        /* 上次载波处理滞后于ADC，下次恢复正常顺序，且DWT跨32位回绕。 */
        sim_cycle_counter.CYCCNT = UINT32_MAX - 4000U;
        complete_samples();
        assert(Adc_CompleteFast());
        sampling_carrier_tick();
        sim_cycle_counter.CYCCNT = 1000U;
        sampling_carrier_tick();
        assert(Adc_IsHealthy());
        sim_cycle_counter.CYCCNT = 6000U;
        sampling_carrier_tick();
        assert(!Adc_IsHealthy() && sim_stop_calls);
        puts("PASS reordered carrier/ADC IRQ and DWT wrap; real stale sample stops");
        return 0;
    }
    sampling_carrier_tick();
    if (scenario == 6U)
    {
        sampling_carrier_tick();
        assert(!Adc_IsHealthy() && sim_stop_calls && !sim_dma_enabled);
        puts("PASS missing next ADC frame latches shutdown");
        return 0;
    }
    AdcAuxSample out = {0};
    assert(!Adc_ReadAux(&out));
    for (unsigned i = 0; i < 2U * R40_AUX_RANKS; ++i)
    {
        s_sampling.aux_dma[i] = 100U + i;
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
        s_sampling.aux_dma[0] = 4096U;
    }
    DMA1_Channel2_IRQHandler();
    if (scenario == 5U || scenario == 8U || scenario == 9U || scenario == 15U || scenario == 16U)
    {
        assert(!Adc_IsHealthy() && !Adc_ReadAux(&out) && !sim_dma_enabled);
        assert(s_aux.sequence == 0U);
        puts("PASS DMA loss/error/corruption cannot expose torn data");
        return 0;
    }
    assert(Adc_ReadAux(&out) && out.sequence == 1U);
    assert(out.motor_temperature_counts == 100U);
    DMA1_Channel2_IRQHandler();
    assert(Adc_ReadAux(&out) && out.sequence == 1U);
    sim_dma_flags = DMA1_FDT2_FLAG;
    DMA1_Channel2_IRQHandler();
    assert(Adc_ReadAux(&out) && out.sequence == 2U);
    assert(out.motor_temperature_counts == 100U + R40_AUX_RANKS);
    AdcTemperatureSample temperature = {0};
    assert(!Adc_ReadTemperature(&temperature));
    s_sampling.temperature_dma[0] = 1200U;
    s_sampling.temperature_dma[1] = 1500U;
    s_sampling.temperature_dma[2] = 1300U;
    s_sampling.temperature_dma[3] = 1501U;
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
        s_sampling.temperature_dma[0] = 4096U;
    }
    if (scenario == 14U)
    {
        s_temperature.sequence = UINT32_MAX;
    }
    DMA1_Channel3_IRQHandler();
    if (scenario >= 10U && scenario <= 12U)
    {
        assert(!Adc_ReadTemperature(&temperature) && !Adc_IsHealthy());
        assert(!sim_dma_enabled && !sim_temperature_dma_enabled && sim_power_off);
        assert(s_temperature.sequence == 0U);
        puts("PASS ADC1 temperature DMA error/loss/range failure stops both streams");
        return 0;
    }
    assert(Adc_ReadTemperature(&temperature) && temperature.mcu_temperature_counts == 1200U);
    assert(temperature.sequence == (scenario == 14U ? 0U : 1U));
    sim_dma_flags = DMA1_FDT3_FLAG;
    DMA1_Channel3_IRQHandler();
    assert(Adc_ReadTemperature(&temperature) && temperature.mcu_temperature_counts == 1300U);
    sim_ms = 3U;
    AdcAuxSample previous = out;
    assert(!Adc_ReadAux(&out) && memcmp(&out, &previous, sizeof(out)) == 0);
    assert(!Adc_ReadAux(NULL));
    assert(!Adc_ReadTemperature(&temperature) && !Adc_ReadTemperature(NULL));
    const float a_per_count =
        R40_ADC_REFERENCE_V / R40_ADC_MAX_COUNTS / R40_CURRENT_GAIN / R40_PHASE_SHUNT_OHM;
    const float v_per_count = R40_ADC_REFERENCE_V / R40_ADC_MAX_COUNTS *
                              (R40_VBUS_TOP_KOHM + R40_VBUS_BOTTOM_KOHM) / R40_VBUS_BOTTOM_KOHM;
    assert(a_per_count > 0.0100f && a_per_count < 0.0101f);
    assert(v_per_count > 0.0252f && v_per_count < 0.0253f);
    puts("PASS real four-conversion sums, coherent DMA halves and board scale");
    return 0;
}
