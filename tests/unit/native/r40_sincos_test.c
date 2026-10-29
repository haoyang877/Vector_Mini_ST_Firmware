/* 新板普通APP的ADC2采集回归；生产代码照常执行，仅寄存器/DMA事件由离线模型注入。 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "../../../firmware/platform/bsp/mcu/at32f456/adc.c"
#include "../../../firmware/platform/bsp/bsp_adc.c"
#include "../../../firmware/platform/bsp/mcu/at32f456/temperature.c"

static uint16_t hardware_average(uint16_t first)
{
    assert(sim_adc[1].ordinary_oversampled && sim_adc[1].ratio == ADC_OVERSAMPLE_RATIO_4 &&
           sim_adc[1].shift == ADC_OVERSAMPLE_SHIFT_2);
    return (uint16_t)((first + (first + 1U) + (first + 2U) + (first + 3U)) >> 2U);
}

static void fill_half(bool second, uint16_t sin_first, uint16_t cos_first)
{
    unsigned base = second ? R40_AUX_RANKS : 0U;
    s_sampling.aux_dma[base] = hardware_average(371U);
    s_sampling.aux_dma[base + 1U] = hardware_average(sin_first);
    s_sampling.aux_dma[base + 2U] = hardware_average(cos_first);
}

static void check_pair(uint32_t sin_counts, uint32_t cos_counts, uint32_t sequence)
{
    assert(g_r40_sincos_snapshot.begin != 0U && (g_r40_sincos_snapshot.begin & 1U) == 0U);
    assert(g_r40_sincos_snapshot.begin == g_r40_sincos_snapshot.end);
    assert(g_r40_sincos_snapshot.pa5_counts == sin_counts);
    assert(g_r40_sincos_snapshot.pa6_counts == cos_counts);
    assert(g_r40_sincos_snapshot.sequence == sequence);
}

int main(int argc, char **argv)
{
    unsigned scenario = argc > 1 ? (unsigned)strtoul(argv[1], NULL, 10) : 0U;
    assert(BSP_ENABLED_OUTPUT_SINCOS && BSP_ENABLED_MOTOR_TEMPERATURE);
    assert(Adc_Init() && Adc_Start());
    assert(sim_adc_common.div == ADC_HCLK_DIV_3 && sim_analog_pins == 0xFFU);
    assert(sim_adc[0].oversampled && sim_adc[0].ratio == ADC_OVERSAMPLE_RATIO_4 &&
           sim_adc[0].shift == ADC_OVERSAMPLE_SHIFT_0);
    assert(sim_adc[0].sample_times[0] == ADC_SAMPLETIME_12_5 &&
           sim_adc[0].sample_times[1] == ADC_SAMPLETIME_12_5 &&
           sim_adc[0].sample_times[2] == ADC_SAMPLETIME_12_5 &&
           sim_adc[0].sample_times[3] == ADC_SAMPLETIME_24_5);
    assert(sim_adc[1].ordinary_length == 3U && sim_dma_config.buffer_size == 6U);
    assert(sim_adc[1].ordinary_channels[0] == ADC_CHANNEL_7 &&
           sim_adc[1].ordinary_channels[1] == ADC_CHANNEL_5 &&
           sim_adc[1].ordinary_channels[2] == ADC_CHANNEL_6);
    assert(sim_adc[1].ordinary_sample_times[0] == ADC_SAMPLETIME_640_5 &&
           sim_adc[1].ordinary_sample_times[1] == ADC_SAMPLETIME_24_5 &&
           sim_adc[1].ordinary_sample_times[2] == ADC_SAMPLETIME_24_5);
    assert(!sim_adc[1].ordinary_oversample_per_trigger && !sim_adc[1].oversampled &&
           sim_adc[1].ordinary_oversample_recovery == ADC_OVERSAMPLE_CONTINUE);
    assert(sim_adc[1].channels[0] == ADC_CHANNEL_4 &&
           sim_adc[1].sample_times[0] == ADC_SAMPLETIME_92_5);
    assert(g_r40_sincos_snapshot.begin == 0U && g_r40_sincos_snapshot.end == 0U);
    assert(sim_adc[1].ordinary_trigger == ADC_ORDINARY_TRIG_TMR6TRGOUT &&
           sim_adc[1].ordinary_edge == ADC_ORDINARY_TRIG_EDGE_RISING);

    fill_half(false, 1000U, 2000U);
    sim_dma_flags = DMA1_HDT2_FLAG;
    DMA1_Channel2_IRQHandler();
    check_pair(1001U, 2001U, 1U);
    assert(BspAdc_UpdateAuxiliary() == BSP_OK);
    BspAdcRawValue ntc_raw;
    BspAdcValue ntc_temperature;
    assert(BspAdc_ReadRawChannel(BSP_ADC_MOTOR_TEMPERATURE, &ntc_raw) == BSP_OK &&
           ntc_raw.counts == 372.0f);
    assert(BspAdc_ReadChannel(BSP_ADC_MOTOR_TEMPERATURE, &ntc_temperature) == BSP_OK &&
           ntc_temperature.value > 24.9f && ntc_temperature.value < 25.1f);

    fill_half(true, 3000U, 1500U);
    sim_ms = 1U;
    sim_dma_flags = DMA1_FDT2_FLAG;
    DMA1_Channel2_IRQHandler();
    check_pair(3001U, 1501U, 2U);
    assert(g_r40_sincos_snapshot.milliseconds == 1U);

    /* ADC2抢占组保持单次原码，不受普通组四次平均/右移影响。 */
    sim_adc[0].flags = ADC_PCCE_FLAG;
    sim_adc[1].flags = ADC_PCCE_FLAG;
    sim_adc[1].result[0] = 2150U;
    assert(Adc_CompleteFast());
    BspAdcRawCurrent bus_raw;
    assert(BspAdc_ReadRawCurrent(&bus_raw) == BSP_OK && bus_raw.bus_current_counts == 2150.0f);

    if (scenario >= 1U && scenario <= 4U)
    {
        fill_half(false, 1000U, 2000U);
        sim_dma_flags = DMA1_HDT2_FLAG;
        if (scenario == 1U)
        {
            sim_dma_flags |= DMA1_DTERR2_FLAG;
        }
        else if (scenario == 2U)
        {
            sim_dma_flags |= DMA1_FDT2_FLAG;
        }
        else
        {
            s_sampling.aux_dma[scenario == 3U ? 1U : 2U] = 4096U;
        }
        DMA1_Channel2_IRQHandler();
        assert(!Adc_IsHealthy() && sim_power_off && !sim_dma_enabled);
        assert(g_r40_sincos_snapshot.begin == 0U && g_r40_sincos_snapshot.end == 0U);
        puts(
            "PASS sin/cos DMA loss/error/range fault invalidates both channels and stops sampling");
        return 0;
    }
    if (scenario == 5U)
    {
        uint32_t old_version = g_r40_sincos_snapshot.end;
        unsigned key = BspSystem_EnterCritical();
        assert(McuAdc_PauseForStorage());
        assert(g_r40_sincos_snapshot.begin == 0U && g_r40_sincos_snapshot.end == 0U);
        sim_storage_auto_samples = true;
        assert(McuAdc_ResumeAfterStorage());
        sim_storage_auto_samples = false;
        assert(Adc_CompleteFast());
        fill_half(false, 1100U, 2100U);
        DMA1_Channel2_IRQHandler();
        check_pair(1101U, 2101U, 3U);
        assert(g_r40_sincos_snapshot.end != old_version && Adc_IsHealthy());
        BspSystem_ExitCritical(key);
    }
    if (scenario == 6U)
    {
        s_sincos_version = UINT32_MAX - 1U;
        s_aux.sequence = UINT32_MAX;
        fill_half(false, 1200U, 2200U);
        sim_dma_flags = DMA1_HDT2_FLAG;
        DMA1_Channel2_IRQHandler();
        check_pair(1201U, 2201U, 0U);
        assert(g_r40_sincos_snapshot.end == 2U);
    }
    puts("PASS sin/cos 4x hardware average: paired halves, NTC units, bus current isolation and "
         "recovery");
    return 0;
}
