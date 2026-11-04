/* 完整编译生产ADC；替身只实现SDK和寄存器副作用。 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "../../../firmware/platform/bsp/mcu/at32f456/adc.c"
#include "../../../firmware/platform/bsp/mcu/at32f456/can.c"

int main(void)
{
    McuCan_Init();
    assert(sim_can.ctrlstat_bit.reset && sim_can.ctrlstat_bit.fd_iso);
    assert(sim_timing.bittime_div == 1U);
    assert(sim_timing.ac_bts1_size == 135U && sim_timing.ac_bts2_size == 45U);
    assert(sim_timing.fd_bts1_size == 27U && sim_timing.fd_bts2_size == 9U);
    assert(sim_filter.code_para.id_type == CAN_ID_EXTENDED);
    assert(sim_filter.code_para.frame_type == CAN_FRAME_DATA);
    assert(!sim_filter.mask_para.id_type && !sim_filter.mask_para.frame_type);
    assert(sim_can.inten == 0U);
    BspCan_Start();
    assert(!sim_can.ctrlstat_bit.reset);
    assert(sim_can.lbtcfg_bit.retlim == CAN_RE_TRANS_TIMES_1);
    assert(sim_can.lbtcfg_bit.realim == CAN_RE_ARBI_TIMES_UNLIMIT);
    McuAdc_Init();
    assert(sim_analog_pins == 0xFFU);
    assert(sim_adc_common.div == ADC_HCLK_DIV_3 && sim_adc_common.tempervintrv_state);
    assert(sim_adc[0].enabled && sim_adc[1].enabled);
    for (unsigned rank = 0; rank < 4; rank++)
    {
        assert(sim_adc[0].channels[rank] == rank);
    }
    assert(sim_adc[0].sample_times[0] == ADC_SAMPLETIME_12_5);
    assert(sim_adc[0].sample_times[3] == ADC_SAMPLETIME_24_5);
    assert(sim_adc[1].channels[0] == 4 && sim_adc[1].sample_times[0] == ADC_SAMPLETIME_92_5);
    assert(sim_adc[0].ordinary_channels[0] == 16 && sim_adc[0].ordinary_channels[1] == 17);
    assert(sim_adc[1].ordinary_channels[0] == 7 && sim_adc[1].ordinary_channels[1] == 5 &&
           sim_adc[1].ordinary_channels[2] == 6);
    assert(sim_adc[0].oversampled && sim_adc[0].ratio == ADC_OVERSAMPLE_RATIO_4 &&
           sim_adc[0].shift == 0);
    assert(!sim_adc[1].oversampled && sim_adc[1].ordinary_oversampled);
    assert(sim_dma_config.buffer_size == 6 && sim_temperature_dma_config.buffer_size == 4);
    assert(!sim_dma_enabled && !sim_temperature_dma_enabled);
    Adc_Start();
    assert(sim_dma_enabled && sim_temperature_dma_enabled);
    assert(sim_adc[0].edge == ADC_PREEMPT_TRIG_EDGE_RISING &&
           sim_adc[1].edge == ADC_PREEMPT_TRIG_EDGE_RISING);
    assert(!BspAdc_CompleteFast());
    sim_adc[0].flags = ADC_PCCE_FLAG;
    assert(!BspAdc_CompleteFast() && !(sim_adc[0].flags & ADC_PCCE_FLAG));
    sim_adc[0].flags = sim_adc[1].flags = ADC_PCCE_FLAG;
    assert(BspAdc_CompleteFast() && !(sim_adc[0].flags & ADC_PCCE_FLAG) &&
           !(sim_adc[1].flags & ADC_PCCE_FLAG));
    for (unsigned remaining = 0; remaining <= 6; remaining++)
    {
        s_aux_dma[0] = 372;
        s_aux_dma[3] = 2000;
        sim_dma_channel.dtcnt = remaining;
        assert(BspAdc_ReadRawChannel(BSP_ADC_MOTOR_TEMPERATURE) ==
               (remaining == 0 || remaining > 3 ? 2000 : 372));
    }
    sim_dma_channel.dtcnt = 3;
    s_aux_dma[0] = 372;
    assert(fabsf(BspAdc_ReadChannel(BSP_ADC_MOTOR_TEMPERATURE) - 25.0f) < 0.1f);
    s_aux_dma[0] = 0;
    assert(isnan(BspAdc_ReadChannel(BSP_ADC_MOTOR_TEMPERATURE)));
    s_aux_dma[0] = 4095;
    assert(isnan(BspAdc_ReadChannel(BSP_ADC_MOTOR_TEMPERATURE)));
    Adc_Stop();
    assert(!sim_dma_enabled && !sim_temperature_dma_enabled);
    assert(sim_adc[0].edge == ADC_PREEMPT_TRIG_EDGE_NONE &&
           sim_adc[1].edge == ADC_PREEMPT_TRIG_EDGE_NONE);
    Adc_Start();
    assert(sim_dma_channel.dtcnt == 6 && sim_temperature_dma_channel.dtcnt == 4);
    assert(sim_dma_enabled && sim_temperature_dma_enabled);
    puts("PASS full AT32 CAN initialization and ADC: ranks, trigger, accumulation domains, DMA "
         "halves, NTC, stop/restart");
    return 0;
}
