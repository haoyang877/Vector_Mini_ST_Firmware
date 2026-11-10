/* 完整编译生产ADC；替身只实现SDK和寄存器副作用。 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "../../../firmware/platform/bsp/mcu/at32f456/adc.c"
#include "../../../firmware/platform/bsp/mcu/at32f456/can.c"

void BspApp_OnCanReceive(void)
{
    assert(0 && "unexpected CAN receive");
}

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
    assert(sim_adc_common.div == ADC_HCLK_DIV_3 && !sim_adc_common.tempervintrv_state);
    assert(sim_adc[0].enabled && sim_adc[1].enabled);
    assert(sim_adc[0].length == 3);
    for (unsigned rank = 0; rank < 3; rank++)
    {
        assert(sim_adc[0].channels[rank] == rank);
    }
    assert(sim_adc[0].sample_times[0] == ADC_SAMPLETIME_12_5);
    assert(sim_adc[1].channels[3] == 3 && sim_adc[1].sample_times[3] == ADC_SAMPLETIME_24_5);
    assert(sim_adc[1].channels[0] == 4 && sim_adc[1].sample_times[0] == ADC_SAMPLETIME_92_5);
    assert(sim_adc[1].length == 4 && sim_adc[1].channels[1] == 5 && sim_adc[1].channels[2] == 6);
    assert(sim_adc[1].ordinary_length == 1 && sim_adc[1].ordinary_channels[0] == 7);
    assert(sim_adc[0].oversampled && sim_adc[0].ratio == ADC_OVERSAMPLE_RATIO_4 &&
           sim_adc[0].shift == 0);
    assert(!sim_adc[1].oversampled && !sim_adc[1].ordinary_oversampled);
    Adc_Start();
    assert(!sim_dma_enabled && !sim_temperature_dma_enabled);
    assert(sim_adc[0].edge == ADC_PREEMPT_TRIG_EDGE_RISING &&
           sim_adc[1].edge == ADC_PREEMPT_TRIG_EDGE_RISING);
    assert(sim_adc[0].trigger == ADC_PREEMPT_TRIG_TMR1TRGOUT &&
           sim_adc[1].trigger == ADC_PREEMPT_TRIG_TMR1TRGOUT);
    assert(sim_adc[1].ordinary_trigger == ADC_ORDINARY_TRIG_TMR6TRGOUT &&
           sim_adc[1].ordinary_edge == ADC_ORDINARY_TRIG_EDGE_RISING);
    assert(!BspAdc_CompleteFast());
    sim_adc[0].flags = ADC_PCCE_FLAG;
    assert(!BspAdc_CompleteFast() && !(sim_adc[0].flags & ADC_PCCE_FLAG));
    sim_adc[0].flags = sim_adc[1].flags = ADC_PCCE_FLAG;
    assert(BspAdc_CompleteFast() && !(sim_adc[0].flags & ADC_PCCE_FLAG) &&
           !(sim_adc[1].flags & ADC_PCCE_FLAG));
    ADC1->pdt4 = 0;
    ADC2->pdt4 = 1206;
    assert(BspAdc_ReadRawChannel(BSP_ADC_BUS_VOLTAGE) == 1206);
    ADC2->pdt1 = 1999;
    ADC2->pdt2 = 1234;
    ADC2->pdt3 = 2345;
    assert(BspAdc_ReadRawChannel(BSP_ADC_BUS_CURRENT) == 1999);
    assert(BspAdc_ReadRawChannel(BSP_ADC_LINEAR_HALL_SIN) == 1234);
    assert(BspAdc_ReadRawChannel(BSP_ADC_LINEAR_HALL_COS) == 2345);
    ADC2->pdt2 = 0;
    ADC2->pdt3 = 4095;
    assert(BspAdc_ReadRawChannel(BSP_ADC_LINEAR_HALL_SIN) == 0);
    assert(BspAdc_ReadRawChannel(BSP_ADC_LINEAR_HALL_COS) == 4095);
    ADC2->odt = 372;
    assert(BspAdc_ReadRawChannel(BSP_ADC_MOTOR_TEMPERATURE) == 372);
    assert(fabsf(BspAdc_ReadChannel(BSP_ADC_MOTOR_TEMPERATURE) - 25.0f) < 0.1f);
    float ntc = -99.0f;
    assert(!BspAdc_ReadMotorTemperature(&ntc) && ntc == -99.0f);
    sim_adc[1].flags |= ADC_OCCE_FLAG;
    assert(BspAdc_ReadMotorTemperature(&ntc) && fabsf(ntc - 25.0f) < 0.1f);
    assert(!BspAdc_ReadMotorTemperature(&ntc));
    ADC2->odt = 0;
    assert(isnan(BspAdc_ReadChannel(BSP_ADC_MOTOR_TEMPERATURE)));
    sim_adc[1].flags |= ADC_OCCE_FLAG;
    assert(BspAdc_ReadMotorTemperature(&ntc) && isnan(ntc));
    Adc_Stop();
    assert(!sim_dma_enabled && !sim_temperature_dma_enabled);
    assert(sim_adc[0].edge == ADC_PREEMPT_TRIG_EDGE_NONE &&
           sim_adc[1].edge == ADC_PREEMPT_TRIG_EDGE_NONE);
    Adc_Start();
    assert(sim_adc[1].edge == ADC_PREEMPT_TRIG_EDGE_RISING);
    puts("PASS AT32 CAN and ADC: shared fast trigger, single Hall conversions, raw NTC, "
         "stop/restart");
    return 0;
}
