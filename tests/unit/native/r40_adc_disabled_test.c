/* 实际端口的温度禁用配置：不使用PA7/内部传感器/DMA3，但继续电流、电压和母线采样。 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../../../firmware/platform/bsp/mcu/at32f456/adc.c"
#include "../../../firmware/platform/bsp/bsp_adc.c"
#include "../../../firmware/platform/bsp/mcu/at32f456/temperature.c"

int main(void)
{
    assert(BSP_ENABLED_MCU_TEMPERATURE == 0);
    assert(BSP_ENABLED_MOTOR_TEMPERATURE == 0);
    assert(Adc_Init());
    assert(sim_analog_pins == 0x1FU && !sim_adc_common.tempervintrv_state);
    assert(sim_adc[1].channels[0] == 4U);
    assert(sim_adc[1].ordinary_channels[1] == 0U);
    assert(sim_adc[0].ordinary_channels[0] == 0U && !sim_adc[0].dma_enabled);
    assert(sim_temperature_dma_config.buffer_size == 0U);
    assert(sim_dma_config.buffer_size == 2U * R40_AUX_RANKS);
    assert(Adc_Start() && sim_dma_enabled == !!BSP_ENABLED_AUX_SAMPLING &&
           !sim_temperature_dma_enabled);
    assert(sim_adc[0].ordinary_edge == ADC_ORDINARY_TRIG_EDGE_NONE);
    assert(sim_adc[0].edge == ADC_PREEMPT_TRIG_EDGE_RISING);
    assert(sim_adc[1].edge == ADC_PREEMPT_TRIG_EDGE_RISING);

    BspAdcRawValue raw;
    assert(BspAdc_UpdateAuxiliary() == BSP_NOT_SUPPORTED);
    assert(BspAdc_ReadRawChannel(BSP_ADC_MCU_TEMPERATURE, &raw) == BSP_NOT_SUPPORTED);
    AdcTemperatureSample temp = {12U, 34U, 56U, 0U};
    assert(!Adc_ReadTemperature(&temp) && temp.sequence == 56U);
    sim_dma_flags = DMA1_FDT3_FLAG;
    DMA1_Channel3_IRQHandler();
    assert(Adc_IsHealthy() && !sim_temperature_dma_enabled);

    s_sampling.aux_dma[0] = 101U;
    s_sampling.aux_dma[R40_AUX_RANKS] = 202U;
    sim_dma_flags = DMA1_HDT2_FLAG;
    DMA1_Channel2_IRQHandler();
    AdcAuxSample aux;
    assert(Adc_ReadAux(&aux) == !!BSP_ENABLED_AUX_SAMPLING);
    sim_dma_flags = DMA1_FDT2_FLAG;
    DMA1_Channel2_IRQHandler();
    assert(Adc_ReadAux(&aux) == !!BSP_ENABLED_AUX_SAMPLING);
    for (unsigned i = 0; i < 4U; ++i)
    {
        sim_adc[0].result[i] = (uint16_t)(1000U + i);
    }
    sim_adc[0].flags = ADC_PCCE_FLAG;
    sim_adc[1].flags |= ADC_PCCE_FLAG;
    assert(Adc_CompleteFast());
    assert(McuAdc_ReadRank(BSP_SAMPLE_PHASE_C_RANK) == 1002U);
    assert(McuAdc_ReadRank(BSP_SAMPLE_VBUS_RANK) == 1003U);
    unsigned key = BspSystem_EnterCritical();
    assert(McuAdc_PauseForStorage());
    assert(!sim_dma_enabled && !sim_temperature_dma_enabled);
    sim_storage_auto_samples = true;
    assert(McuAdc_ResumeAfterStorage());
    sim_storage_auto_samples = false;
    assert(sim_dma_enabled == !!BSP_ENABLED_AUX_SAMPLING && !sim_temperature_dma_enabled);
    assert(sim_adc[0].ordinary_edge == ADC_ORDINARY_TRIG_EDGE_NONE);
    assert(Adc_CompleteFast());
    DMA1_Channel2_IRQHandler();
    assert(Adc_IsHealthy());
    BspSystem_ExitCritical(key);
    s_sampling.aux_dma[0] = 4096U;
    sim_dma_flags = DMA1_HDT2_FLAG;
    DMA1_Channel2_IRQHandler();
    assert(Adc_IsHealthy() == !BSP_ENABLED_AUX_SAMPLING && sim_power_off && !sim_dma_enabled);
    puts("PASS temperature disabled: PA7/TS/VREF/DMA3 inactive, no fake temperature, ADC bus and "
         "phase sampling retained");
    return 0;
}
