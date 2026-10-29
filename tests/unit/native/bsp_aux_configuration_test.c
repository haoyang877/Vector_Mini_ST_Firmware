/* 执行真实ADC初始化/启动/存储恢复；不同辅助组合不得访问已禁用采集资源。 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../../../firmware/platform/bsp/mcu/at32f456/adc.c"
#include "../../../firmware/platform/bsp/bsp_adc.c"
#include "../../../firmware/platform/bsp/mcu/at32f456/temperature.c"

int main(void)
{
    assert(Adc_Init());
    unsigned expected_pins = 0xFU | (BSP_ENABLED_BUS_CURRENT ? 0x10U : 0U) |
                             (BSP_ENABLED_MOTOR_TEMPERATURE ? 0x80U : 0U) |
                             (BSP_ENABLED_OUTPUT_SINCOS ? 0x60U : 0U);
    assert(sim_analog_pins == expected_pins);
    assert(sim_adc[1].enabled == !!(BSP_ENABLED_AUX_SAMPLING || BSP_ENABLED_BUS_CURRENT));
    assert(sim_adc[1].ordinary_length == (R40_AUX_RANKS             ? R40_AUX_RANKS
                                          : BSP_ENABLED_BUS_CURRENT ? 1U
                                                                    : 0U));
    assert(sim_dma_config.buffer_size == 2U * R40_AUX_RANKS);
    assert(sim_temperature_dma_config.buffer_size == (BSP_ENABLED_MCU_TEMPERATURE ? 4U : 0U));
    assert(Adc_Start());
    assert(sim_dma_enabled == !!BSP_ENABLED_AUX_SAMPLING);
    assert(sim_temperature_dma_enabled == !!BSP_ENABLED_MCU_TEMPERATURE);
    assert(((sim_nvic_enabled >> DMA1_Channel2_IRQn) & 1U) == !!BSP_ENABLED_AUX_SAMPLING);
    assert(((sim_nvic_enabled >> DMA1_Channel3_IRQn) & 1U) == !!BSP_ENABLED_MCU_TEMPERATURE);
    AdcAuxSample sample = {456U, 789U, 0U};
#if BSP_ENABLED_AUX_SAMPLING
    for (unsigned i = 0U; i < R40_AUX_RANKS; ++i)
    {
        s_sampling.aux_dma[i] = (uint16_t)(1000U + i);
    }
    sim_dma_flags = DMA1_HDT2_FLAG;
    DMA1_Channel2_IRQHandler();
    assert(Adc_ReadAux(&sample));
    assert(sample.motor_temperature_counts == (BSP_ENABLED_MOTOR_TEMPERATURE ? 1000U : 0U));
    assert(sample.sequence == 1U);
#else
    DMA1_Channel2_IRQHandler();
    assert(!Adc_ReadAux(&sample));
    assert(sample.motor_temperature_counts == 456U && sample.sequence == 789U);
    /* 禁用的ADC2即便残留错误位，也不能污染仍启用的ADC1快速采样。 */
    sim_adc[1].flags = BSP_ENABLED_BUS_CURRENT ? ADC_PCCE_FLAG : ADC_TCF_FLAG | ADC_OCCO_FLAG;
    sim_adc[0].flags = ADC_PCCE_FLAG;
    assert(Adc_CompleteFast() && Adc_IsHealthy());
    sim_adc[1].flags = 0U;
#endif
    unsigned key = BspSystem_EnterCritical();
    assert(McuAdc_PauseForStorage());
    sim_storage_auto_samples = true;
    assert(McuAdc_ResumeAfterStorage());
    assert(sim_dma_enabled == !!BSP_ENABLED_AUX_SAMPLING);
    assert(sim_temperature_dma_enabled == !!BSP_ENABLED_MCU_TEMPERATURE);
    assert(Adc_IsHealthy());
    BspSystem_ExitCritical(key);
    puts("PASS optional ADC configuration: pins, ranks, DMA, IRQ, sample, storage recovery");
    return 0;
}
