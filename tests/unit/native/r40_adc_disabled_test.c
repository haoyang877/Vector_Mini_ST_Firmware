/* 实际端口的温度禁用配置：不使用PA7/内部传感器/DMA3，但继续电流、电压和母线采样。 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../../../firmware/platform/at32f456/ports/motor_sensing_at32f456.c"
#include "../../../firmware/platform/at32f456/ports/temperature_at32f456.c"
#include "../../../firmware/platform/at32f456/ports/motor_temperature_at32f456.c"

int main(void)
{
    assert(R40_TEMPERATURE_SENSING_ENABLED == 0);
    assert(R40_MOTOR_NTC_ENABLED == 0);
    assert(R40_AdcInit());
    assert(sim_analog_pins == (R40_OUTPUT_SINCOS_DIAGNOSTICS ? 0x7FU : 0x1FU) &&
           !sim_adc_common.tempervintrv_state);
    assert(sim_adc[1].ordinary_length == R40_AUX_RANKS && sim_adc[1].ordinary_channels[0] == 4U);
#if R40_OUTPUT_SINCOS_DIAGNOSTICS
    assert(sim_adc[1].ordinary_channels[1] == 5U && sim_adc[1].ordinary_channels[2] == 6U);
#else
    assert(sim_adc[1].ordinary_channels[1] == 0U);
#endif
    assert(sim_adc[0].ordinary_channels[0] == 0U && !sim_adc[0].dma_enabled);
    assert(sim_temperature_dma_config.buffer_size == 0U);
    assert(sim_dma_config.buffer_size == 2U * R40_AUX_RANKS);
    assert(R40_AdcStart() && sim_dma_enabled && !sim_temperature_dma_enabled);
    assert(sim_adc[0].ordinary_edge == ADC_ORDINARY_TRIG_EDGE_NONE);
    assert(sim_adc[0].edge == ADC_PREEMPT_TRIG_EDGE_RISING);
    assert(sim_adc[1].ordinary_edge == ADC_ORDINARY_TRIG_EDGE_RISING);

    MotorHwTemperaturePoll poll = {true, true, 12U, 34U, 56.0f, 78.0f};
    motor_hw_temperature_poll(&poll);
    assert(!poll.sample_ready && !poll.conversion_ok);
    assert(poll.raw_ts == 12U && poll.raw_vref == 34U && poll.celsius == 56.0f);
    R40TemperatureSamples temp = {12U, 34U, 56U};
    assert(!R40_AdcReadTemperature(&temp) && temp.sequence == 56U);
    R40MotorTemperatureSample motor_temp;
    assert(!R40_MotorTemperaturePoll(&motor_temp));
    assert(motor_temp.status == R40_MOTOR_TEMPERATURE_DISABLED && isnan(motor_temp.celsius));
    sim_dma_flags = DMA1_FDT3_FLAG;
    DMA1_Channel3_IRQHandler();
    assert(R40_AdcHealthy() && !sim_temperature_dma_enabled);

    s_aux_dma[0] = 101U;
    s_aux_dma[R40_AUX_RANKS] = 202U;
#if R40_OUTPUT_SINCOS_DIAGNOSTICS
    s_aux_dma[1] = 1500U;
    s_aux_dma[2] = 2500U;
#endif
    sim_dma_flags = DMA1_HDT2_FLAG;
    DMA1_Channel2_IRQHandler();
    R40AuxSamples aux;
    assert(R40_AdcReadAux(&aux) && aux.bus_current_counts == 101U &&
           aux.motor_temperature_counts == 0U);
#if R40_OUTPUT_SINCOS_DIAGNOSTICS
    assert(g_r40_sincos_snapshot.pa5_counts == 1500U && g_r40_sincos_snapshot.pa6_counts == 2500U);
#endif
    sim_dma_flags = DMA1_FDT2_FLAG;
    DMA1_Channel2_IRQHandler();
    assert(R40_AdcReadAux(&aux) && aux.bus_current_counts == 202U && aux.sequence == 2U);
    for (unsigned i = 0; i < 4U; ++i)
    {
        sim_adc[0].result[i] = (uint16_t)(1000U + i);
    }
    sim_adc[0].flags = ADC_PCCE_FLAG;
    assert(R40_AdcComplete());
    assert(motor_hw_current_sample_raw(MOTOR_HW_CURRENT_PHASE_C) == 1002U);
    assert(motor_hw_vbus_sample_raw() == 1003U);
    s_aux_dma[0] = 4096U;
    sim_dma_flags = DMA1_HDT2_FLAG;
    DMA1_Channel2_IRQHandler();
    assert(!R40_AdcHealthy() && sim_power_off && !sim_dma_enabled);
    puts("PASS temperature disabled: PA7/TS/VREF/DMA3 inactive, no fake temperature, ADC bus and "
         "phase sampling retained");
    return 0;
}
