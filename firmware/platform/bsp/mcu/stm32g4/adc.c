/**
 * @file adc.c
 * @brief STM32G4 ADC硬件采样、启停、中断和片内校准值读取。
 */
#include "../../bsp_internal.h"
#include "bsp_adc.h"
#include "board_config.h"
#include "adc.h"
#include "tim.h"
#include "stm32g4xx_ll_adc.h"
#include <math.h>

static volatile float s_phase_offsets[3] = {NAN, NAN, NAN};
static volatile float s_bus_offset = NAN;

static void calibrate(ADC_HandleTypeDef *adc)
{
    HAL_ADCEx_Calibration_Start(adc, ADC_SINGLE_ENDED);
    /* ADC时钟为HCLK/4，校准结束后等待至少4个ADC周期再使能。 */
    for (uint32_t cycles = 0U; cycles < 4U * LL_ADC_DELAY_CALIB_ENABLE_ADC_CYCLES; ++cycles)
    {
        __NOP();
    }
    LL_ADC_Enable(adc->Instance);
    while (!LL_ADC_IsActiveFlag_ADRDY(adc->Instance))
    {
    }
}

void McuAdc_Init(void)
{
    MX_ADC2_Init();
#if BSP_ENABLED_INTERNAL_ANALOG_SAMPLING
    MX_ADC1_Init();
    calibrate(&hadc1);
#endif
    calibrate(&hadc2);
}

void Adc_Start(void)
{
    ADC2->ISR = ADC_ISR_JEOS | ADC_ISR_JEOC;
    NVIC_ClearPendingIRQ(ADC1_2_IRQn);
    ADC2->IER = (ADC2->IER & ~ADC_IER_JEOCIE) | ADC_IER_JEOSIE;
#if BSP_ENABLED_INTERNAL_ANALOG_SAMPLING
    ADC1->ISR = ADC_ISR_JEOS | ADC_ISR_JEOC;
    LL_ADC_INJ_StartConversion(ADC1);
#endif
    LL_ADC_INJ_StartConversion(ADC2);
    TIM1->CCER |= TIM_CCER_CC4E;
}

void Adc_Stop(void)
{
    ADC2->IER &= ~ADC_IER_JEOSIE;
    if (LL_ADC_INJ_IsConversionOngoing(ADC2))
    {
        LL_ADC_INJ_StopConversion(ADC2);
        while (LL_ADC_INJ_IsConversionOngoing(ADC2))
        {
        }
    }
#if BSP_ENABLED_INTERNAL_ANALOG_SAMPLING
    if (LL_ADC_INJ_IsConversionOngoing(ADC1))
    {
        LL_ADC_INJ_StopConversion(ADC1);
        while (LL_ADC_INJ_IsConversionOngoing(ADC1))
        {
        }
    }
    ADC1->ISR = ADC_ISR_JEOS | ADC_ISR_JEOC;
#endif
    ADC2->ISR = ADC_ISR_JEOS | ADC_ISR_JEOC;
}

bool Adc_CompleteFast(void)
{
    /* HAL完成回调返回后清JEOS，应用读取时寄存器仍保存本拍结果。 */
    return (ADC2->ISR & ADC_ISR_JEOS) != 0U;
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *adc)
{
    if (adc == &hadc2 && Adc_CompleteFast())
    {
        BspApp_OnFastSample();
    }
}

float BspAdc_ReadRawChannel(BspAdcChannel channel)
{
    switch (channel)
    {
    case BSP_ADC_PHASE_CURRENT_A:
        return (float)ADC2->JDR1 * BSP_ADC_SUM_TO_COUNTS;
    case BSP_ADC_PHASE_CURRENT_B:
        return (float)ADC2->JDR2 * BSP_ADC_SUM_TO_COUNTS;
    case BSP_ADC_PHASE_CURRENT_C:
        return (float)ADC2->JDR3 * BSP_ADC_SUM_TO_COUNTS;
    case BSP_ADC_BUS_VOLTAGE:
        return (float)ADC2->JDR4 * BSP_ADC_SUM_TO_COUNTS;
#if BSP_ENABLED_INTERNAL_ANALOG_SAMPLING
    case BSP_ADC_MCU_TEMPERATURE:
    case BSP_ADC_ANALOG_SUPPLY_VOLTAGE:
        return channel == BSP_ADC_MCU_TEMPERATURE ? (float)ADC1->JDR1 : (float)ADC1->JDR2;
#endif
    default:
        return NAN;
    }
}

float BspAdc_ReadChannel(BspAdcChannel channel)
{
    float counts = BspAdc_ReadRawChannel(channel);
    switch (channel)
    {
    case BSP_ADC_PHASE_CURRENT_A:
    case BSP_ADC_PHASE_CURRENT_B:
    case BSP_ADC_PHASE_CURRENT_C:
        return -(counts - s_phase_offsets[channel]) * BSP_ADC_CURRENT_A_PER_COUNT;
    case BSP_ADC_BUS_VOLTAGE:
        return counts * BSP_ADC_VBUS_V_PER_COUNT;
#if BSP_ENABLED_BUS_CURRENT
    case BSP_ADC_BUS_CURRENT:
        return (counts - s_bus_offset) * (BUS_CURRENT_ADC_REFERENCE_V / BUS_CURRENT_ADC_MAX_COUNTS /
                                          BUS_CURRENT_GAIN / BUS_CURRENT_SHUNT_OHM);
#endif
#if BSP_ENABLED_MOTOR_TEMPERATURE
    case BSP_ADC_MOTOR_TEMPERATURE:
    {
        if (counts == 0.0f || counts >= CURRENT_SENSE_PROFILE_ADC_MAX_COUNTS)
        {
            return NAN;
        }
        float resistance_ohm =
            BSP_ADC_NTC_PULLDOWN_OHM * (CURRENT_SENSE_PROFILE_ADC_MAX_COUNTS - counts) / counts;
        return 1.0f / (1.0f / 298.15f +
                       logf(resistance_ohm / BSP_ADC_NTC_R25_OHM) / BSP_ADC_NTC_BETA_K) -
               273.15f;
    }
#endif
    case BSP_ADC_ANALOG_SUPPLY_VOLTAGE:
        if (counts == 0.0f)
        {
            return NAN;
        }
        return 3.0f * (float)*VREFINT_CAL_ADDR / counts;
    case BSP_ADC_MCU_TEMPERATURE:
    {
        float reference_counts = BspAdc_ReadRawChannel(BSP_ADC_ANALOG_SUPPLY_VOLTAGE);
        float cal30 = (float)*TEMPSENSOR_CAL1_ADDR;
        float cal130 = (float)*TEMPSENSOR_CAL2_ADDR;
        if (reference_counts == 0.0f || cal130 == cal30)
        {
            return NAN;
        }
        float normalized = counts * (float)*VREFINT_CAL_ADDR / reference_counts;
        return 30.0f + (normalized - cal30) * 100.0f / (cal130 - cal30);
    }
    default:
        return NAN;
    }
}

void BspAdc_SetPhaseOffsets(const float offsets_counts[3])
{
    for (unsigned phase = 0U; phase < 3U; ++phase)
    {
        s_phase_offsets[phase] = offsets_counts[phase];
    }
}

void BspAdc_SetBusCurrentOffset(float offset_counts)
{
    s_bus_offset = offset_counts;
}
