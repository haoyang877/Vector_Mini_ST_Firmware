/**
 * @file adc.c
 * @brief STM32G4 ADC硬件采样、启停、中断和片内校准值读取。
 */
#include "../../bsp_internal.h"
#include "bsp_adc.h"
#include "mcu_resources.h"
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
    MCU_ADC_FAST_INIT();
#if BSP_ENABLED_INTERNAL_ANALOG_SAMPLING
    MCU_ADC_INTERNAL_INIT();
    calibrate(&MCU_ADC_INTERNAL_HANDLE);
#endif
    calibrate(&MCU_ADC_FAST_HANDLE);
}

void Adc_Start(void)
{
    MCU_ADC_FAST->ISR = ADC_ISR_JEOS | ADC_ISR_JEOC;
    NVIC_ClearPendingIRQ(MCU_ADC_IRQ);
    MCU_ADC_FAST->IER = (MCU_ADC_FAST->IER & ~ADC_IER_JEOCIE) | ADC_IER_JEOSIE;
#if BSP_ENABLED_INTERNAL_ANALOG_SAMPLING
    MCU_ADC_INTERNAL->ISR = ADC_ISR_JEOS | ADC_ISR_JEOC;
    LL_ADC_INJ_StartConversion(MCU_ADC_INTERNAL);
#endif
    LL_ADC_INJ_StartConversion(MCU_ADC_FAST);
    MCU_PWM_TIMER->CCER |= MCU_ADC_TRIGGER_ENABLE_MASK;
}

void Adc_Stop(void)
{
    MCU_ADC_FAST->IER &= ~ADC_IER_JEOSIE;
    if (LL_ADC_INJ_IsConversionOngoing(MCU_ADC_FAST))
    {
        LL_ADC_INJ_StopConversion(MCU_ADC_FAST);
        while (LL_ADC_INJ_IsConversionOngoing(MCU_ADC_FAST))
        {
        }
    }
#if BSP_ENABLED_INTERNAL_ANALOG_SAMPLING
    if (LL_ADC_INJ_IsConversionOngoing(MCU_ADC_INTERNAL))
    {
        LL_ADC_INJ_StopConversion(MCU_ADC_INTERNAL);
        while (LL_ADC_INJ_IsConversionOngoing(MCU_ADC_INTERNAL))
        {
        }
    }
    MCU_ADC_INTERNAL->ISR = ADC_ISR_JEOS | ADC_ISR_JEOC;
#endif
    MCU_ADC_FAST->ISR = ADC_ISR_JEOS | ADC_ISR_JEOC;
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *adc)
{
    /* HAL完成回调返回后才清JEOS，应用读取时寄存器仍保存本拍结果。 */
    if (adc == &MCU_ADC_FAST_HANDLE && (MCU_ADC_FAST->ISR & ADC_ISR_JEOS) != 0U)
    {
        BspApp_OnFastSample();
    }
}

float BspAdc_ReadRawChannel(BspAdcChannel channel)
{
    switch (channel)
    {
    case BSP_ADC_PHASE_CURRENT_A:
        return (float)MCU_ADC_PHASE_A_RESULT * BSP_ADC_SUM_TO_COUNTS;
    case BSP_ADC_PHASE_CURRENT_B:
        return (float)MCU_ADC_PHASE_B_RESULT * BSP_ADC_SUM_TO_COUNTS;
    case BSP_ADC_PHASE_CURRENT_C:
        return (float)MCU_ADC_PHASE_C_RESULT * BSP_ADC_SUM_TO_COUNTS;
    case BSP_ADC_BUS_VOLTAGE:
        return (float)MCU_ADC_BUS_VOLTAGE_RESULT * BSP_ADC_SUM_TO_COUNTS;
#if BSP_ENABLED_INTERNAL_ANALOG_SAMPLING
    case BSP_ADC_MCU_TEMPERATURE:
    case BSP_ADC_ANALOG_SUPPLY_VOLTAGE:
        return channel == BSP_ADC_MCU_TEMPERATURE ? (float)MCU_ADC_TEMPERATURE_RESULT
                                                  : (float)MCU_ADC_REFERENCE_RESULT;
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

bool BspAdc_ReadMotorTemperature(float *celsius)
{
    /* STM32G431当前产品未安装电机NTC，不能用片内温度替代。 */
    (void)celsius;
    return false;
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
