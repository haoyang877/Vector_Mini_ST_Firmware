#include "bsp_system.h"
#include "bsp_pwm.h"
#include "bsp_internal.h"

#include <stddef.h>
#include "adc.h"
#include "tim.h"
#include "stm32g4xx_ll_adc.h"

/* CubeMX先准备外设；本模块拥有校准、采样启动和原码读取，保持原HAL调用顺序。 */
static bool s_sampling_started, s_sampling_fault, s_storage_owned, s_storage_was_started;

bool McuAdc_Init(void)
{
    /* 保持旧板原有HAL错误处理，不借职责搬迁增加或删除启动条件。 */
    if (BSP_ENABLED_INTERNAL_ANALOG_SAMPLING)
    {
        HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    }
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
    return true;
}

bool McuAdc_Start(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    if (BSP_ENABLED_INTERNAL_ANALOG_SAMPLING)
    {
        HAL_ADCEx_InjectedStart(&hadc1);
    }
    HAL_ADCEx_InjectedStart(&hadc2);
    __HAL_ADC_DISABLE_IT(&hadc2, ADC_IT_JEOC);
    __HAL_ADC_ENABLE_IT(&hadc2, ADC_IT_JEOS);
    s_sampling_started = true;
    return true;
}

bool McuAdc_CompleteFast(void)
{
    /* HAL回调之后才清JEOS，保持既有IRQ确认与APP执行顺序。 */
    return __HAL_ADC_GET_FLAG(&hadc2, ADC_FLAG_JEOS) != 0U;
}

bool Adc_IsHealthy(void)
{
    /* 旧板没有AT32载波丢帧锁存；不以统一接口伪称增加了此保护。 */
    return s_sampling_started && !s_sampling_fault;
}

bool McuAdc_PollTemperature(AdcTemperatureSample *out)
{
    if (!BSP_ENABLED_INTERNAL_ANALOG_SAMPLING || out == NULL ||
        !__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_JEOS))
    {
        return false;
    }
    out->mcu_temperature_counts = ADC1->JDR1;
    out->reference_counts = ADC1->JDR2;
    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_JEOS | ADC_FLAG_JEOC);
    out->sequence = 0U; /* 完整帧序号和时间戳仅由公共发布入口填写。 */
    out->sampled_ms = 0U;
    return true;
}

void McuAdc_ContinueTemperature(void)
{
    if (BSP_ENABLED_INTERNAL_ANALOG_SAMPLING && !LL_ADC_INJ_IsConversionOngoing(ADC1))
    {
        LL_ADC_INJ_StartConversion(ADC1);
    }
}

uint16_t McuAdc_ReadRank(unsigned rank)
{
    switch (rank)
    {
    case 0U:
        return (uint16_t)ADC2->JDR1;
    case 1U:
        return (uint16_t)ADC2->JDR2;
    case 2U:
        return (uint16_t)ADC2->JDR3;
    case 3U:
        return (uint16_t)ADC2->JDR4;
    default:
        return 0U;
    }
}

uint16_t McuAdc_ReadBusCurrent(void)
{
    return 0U;
}

bool McuAdc_PauseForStorage(void)
{
    if (__get_IPSR() != 0U || __get_PRIMASK() == 0U || !BspPwm_ArePhasesDisabled() ||
        s_storage_owned || s_sampling_fault)
    {
        return false;
    }
    s_storage_owned = true;
    s_storage_was_started = s_sampling_started;
    if (!s_sampling_started)
    {
        return true;
    }
    __HAL_ADC_DISABLE_IT(&hadc2, ADC_IT_JEOS);
    HAL_StatusTypeDef main_status = HAL_ADCEx_InjectedStop(&hadc2);
    HAL_StatusTypeDef auxiliary_status = HAL_OK;
    if (BSP_ENABLED_INTERNAL_ANALOG_SAMPLING)
    {
        auxiliary_status = HAL_ADCEx_InjectedStop(&hadc1);
    }
    s_sampling_started = false;
    AdcFrames_Invalidate();
    if (main_status != HAL_OK || auxiliary_status != HAL_OK)
    {
        s_sampling_fault = true;
        BspPwm_Disable();
        return false;
    }
    __HAL_ADC_CLEAR_FLAG(&hadc2, ADC_FLAG_JEOS | ADC_FLAG_JEOC);
    return true;
}

bool McuAdc_ResumeAfterStorage(void)
{
    if (!s_storage_owned || s_sampling_fault || !BspPwm_ArePhasesDisabled())
    {
        return false;
    }
    if (!s_storage_was_started)
    {
        s_storage_owned = false;
        return true;
    }
    __HAL_ADC_CLEAR_FLAG(&hadc2, ADC_FLAG_JEOS | ADC_FLAG_JEOC);
    HAL_StatusTypeDef auxiliary_status = HAL_OK;
    if (BSP_ENABLED_INTERNAL_ANALOG_SAMPLING)
    {
        __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_JEOS | ADC_FLAG_JEOC);
        auxiliary_status = HAL_ADCEx_InjectedStart(&hadc1);
    }
    HAL_StatusTypeDef main_status = HAL_ADCEx_InjectedStart(&hadc2);
    if (main_status != HAL_OK || auxiliary_status != HAL_OK)
    {
        s_sampling_fault = true;
        BspPwm_Disable();
        return false;
    }
    s_sampling_started = true;
    s_storage_owned = false;
    __HAL_ADC_ENABLE_IT(&hadc2, ADC_IT_JEOS);
    return true;
}
