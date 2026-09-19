#include "motor_sensing.h"

#include "adc.h"
#include "hw_conf.h"
#include "mcu_temperature.h"
#include "stm32g4xx_ll_adc.h"

/* 感测端口：实现 platform/api/motor_sensing.h 契约。
 * 寄存器与 LL/HAL 访问只存在于本层；通道选择宏来自板级 hw_conf.h。 */

uint16_t motor_hw_current_sample_raw(MotorHwCurrentPhase phase)
{
    switch (phase)
    {
    case MOTOR_HW_CURRENT_PHASE_A:
        return (uint16_t)CURRENT_ADC->IA_ADC_CHANNEL;
    case MOTOR_HW_CURRENT_PHASE_B:
        return (uint16_t)CURRENT_ADC->IB_ADC_CHANNEL;
    case MOTOR_HW_CURRENT_PHASE_C:
        return (uint16_t)CURRENT_ADC->IC_ADC_CHANNEL;
    default:
        return 0U;
    }
}

uint16_t motor_hw_vbus_sample_raw(void)
{
    return (uint16_t)VBUS_ADC->VBUS_ADC_CHANNEL;
}

void motor_hw_temperature_poll(MotorHwTemperaturePoll_TypeDef *result)
{
    result->sample_ready = false;
    result->conversion_ok = false;

    /* 软件触发的 ADC1 序列在 1 kHz 监督调用之间完成：仅在 JEOS 时读取两个
     * rank 并清标志，随后请求下一次转换。 */
    if (__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_JEOS))
    {
        result->raw_ts = ADC1->JDR1;
        result->raw_vref = ADC1->JDR2;
        __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_JEOS | ADC_FLAG_JEOC);
        result->sample_ready = true;
        result->conversion_ok = McuTemperature_Convert(result->raw_ts,
                                                       result->raw_vref,
                                                       *TEMPSENSOR_CAL1_ADDR,
                                                       *TEMPSENSOR_CAL2_ADDR,
                                                       *VREFINT_CAL_ADDR,
                                                       &result->celsius,
                                                       &result->vdda_mv);
    }
    if (!LL_ADC_INJ_IsConversionOngoing(ADC1))
    {
        LL_ADC_INJ_StartConversion(ADC1);
    }
}
