/* 执行实际STM32采样BSP、温度端口与启动函数；仅厂商寄存器和HAL事件由桩替代。 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "bsp_internal.h"
#include "bsp_pwm.h"
#include "sampling_scale.h"
#include "mcu_temperature.h"

enum
{
    ADC_SINGLE_ENDED = 0,
    TIM_CHANNEL_1 = 1,
    TIM_CHANNEL_2,
    TIM_CHANNEL_3,
    TIM_CHANNEL_4,
    ADC_FLAG_JEOS = 1,
    ADC_FLAG_JEOC = 2,
    ADC_IT_JEOC = 2,
    ADC_IT_JEOS = 1
};
static int hadc1, hadc2, htim1, htim7;
static unsigned events[32], event_count, flags1, flags2, irq_mask = ADC_IT_JEOC;
static bool busy, s_sampling_started;
static unsigned starts;
static bool s_sampling_fault;
static unsigned s_startup_state = 2U;
#define HAL_OK 0
uint32_t BspSystem_TimeMs(void)
{
    return 0U;
}
uint32_t BspSystem_TimeUs(void)
{
    return 0U;
}
uint32_t BspSystem_EnterCritical(void)
{
    return 0U;
}
void BspSystem_ExitCritical(uint32_t key)
{
    (void)key;
}
void BspPwm_Disable(void)
{
    assert(false);
}
bool BspPwm_ArePhasesDisabled(void)
{
    return true;
}
static struct
{
    uint32_t JDR1, JDR2, JDR3, JDR4;
} current_adc, temperature_adc;
#define ADC2 (&current_adc)
#define ADC1 (&temperature_adc)
static uint16_t trim[3] = {1000U, 1400U, 1500U};
#define TEMPSENSOR_CAL1_ADDR (&trim[0])
#define TEMPSENSOR_CAL2_ADDR (&trim[1])
#define VREFINT_CAL_ADDR (&trim[2])
#define __HAL_ADC_GET_FLAG(handle, mask) (((handle) == &hadc1 ? flags1 : flags2) & (mask))
#define __HAL_ADC_CLEAR_FLAG(handle, mask)                                                         \
    do                                                                                             \
    {                                                                                              \
        assert((handle) == &hadc1);                                                                \
        flags1 &= ~(mask);                                                                         \
    } while (0)

/** @brief 记录硬件操作的实际先后次序。 */
static void record(unsigned event)
{
    assert(event_count < 32U);
    events[event_count++] = event;
}
/** @brief 记录两路ADC校准，不模拟新的错误处理。 */
static void HAL_ADCEx_Calibration_Start(int *adc, int mode)
{
    assert(mode == ADC_SINGLE_ENDED);
    record(adc == &hadc1 ? 1U : 2U);
}
/** @brief 记录功率主通道或采样CH4启动。 */
static void HAL_TIM_PWM_Start(int *timer, int channel)
{
    assert(timer == &htim1);
    record(10U + (unsigned)channel);
}
/** @brief 记录功率互补通道启动。 */
static void HAL_TIMEx_OCN_Start(int *timer, int channel)
{
    assert(timer == &htim1);
    record(40U + (unsigned)channel);
}
/** @brief 采样夹具在功率接口边界记录原有六路启动顺序；PWM实现另有完整等价测试。 */
BspResult BspPwm_Enable(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_OCN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_OCN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_OCN_Start(&htim1, TIM_CHANNEL_3);
    return BSP_OK;
}
/** @brief 记录注入序列启动。 */
static void HAL_ADCEx_InjectedStart(int *adc)
{
    record(adc == &hadc1 ? 21U : 22U);
}
/** @brief 记录逐rank中断关闭。 */
static void __HAL_ADC_DISABLE_IT(int *adc, int mask)
{
    assert(adc == &hadc2 && mask == ADC_IT_JEOC);
    irq_mask &= ~(unsigned)mask;
    record(30U);
}
/** @brief 记录完整序列中断开放。 */
static void __HAL_ADC_ENABLE_IT(int *adc, int mask)
{
    assert(adc == &hadc2 && mask == ADC_IT_JEOS);
    irq_mask |= (unsigned)mask;
    record(31U);
}
/** @brief 核对监督时钟仍在采样序列之后启动。 */
static int HAL_TIM_Base_Start_IT(int *timer)
{
    assert(timer == &htim7);
    record(70U);
    return HAL_OK;
}
/** @brief 返回模拟的温度转换忙状态。 */
static bool LL_ADC_INJ_IsConversionOngoing(void *adc)
{
    assert(adc == ADC1);
    return busy;
}
/** @brief 记录换算之后的软件温度触发。 */
static void LL_ADC_INJ_StartConversion(void *adc)
{
    assert(adc == ADC1 && flags1 == 0U);
    busy = true;
    ++starts;
}

/* STM32_SAMPLING_PRODUCTION */

int main(void)
{
    const unsigned disabled_order[] = {1U, 2U, 14U, 21U, 22U, 30U, 31U, 70U};
    assert(!Adc_IsHealthy());
    assert(Adc_Init() && Bsp_Start() == BSP_OK);
    assert(event_count == sizeof(disabled_order) / sizeof(disabled_order[0]));
    assert(memcmp(events, disabled_order, sizeof(disabled_order)) == 0);
    assert(irq_mask == ADC_IT_JEOS && Adc_IsHealthy());
    unsigned previous_events = event_count;
    assert(Bsp_Start() == BSP_NOT_READY && event_count == previous_events);

    current_adc.JDR1 = 10U;
    current_adc.JDR2 = 8000U;
    current_adc.JDR3 = 16380U;
    current_adc.JDR4 = 12000U;
    assert(McuAdc_ReadRank(BSP_SAMPLE_PHASE_A_RANK) == 10U);
    assert(McuAdc_ReadRank(BSP_SAMPLE_PHASE_B_RANK) == 8000U);
    assert(McuAdc_ReadRank(BSP_SAMPLE_PHASE_C_RANK) == 16380U);
    assert(McuAdc_ReadRank(99U) == 0U);
    assert(McuAdc_ReadRank(BSP_SAMPLE_VBUS_RANK) == 12000U);
    flags1 = ADC_FLAG_JEOS;
    assert(!Adc_CompleteFast());
    flags2 = ADC_FLAG_JEOS | ADC_FLAG_JEOC;
    assert(Adc_CompleteFast() && flags2 == (ADC_FLAG_JEOS | ADC_FLAG_JEOC));
    AdcAuxSample aux = {2U, 3U, 0U}, previous = aux;
    assert(!Adc_ReadAux(&aux) && memcmp(&aux, &previous, sizeof(aux)) == 0);

    BspAdcValue value;
    BspAdcRawValue raw;
    temperature_adc.JDR1 = 1000U;
    temperature_adc.JDR2 = 1500U;
    flags1 = ADC_FLAG_JEOS | ADC_FLAG_JEOC;
    busy = false;
    assert(BspAdc_UpdateAuxiliary() == BSP_OK);
    assert(BspAdc_ReadChannel(BSP_ADC_MCU_TEMPERATURE, &value) == BSP_OK && value.value == 30.0f);
    assert(BspAdc_ReadChannel(BSP_ADC_ANALOG_SUPPLY_VOLTAGE, &value) == BSP_OK &&
           value.value == 3.0f);
    assert(BspAdc_ReadRawChannel(BSP_ADC_MCU_TEMPERATURE, &raw) == BSP_OK);
    assert(raw.counts == 1000.0f && raw.reference_counts == 1500.0f && raw.sequence == 1U);
    assert(flags1 == 0U && starts == 1U && busy);
    assert(BspAdc_UpdateAuxiliary() == BSP_NOT_READY && starts == 1U);
    assert(s_temperature.sequence == 1U);
    s_temperature.sequence = UINT32_MAX - 1U;
    flags1 = ADC_FLAG_JEOS;
    assert(BspAdc_UpdateAuxiliary() == BSP_OK);
    assert(BspAdc_ReadRawChannel(BSP_ADC_MCU_TEMPERATURE, &raw) == BSP_OK &&
           raw.sequence == UINT32_MAX);
    assert(!Adc_ReadTemperature(NULL));
    temperature_adc.JDR1 = 0U;
    flags1 = ADC_FLAG_JEOS;
    busy = false;
    assert(BspAdc_UpdateAuxiliary() == BSP_OK && starts == 2U);
    assert(BspAdc_ReadRawChannel(BSP_ADC_MCU_TEMPERATURE, &raw) == BSP_OK && raw.sequence == 0U);
    assert(BspAdc_ReadChannel(BSP_ADC_MCU_TEMPERATURE, &value) == BSP_NOT_CALIBRATED);
    assert(BspAdc_ReadChannel(BSP_ADC_ANALOG_SUPPLY_VOLTAGE, &value) == BSP_OK &&
           value.value == 3.0f);
    puts("PASS STM32 actual sampling chain: startup order, IRQ ack, raw scale, unavailable aux, "
         "temperature conversion/trigger, duplicate and wrap");
    return 0;
}
