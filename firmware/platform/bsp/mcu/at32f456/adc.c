/**
 * @file adc.c
 * @brief AT32 ADC硬件采样、启停、中断和单位换算。
 */
#include "../../bsp_internal.h"
#include "bsp_adc.h"
#include "mcu_resources.h"
#include "at32f45x.h"
#include <math.h>

#define ADC_AUX_FAST_RANKS (BSP_ENABLED_BUS_CURRENT + 2U * BSP_ENABLED_LINEAR_HALL)

static volatile float s_phase_offsets[3] = {NAN, NAN, NAN};
static volatile float s_bus_offset = NAN;

/* 等待硬件就绪并执行片内校准，只在初始化时调用。 */
static void adc_calibrate(adc_type *adc)
{
    adc_enable(adc, TRUE);
    while (adc_flag_get(adc, ADC_RDY_FLAG) == RESET)
    {
    }
    adc_calibration_init(adc);
    while (adc_calibration_init_status_get(adc) != RESET)
    {
    }
    adc_calibration_start(adc);
    while (adc_calibration_status_get(adc) != RESET)
    {
    }
}

void McuAdc_Init(void)
{
    /* 默认GPIO时钟已由PWM初始化；独立换端口时补开对应时钟。 */
    if (MCU_ADC_GPIO_CLOCK != MCU_PWM_HIGH_CLOCK && MCU_ADC_GPIO_CLOCK != MCU_PWM_LOW_CLOCK)
    {
        crm_periph_clock_enable(MCU_ADC_GPIO_CLOCK, TRUE);
    }
    crm_periph_clock_enable(MCU_ADC_FAST_CLOCK, TRUE);
    if (BSP_ENABLED_MOTOR_TEMPERATURE || ADC_AUX_FAST_RANKS != 0U)
    {
        crm_periph_clock_enable(MCU_ADC_AUX_CLOCK, TRUE);
    }
    gpio_init_type gpio;
    gpio_default_para_init(&gpio);
    gpio.gpio_mode = GPIO_MODE_ANALOG;
    gpio.gpio_pull = GPIO_PULL_NONE;
    gpio.gpio_pins =
        MCU_ADC_PHASE_A_PIN | MCU_ADC_PHASE_B_PIN | MCU_ADC_PHASE_C_PIN | MCU_ADC_BUS_VOLTAGE_PIN |
        (BSP_ENABLED_BUS_CURRENT ? MCU_ADC_BUS_CURRENT_PIN : 0U) |
        (BSP_ENABLED_MOTOR_TEMPERATURE ? MCU_ADC_MOTOR_TEMPERATURE_PIN : 0U) |
        (BSP_ENABLED_LINEAR_HALL ? MCU_ADC_LINEAR_HALL_SIN_PIN | MCU_ADC_LINEAR_HALL_COS_PIN : 0U);
    gpio_init(MCU_ADC_GPIO_PORT, &gpio);
    adc_reset();
    adc_common_config_type common;
    adc_common_default_para_init(&common);
    common.combine_mode = ADC_INDEPENDENT_MODE;
    /* HCLK为180MHz；两ADC共用60MHz，三相与母线电压分别选择采样时间。 */
    common.div = ADC_HCLK_DIV_3;
    common.common_dma_mode = ADC_COMMON_DMAMODE_DISABLE;
    common.tempervintrv_state = FALSE;
    adc_common_config(&common);
    adc_base_config_type base;
    adc_base_default_para_init(&base);
    base.sequence_mode = TRUE;
    base.repeat_mode = FALSE;
    base.data_align = ADC_RIGHT_ALIGNMENT;
    if (BSP_ENABLED_MOTOR_TEMPERATURE || ADC_AUX_FAST_RANKS != 0U)
    {
        base.ordinary_channel_length = 1U;
        adc_base_config(MCU_ADC_AUX, &base);
    }
    base.ordinary_channel_length = 1U;
    adc_base_config(MCU_ADC_FAST, &base);
    adc_resolution_set(MCU_ADC_FAST, ADC_RESOLUTION_12B);
    if (BSP_ENABLED_MOTOR_TEMPERATURE || ADC_AUX_FAST_RANKS != 0U)
    {
        adc_resolution_set(MCU_ADC_AUX, ADC_RESOLUTION_12B);
    }
    adc_preempt_channel_length_set(MCU_ADC_FAST, 4U);
    const adc_channel_select_type fast_channels[] = {MCU_ADC_PHASE_A_CHANNEL,
                                                     MCU_ADC_PHASE_B_CHANNEL,
                                                     MCU_ADC_PHASE_C_CHANNEL,
                                                     MCU_ADC_BUS_VOLTAGE_CHANNEL};
    for (uint8_t rank = 1U; rank <= 4U; ++rank)
    {
        adc_preempt_channel_set(MCU_ADC_FAST,
                                fast_channels[rank - 1U],
                                rank,
                                rank <= 3U ? ADC_SAMPLETIME_12_5 : ADC_SAMPLETIME_24_5);
    }
    adc_oversample_ratio_shift_set(MCU_ADC_FAST, ADC_OVERSAMPLE_RATIO_4, ADC_OVERSAMPLE_SHIFT_0);
    adc_preempt_oversample_enable(MCU_ADC_FAST, TRUE);
    adc_preempt_conversion_trigger_set(
        MCU_ADC_FAST, MCU_ADC_FAST_TRIGGER, ADC_PREEMPT_TRIG_EDGE_NONE);
    adc_convert_fail_auto_abort_enable(MCU_ADC_FAST, TRUE);
    if (ADC_AUX_FAST_RANKS)
    {
        adc_preempt_channel_length_set(MCU_ADC_AUX, ADC_AUX_FAST_RANKS);
#if BSP_ENABLED_BUS_CURRENT
        adc_preempt_channel_set(MCU_ADC_AUX, MCU_ADC_BUS_CURRENT_CHANNEL, 1U, ADC_SAMPLETIME_92_5);
#endif
#if BSP_ENABLED_LINEAR_HALL
        adc_preempt_channel_set(MCU_ADC_AUX,
                                MCU_ADC_LINEAR_HALL_SIN_CHANNEL,
                                BSP_ENABLED_BUS_CURRENT + 1U,
                                ADC_SAMPLETIME_24_5);
        adc_preempt_channel_set(MCU_ADC_AUX,
                                MCU_ADC_LINEAR_HALL_COS_CHANNEL,
                                BSP_ENABLED_BUS_CURRENT + 2U,
                                ADC_SAMPLETIME_24_5);
#endif
        /* 同一 TMR1 边沿触发，一拍内依次转换母线电流、sin 和 cos，各取一次。 */
        adc_preempt_oversample_enable(MCU_ADC_AUX, FALSE);
        adc_preempt_conversion_trigger_set(
            MCU_ADC_AUX, MCU_ADC_FAST_TRIGGER, ADC_PREEMPT_TRIG_EDGE_NONE);
        adc_convert_fail_auto_abort_enable(MCU_ADC_AUX, TRUE);
    }
#if BSP_ENABLED_MOTOR_TEMPERATURE
    adc_ordinary_channel_set(
        MCU_ADC_AUX, MCU_ADC_MOTOR_TEMPERATURE_CHANNEL, 1U, ADC_SAMPLETIME_640_5);
    adc_ordinary_conversion_trigger_set(
        MCU_ADC_AUX, MCU_ADC_SLOW_TRIGGER, ADC_ORDINARY_TRIG_EDGE_NONE);
#endif
    adc_calibrate(MCU_ADC_FAST);
    if (BSP_ENABLED_MOTOR_TEMPERATURE || ADC_AUX_FAST_RANKS != 0U)
    {
        adc_calibrate(MCU_ADC_AUX);
    }
    /* 采样触发通道只产生内部事件；触发边沿在Adc_Start中开放。 */
    tmr_output_config_type trigger;
    tmr_output_default_para_init(&trigger);
    trigger.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_B;
    trigger.oc_output_state = FALSE;
    trigger.occ_output_state = FALSE;
    tmr_output_channel_config(MCU_ADC_TRIGGER_TIMER, MCU_ADC_TRIGGER_CHANNEL, &trigger);
    /* 20kHz中心对齐PWM：三相4倍采样共约5us，以25us计数峰值为中心。
     * 触发放在22.5us，避免首相采样过早落在开关沿附近。 */
    tmr_channel_value_set(
        MCU_ADC_TRIGGER_TIMER, MCU_ADC_TRIGGER_CHANNEL, AT32_TIMER_HZ * 9U / (20U * FOC_FREQ));
    tmr_primary_mode_select(MCU_ADC_TRIGGER_TIMER, MCU_ADC_TRIGGER_OUTPUT);
    tmr_primary_mode_select(MCU_CONTROL_TIMER, TMR_PRIMARY_SEL_OVERFLOW);
    adc_interrupt_enable(MCU_ADC_FAST, ADC_PCCE_INT, TRUE);
    NVIC_SetPriority(MCU_ADC_IRQ, MCU_ADC_IRQ_PRIORITY);
    NVIC_EnableIRQ(MCU_ADC_IRQ);
}

void Adc_Start(void)
{
    adc_flag_clear(MCU_ADC_FAST, ADC_PCCE_FLAG | ADC_TCF_FLAG | ADC_OCCO_FLAG);
    if (BSP_ENABLED_MOTOR_TEMPERATURE || ADC_AUX_FAST_RANKS != 0U)
    {
        adc_flag_clear(MCU_ADC_AUX, ADC_PCCE_FLAG | ADC_TCF_FLAG | ADC_OCCO_FLAG);
    }
    NVIC_ClearPendingIRQ(MCU_ADC_IRQ);
#if BSP_ENABLED_MOTOR_TEMPERATURE
    adc_ordinary_conversion_trigger_set(
        MCU_ADC_AUX, MCU_ADC_SLOW_TRIGGER, ADC_ORDINARY_TRIG_EDGE_RISING);
#endif
    adc_preempt_conversion_trigger_set(
        MCU_ADC_FAST, MCU_ADC_FAST_TRIGGER, ADC_PREEMPT_TRIG_EDGE_RISING);
    if (ADC_AUX_FAST_RANKS)
    {
        adc_preempt_conversion_trigger_set(
            MCU_ADC_AUX, MCU_ADC_FAST_TRIGGER, ADC_PREEMPT_TRIG_EDGE_RISING);
    }
}

void Adc_Stop(void)
{
    adc_preempt_conversion_trigger_set(
        MCU_ADC_FAST, MCU_ADC_FAST_TRIGGER, ADC_PREEMPT_TRIG_EDGE_NONE);
    if (BSP_ENABLED_MOTOR_TEMPERATURE || ADC_AUX_FAST_RANKS != 0U)
    {
        adc_preempt_conversion_trigger_set(
            MCU_ADC_AUX, MCU_ADC_FAST_TRIGGER, ADC_PREEMPT_TRIG_EDGE_NONE);
        adc_ordinary_conversion_trigger_set(
            MCU_ADC_AUX, MCU_ADC_SLOW_TRIGGER, ADC_ORDINARY_TRIG_EDGE_NONE);
        adc_conversion_stop(MCU_ADC_AUX);
    }
    adc_conversion_stop(MCU_ADC_FAST);
    while (adc_conversion_stop_status_get(MCU_ADC_FAST) != RESET ||
           ((BSP_ENABLED_MOTOR_TEMPERATURE || ADC_AUX_FAST_RANKS != 0U) &&
            adc_conversion_stop_status_get(MCU_ADC_AUX) != RESET))
    {
    }
}

bool BspAdc_CompleteFast(void)
{
    if (!adc_flag_get(MCU_ADC_FAST, ADC_PCCE_FLAG))
    {
        return false;
    }
    if (ADC_AUX_FAST_RANKS && !adc_flag_get(MCU_ADC_AUX, ADC_PCCE_FLAG))
    {
        adc_flag_clear(MCU_ADC_FAST, ADC_PCCE_FLAG);
        return false;
    }
    adc_flag_clear(MCU_ADC_FAST, ADC_PCCE_FLAG);
    if (ADC_AUX_FAST_RANKS)
    {
        adc_flag_clear(MCU_ADC_AUX, ADC_PCCE_FLAG);
    }
    return true;
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
#if BSP_ENABLED_BUS_CURRENT
    case BSP_ADC_BUS_CURRENT:
        return (float)MCU_ADC_BUS_CURRENT_RESULT;
#endif
#if BSP_ENABLED_MOTOR_TEMPERATURE
    case BSP_ADC_MOTOR_TEMPERATURE:
        return (float)MCU_ADC_MOTOR_TEMPERATURE_RESULT;
#endif
#if BSP_ENABLED_LINEAR_HALL
    case BSP_ADC_LINEAR_HALL_SIN:
        return (float)MCU_ADC_LINEAR_HALL_SIN_RESULT;
    case BSP_ADC_LINEAR_HALL_COS:
        return (float)MCU_ADC_LINEAR_HALL_COS_RESULT;
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
