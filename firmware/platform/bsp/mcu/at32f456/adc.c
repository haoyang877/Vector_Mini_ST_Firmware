/**
 * @file adc.c
 * @brief AT32 ADC硬件采样、DMA、启停、中断和片内参数换算。
 */
#include "../../bsp_internal.h"
#include "bsp_adc.h"
#include "board_config.h"
#include "at32f45x.h"
#include <math.h>

#define ADC_AUX_RANKS (BSP_ENABLED_MOTOR_TEMPERATURE + 2U * BSP_ENABLED_OUTPUT_SINCOS)

static volatile float s_phase_offsets[3] = {NAN, NAN, NAN};
static volatile float s_bus_offset = NAN;

static volatile uint16_t s_aux_dma[2U * (ADC_AUX_RANKS ? ADC_AUX_RANKS : 1U)];
static volatile uint16_t s_temperature_dma[4];

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
    crm_periph_clock_enable(CRM_ADC1_PERIPH_CLOCK, TRUE);
    if (BSP_ENABLED_AUX_SAMPLING || BSP_ENABLED_BUS_CURRENT)
    {
        crm_periph_clock_enable(CRM_ADC2_PERIPH_CLOCK, TRUE);
    }
    if (BSP_ENABLED_AUX_SAMPLING || BSP_ENABLED_INTERNAL_ANALOG_SAMPLING)
    {
        crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
    }
    gpio_init_type gpio;
    gpio_default_para_init(&gpio);
    gpio.gpio_mode = GPIO_MODE_ANALOG;
    gpio.gpio_pull = GPIO_PULL_NONE;
    gpio.gpio_pins = GPIO_PINS_0 | GPIO_PINS_1 | GPIO_PINS_2 | GPIO_PINS_3 |
                     (BSP_ENABLED_BUS_CURRENT ? GPIO_PINS_4 : 0U) |
                     (BSP_ENABLED_MOTOR_TEMPERATURE ? GPIO_PINS_7 : 0U) |
                     (BSP_ENABLED_OUTPUT_SINCOS ? GPIO_PINS_5 | GPIO_PINS_6 : 0U);
    gpio_init(GPIOA, &gpio);
    adc_reset();
    adc_common_config_type common;
    adc_common_default_para_init(&common);
    common.combine_mode = ADC_INDEPENDENT_MODE;
    /* HCLK为180MHz；两ADC共用60MHz，三相与母线电压分别选择采样时间。 */
    common.div = ADC_HCLK_DIV_3;
    common.common_dma_mode = ADC_COMMON_DMAMODE_DISABLE;
    common.tempervintrv_state = BSP_ENABLED_INTERNAL_ANALOG_SAMPLING ? TRUE : FALSE;
    adc_common_config(&common);
    adc_base_config_type base;
    adc_base_default_para_init(&base);
    base.sequence_mode = TRUE;
    base.repeat_mode = FALSE;
    base.data_align = ADC_RIGHT_ALIGNMENT;
    if (BSP_ENABLED_AUX_SAMPLING || BSP_ENABLED_BUS_CURRENT)
    {
        base.ordinary_channel_length = ADC_AUX_RANKS ? ADC_AUX_RANKS : 1U;
        adc_base_config(ADC2, &base);
    }
    base.ordinary_channel_length = BSP_ENABLED_INTERNAL_ANALOG_SAMPLING ? 2U : 1U;
    adc_base_config(ADC1, &base);
    adc_resolution_set(ADC1, ADC_RESOLUTION_12B);
    if (BSP_ENABLED_AUX_SAMPLING || BSP_ENABLED_BUS_CURRENT)
    {
        adc_resolution_set(ADC2, ADC_RESOLUTION_12B);
    }
    adc_preempt_channel_length_set(ADC1, 4U);
    for (uint8_t rank = 1U; rank <= 4U; ++rank)
    {
        adc_preempt_channel_set(ADC1,
                                (adc_channel_select_type)(rank - 1U),
                                rank,
                                rank <= 3U ? ADC_SAMPLETIME_12_5 : ADC_SAMPLETIME_24_5);
    }
    adc_oversample_ratio_shift_set(ADC1, ADC_OVERSAMPLE_RATIO_4, ADC_OVERSAMPLE_SHIFT_0);
    adc_preempt_oversample_enable(ADC1, TRUE);
    adc_preempt_conversion_trigger_set(
        ADC1, ADC_PREEMPT_TRIG_TMR1TRGOUT, ADC_PREEMPT_TRIG_EDGE_NONE);
    adc_convert_fail_auto_abort_enable(ADC1, TRUE);
    if (BSP_ENABLED_BUS_CURRENT)
    {
        adc_preempt_channel_length_set(ADC2, 1U);
        /* 92.5/60MHz建立时间约1.54us；加转换约1.75us，早于ADC1四rank完成。
         * 不启用母线过采样，不混用三相四次和域；实板需比较偏置、噪声及周期预算。 */
        adc_preempt_channel_set(ADC2, ADC_CHANNEL_4, 1U, ADC_SAMPLETIME_92_5);
        adc_preempt_conversion_trigger_set(
            ADC2, ADC_PREEMPT_TRIG_TMR1TRGOUT, ADC_PREEMPT_TRIG_EDGE_NONE);
        adc_convert_fail_auto_abort_enable(ADC2, TRUE);
    }
#if BSP_ENABLED_AUX_SAMPLING
    const adc_channel_select_type aux_channels[] = {
#if BSP_ENABLED_MOTOR_TEMPERATURE
        ADC_CHANNEL_7,
#endif
#if BSP_ENABLED_OUTPUT_SINCOS
        ADC_CHANNEL_5,
        ADC_CHANNEL_6,
#endif
    };
    for (uint8_t rank = 1U; rank <= ADC_AUX_RANKS; ++rank)
    {
        const adc_channel_select_type channel = aux_channels[rank - 1U];
        adc_ordinary_channel_set(ADC2,
                                 channel,
                                 rank,
                                 channel == ADC_CHANNEL_5 || channel == ADC_CHANNEL_6
                                     ? ADC_SAMPLETIME_24_5
                                     : ADC_SAMPLETIME_640_5);
    }
#if BSP_ENABLED_OUTPUT_SINCOS
    /* 普通组统一4次硬件平均，NTC与两路霍尔仍为12位原码；母线抢占组不做过采样。
     * 单次TMR6触发完成4次累加；母线抢占结束后继续普通组累加，避免反复丢弃进度。 */
    adc_oversample_ratio_shift_set(ADC2, ADC_OVERSAMPLE_RATIO_4, ADC_OVERSAMPLE_SHIFT_2);
    adc_ordinary_oversample_trig_enable(ADC2, FALSE);
    adc_ordinary_oversample_restart_set(ADC2, ADC_OVERSAMPLE_CONTINUE);
    adc_ordinary_oversample_enable(ADC2, TRUE);
    adc_preempt_oversample_enable(ADC2, FALSE);
#endif
#endif
    if (BSP_ENABLED_INTERNAL_ANALOG_SAMPLING)
    {
        adc_ordinary_channel_set(ADC1, ADC_CHANNEL_16, 1U, ADC_SAMPLETIME_640_5);
        adc_ordinary_channel_set(ADC1, ADC_CHANNEL_17, 2U, ADC_SAMPLETIME_640_5);
    }
    adc_ordinary_conversion_trigger_set(
        ADC1, ADC_ORDINARY_TRIG_TMR6TRGOUT, ADC_ORDINARY_TRIG_EDGE_NONE);
    if (BSP_ENABLED_AUX_SAMPLING)
    {
        adc_ordinary_conversion_trigger_set(
            ADC2, ADC_ORDINARY_TRIG_TMR6TRGOUT, ADC_ORDINARY_TRIG_EDGE_NONE);
    }
    if (BSP_ENABLED_AUX_SAMPLING)
    {
        adc_occe_each_conversion_enable(ADC2, TRUE);
        adc_dma_mode_enable(ADC2, TRUE);
        adc_dma_request_repeat_enable(ADC2, TRUE);
    }
    if (BSP_ENABLED_INTERNAL_ANALOG_SAMPLING)
    {
        adc_occe_each_conversion_enable(ADC1, TRUE);
        adc_dma_mode_enable(ADC1, TRUE);
        adc_dma_request_repeat_enable(ADC1, TRUE);
    }
    dma_init_type dma;
    if (BSP_ENABLED_AUX_SAMPLING)
    {
        dma_channel_enable(DMA1_CHANNEL2, FALSE);
        dma_reset(DMA1_CHANNEL2);
        dma_default_para_init(&dma);
        dma.direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
        dma.peripheral_base_addr = (uint32_t)(uintptr_t)&ADC2->odt;
        dma.memory_base_addr = (uint32_t)(uintptr_t)s_aux_dma;
        dma.buffer_size = 2U * ADC_AUX_RANKS;
        dma.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
        dma.memory_data_width = DMA_MEMORY_DATA_WIDTH_HALFWORD;
        dma.memory_inc_enable = TRUE;
        dma.loop_mode_enable = TRUE;
        dma.priority = DMA_PRIORITY_HIGH;
        dma_init(DMA1_CHANNEL2, &dma);
        dmamux_enable(DMA1, TRUE);
        dmamux_init(DMA1MUX_CHANNEL2, DMAMUX_DMAREQ_ID_ADC2);
    }
    if (BSP_ENABLED_INTERNAL_ANALOG_SAMPLING)
    {
        dma_channel_enable(DMA1_CHANNEL3, FALSE);
        dma_reset(DMA1_CHANNEL3);
        if (!BSP_ENABLED_AUX_SAMPLING)
        {
            dma_default_para_init(&dma);
        }
        dma.direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
        dma.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
        dma.memory_data_width = DMA_MEMORY_DATA_WIDTH_HALFWORD;
        dma.memory_inc_enable = TRUE;
        dma.loop_mode_enable = TRUE;
        dma.priority = DMA_PRIORITY_HIGH;
        dma.peripheral_base_addr = (uint32_t)(uintptr_t)&ADC1->odt;
        dma.memory_base_addr = (uint32_t)(uintptr_t)s_temperature_dma;
        dma.buffer_size = 4U;
        dma_init(DMA1_CHANNEL3, &dma);
        if (!BSP_ENABLED_AUX_SAMPLING)
        {
            dmamux_enable(DMA1, TRUE);
        }
        dmamux_init(DMA1MUX_CHANNEL3, DMAMUX_DMAREQ_ID_ADC1);
    }
    adc_calibrate(ADC1);
    if (BSP_ENABLED_AUX_SAMPLING || BSP_ENABLED_BUS_CURRENT)
    {
        adc_calibrate(ADC2);
    }
    /* 数据手册规定内部温度传感器上电建立<=100us；至少18000个内核周期。 */
    for (uint32_t i = 0U; BSP_ENABLED_INTERNAL_ANALOG_SAMPLING && i < AT32_CORE_HZ / 10000U; ++i)
    {
        __NOP();
    }
    /* 采样触发通道只产生内部事件；触发边沿在Adc_Start中开放。 */
    tmr_output_config_type trigger;
    tmr_output_default_para_init(&trigger);
    trigger.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_B;
    trigger.oc_output_state = FALSE;
    trigger.occ_output_state = FALSE;
    tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_1, &trigger);
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_1, AT32_TIMER_HZ * 3U / (8U * FOC_FREQ));
    tmr_primary_mode_select(TMR1, TMR_PRIMARY_SEL_C1ORAW);
    tmr_primary_mode_select(TMR6, TMR_PRIMARY_SEL_OVERFLOW);
    adc_interrupt_enable(ADC1, ADC_PCCE_INT, TRUE);
    NVIC_SetPriority(ADC1_2_IRQn, 0U);
    NVIC_EnableIRQ(ADC1_2_IRQn);
}

void Adc_Start(void)
{
    adc_flag_clear(ADC1, ADC_PCCE_FLAG | ADC_TCF_FLAG | ADC_OCCO_FLAG);
    if (BSP_ENABLED_AUX_SAMPLING || BSP_ENABLED_BUS_CURRENT)
    {
        adc_flag_clear(ADC2, ADC_PCCE_FLAG | ADC_TCF_FLAG | ADC_OCCO_FLAG);
    }
    NVIC_ClearPendingIRQ(ADC1_2_IRQn);
#if BSP_ENABLED_AUX_SAMPLING
    dma_flag_clear(DMA1_GL2_FLAG);
    DMA1_CHANNEL2->maddr = (uint32_t)(uintptr_t)s_aux_dma;
    dma_data_number_set(DMA1_CHANNEL2, 2U * ADC_AUX_RANKS);
    dma_channel_enable(DMA1_CHANNEL2, TRUE);
    adc_ordinary_conversion_trigger_set(
        ADC2, ADC_ORDINARY_TRIG_TMR6TRGOUT, ADC_ORDINARY_TRIG_EDGE_RISING);
#endif
#if BSP_ENABLED_INTERNAL_ANALOG_SAMPLING
    dma_flag_clear(DMA1_GL3_FLAG);
    DMA1_CHANNEL3->maddr = (uint32_t)(uintptr_t)s_temperature_dma;
    dma_data_number_set(DMA1_CHANNEL3, 4U);
    dma_channel_enable(DMA1_CHANNEL3, TRUE);
    adc_ordinary_conversion_trigger_set(
        ADC1, ADC_ORDINARY_TRIG_TMR6TRGOUT, ADC_ORDINARY_TRIG_EDGE_RISING);
#endif
    adc_preempt_conversion_trigger_set(
        ADC1, ADC_PREEMPT_TRIG_TMR1TRGOUT, ADC_PREEMPT_TRIG_EDGE_RISING);
    if (BSP_ENABLED_BUS_CURRENT)
    {
        adc_preempt_conversion_trigger_set(
            ADC2, ADC_PREEMPT_TRIG_TMR1TRGOUT, ADC_PREEMPT_TRIG_EDGE_RISING);
    }
}

void Adc_Stop(void)
{
    adc_preempt_conversion_trigger_set(
        ADC1, ADC_PREEMPT_TRIG_TMR1TRGOUT, ADC_PREEMPT_TRIG_EDGE_NONE);
    adc_ordinary_conversion_trigger_set(
        ADC1, ADC_ORDINARY_TRIG_TMR6TRGOUT, ADC_ORDINARY_TRIG_EDGE_NONE);
    if (BSP_ENABLED_AUX_SAMPLING || BSP_ENABLED_BUS_CURRENT)
    {
        adc_preempt_conversion_trigger_set(
            ADC2, ADC_PREEMPT_TRIG_TMR1TRGOUT, ADC_PREEMPT_TRIG_EDGE_NONE);
        adc_ordinary_conversion_trigger_set(
            ADC2, ADC_ORDINARY_TRIG_TMR6TRGOUT, ADC_ORDINARY_TRIG_EDGE_NONE);
        adc_conversion_stop(ADC2);
    }
    adc_conversion_stop(ADC1);
    while (adc_conversion_stop_status_get(ADC1) != RESET ||
           ((BSP_ENABLED_AUX_SAMPLING || BSP_ENABLED_BUS_CURRENT) &&
            adc_conversion_stop_status_get(ADC2) != RESET))
    {
    }
#if BSP_ENABLED_AUX_SAMPLING
    dma_channel_enable(DMA1_CHANNEL2, FALSE);
#endif
#if BSP_ENABLED_INTERNAL_ANALOG_SAMPLING
    dma_channel_enable(DMA1_CHANNEL3, FALSE);
#endif
}

bool Adc_CompleteFast(void)
{
    if (!adc_flag_get(ADC1, ADC_PCCE_FLAG))
    {
        return false;
    }
    if (BSP_ENABLED_BUS_CURRENT && !adc_flag_get(ADC2, ADC_PCCE_FLAG))
    {
        adc_flag_clear(ADC1, ADC_PCCE_FLAG);
        return false;
    }
    adc_flag_clear(ADC1, ADC_PCCE_FLAG);
    if (BSP_ENABLED_BUS_CURRENT)
    {
        adc_flag_clear(ADC2, ADC_PCCE_FLAG);
    }
    return true;
}

#if defined(BSP_APP_LIFECYCLE) && BSP_APP_LIFECYCLE
void ADC1_2_IRQHandler(void)
{
    if (Adc_CompleteFast())
    {
        BspApp_OnFastSample();
    }
}
#endif

float BspAdc_ReadRawChannel(BspAdcChannel channel)
{
    switch (channel)
    {
    case BSP_ADC_PHASE_CURRENT_A:
        return (float)ADC1->pdt1 * BSP_ADC_SUM_TO_COUNTS;
    case BSP_ADC_PHASE_CURRENT_B:
        return (float)ADC1->pdt2 * BSP_ADC_SUM_TO_COUNTS;
    case BSP_ADC_PHASE_CURRENT_C:
        return (float)ADC1->pdt3 * BSP_ADC_SUM_TO_COUNTS;
    case BSP_ADC_BUS_VOLTAGE:
        return (float)ADC1->pdt4 * BSP_ADC_SUM_TO_COUNTS;
#if BSP_ENABLED_BUS_CURRENT
    case BSP_ADC_BUS_CURRENT:
        return (float)ADC2->pdt1;
#endif
#if BSP_ENABLED_MOTOR_TEMPERATURE
    case BSP_ADC_MOTOR_TEMPERATURE:
    {
        uint32_t remaining = DMA1_CHANNEL2->dtcnt;
        uint32_t offset = remaining == 0U || remaining > ADC_AUX_RANKS ? ADC_AUX_RANKS : 0U;
        return (float)s_aux_dma[offset];
    }
#endif
#if BSP_ENABLED_INTERNAL_ANALOG_SAMPLING
    case BSP_ADC_MCU_TEMPERATURE:
    case BSP_ADC_ANALOG_SUPPLY_VOLTAGE:
    {
        uint32_t remaining = DMA1_CHANNEL3->dtcnt;
        uint32_t offset = remaining == 0U || remaining > 2U ? 2U : 0U;
        return s_temperature_dma[offset + (channel == BSP_ADC_ANALOG_SUPPLY_VOLTAGE)];
    }
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
        if (AT32_INTERNAL_REFERENCE_MV <= 0.0f)
        {
            return NAN;
        }
        return AT32_INTERNAL_REFERENCE_MV * AT32_ADC_MAX_COUNTS / counts / 1000.0f;
    case BSP_ADC_MCU_TEMPERATURE:
#if AT32_TEMPERATURE_CALIBRATED
    {
        float reference_counts = BspAdc_ReadRawChannel(BSP_ADC_ANALOG_SUPPLY_VOLTAGE);
        if (AT32_INTERNAL_REFERENCE_MV <= 0.0f || reference_counts == 0.0f ||
            AT32_TEMPERATURE_SLOPE_MV_PER_C == 0.0f)
        {
            return NAN;
        }
        return AT32_TEMPERATURE_CAL_C +
               (counts * AT32_INTERNAL_REFERENCE_MV / reference_counts - AT32_TEMPERATURE_CAL_MV) /
                   AT32_TEMPERATURE_SLOPE_MV_PER_C;
    }
#else
        return NAN;
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
