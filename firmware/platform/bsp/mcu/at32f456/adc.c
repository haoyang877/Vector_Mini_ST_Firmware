/**
 * @file adc.c
 * @brief AT32 ADC硬件采样、DMA、启停、中断和片内参数换算。
 */
#include "../../bsp_internal.h"
#include "bsp_adc.h"
#include "mcu_resources.h"
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
    /* 默认GPIO时钟已由PWM初始化；独立换端口时补开对应时钟。 */
    if (MCU_ADC_GPIO_CLOCK != MCU_PWM_HIGH_CLOCK && MCU_ADC_GPIO_CLOCK != MCU_PWM_LOW_CLOCK)
    {
        crm_periph_clock_enable(MCU_ADC_GPIO_CLOCK, TRUE);
    }
    crm_periph_clock_enable(MCU_ADC_FAST_CLOCK, TRUE);
    if (BSP_ENABLED_AUX_SAMPLING || BSP_ENABLED_BUS_CURRENT)
    {
        crm_periph_clock_enable(MCU_ADC_AUX_CLOCK, TRUE);
    }
    if (BSP_ENABLED_AUX_SAMPLING || BSP_ENABLED_INTERNAL_ANALOG_SAMPLING)
    {
        crm_periph_clock_enable(MCU_ADC_DMA_CLOCK, TRUE);
    }
    gpio_init_type gpio;
    gpio_default_para_init(&gpio);
    gpio.gpio_mode = GPIO_MODE_ANALOG;
    gpio.gpio_pull = GPIO_PULL_NONE;
    gpio.gpio_pins =
        MCU_ADC_PHASE_A_PIN | MCU_ADC_PHASE_B_PIN | MCU_ADC_PHASE_C_PIN | MCU_ADC_BUS_VOLTAGE_PIN |
        (BSP_ENABLED_BUS_CURRENT ? MCU_ADC_BUS_CURRENT_PIN : 0U) |
        (BSP_ENABLED_MOTOR_TEMPERATURE ? MCU_ADC_MOTOR_TEMPERATURE_PIN : 0U) |
        (BSP_ENABLED_OUTPUT_SINCOS ? MCU_ADC_OUTPUT_SIN_PIN | MCU_ADC_OUTPUT_COS_PIN : 0U);
    gpio_init(MCU_ADC_GPIO_PORT, &gpio);
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
        adc_base_config(MCU_ADC_AUX, &base);
    }
    base.ordinary_channel_length = BSP_ENABLED_INTERNAL_ANALOG_SAMPLING ? 2U : 1U;
    adc_base_config(MCU_ADC_FAST, &base);
    adc_resolution_set(MCU_ADC_FAST, ADC_RESOLUTION_12B);
    if (BSP_ENABLED_AUX_SAMPLING || BSP_ENABLED_BUS_CURRENT)
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
    if (BSP_ENABLED_BUS_CURRENT)
    {
        adc_preempt_channel_length_set(MCU_ADC_AUX, 1U);
        /* 92.5/60MHz建立时间约1.54us；加转换约1.75us，早于ADC1四rank完成。
         * 不启用母线过采样，不混用三相四次和域；实板需比较偏置、噪声及周期预算。 */
        adc_preempt_channel_set(MCU_ADC_AUX, MCU_ADC_BUS_CURRENT_CHANNEL, 1U, ADC_SAMPLETIME_92_5);
        adc_preempt_conversion_trigger_set(
            MCU_ADC_AUX, MCU_ADC_FAST_TRIGGER, ADC_PREEMPT_TRIG_EDGE_NONE);
        adc_convert_fail_auto_abort_enable(MCU_ADC_AUX, TRUE);
    }
#if BSP_ENABLED_AUX_SAMPLING
    const adc_channel_select_type aux_channels[] = {
#if BSP_ENABLED_MOTOR_TEMPERATURE
        MCU_ADC_MOTOR_TEMPERATURE_CHANNEL,
#endif
#if BSP_ENABLED_OUTPUT_SINCOS
        MCU_ADC_OUTPUT_SIN_CHANNEL,
        MCU_ADC_OUTPUT_COS_CHANNEL,
#endif
    };
    for (uint8_t rank = 1U; rank <= ADC_AUX_RANKS; ++rank)
    {
        const adc_channel_select_type channel = aux_channels[rank - 1U];
        adc_ordinary_channel_set(MCU_ADC_AUX,
                                 channel,
                                 rank,
                                 channel == MCU_ADC_OUTPUT_SIN_CHANNEL ||
                                         channel == MCU_ADC_OUTPUT_COS_CHANNEL
                                     ? ADC_SAMPLETIME_24_5
                                     : ADC_SAMPLETIME_640_5);
    }
#if BSP_ENABLED_OUTPUT_SINCOS
    /* 普通组统一4次硬件平均，NTC与两路霍尔仍为12位原码；母线抢占组不做过采样。
     * 单次TMR6触发完成4次累加；母线抢占结束后继续普通组累加，避免反复丢弃进度。 */
    adc_oversample_ratio_shift_set(MCU_ADC_AUX, ADC_OVERSAMPLE_RATIO_4, ADC_OVERSAMPLE_SHIFT_2);
    adc_ordinary_oversample_trig_enable(MCU_ADC_AUX, FALSE);
    adc_ordinary_oversample_restart_set(MCU_ADC_AUX, ADC_OVERSAMPLE_CONTINUE);
    adc_ordinary_oversample_enable(MCU_ADC_AUX, TRUE);
    adc_preempt_oversample_enable(MCU_ADC_AUX, FALSE);
#endif
#endif
    if (BSP_ENABLED_INTERNAL_ANALOG_SAMPLING)
    {
        adc_ordinary_channel_set(
            MCU_ADC_FAST, MCU_ADC_TEMPERATURE_CHANNEL, 1U, ADC_SAMPLETIME_640_5);
        adc_ordinary_channel_set(MCU_ADC_FAST, MCU_ADC_REFERENCE_CHANNEL, 2U, ADC_SAMPLETIME_640_5);
    }
    adc_ordinary_conversion_trigger_set(
        MCU_ADC_FAST, MCU_ADC_SLOW_TRIGGER, ADC_ORDINARY_TRIG_EDGE_NONE);
    if (BSP_ENABLED_AUX_SAMPLING)
    {
        adc_ordinary_conversion_trigger_set(
            MCU_ADC_AUX, MCU_ADC_SLOW_TRIGGER, ADC_ORDINARY_TRIG_EDGE_NONE);
    }
    if (BSP_ENABLED_AUX_SAMPLING)
    {
        adc_occe_each_conversion_enable(MCU_ADC_AUX, TRUE);
        adc_dma_mode_enable(MCU_ADC_AUX, TRUE);
        adc_dma_request_repeat_enable(MCU_ADC_AUX, TRUE);
    }
    if (BSP_ENABLED_INTERNAL_ANALOG_SAMPLING)
    {
        adc_occe_each_conversion_enable(MCU_ADC_FAST, TRUE);
        adc_dma_mode_enable(MCU_ADC_FAST, TRUE);
        adc_dma_request_repeat_enable(MCU_ADC_FAST, TRUE);
    }
    dma_init_type dma;
    if (BSP_ENABLED_AUX_SAMPLING)
    {
        dma_channel_enable(MCU_ADC_AUX_DMA_CHANNEL, FALSE);
        dma_reset(MCU_ADC_AUX_DMA_CHANNEL);
        dma_default_para_init(&dma);
        dma.direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
        dma.peripheral_base_addr = (uint32_t)(uintptr_t)&MCU_ADC_AUX_RESULT;
        dma.memory_base_addr = (uint32_t)(uintptr_t)s_aux_dma;
        dma.buffer_size = 2U * ADC_AUX_RANKS;
        dma.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
        dma.memory_data_width = DMA_MEMORY_DATA_WIDTH_HALFWORD;
        dma.memory_inc_enable = TRUE;
        dma.loop_mode_enable = TRUE;
        dma.priority = DMA_PRIORITY_HIGH;
        dma_init(MCU_ADC_AUX_DMA_CHANNEL, &dma);
        dmamux_enable(MCU_ADC_DMA, TRUE);
        dmamux_init(MCU_ADC_AUX_DMA_MUX, MCU_ADC_AUX_DMA_REQUEST);
    }
    if (BSP_ENABLED_INTERNAL_ANALOG_SAMPLING)
    {
        dma_channel_enable(MCU_ADC_INTERNAL_DMA_CHANNEL, FALSE);
        dma_reset(MCU_ADC_INTERNAL_DMA_CHANNEL);
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
        dma.peripheral_base_addr = (uint32_t)(uintptr_t)&MCU_ADC_INTERNAL_RESULT;
        dma.memory_base_addr = (uint32_t)(uintptr_t)s_temperature_dma;
        dma.buffer_size = 4U;
        dma_init(MCU_ADC_INTERNAL_DMA_CHANNEL, &dma);
        if (!BSP_ENABLED_AUX_SAMPLING)
        {
            dmamux_enable(MCU_ADC_DMA, TRUE);
        }
        dmamux_init(MCU_ADC_INTERNAL_DMA_MUX, MCU_ADC_INTERNAL_DMA_REQUEST);
    }
    adc_calibrate(MCU_ADC_FAST);
    if (BSP_ENABLED_AUX_SAMPLING || BSP_ENABLED_BUS_CURRENT)
    {
        adc_calibrate(MCU_ADC_AUX);
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
    tmr_output_channel_config(MCU_ADC_TRIGGER_TIMER, MCU_ADC_TRIGGER_CHANNEL, &trigger);
    tmr_channel_value_set(
        MCU_ADC_TRIGGER_TIMER, MCU_ADC_TRIGGER_CHANNEL, AT32_TIMER_HZ * 3U / (8U * FOC_FREQ));
    tmr_primary_mode_select(MCU_ADC_TRIGGER_TIMER, MCU_ADC_TRIGGER_OUTPUT);
    tmr_primary_mode_select(MCU_CONTROL_TIMER, TMR_PRIMARY_SEL_OVERFLOW);
    adc_interrupt_enable(MCU_ADC_FAST, ADC_PCCE_INT, TRUE);
    NVIC_SetPriority(MCU_ADC_IRQ, MCU_ADC_IRQ_PRIORITY);
    NVIC_EnableIRQ(MCU_ADC_IRQ);
}

void Adc_Start(void)
{
    adc_flag_clear(MCU_ADC_FAST, ADC_PCCE_FLAG | ADC_TCF_FLAG | ADC_OCCO_FLAG);
    if (BSP_ENABLED_AUX_SAMPLING || BSP_ENABLED_BUS_CURRENT)
    {
        adc_flag_clear(MCU_ADC_AUX, ADC_PCCE_FLAG | ADC_TCF_FLAG | ADC_OCCO_FLAG);
    }
    NVIC_ClearPendingIRQ(MCU_ADC_IRQ);
#if BSP_ENABLED_AUX_SAMPLING
    dma_flag_clear(MCU_ADC_AUX_DMA_CLEAR_FLAG);
    MCU_ADC_AUX_DMA_CHANNEL->maddr = (uint32_t)(uintptr_t)s_aux_dma;
    dma_data_number_set(MCU_ADC_AUX_DMA_CHANNEL, 2U * ADC_AUX_RANKS);
    dma_channel_enable(MCU_ADC_AUX_DMA_CHANNEL, TRUE);
    adc_ordinary_conversion_trigger_set(
        MCU_ADC_AUX, MCU_ADC_SLOW_TRIGGER, ADC_ORDINARY_TRIG_EDGE_RISING);
#endif
#if BSP_ENABLED_INTERNAL_ANALOG_SAMPLING
    dma_flag_clear(MCU_ADC_INTERNAL_DMA_CLEAR_FLAG);
    MCU_ADC_INTERNAL_DMA_CHANNEL->maddr = (uint32_t)(uintptr_t)s_temperature_dma;
    dma_data_number_set(MCU_ADC_INTERNAL_DMA_CHANNEL, 4U);
    dma_channel_enable(MCU_ADC_INTERNAL_DMA_CHANNEL, TRUE);
    adc_ordinary_conversion_trigger_set(
        MCU_ADC_FAST, MCU_ADC_SLOW_TRIGGER, ADC_ORDINARY_TRIG_EDGE_RISING);
#endif
    adc_preempt_conversion_trigger_set(
        MCU_ADC_FAST, MCU_ADC_FAST_TRIGGER, ADC_PREEMPT_TRIG_EDGE_RISING);
    if (BSP_ENABLED_BUS_CURRENT)
    {
        adc_preempt_conversion_trigger_set(
            MCU_ADC_AUX, MCU_ADC_FAST_TRIGGER, ADC_PREEMPT_TRIG_EDGE_RISING);
    }
}

void Adc_Stop(void)
{
    adc_preempt_conversion_trigger_set(
        MCU_ADC_FAST, MCU_ADC_FAST_TRIGGER, ADC_PREEMPT_TRIG_EDGE_NONE);
    adc_ordinary_conversion_trigger_set(
        MCU_ADC_FAST, MCU_ADC_SLOW_TRIGGER, ADC_ORDINARY_TRIG_EDGE_NONE);
    if (BSP_ENABLED_AUX_SAMPLING || BSP_ENABLED_BUS_CURRENT)
    {
        adc_preempt_conversion_trigger_set(
            MCU_ADC_AUX, MCU_ADC_FAST_TRIGGER, ADC_PREEMPT_TRIG_EDGE_NONE);
        adc_ordinary_conversion_trigger_set(
            MCU_ADC_AUX, MCU_ADC_SLOW_TRIGGER, ADC_ORDINARY_TRIG_EDGE_NONE);
        adc_conversion_stop(MCU_ADC_AUX);
    }
    adc_conversion_stop(MCU_ADC_FAST);
    while (adc_conversion_stop_status_get(MCU_ADC_FAST) != RESET ||
           ((BSP_ENABLED_AUX_SAMPLING || BSP_ENABLED_BUS_CURRENT) &&
            adc_conversion_stop_status_get(MCU_ADC_AUX) != RESET))
    {
    }
#if BSP_ENABLED_AUX_SAMPLING
    dma_channel_enable(MCU_ADC_AUX_DMA_CHANNEL, FALSE);
#endif
#if BSP_ENABLED_INTERNAL_ANALOG_SAMPLING
    dma_channel_enable(MCU_ADC_INTERNAL_DMA_CHANNEL, FALSE);
#endif
}

bool BspAdc_CompleteFast(void)
{
    if (!adc_flag_get(MCU_ADC_FAST, ADC_PCCE_FLAG))
    {
        return false;
    }
    if (BSP_ENABLED_BUS_CURRENT && !adc_flag_get(MCU_ADC_AUX, ADC_PCCE_FLAG))
    {
        adc_flag_clear(MCU_ADC_FAST, ADC_PCCE_FLAG);
        return false;
    }
    adc_flag_clear(MCU_ADC_FAST, ADC_PCCE_FLAG);
    if (BSP_ENABLED_BUS_CURRENT)
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
    {
        uint32_t remaining = MCU_ADC_AUX_DMA_CHANNEL->dtcnt;
        uint32_t offset = remaining == 0U || remaining > ADC_AUX_RANKS ? ADC_AUX_RANKS : 0U;
        return (float)s_aux_dma[offset + MCU_ADC_MOTOR_TEMPERATURE_DMA_INDEX];
    }
#endif
#if BSP_ENABLED_OUTPUT_SINCOS
    case BSP_ADC_OUTPUT_SIN:
    case BSP_ADC_OUTPUT_COS:
    {
        uint32_t remaining = MCU_ADC_AUX_DMA_CHANNEL->dtcnt;
        uint32_t offset = remaining == 0U || remaining > ADC_AUX_RANKS ? ADC_AUX_RANKS : 0U;
        uint32_t index = BSP_ENABLED_MOTOR_TEMPERATURE + (channel == BSP_ADC_OUTPUT_COS ? 1U : 0U);
        return (float)s_aux_dma[offset + index];
    }
#endif
#if BSP_ENABLED_INTERNAL_ANALOG_SAMPLING
    case BSP_ADC_MCU_TEMPERATURE:
    case BSP_ADC_ANALOG_SUPPLY_VOLTAGE:
    {
        uint32_t remaining = MCU_ADC_INTERNAL_DMA_CHANNEL->dtcnt;
        uint32_t offset = remaining == 0U || remaining > 2U ? 2U : 0U;
        return s_temperature_dma[offset + (channel == BSP_ADC_ANALOG_SUPPLY_VOLTAGE
                                               ? MCU_ADC_REFERENCE_DMA_INDEX
                                               : MCU_ADC_TEMPERATURE_DMA_INDEX)];
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
        return AT32_INTERNAL_REFERENCE_MV <= 0.0f
                   ? NAN
                   : AT32_INTERNAL_REFERENCE_MV * AT32_ADC_MAX_COUNTS / counts / 1000.0f;
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
