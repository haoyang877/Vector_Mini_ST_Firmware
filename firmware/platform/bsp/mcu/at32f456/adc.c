/* AT32 ADC硬件所有者：配置、DMA、IRQ、故障和存储暂停共享本文件私有状态。 */
#include "bsp_internal.h"
#include "at32f45x.h"
#include "control_config.h"
#include "sampling_scale.h"
#include "bsp_config.h"
#include "bsp_system.h"
#include "bsp_pwm.h"
#include <stddef.h>

#define R40_ADC_WAIT_ITERATIONS 200000U
#define R40_FAST_SUM_MAX (R40_ADC_MAX_COUNTS * R40_ADC_ACCUMULATIONS)
#define R40_AUX_RANKS (BSP_ENABLED_MOTOR_TEMPERATURE + 2U * BSP_ENABLED_OUTPUT_SINCOS)

/** @brief AT32 ADC唯一私有状态，不跨文件暴露。 */
typedef struct
{
    volatile uint16_t aux_dma[2U * (R40_AUX_RANKS ? R40_AUX_RANKS : 1U)];
    volatile uint16_t temperature_dma[4];
    volatile uint16_t fast[4];
    volatile uint16_t fast_bus_current_counts;
    volatile uint32_t fast_sequence, last_adc_cycles;
    uint32_t last_carrier_cycles, carrier_sequence;
    volatile bool fault;
    bool initialized, started, carrier_seen;
    bool storage_owned, storage_was_started;
} AdcState;

#if MOTOR_SENSING_ADC_SAMPLE_COUNT != R40_ADC_ACCUMULATIONS
#error "R40 ADC accumulation must match the frozen raw sample domain"
#endif

static AdcState s_sampling;

#if BSP_ENABLED_OUTPUT_SINCOS
/** @brief MCU私有只读Watch快照；begin=end且为非零偶数时才可采信，原码0..4095。 */
typedef struct
{
    uint32_t begin, sequence, milliseconds, pa5_counts, pa6_counts, end;
} R40SinCosSnapshot;
volatile R40SinCosSnapshot g_r40_sincos_snapshot;
static uint32_t s_sincos_version;

static void invalidate_sincos(void)
{
    g_r40_sincos_snapshot.begin = 0U;
    __DMB();
    g_r40_sincos_snapshot.end = 0U;
}

static void publish_sincos(const uint16_t *raw, uint32_t sequence, uint32_t milliseconds)
{
    s_sincos_version += 2U;
    if (s_sincos_version == 0U)
    {
        s_sincos_version = 2U;
    }
    g_r40_sincos_snapshot.begin = s_sincos_version - 1U;
    __DMB();
    g_r40_sincos_snapshot.sequence = sequence;
    g_r40_sincos_snapshot.milliseconds = milliseconds;
    g_r40_sincos_snapshot.pa5_counts = raw[BSP_ENABLED_MOTOR_TEMPERATURE];
    g_r40_sincos_snapshot.pa6_counts = raw[BSP_ENABLED_MOTOR_TEMPERATURE + 1U];
    __DMB();
    g_r40_sincos_snapshot.end = s_sincos_version;
    __DMB();
    g_r40_sincos_snapshot.begin = s_sincos_version;
}
#else
#define invalidate_sincos() ((void)0)
#endif

static void sampling_fault(void)
{
    s_sampling.fault = true;
    BspPwm_Disable();
    invalidate_sincos();
    adc_interrupt_enable(ADC1, ADC_PCCE_INT | ADC_TCF_INT | ADC_OCCO_INT, FALSE);
    if (BSP_ENABLED_AUX_SAMPLING)
    {
        adc_interrupt_enable(ADC2, ADC_OCCO_INT | ADC_TCF_INT, FALSE);
    }
    tmr_interrupt_enable(TMR1, TMR_C1_INT, FALSE);
    if (BSP_ENABLED_AUX_SAMPLING)
    {
        dma_interrupt_enable(DMA1_CHANNEL2, DMA_HDT_INT | DMA_FDT_INT | DMA_DTERR_INT, FALSE);
    }
    if (BSP_ENABLED_INTERNAL_ANALOG_SAMPLING)
    {
        dma_interrupt_enable(DMA1_CHANNEL3, DMA_HDT_INT | DMA_FDT_INT | DMA_DTERR_INT, FALSE);
    }
    adc_preempt_conversion_trigger_set(
        ADC1, ADC_PREEMPT_TRIG_TMR1TRGOUT, ADC_PREEMPT_TRIG_EDGE_NONE);
    if (BSP_ENABLED_BUS_CURRENT)
    {
        adc_preempt_conversion_trigger_set(
            ADC2, ADC_PREEMPT_TRIG_TMR1TRGOUT, ADC_PREEMPT_TRIG_EDGE_NONE);
    }
    if (BSP_ENABLED_AUX_SAMPLING)
    {
        adc_ordinary_conversion_trigger_set(
            ADC2, ADC_ORDINARY_TRIG_TMR6TRGOUT, ADC_ORDINARY_TRIG_EDGE_NONE);
    }
    adc_ordinary_conversion_trigger_set(
        ADC1, ADC_ORDINARY_TRIG_TMR6TRGOUT, ADC_ORDINARY_TRIG_EDGE_NONE);
    if (BSP_ENABLED_AUX_SAMPLING)
    {
        dma_channel_enable(DMA1_CHANNEL2, FALSE);
    }
    if (BSP_ENABLED_INTERNAL_ANALOG_SAMPLING)
    {
        dma_channel_enable(DMA1_CHANNEL3, FALSE);
    }
}

static bool adc_ready(adc_type *adc)
{
    adc_enable(adc, TRUE);
    uint32_t remaining = R40_ADC_WAIT_ITERATIONS;
    while (adc_flag_get(adc, ADC_RDY_FLAG) == RESET && --remaining != 0U)
    {
    }
    if (remaining == 0U)
    {
        return false;
    }
    adc_calibration_init(adc);
    remaining = R40_ADC_WAIT_ITERATIONS;
    while (adc_calibration_init_status_get(adc) != RESET && --remaining != 0U)
    {
    }
    if (remaining == 0U)
    {
        return false;
    }
    adc_calibration_start(adc);
    remaining = R40_ADC_WAIT_ITERATIONS;
    while (adc_calibration_status_get(adc) != RESET && --remaining != 0U)
    {
    }
    return remaining != 0U;
}

static bool configure_sampling(void)
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
        base.ordinary_channel_length = R40_AUX_RANKS ? R40_AUX_RANKS : 1U;
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
    for (uint8_t rank = 1U; rank <= R40_AUX_RANKS; ++rank)
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
        dma.memory_base_addr = (uint32_t)(uintptr_t)s_sampling.aux_dma;
        dma.buffer_size = 2U * R40_AUX_RANKS;
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
        dma.memory_base_addr = (uint32_t)(uintptr_t)s_sampling.temperature_dma;
        dma.buffer_size = 4U;
        dma_init(DMA1_CHANNEL3, &dma);
        if (!BSP_ENABLED_AUX_SAMPLING)
        {
            dmamux_enable(DMA1, TRUE);
        }
        dmamux_init(DMA1MUX_CHANNEL3, DMAMUX_DMAREQ_ID_ADC1);
    }
    if (!adc_ready(ADC1) ||
        ((BSP_ENABLED_AUX_SAMPLING || BSP_ENABLED_BUS_CURRENT) && !adc_ready(ADC2)))
    {
        return false;
    }
    /* 数据手册规定内部温度传感器上电建立<=100us；至少18000个内核周期。 */
    for (uint32_t i = 0U; BSP_ENABLED_INTERNAL_ANALOG_SAMPLING && i < R40_CORE_HZ / 10000U; ++i)
    {
        __NOP();
    }
    return true;
}

bool McuAdc_Init(void)
{
    if (s_sampling.initialized || !BspPwm_ArePhasesDisabled())
    {
        return false;
    }
    if (!configure_sampling())
    {
        sampling_fault();
        return false;
    }
    s_sampling.initialized = true;
    return true;
}

bool McuAdc_Start(void)
{
    if (!s_sampling.initialized || s_sampling.started || s_sampling.fault ||
        !BspPwm_ArePhasesDisabled())
    {
        return false;
    }
    /* TRGO在上数3/4处上升，每载波一次；CH1没有外部功率引脚。
     * 低调制比下50%比较沿位于上数中点，不能在该处开始相电流转换。
     * 60MHz ADC的三相12.5周期/母线24.5周期共16次转换约7.47us，触发仍在18.75us。
     * 高调制比的有效采样窗口仍需单独验收，不能以本低压台架结果替代。 */
    tmr_output_config_type trigger;
    tmr_output_default_para_init(&trigger);
    trigger.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_B;
    trigger.oc_output_state = FALSE;
    trigger.occ_output_state = FALSE;
    tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_1, &trigger);
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_1, R40_TIMER_HZ * 3U / (8U * FOC_FREQ));
    tmr_primary_mode_select(TMR1, TMR_PRIMARY_SEL_C1ORAW);
    tmr_primary_mode_select(TMR6, TMR_PRIMARY_SEL_OVERFLOW);
    adc_flag_clear(ADC1, ADC_PCCE_FLAG | ADC_TCF_FLAG | ADC_OCCO_FLAG);
    if (BSP_ENABLED_AUX_SAMPLING || BSP_ENABLED_BUS_CURRENT)
    {
        adc_flag_clear(ADC2, ADC_PCCE_FLAG | ADC_TCF_FLAG | ADC_OCCO_FLAG);
    }
    if (BSP_ENABLED_AUX_SAMPLING)
    {
        dma_flag_clear(DMA1_GL2_FLAG);
    }
    if (BSP_ENABLED_INTERNAL_ANALOG_SAMPLING)
    {
        dma_flag_clear(DMA1_GL3_FLAG);
    }
    if (BSP_ENABLED_AUX_SAMPLING)
    {
        dma_interrupt_enable(DMA1_CHANNEL2, DMA_HDT_INT | DMA_FDT_INT | DMA_DTERR_INT, TRUE);
    }
    if (BSP_ENABLED_INTERNAL_ANALOG_SAMPLING)
    {
        dma_interrupt_enable(DMA1_CHANNEL3, DMA_HDT_INT | DMA_FDT_INT | DMA_DTERR_INT, TRUE);
        NVIC_SetPriority(DMA1_Channel3_IRQn, 1U);
        NVIC_EnableIRQ(DMA1_Channel3_IRQn);
    }
    adc_interrupt_enable(ADC1, ADC_PCCE_INT | ADC_TCF_INT | ADC_OCCO_INT, TRUE);
    if (BSP_ENABLED_AUX_SAMPLING)
    {
        adc_interrupt_enable(ADC2, ADC_OCCO_INT | ADC_TCF_INT, TRUE);
    }
    tmr_flag_clear(TMR1, TMR_C1_FLAG);
    tmr_interrupt_enable(TMR1, TMR_C1_INT, TRUE);
    NVIC_SetPriority(ADC1_2_IRQn, 0U);
    NVIC_SetPriority(TMR1_CH_IRQn, 1U);
    if (BSP_ENABLED_AUX_SAMPLING)
    {
        NVIC_SetPriority(DMA1_Channel2_IRQn, 1U);
    }
    NVIC_EnableIRQ(ADC1_2_IRQn);
    NVIC_EnableIRQ(TMR1_CH_IRQn);
    if (BSP_ENABLED_AUX_SAMPLING)
    {
        NVIC_EnableIRQ(DMA1_Channel2_IRQn);
    }
    s_sampling.started = true;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    s_sampling.last_adc_cycles = s_sampling.last_carrier_cycles = DWT->CYCCNT;
    if (BSP_ENABLED_AUX_SAMPLING)
    {
        dma_channel_enable(DMA1_CHANNEL2, TRUE);
    }
    if (BSP_ENABLED_INTERNAL_ANALOG_SAMPLING)
    {
        dma_channel_enable(DMA1_CHANNEL3, TRUE);
        adc_ordinary_conversion_trigger_set(
            ADC1, ADC_ORDINARY_TRIG_TMR6TRGOUT, ADC_ORDINARY_TRIG_EDGE_RISING);
    }
    adc_preempt_conversion_trigger_set(
        ADC1, ADC_PREEMPT_TRIG_TMR1TRGOUT, ADC_PREEMPT_TRIG_EDGE_RISING);
    if (BSP_ENABLED_BUS_CURRENT)
    {
        adc_preempt_conversion_trigger_set(
            ADC2, ADC_PREEMPT_TRIG_TMR1TRGOUT, ADC_PREEMPT_TRIG_EDGE_RISING);
    }
    if (BSP_ENABLED_AUX_SAMPLING)
    {
        adc_ordinary_conversion_trigger_set(
            ADC2, ADC_ORDINARY_TRIG_TMR6TRGOUT, ADC_ORDINARY_TRIG_EDGE_RISING);
    }
    return true;
}

bool McuAdc_CompleteFast(void)
{
    if (!s_sampling.started || s_sampling.fault)
    {
        return false;
    }
    if (adc_flag_get(ADC1, ADC_TCF_FLAG | ADC_OCCO_FLAG) ||
        ((BSP_ENABLED_AUX_SAMPLING || BSP_ENABLED_BUS_CURRENT) &&
         (adc_flag_get(ADC2, ADC_TCF_FLAG) || adc_flag_get(ADC2, ADC_OCCO_FLAG))))
    {
        adc_flag_clear(ADC1, ADC_TCF_FLAG | ADC_PCCE_FLAG | ADC_OCCO_FLAG);
        if (BSP_ENABLED_AUX_SAMPLING)
        {
            adc_flag_clear(ADC2, ADC_PCCE_FLAG | ADC_TCF_FLAG | ADC_OCCO_FLAG);
        }
        sampling_fault();
        return false;
    }
    if (!adc_flag_get(ADC1, ADC_PCCE_FLAG))
    {
        return false;
    }
    if (BSP_ENABLED_BUS_CURRENT)
    {
        /* ADC2应先于ADC1完成；没有当拍完成位就锁存采集故障，绝不沿用旧母线缓存。 */
        if (!adc_flag_get(ADC2, ADC_PCCE_FLAG))
        {
            sampling_fault();
            return false;
        }
        uint16_t bus_counts = adc_preempt_conversion_data_get(ADC2, ADC_PREEMPT_CHANNEL_1);
        adc_flag_clear(ADC2, ADC_PCCE_FLAG);
        if (bus_counts > R40_ADC_MAX_COUNTS)
        {
            sampling_fault();
            return false;
        }
        s_sampling.fast_bus_current_counts = bus_counts;
    }
    uint16_t values[4];
    for (uint32_t rank = 0U; rank < 4U; ++rank)
    {
        values[rank] = adc_preempt_conversion_data_get(ADC1, (adc_preempt_channel_type)rank);
        if (values[rank] > R40_FAST_SUM_MAX)
        {
            sampling_fault();
            return false;
        }
    }
    adc_flag_clear(ADC1, ADC_PCCE_FLAG);
    if (adc_flag_get(ADC1, ADC_TCF_FLAG))
    {
        sampling_fault();
        return false;
    }
    for (uint32_t rank = 0U; rank < 4U; ++rank)
    {
        s_sampling.fast[rank] = values[rank];
    }
    ++s_sampling.fast_sequence;
    s_sampling.last_adc_cycles = DWT->CYCCNT;
    return true;
}

static void sampling_carrier_tick(void)
{
    /* 载波IRQ低于ADC优先级；序号、时间戳和DWT必须同一快照，
     * 防止ADC在比较中更新后形成无符号时间差下溢，误报整周期漏采。 */
    uint32_t mask = BspSystem_EnterCritical();
    if (!s_sampling.started || s_sampling.fault)
    {
        BspSystem_ExitCritical(mask);
        return;
    }
    uint32_t now_cycles = DWT->CYCCNT;
    /* 上次载波IRQ可能在当拍ADC完成之后才执行；下次载波IRQ若恢复正常顺序，
     * 序列号相同不代表漏采。只在一整周期没有新数据，或周期计数停走时关断。 */
    if (s_sampling.carrier_seen && s_sampling.carrier_sequence == s_sampling.fast_sequence &&
        ((uint32_t)(now_cycles - s_sampling.last_adc_cycles) >= R40_CORE_HZ / FOC_FREQ ||
         now_cycles == s_sampling.last_carrier_cycles))
    {

        sampling_fault();
        BspSystem_ExitCritical(mask);
        return;
    }
    s_sampling.carrier_sequence = s_sampling.fast_sequence;
    s_sampling.last_carrier_cycles = now_cycles;
    s_sampling.carrier_seen = true;
    BspSystem_ExitCritical(mask);
}

bool Adc_IsHealthy(void)
{
    return s_sampling.started && !s_sampling.fault;
}

void TMR1_CH_IRQHandler(void)
{
    if (tmr_flag_get(TMR1, TMR_C1_FLAG))
    {
        tmr_flag_clear(TMR1, TMR_C1_FLAG);
        sampling_carrier_tick();
    }
}

void DMA1_Channel2_IRQHandler(void)
{
#if BSP_ENABLED_AUX_SAMPLING
    bool first = dma_flag_get(DMA1_HDT2_FLAG) != RESET;
    bool second = dma_flag_get(DMA1_FDT2_FLAG) != RESET;
    if (!s_sampling.started || s_sampling.fault || dma_flag_get(DMA1_DTERR2_FLAG) ||
        (first && second))
    {
        dma_flag_clear(DMA1_GL2_FLAG);
        sampling_fault();
        return;
    }
    if (!first && !second)
    {
        return;
    }
    uint32_t offset = first ? 0U : R40_AUX_RANKS;
    uint16_t raw[R40_AUX_RANKS];
    for (unsigned rank = 0U; rank < R40_AUX_RANKS; ++rank)
    {
        raw[rank] = s_sampling.aux_dma[offset + rank];
        if (raw[rank] > R40_ADC_MAX_COUNTS)
        {
            sampling_fault();
            return;
        }
    }
#if BSP_ENABLED_OUTPUT_SINCOS
    /* 复制若跨过下一半完成，不发布可能被覆盖的两路配对值。 */
    if (dma_flag_get(DMA1_DTERR2_FLAG | (first ? DMA1_FDT2_FLAG : DMA1_HDT2_FLAG)))
    {
        sampling_fault();
        return;
    }
#endif
    AdcAuxSample samples = {BSP_ENABLED_MOTOR_TEMPERATURE ? raw[0] : 0U, 0U, 0U};
    dma_flag_clear(first ? DMA1_HDT2_FLAG : DMA1_FDT2_FLAG);
    uint32_t now_ms = BspSystem_TimeMs();
    samples.sequence = AdcFrames_PublishAux(&samples, now_ms);
#if BSP_ENABLED_OUTPUT_SINCOS
    publish_sincos(raw, samples.sequence, now_ms);
#endif
#endif
}

void DMA1_Channel3_IRQHandler(void)
{
#if BSP_ENABLED_INTERNAL_ANALOG_SAMPLING
    bool first = dma_flag_get(DMA1_HDT3_FLAG) != RESET;
    bool second = dma_flag_get(DMA1_FDT3_FLAG) != RESET;
    if (!s_sampling.started || s_sampling.fault || dma_flag_get(DMA1_DTERR3_FLAG) ||
        (first && second))
    {
        dma_flag_clear(DMA1_GL3_FLAG);
        sampling_fault();
        return;
    }
    if (!first && !second)
    {
        return;
    }
    uint32_t offset = first ? 0U : 2U;
    AdcTemperatureSample samples = {
        s_sampling.temperature_dma[offset], s_sampling.temperature_dma[offset + 1U], 0U, 0U};
    if (samples.mcu_temperature_counts > R40_ADC_MAX_COUNTS ||
        samples.reference_counts > R40_ADC_MAX_COUNTS)
    {
        sampling_fault();
        return;
    }
    dma_flag_clear(first ? DMA1_HDT3_FLAG : DMA1_FDT3_FLAG);
    AdcFrames_PublishTemperature(&samples, BspSystem_TimeMs());
#endif
}

void McuAdc_ContinueTemperature(void)
{
    /* TMR6硬件触发普通序列，不添加软件转换。 */
}

uint16_t McuAdc_ReadRank(unsigned rank)
{
    return rank < 4U && !s_sampling.fault ? s_sampling.fast[rank] : 0U;
}

bool McuAdc_PollTemperature(AdcTemperatureSample *out)
{
    (void)out;
    return false;
}

uint16_t McuAdc_ReadBusCurrent(void)
{
    return s_sampling.fast_bus_current_counts;
}

bool McuAdc_PauseForStorage(void)
{
    if (__get_IPSR() != 0U || __get_PRIMASK() == 0U || !BspPwm_ArePhasesDisabled() ||
        s_sampling.fault || s_sampling.storage_owned)
    {
        return false;
    }
    s_sampling.storage_owned = true;
    s_sampling.storage_was_started = s_sampling.started;
    if (!s_sampling.started)
    {
        return true;
    }
    adc_preempt_conversion_trigger_set(
        ADC1, ADC_PREEMPT_TRIG_TMR1TRGOUT, ADC_PREEMPT_TRIG_EDGE_NONE);
    if (BSP_ENABLED_BUS_CURRENT)
    {
        adc_preempt_conversion_trigger_set(
            ADC2, ADC_PREEMPT_TRIG_TMR1TRGOUT, ADC_PREEMPT_TRIG_EDGE_NONE);
    }
    adc_ordinary_conversion_trigger_set(
        ADC1, ADC_ORDINARY_TRIG_TMR6TRGOUT, ADC_ORDINARY_TRIG_EDGE_NONE);
    if (BSP_ENABLED_AUX_SAMPLING)
    {
        adc_ordinary_conversion_trigger_set(
            ADC2, ADC_ORDINARY_TRIG_TMR6TRGOUT, ADC_ORDINARY_TRIG_EDGE_NONE);
    }
    adc_conversion_stop(ADC1);
    if (BSP_ENABLED_AUX_SAMPLING || BSP_ENABLED_BUS_CURRENT)
    {
        adc_conversion_stop(ADC2);
    }
    uint32_t remaining = R40_ADC_WAIT_ITERATIONS;
    while ((adc_conversion_stop_status_get(ADC1) != RESET ||
            ((BSP_ENABLED_AUX_SAMPLING || BSP_ENABLED_BUS_CURRENT) &&
             adc_conversion_stop_status_get(ADC2) != RESET)) &&
           --remaining != 0U)
    {
    }
    if (remaining == 0U)
    {
        sampling_fault();
        return false;
    }
    if (BSP_ENABLED_AUX_SAMPLING)
    {
        dma_channel_enable(DMA1_CHANNEL2, FALSE);
    }
    if (BSP_ENABLED_INTERNAL_ANALOG_SAMPLING)
    {
        dma_channel_enable(DMA1_CHANNEL3, FALSE);
    }
    s_sampling.started = false;
    AdcFrames_Invalidate();
    invalidate_sincos();
    return true;
}

bool McuAdc_ResumeAfterStorage(void)
{
    if (!s_sampling.storage_owned || __get_IPSR() != 0U || __get_PRIMASK() == 0U ||
        !BspPwm_ArePhasesDisabled() || s_sampling.fault)
    {
        return false;
    }
    if (!s_sampling.storage_was_started)
    {
        s_sampling.storage_owned = false;
        return true;
    }
    adc_flag_clear(ADC1, ADC_PCCE_FLAG | ADC_TCF_FLAG | ADC_OCCO_FLAG);
    if (BSP_ENABLED_AUX_SAMPLING || BSP_ENABLED_BUS_CURRENT)
    {
        adc_flag_clear(ADC2, ADC_PCCE_FLAG | ADC_TCF_FLAG | ADC_OCCO_FLAG);
        if (BSP_ENABLED_AUX_SAMPLING)
        {
            dma_flag_clear(DMA1_GL2_FLAG);
        }
    }
    if (BSP_ENABLED_INTERNAL_ANALOG_SAMPLING)
    {
        dma_flag_clear(DMA1_GL3_FLAG);
    }
    tmr_flag_clear(TMR1, TMR_C1_FLAG);
    NVIC_ClearPendingIRQ(ADC1_2_IRQn);
    NVIC_ClearPendingIRQ(TMR1_CH_IRQn);
    if (BSP_ENABLED_AUX_SAMPLING)
    {
        NVIC_ClearPendingIRQ(DMA1_Channel2_IRQn);
    }
    if (BSP_ENABLED_INTERNAL_ANALOG_SAMPLING)
    {
        NVIC_ClearPendingIRQ(DMA1_Channel3_IRQn);
    }
    if (BSP_ENABLED_AUX_SAMPLING)
    {
        DMA1_CHANNEL2->maddr = (uint32_t)(uintptr_t)s_sampling.aux_dma;
        dma_data_number_set(DMA1_CHANNEL2, 2U * R40_AUX_RANKS);
    }
    if (BSP_ENABLED_INTERNAL_ANALOG_SAMPLING)
    {
        DMA1_CHANNEL3->maddr = (uint32_t)(uintptr_t)s_sampling.temperature_dma;
        dma_data_number_set(DMA1_CHANNEL3, 4U);
        dma_channel_enable(DMA1_CHANNEL3, TRUE);
        adc_ordinary_conversion_trigger_set(
            ADC1, ADC_ORDINARY_TRIG_TMR6TRGOUT, ADC_ORDINARY_TRIG_EDGE_RISING);
    }
    uint32_t remaining;
    uint32_t started;
    if (BSP_ENABLED_AUX_SAMPLING)
    {
        dma_channel_enable(DMA1_CHANNEL2, TRUE);
        adc_ordinary_conversion_trigger_set(
            ADC2, ADC_ORDINARY_TRIG_TMR6TRGOUT, ADC_ORDINARY_TRIG_EDGE_RISING);
        /* 先等辅助首帧，再放开快采样，避免等待辅助期间积累多个快帧。 */
        remaining = R40_ADC_WAIT_ITERATIONS;
        started = DWT->CYCCNT;
        while (dma_flag_get(DMA1_HDT2_FLAG) == RESET && --remaining != 0U &&
               (uint32_t)(DWT->CYCCNT - started) < R40_CORE_HZ / 1000U)
        {
        }
        if (remaining == 0U || dma_flag_get(DMA1_HDT2_FLAG) == RESET ||
            dma_flag_get(DMA1_DTERR2_FLAG | DMA1_FDT2_FLAG) != RESET)
        {
            sampling_fault();
            return false;
        }
    }
    s_sampling.carrier_seen = false;
    s_sampling.last_adc_cycles = s_sampling.last_carrier_cycles = DWT->CYCCNT;
    s_sampling.started = true;
    adc_preempt_conversion_trigger_set(
        ADC1, ADC_PREEMPT_TRIG_TMR1TRGOUT, ADC_PREEMPT_TRIG_EDGE_RISING);
    if (BSP_ENABLED_BUS_CURRENT)
    {
        adc_preempt_conversion_trigger_set(
            ADC2, ADC_PREEMPT_TRIG_TMR1TRGOUT, ADC_PREEMPT_TRIG_EDGE_RISING);
    }
    remaining = R40_ADC_WAIT_ITERATIONS;
    started = DWT->CYCCNT;
    while (adc_flag_get(ADC1, ADC_PCCE_FLAG) == RESET && --remaining != 0U &&
           (uint32_t)(DWT->CYCCNT - started) < R40_CORE_HZ / 10000U)
    {
    }
    if (remaining == 0U || adc_flag_get(ADC1, ADC_PCCE_FLAG) == RESET ||
        adc_flag_get(ADC1, ADC_TCF_FLAG | ADC_OCCO_FLAG) != RESET)
    {
        sampling_fault();
        return false;
    }
    s_sampling.storage_owned = false;
    return true;
}
