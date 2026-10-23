#include "motor_sensing.h"

#include <stddef.h>
#include <string.h>

#include "at32f45x.h"
#include "critical_hw.h"
#include "control_config.h"
#include "power_stage_hw.h"
#include "r40_adc.h"
#include "r40_clock.h"
#include "r40_sincos_diagnostic.h"
#include "time_hw.h"
#include "../../config/boards/r40_analog.h"

/* ADC1抢占序列：PA0/1/2相电流、PA3母线电压，均为硬件4次累加、不右移。
 * ADC2普通序列：PA4母线电流、可选PA7电机温度，诊断目标追加PA5/PA6输出轴原始信号。
 * 内部温度和VREF只能由ADC1普通序列取得，使用DMA1通道3，不占用抢占序列。 */
#define R40_ADC_WAIT_ITERATIONS 200000U
#define R40_FAST_SUM_MAX (R40_ADC_MAX_COUNTS * R40_ADC_ACCUMULATIONS)
#define R40_AUX_BASE_RANKS (R40_MOTOR_NTC_ENABLED ? 2U : 1U)
#define R40_AUX_RANKS (R40_AUX_BASE_RANKS + (R40_OUTPUT_SINCOS_DIAGNOSTICS ? 2U : 0U))

#if R40_OUTPUT_SINCOS_DIAGNOSTICS
/* 板级诊断对象，不是BSP接口。原始采样2kHz，Keil台架发布2kHz，其余快照50Hz。
 * 读取方必须核对begin=end且为非零偶数；PA5/PA6的sin/cos次序不在本层推断。 */
volatile R40SinCosDiagnostic g_r40_sincos_snapshot;
static uint8_t s_sincos_divider;
#endif

#if MOTOR_SENSING_ADC_SAMPLE_COUNT != R40_ADC_ACCUMULATIONS
#error "R40 ADC accumulation must match the frozen raw sample domain"
#endif

static volatile uint16_t s_aux_dma[2U * R40_AUX_RANKS];
static volatile uint16_t s_temperature_dma[4];
static volatile uint16_t s_fast[4];
static volatile uint32_t s_fast_sequence;
static volatile uint32_t s_last_adc_cycles;
static uint32_t s_last_carrier_cycles;
/* 私有诊断原因，供验证入口保留首个采样关断位置。 */
volatile uint32_t g_r40_adc_fault_reason;
volatile uint32_t g_r40_adc_stall_context[4];
static volatile bool s_fault;
static bool s_initialized;
static bool s_started;
static bool s_carrier_seen;
static uint32_t s_carrier_sequence;
static volatile R40AuxSamples s_aux;
static volatile uint32_t s_aux_ms;
static volatile R40TemperatureSamples s_temperature;
static volatile uint32_t s_temperature_ms;
static volatile bool s_aux_seen;
static volatile bool s_temperature_seen;

static void sampling_fault(uint32_t reason)
{
    if (!s_fault)
    {
        g_r40_adc_fault_reason = reason;
    }
    s_fault = true;
    power_stage_hw_stop();
    adc_interrupt_enable(ADC1, ADC_PCCE_INT | ADC_TCF_INT | ADC_OCCO_INT, FALSE);
    adc_interrupt_enable(ADC2, ADC_OCCO_INT | ADC_TCF_INT, FALSE);
    tmr_interrupt_enable(TMR1, TMR_C1_INT, FALSE);
    dma_interrupt_enable(DMA1_CHANNEL2, DMA_HDT_INT | DMA_FDT_INT | DMA_DTERR_INT, FALSE);
    dma_interrupt_enable(DMA1_CHANNEL3, DMA_HDT_INT | DMA_FDT_INT | DMA_DTERR_INT, FALSE);
    adc_preempt_conversion_trigger_set(
        ADC1, ADC_PREEMPT_TRIG_TMR1TRGOUT, ADC_PREEMPT_TRIG_EDGE_NONE);
    adc_ordinary_conversion_trigger_set(
        ADC2, ADC_ORDINARY_TRIG_TMR6TRGOUT, ADC_ORDINARY_TRIG_EDGE_NONE);
    adc_ordinary_conversion_trigger_set(
        ADC1, ADC_ORDINARY_TRIG_TMR6TRGOUT, ADC_ORDINARY_TRIG_EDGE_NONE);
    dma_channel_enable(DMA1_CHANNEL2, FALSE);
    dma_channel_enable(DMA1_CHANNEL3, FALSE);
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

bool R40_AdcInit(void)
{
    if (s_initialized || !power_stage_hw_phases_disabled())
    {
        return false;
    }
    crm_periph_clock_enable(CRM_ADC1_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_ADC2_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
    gpio_init_type gpio;
    gpio_default_para_init(&gpio);
    gpio.gpio_mode = GPIO_MODE_ANALOG;
    gpio.gpio_pull = GPIO_PULL_NONE;
    gpio.gpio_pins = GPIO_PINS_0 | GPIO_PINS_1 | GPIO_PINS_2 | GPIO_PINS_3 | GPIO_PINS_4 |
                     (R40_MOTOR_NTC_ENABLED ? GPIO_PINS_7 : 0U) |
                     (R40_OUTPUT_SINCOS_DIAGNOSTICS ? GPIO_PINS_5 | GPIO_PINS_6 : 0U);
    gpio_init(GPIOA, &gpio);
    adc_reset();
    adc_common_config_type common;
    adc_common_default_para_init(&common);
    common.combine_mode = ADC_INDEPENDENT_MODE;
    common.div = ADC_HCLK_DIV_6;
    common.common_dma_mode = ADC_COMMON_DMAMODE_DISABLE;
    common.tempervintrv_state = R40_TEMPERATURE_SENSING_ENABLED ? TRUE : FALSE;
    adc_common_config(&common);
    adc_base_config_type base;
    adc_base_default_para_init(&base);
    base.sequence_mode = TRUE;
    base.repeat_mode = FALSE;
    base.data_align = ADC_RIGHT_ALIGNMENT;
    base.ordinary_channel_length = R40_AUX_RANKS;
    adc_base_config(ADC2, &base);
    base.ordinary_channel_length = R40_TEMPERATURE_SENSING_ENABLED ? 2U : 1U;
    adc_base_config(ADC1, &base);
    adc_resolution_set(ADC1, ADC_RESOLUTION_12B);
    adc_resolution_set(ADC2, ADC_RESOLUTION_12B);
    adc_preempt_channel_length_set(ADC1, 4U);
    for (uint8_t rank = 1U; rank <= 4U; ++rank)
    {
        adc_preempt_channel_set(
            ADC1, (adc_channel_select_type)(rank - 1U), rank, ADC_SAMPLETIME_12_5);
    }
    adc_oversample_ratio_shift_set(ADC1, ADC_OVERSAMPLE_RATIO_4, ADC_OVERSAMPLE_SHIFT_0);
    adc_preempt_oversample_enable(ADC1, TRUE);
    adc_preempt_conversion_trigger_set(
        ADC1, ADC_PREEMPT_TRIG_TMR1TRGOUT, ADC_PREEMPT_TRIG_EDGE_NONE);
    adc_convert_fail_auto_abort_enable(ADC1, TRUE);
    const adc_channel_select_type aux_channels[] = {
        ADC_CHANNEL_4,
#if R40_MOTOR_NTC_ENABLED
        ADC_CHANNEL_7,
#endif
#if R40_OUTPUT_SINCOS_DIAGNOSTICS
        ADC_CHANNEL_5,
        ADC_CHANNEL_6,
#endif
    };
    for (uint8_t rank = 1U; rank <= R40_AUX_RANKS; ++rank)
    {
        adc_ordinary_channel_set(ADC2, aux_channels[rank - 1U], rank, ADC_SAMPLETIME_640_5);
    }
    if (R40_TEMPERATURE_SENSING_ENABLED)
    {
        adc_ordinary_channel_set(ADC1, ADC_CHANNEL_16, 1U, ADC_SAMPLETIME_640_5);
        adc_ordinary_channel_set(ADC1, ADC_CHANNEL_17, 2U, ADC_SAMPLETIME_640_5);
    }
    adc_ordinary_conversion_trigger_set(
        ADC1, ADC_ORDINARY_TRIG_TMR6TRGOUT, ADC_ORDINARY_TRIG_EDGE_NONE);
    adc_ordinary_conversion_trigger_set(
        ADC2, ADC_ORDINARY_TRIG_TMR6TRGOUT, ADC_ORDINARY_TRIG_EDGE_NONE);
    adc_occe_each_conversion_enable(ADC2, TRUE);
    adc_dma_mode_enable(ADC2, TRUE);
    adc_dma_request_repeat_enable(ADC2, TRUE);
    if (R40_TEMPERATURE_SENSING_ENABLED)
    {
        adc_occe_each_conversion_enable(ADC1, TRUE);
        adc_dma_mode_enable(ADC1, TRUE);
        adc_dma_request_repeat_enable(ADC1, TRUE);
    }
    dma_channel_enable(DMA1_CHANNEL2, FALSE);
    dma_reset(DMA1_CHANNEL2);
    dma_init_type dma;
    dma_default_para_init(&dma);
    dma.direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
    dma.peripheral_base_addr = (uint32_t)(uintptr_t)&ADC2->odt;
    dma.memory_base_addr = (uint32_t)(uintptr_t)s_aux_dma;
    dma.buffer_size = 2U * R40_AUX_RANKS;
    dma.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
    dma.memory_data_width = DMA_MEMORY_DATA_WIDTH_HALFWORD;
    dma.memory_inc_enable = TRUE;
    dma.loop_mode_enable = TRUE;
    dma.priority = DMA_PRIORITY_HIGH;
    dma_init(DMA1_CHANNEL2, &dma);
    dmamux_enable(DMA1, TRUE);
    dmamux_init(DMA1MUX_CHANNEL2, DMAMUX_DMAREQ_ID_ADC2);
    if (R40_TEMPERATURE_SENSING_ENABLED)
    {
        dma_channel_enable(DMA1_CHANNEL3, FALSE);
        dma_reset(DMA1_CHANNEL3);
        dma.peripheral_base_addr = (uint32_t)(uintptr_t)&ADC1->odt;
        dma.memory_base_addr = (uint32_t)(uintptr_t)s_temperature_dma;
        dma.buffer_size = 4U;
        dma_init(DMA1_CHANNEL3, &dma);
        dmamux_init(DMA1MUX_CHANNEL3, DMAMUX_DMAREQ_ID_ADC1);
    }
    if (!adc_ready(ADC1) || !adc_ready(ADC2))
    {
        sampling_fault(1U);
        return false;
    }
    /* 数据手册规定内部温度传感器上电建立<=100us；至少18000个内核周期。 */
    for (uint32_t i = 0U; R40_TEMPERATURE_SENSING_ENABLED && i < R40_CORE_HZ / 10000U; ++i)
    {
        __NOP();
    }
    s_initialized = true;
    return true;
}

bool R40_AdcStart(void)
{
    if (!s_initialized || s_started || s_fault || !power_stage_hw_phases_disabled())
    {
        return false;
    }
    /* TRGO在上数3/4处上升，每载波一次；CH1没有外部功率引脚。
     * 低调制比下50%比较沿位于上数中点，不能在该处开始相电流转换。
     * 30MHz ADC的16次转换约13.34us，从18.75us开始覆盖顶部安静区。
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
    adc_flag_clear(ADC2, ADC_TCF_FLAG | ADC_OCCO_FLAG);
    dma_flag_clear(DMA1_GL2_FLAG);
    dma_flag_clear(DMA1_GL3_FLAG);
    dma_interrupt_enable(DMA1_CHANNEL2, DMA_HDT_INT | DMA_FDT_INT | DMA_DTERR_INT, TRUE);
    if (R40_TEMPERATURE_SENSING_ENABLED)
    {
        dma_interrupt_enable(DMA1_CHANNEL3, DMA_HDT_INT | DMA_FDT_INT | DMA_DTERR_INT, TRUE);
        NVIC_SetPriority(DMA1_Channel3_IRQn, 1U);
        NVIC_EnableIRQ(DMA1_Channel3_IRQn);
    }
    adc_interrupt_enable(ADC1, ADC_PCCE_INT | ADC_TCF_INT | ADC_OCCO_INT, TRUE);
    adc_interrupt_enable(ADC2, ADC_OCCO_INT | ADC_TCF_INT, TRUE);
    tmr_flag_clear(TMR1, TMR_C1_FLAG);
    tmr_interrupt_enable(TMR1, TMR_C1_INT, TRUE);
    NVIC_SetPriority(ADC1_2_IRQn, 0U);
    NVIC_SetPriority(TMR1_CH_IRQn, 1U);
    NVIC_SetPriority(DMA1_Channel2_IRQn, 1U);
    NVIC_EnableIRQ(ADC1_2_IRQn);
    NVIC_EnableIRQ(TMR1_CH_IRQn);
    NVIC_EnableIRQ(DMA1_Channel2_IRQn);
    s_started = true;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    s_last_adc_cycles = s_last_carrier_cycles = DWT->CYCCNT;
    dma_channel_enable(DMA1_CHANNEL2, TRUE);
    if (R40_TEMPERATURE_SENSING_ENABLED)
    {
        dma_channel_enable(DMA1_CHANNEL3, TRUE);
        adc_ordinary_conversion_trigger_set(
            ADC1, ADC_ORDINARY_TRIG_TMR6TRGOUT, ADC_ORDINARY_TRIG_EDGE_RISING);
    }
    adc_preempt_conversion_trigger_set(
        ADC1, ADC_PREEMPT_TRIG_TMR1TRGOUT, ADC_PREEMPT_TRIG_EDGE_RISING);
    adc_ordinary_conversion_trigger_set(
        ADC2, ADC_ORDINARY_TRIG_TMR6TRGOUT, ADC_ORDINARY_TRIG_EDGE_RISING);
    return true;
}

bool R40_AdcComplete(void)
{
    if (!s_started || s_fault)
    {
        return false;
    }
    if (adc_flag_get(ADC1, ADC_TCF_FLAG | ADC_OCCO_FLAG) || adc_flag_get(ADC2, ADC_TCF_FLAG) ||
        adc_flag_get(ADC2, ADC_OCCO_FLAG))
    {
        adc_flag_clear(ADC1, ADC_TCF_FLAG | ADC_PCCE_FLAG | ADC_OCCO_FLAG);
        adc_flag_clear(ADC2, ADC_TCF_FLAG | ADC_OCCO_FLAG);
        sampling_fault(2U);
        return false;
    }
    if (!adc_flag_get(ADC1, ADC_PCCE_FLAG))
    {
        return false;
    }
    uint16_t values[4];
    for (uint32_t rank = 0U; rank < 4U; ++rank)
    {
        values[rank] = adc_preempt_conversion_data_get(ADC1, (adc_preempt_channel_type)rank);
        if (values[rank] > R40_FAST_SUM_MAX)
        {
            sampling_fault(3U);
            return false;
        }
    }
    adc_flag_clear(ADC1, ADC_PCCE_FLAG);
    if (adc_flag_get(ADC1, ADC_TCF_FLAG))
    {
        sampling_fault(4U);
        return false;
    }
    for (uint32_t rank = 0U; rank < 4U; ++rank)
    {
        s_fast[rank] = values[rank];
    }
    ++s_fast_sequence;
    s_last_adc_cycles = DWT->CYCCNT;
    return true;
}

void R40_AdcCarrierTick(void)
{
    /* 载波IRQ低于ADC优先级；序号、时间戳和DWT必须同一快照，
     * 防止ADC在比较中更新后形成无符号时间差下溢，误报整周期漏采。 */
    uint32_t mask = critical_hw_enter();
    if (!s_started || s_fault)
    {
        critical_hw_exit(mask);
        return;
    }
    uint32_t now_cycles = DWT->CYCCNT;
    /* 上次载波IRQ可能在当拍ADC完成之后才执行；下次载波IRQ若恢复正常顺序，
     * 序列号相同不代表漏采。只在一整周期没有新数据，或周期计数停走时关断。 */
    if (s_carrier_seen && s_carrier_sequence == s_fast_sequence &&
        ((uint32_t)(now_cycles - s_last_adc_cycles) >= R40_CORE_HZ / FOC_FREQ ||
         now_cycles == s_last_carrier_cycles))
    {
        g_r40_adc_stall_context[0] = now_cycles - s_last_adc_cycles;
        g_r40_adc_stall_context[1] = now_cycles - s_last_carrier_cycles;
        g_r40_adc_stall_context[2] = ADC1->sts;
        g_r40_adc_stall_context[3] = TMR1->cval;
        sampling_fault(5U);
        critical_hw_exit(mask);
        return;
    }
    s_carrier_sequence = s_fast_sequence;
    s_last_carrier_cycles = now_cycles;
    s_carrier_seen = true;
    critical_hw_exit(mask);
}

uint16_t motor_hw_current_sample_raw(MotorHwCurrentPhase phase)
{
    return (uint32_t)phase < 3U && !s_fault ? s_fast[(uint32_t)phase] : 0U;
}

uint16_t motor_hw_vbus_sample_raw(void)
{
    return !s_fault ? s_fast[3] : 0U;
}

bool R40_AdcReadAux(R40AuxSamples *out)
{
    if (out == NULL)
    {
        return false;
    }
    uint32_t mask = critical_hw_enter();
    bool valid =
        s_started && !s_fault && s_aux_seen && (uint32_t)(time_hw_now_ms() - s_aux_ms) <= 2U;
    if (valid)
    {
        *out = s_aux;
    }
    critical_hw_exit(mask);
    return valid;
}

bool R40_AdcHealthy(void)
{
    return s_started && !s_fault;
}

bool R40_AdcReadTemperature(R40TemperatureSamples *out)
{
    if (out == NULL)
    {
        return false;
    }
    uint32_t mask = critical_hw_enter();
    bool valid = R40_TEMPERATURE_SENSING_ENABLED && s_started && !s_fault && s_temperature_seen &&
                 (uint32_t)(time_hw_now_ms() - s_temperature_ms) <= 2U;
    if (valid)
    {
        *out = s_temperature;
    }
    critical_hw_exit(mask);
    return valid;
}

void TMR1_CH_IRQHandler(void)
{
    if (tmr_flag_get(TMR1, TMR_C1_FLAG))
    {
        tmr_flag_clear(TMR1, TMR_C1_FLAG);
        R40_AdcCarrierTick();
    }
}

void DMA1_Channel2_IRQHandler(void)
{
    bool first = dma_flag_get(DMA1_HDT2_FLAG) != RESET;
    bool second = dma_flag_get(DMA1_FDT2_FLAG) != RESET;
    if (!s_started || s_fault || dma_flag_get(DMA1_DTERR2_FLAG) || (first && second))
    {
        dma_flag_clear(DMA1_GL2_FLAG);
        sampling_fault(6U);
        return;
    }
    if (!first && !second)
    {
        return;
    }
    uint32_t offset = first ? 0U : R40_AUX_RANKS;
    R40AuxSamples samples = {s_aux_dma[offset],
                             R40_MOTOR_NTC_ENABLED ? s_aux_dma[offset + 1U] : 0U,
                             s_aux.sequence + 1U};
    if (samples.bus_current_counts > R40_ADC_MAX_COUNTS ||
        samples.motor_temperature_counts > R40_ADC_MAX_COUNTS)
    {
        sampling_fault(7U);
        return;
    }
#if R40_OUTPUT_SINCOS_DIAGNOSTICS
    uint16_t pa5 = s_aux_dma[offset + R40_AUX_BASE_RANKS];
    uint16_t pa6 = s_aux_dma[offset + R40_AUX_BASE_RANKS + 1U];
    if (pa5 > R40_ADC_MAX_COUNTS || pa6 > R40_ADC_MAX_COUNTS)
    {
        sampling_fault(8U);
        return;
    }
    if (s_sincos_divider == 0U)
    {
        uint32_t version = g_r40_sincos_snapshot.begin + 2U;
        g_r40_sincos_snapshot.begin = version | 1U;
        __DMB();
        g_r40_sincos_snapshot.sequence = samples.sequence;
        g_r40_sincos_snapshot.milliseconds = time_hw_now_ms();
        g_r40_sincos_snapshot.pa5_counts = pa5;
        g_r40_sincos_snapshot.pa6_counts = pa6;
        g_r40_sincos_snapshot.end = version;
        __DMB();
        g_r40_sincos_snapshot.begin = version;
    }
#if defined(R40_KEIL_OPENLOOP) && R40_KEIL_OPENLOOP
    const uint8_t diagnostic_divider = 1U;
#else
    const uint8_t diagnostic_divider = 40U;
#endif
    if (++s_sincos_divider == diagnostic_divider)
    {
        s_sincos_divider = 0U;
    }
#endif
    dma_flag_clear(first ? DMA1_HDT2_FLAG : DMA1_FDT2_FLAG);
    s_aux = samples;
    s_aux_ms = time_hw_now_ms();
    s_aux_seen = true;
}

void DMA1_Channel3_IRQHandler(void)
{
    if (!R40_TEMPERATURE_SENSING_ENABLED)
    {
        return;
    }
    bool first = dma_flag_get(DMA1_HDT3_FLAG) != RESET;
    bool second = dma_flag_get(DMA1_FDT3_FLAG) != RESET;
    if (!s_started || s_fault || dma_flag_get(DMA1_DTERR3_FLAG) || (first && second))
    {
        dma_flag_clear(DMA1_GL3_FLAG);
        sampling_fault(9U);
        return;
    }
    if (!first && !second)
    {
        return;
    }
    uint32_t offset = first ? 0U : 2U;
    R40TemperatureSamples samples = {
        s_temperature_dma[offset], s_temperature_dma[offset + 1U], s_temperature.sequence + 1U};
    if (samples.mcu_temperature_counts > R40_ADC_MAX_COUNTS ||
        samples.reference_counts > R40_ADC_MAX_COUNTS)
    {
        sampling_fault(10U);
        return;
    }
    dma_flag_clear(first ? DMA1_HDT3_FLAG : DMA1_FDT3_FLAG);
    s_temperature = samples;
    s_temperature_ms = time_hw_now_ms();
    s_temperature_seen = true;
}
