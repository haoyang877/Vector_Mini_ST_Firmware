#include "r40_platform.h"

#include "at32f45x.h"
#include "control_config.h"
#include "power_stage_hw.h"
#include "r40_clock.h"

/* PWM死区本阶段保守取200ns；最终值须在驱动器与MOS波形验收后确定。 */
#define R40_DEADTIME_NS 200U
/* 中央对齐完整周期为2*PR，单向监督计数周期才是PR+1（手册15.4.3.2）。 */
#define R40_PWM_TOP (R40_TIMER_HZ / (2U * FOC_FREQ))
#define R40_SUPERVISOR_TOP (1000000U / SUPERVISOR_FREQ - 1U)

#if (R40_TIMER_HZ % (2U * FOC_FREQ)) != 0U || (1000000U % SUPERVISOR_FREQ) != 0U
#error "R40 timer clock must divide the frozen control frequencies exactly"
#endif

static bool s_initialized;
static bool s_timing_started;

static void gate_pins_low(void)
{
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    gpio_bits_reset(GPIOA, GPIO_PINS_9 | GPIO_PINS_10 | GPIO_PINS_11);
    gpio_bits_reset(GPIOB, GPIO_PINS_0 | GPIO_PINS_1 | GPIO_PINS_4);
    gpio_bits_set(GPIOB, GPIO_PINS_12);
    gpio_init_type pins;
    gpio_default_para_init(&pins);
    pins.gpio_mode = GPIO_MODE_OUTPUT;
    pins.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    pins.gpio_pull = GPIO_PULL_DOWN;
    pins.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
    pins.gpio_pins = GPIO_PINS_9 | GPIO_PINS_10 | GPIO_PINS_11;
    gpio_init(GPIOA, &pins);
    pins.gpio_pins = GPIO_PINS_0 | GPIO_PINS_1 | GPIO_PINS_4;
    gpio_init(GPIOB, &pins);
    pins.gpio_pins = GPIO_PINS_12;
    pins.gpio_pull = GPIO_PULL_UP;
    gpio_init(GPIOB, &pins);
}

static void timer_init_disabled(void)
{
    crm_periph_clock_enable(CRM_TMR1_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_TMR6_PERIPH_CLOCK, TRUE);
    tmr_reset(TMR1);
    tmr_reset(TMR6);
    tmr_base_init(TMR1, R40_PWM_TOP, 0U);
    tmr_cnt_dir_set(TMR1, TMR_COUNT_TWO_WAY_2);
    tmr_clock_source_div_set(TMR1, TMR_CLOCK_DIV1);
    tmr_period_buffer_enable(TMR1, TRUE);
    tmr_output_config_type output;
    tmr_output_default_para_init(&output);
    /* 模式A使CCR增大对应上桥导通时间增大；不沿用SDK的反向模式B。 */
    output.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
    output.oc_output_state = FALSE;
    output.occ_output_state = FALSE;
    output.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
    output.occ_polarity = TMR_OUTPUT_ACTIVE_HIGH;
    output.oc_idle_state = FALSE;
    output.occ_idle_state = FALSE;
    for (uint32_t phase = 0U; phase < 3U; ++phase)
    {
        tmr_channel_select_type channel =
            (tmr_channel_select_type)(TMR_SELECT_CHANNEL_2 + 2U * phase);
        tmr_output_channel_config(TMR1, channel, &output);
        tmr_channel_value_set(TMR1, channel, R40_PWM_TOP / 2U);
        tmr_output_channel_buffer_enable(TMR1, channel, TRUE);
    }
    tmr_brkdt_config_type protection;
    tmr_brkdt_default_para_init(&protection);
    protection.auto_output_enable = FALSE;
    protection.fcsoen_state = TRUE;
    protection.fcsodis_state = TRUE;
    protection.brk_enable = FALSE;
    protection.deadtime = (R40_TIMER_HZ / 1000000U) * R40_DEADTIME_NS / 1000U;
    tmr_brkdt_config(TMR1, &protection);
    power_stage_hw_stop();
    tmr_base_init(TMR6, R40_SUPERVISOR_TOP, R40_TIMER_HZ / 1000000U - 1U);
    tmr_flag_clear(TMR6, TMR_OVF_FLAG);
}

static void gate_pins_timer_mux(void)
{
    const uint16_t upper_pins = GPIO_PINS_9 | GPIO_PINS_10 | GPIO_PINS_11;
    const uint16_t lower_pins = GPIO_PINS_0 | GPIO_PINS_1 | GPIO_PINS_4;
    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE9, GPIO_MUX_1);
    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE10, GPIO_MUX_1);
    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE11, GPIO_MUX_1);
    gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE0, GPIO_MUX_1);
    gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE1, GPIO_MUX_1);
    gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE4, GPIO_MUX_1);
    gpio_init_type pins;
    gpio_default_para_init(&pins);
    pins.gpio_mode = GPIO_MODE_MUX;
    pins.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    pins.gpio_pull = GPIO_PULL_DOWN;
    pins.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
    pins.gpio_pins = upper_pins;
    gpio_init(GPIOA, &pins);
    pins.gpio_pins = lower_pins;
    gpio_init(GPIOB, &pins);
}

bool R40_PlatformInit(void)
{
    if (s_initialized)
    {
        return false;
    }
    gate_pins_low();
    if (!R40_ClockInit())
    {
        return false;
    }
    timer_init_disabled();
    if (!power_stage_hw_phases_disabled())
    {
        return false;
    }
    gate_pins_timer_mux();
    s_initialized = true;
    return true;
}

bool R40_TimingStart(void)
{
    if (!s_initialized || s_timing_started || !power_stage_hw_phases_disabled())
    {
        return false;
    }
    if (SysTick_Config(R40_CORE_HZ / 1000U) != 0U)
    {
        return false;
    }
    NVIC_SetPriority(SysTick_IRQn, 3U);
    tmr_flag_clear(TMR6, TMR_OVF_FLAG);
    tmr_interrupt_enable(TMR6, TMR_OVF_INT, TRUE);
    NVIC_SetPriority(TMR6_DAC_GLOBAL_IRQn, 2U);
    NVIC_EnableIRQ(TMR6_DAC_GLOBAL_IRQn);
    tmr_counter_enable(TMR1, TRUE);
    tmr_counter_enable(TMR6, TRUE);
    s_timing_started = true;
    return power_stage_hw_phases_disabled();
}
