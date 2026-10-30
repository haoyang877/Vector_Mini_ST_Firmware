/**
 * @file pwm.c
 * @brief AT32三相PWM初始化、寄存器读写和输出启停。
 */
#include "../../bsp_internal.h"
#include "bsp_pwm.h"
#include "board_config.h"
#include "at32f45x.h"

#define PWM_PHASE_CHANNEL_MASK 0x5550U
#define PWM_MASTER_ENABLE_MASK 0x8000U

void McuPwm_Init(void)
{
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_TMR1_PERIPH_CLOCK, TRUE);
    tmr_reset(TMR1);
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
    const uint32_t period_counts = AT32_TIMER_HZ / (2U * FOC_FREQ);
    tmr_base_init(TMR1, period_counts, 0U);
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
        tmr_channel_value_set(TMR1, channel, period_counts / 2U);
        tmr_output_channel_buffer_enable(TMR1, channel, TRUE);
    }
    tmr_brkdt_config_type timing;
    tmr_brkdt_default_para_init(&timing);
    timing.auto_output_enable = FALSE;
    timing.fcsoen_state = TRUE;
    timing.fcsodis_state = TRUE;
    timing.brk_enable = FALSE;
    timing.deadtime = (AT32_TIMER_HZ / 1000000U) * BSP_PWM_DEADTIME_NS / 1000U;
    tmr_brkdt_config(TMR1, &timing);
    tmr_output_enable(TMR1, FALSE);
    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE9, GPIO_MUX_1);
    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE10, GPIO_MUX_1);
    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE11, GPIO_MUX_1);
    gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE0, GPIO_MUX_1);
    gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE1, GPIO_MUX_1);
    gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE4, GPIO_MUX_1);
    pins.gpio_mode = GPIO_MODE_MUX;
    pins.gpio_pull = GPIO_PULL_DOWN;
    pins.gpio_pins = GPIO_PINS_9 | GPIO_PINS_10 | GPIO_PINS_11;
    gpio_init(GPIOA, &pins);
    pins.gpio_pins = GPIO_PINS_0 | GPIO_PINS_1 | GPIO_PINS_4;
    gpio_init(GPIOB, &pins);
}

void BspPwm_SetDuty(float phase_a, float phase_b, float phase_c)
{
    float period_counts = (float)TMR1->pr;
    TMR1->c2dt = (uint16_t)(phase_a * period_counts);
    TMR1->c3dt = (uint16_t)(phase_b * period_counts);
    TMR1->c4dt = (uint16_t)(phase_c * period_counts);
}

void BspPwm_SetPhaseDuty(BspPwmPhase phase, float duty)
{
    uint16_t counts = (uint16_t)(duty * (float)TMR1->pr);
    switch (phase)
    {
    case BSP_PWM_PHASE_A:
        TMR1->c2dt = counts;
        break;
    case BSP_PWM_PHASE_B:
        TMR1->c3dt = counts;
        break;
    case BSP_PWM_PHASE_C:
        TMR1->c4dt = counts;
        break;
    }
}

void BspPwm_Enable(void)
{
    /* 六路相通道及互补输出，最后开放主输出。 */
    TMR1->cctrl |= PWM_PHASE_CHANNEL_MASK;
    TMR1->brk |= PWM_MASTER_ENABLE_MASK;
}

void BspPwm_Disable(void)
{
    TMR1->brk &= ~PWM_MASTER_ENABLE_MASK;
    TMR1->cctrl &= ~PWM_PHASE_CHANNEL_MASK;
}

bool BspPwm_ArePhasesDisabled(void)
{
    return (TMR1->cctrl & PWM_PHASE_CHANNEL_MASK) == 0U;
}
