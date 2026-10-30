/**
 * @file pwm.c
 * @brief AT32三相PWM初始化、寄存器读写和输出启停。
 */
#include "../../bsp_internal.h"
#include "bsp_pwm.h"
#include "mcu_resources.h"
#include "at32f45x.h"

void McuPwm_Init(void)
{
    crm_periph_clock_enable(MCU_PWM_HIGH_CLOCK, TRUE);
    crm_periph_clock_enable(MCU_PWM_LOW_CLOCK, TRUE);
    crm_periph_clock_enable(MCU_PWM_CLOCK, TRUE);
    if (BSP_ENCODER_GPIO_CLOCK != MCU_PWM_HIGH_CLOCK && BSP_ENCODER_GPIO_CLOCK != MCU_PWM_LOW_CLOCK)
    {
        crm_periph_clock_enable(BSP_ENCODER_GPIO_CLOCK, TRUE);
    }
    tmr_reset(MCU_PWM_TIMER);
    gpio_bits_reset(MCU_PWM_HIGH_PORT,
                    MCU_PWM_A_HIGH_PIN | MCU_PWM_B_HIGH_PIN | MCU_PWM_C_HIGH_PIN);
    gpio_bits_reset(MCU_PWM_LOW_PORT, MCU_PWM_A_LOW_PIN | MCU_PWM_B_LOW_PIN | MCU_PWM_C_LOW_PIN);
    gpio_bits_set(BSP_ENCODER_GPIO_PORT, 1U << BSP_ENCODER_CS_PIN);
    gpio_init_type pins;
    gpio_default_para_init(&pins);
    pins.gpio_mode = GPIO_MODE_OUTPUT;
    pins.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    pins.gpio_pull = GPIO_PULL_DOWN;
    pins.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
    pins.gpio_pins = MCU_PWM_A_HIGH_PIN | MCU_PWM_B_HIGH_PIN | MCU_PWM_C_HIGH_PIN;
    gpio_init(MCU_PWM_HIGH_PORT, &pins);
    pins.gpio_pins = MCU_PWM_A_LOW_PIN | MCU_PWM_B_LOW_PIN | MCU_PWM_C_LOW_PIN;
    gpio_init(MCU_PWM_LOW_PORT, &pins);
    pins.gpio_pins = 1U << BSP_ENCODER_CS_PIN;
    pins.gpio_pull = GPIO_PULL_UP;
    gpio_init(BSP_ENCODER_GPIO_PORT, &pins);
    const uint32_t period_counts = AT32_TIMER_HZ / (2U * FOC_FREQ);
    tmr_base_init(MCU_PWM_TIMER, period_counts, 0U);
    tmr_cnt_dir_set(MCU_PWM_TIMER, TMR_COUNT_TWO_WAY_2);
    tmr_clock_source_div_set(MCU_PWM_TIMER, TMR_CLOCK_DIV1);
    tmr_period_buffer_enable(MCU_PWM_TIMER, TRUE);
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
    const tmr_channel_select_type phase_channels[] = {
        MCU_PWM_PHASE_A_CHANNEL, MCU_PWM_PHASE_B_CHANNEL, MCU_PWM_PHASE_C_CHANNEL};
    for (uint32_t phase = 0U; phase < 3U; ++phase)
    {
        tmr_channel_select_type channel = phase_channels[phase];
        tmr_output_channel_config(MCU_PWM_TIMER, channel, &output);
        tmr_channel_value_set(MCU_PWM_TIMER, channel, period_counts / 2U);
        tmr_output_channel_buffer_enable(MCU_PWM_TIMER, channel, TRUE);
    }
    tmr_brkdt_config_type timing;
    tmr_brkdt_default_para_init(&timing);
    timing.auto_output_enable = FALSE;
    timing.fcsoen_state = TRUE;
    timing.fcsodis_state = TRUE;
    timing.brk_enable = FALSE;
    timing.deadtime = (AT32_TIMER_HZ / 1000000U) * BSP_PWM_DEADTIME_NS / 1000U;
    tmr_brkdt_config(MCU_PWM_TIMER, &timing);
    tmr_output_enable(MCU_PWM_TIMER, FALSE);
    gpio_pin_mux_config(MCU_PWM_HIGH_PORT, MCU_PWM_A_HIGH_SOURCE, MCU_PWM_GPIO_AF);
    gpio_pin_mux_config(MCU_PWM_HIGH_PORT, MCU_PWM_B_HIGH_SOURCE, MCU_PWM_GPIO_AF);
    gpio_pin_mux_config(MCU_PWM_HIGH_PORT, MCU_PWM_C_HIGH_SOURCE, MCU_PWM_GPIO_AF);
    gpio_pin_mux_config(MCU_PWM_LOW_PORT, MCU_PWM_A_LOW_SOURCE, MCU_PWM_GPIO_AF);
    gpio_pin_mux_config(MCU_PWM_LOW_PORT, MCU_PWM_B_LOW_SOURCE, MCU_PWM_GPIO_AF);
    gpio_pin_mux_config(MCU_PWM_LOW_PORT, MCU_PWM_C_LOW_SOURCE, MCU_PWM_GPIO_AF);
    pins.gpio_mode = GPIO_MODE_MUX;
    pins.gpio_pull = GPIO_PULL_DOWN;
    pins.gpio_pins = MCU_PWM_A_HIGH_PIN | MCU_PWM_B_HIGH_PIN | MCU_PWM_C_HIGH_PIN;
    gpio_init(MCU_PWM_HIGH_PORT, &pins);
    pins.gpio_pins = MCU_PWM_A_LOW_PIN | MCU_PWM_B_LOW_PIN | MCU_PWM_C_LOW_PIN;
    gpio_init(MCU_PWM_LOW_PORT, &pins);
}

void BspPwm_SetDuty(float phase_a, float phase_b, float phase_c)
{
    float period_counts = (float)MCU_PWM_TIMER->pr;
    MCU_PWM_PHASE_A_COMPARE = (uint16_t)(phase_a * period_counts);
    MCU_PWM_PHASE_B_COMPARE = (uint16_t)(phase_b * period_counts);
    MCU_PWM_PHASE_C_COMPARE = (uint16_t)(phase_c * period_counts);
}

void BspPwm_SetPhaseDuty(BspPwmPhase phase, float duty)
{
    uint16_t counts = (uint16_t)(duty * (float)MCU_PWM_TIMER->pr);
    switch (phase)
    {
    case BSP_PWM_PHASE_A:
        MCU_PWM_PHASE_A_COMPARE = counts;
        break;
    case BSP_PWM_PHASE_B:
        MCU_PWM_PHASE_B_COMPARE = counts;
        break;
    case BSP_PWM_PHASE_C:
        MCU_PWM_PHASE_C_COMPARE = counts;
        break;
    }
}

void BspPwm_Enable(void)
{
    /* 六路相通道及互补输出，最后开放主输出。 */
    MCU_PWM_TIMER->cctrl |= MCU_PWM_PHASE_CHANNEL_MASK;
    MCU_PWM_TIMER->brk |= MCU_PWM_MASTER_ENABLE_MASK;
}

void BspPwm_Disable(void)
{
    MCU_PWM_TIMER->brk &= ~MCU_PWM_MASTER_ENABLE_MASK;
    MCU_PWM_TIMER->cctrl &= ~MCU_PWM_PHASE_CHANNEL_MASK;
}

bool BspPwm_ArePhasesDisabled(void)
{
    return (MCU_PWM_TIMER->cctrl & MCU_PWM_PHASE_CHANNEL_MASK) == 0U;
}
