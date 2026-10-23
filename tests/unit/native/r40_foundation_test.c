/* 验证真实R40启动、关相及时基端口；替身只提供寄存器和SDK调用效果。 */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "at32f45x.h"
#include "critical_hw.h"
#include "power_stage_hw.h"
#include "motor_hw.h"
#include "r40_clock.h"
#include "r40_platform.h"
#include "time_hw.h"

tmr_type fake_tmr1, fake_tmr6;
gpio_type fake_gpioa = {0xFFFFU}, fake_gpiob = {0xFFFFU};
crm_type fake_crm;
unsigned fake_failure, fake_primask, fake_pll_n, fake_pll_m, fake_pll_p;
unsigned fake_timer_channels, fake_mux_count, fake_core_clock;
uint32_t system_core_clock;
static unsigned s_main_outputs = 0x1110U, s_complement_outputs = 0x4440U;

void SysTick_Handler(void);
void crm_periph_clock_enable(unsigned id, unsigned enabled)
{
    (void)id;
    assert(enabled);
}
void flash_psr_set(unsigned cycles)
{
    assert(cycles == 5U);
}
void pwc_ldo_output_voltage_set(unsigned level)
{
    assert(level == 3U);
}
void crm_clock_source_enable(unsigned source, unsigned enabled)
{
    (void)source;
    assert(enabled);
}
unsigned crm_flag_get(unsigned flag)
{
    return fake_failure == flag ? RESET : SET;
}
void crm_pll_config(unsigned source, unsigned n, unsigned m, unsigned p)
{
    assert(source == CRM_PLL_SOURCE_HEXT);
    fake_pll_n = n;
    fake_pll_m = m;
    fake_pll_p = p;
}
void crm_ahb_div_set(unsigned value)
{
    assert(value == 0U);
}
void crm_apb1_div_set(unsigned value)
{
    assert(value == 0U);
}
void crm_apb2_div_set(unsigned value)
{
    assert(value == 0U);
}
void crm_apb3_div_set(unsigned value)
{
    fake_crm.misc2_bit.apb3div = value;
}
void crm_auto_step_mode_enable(unsigned enabled)
{
    (void)enabled;
}
void crm_sysclk_switch(unsigned source)
{
    assert(source == CRM_SCLK_PLL);
}
unsigned crm_sysclk_switch_status_get(void)
{
    return fake_failure == 3U ? 0U : CRM_SCLK_PLL;
}
void system_core_clock_update(void)
{
    assert(fake_pll_p == CRM_PLL_FP_4);
    system_core_clock = R40_HEXT_HZ / fake_pll_m * fake_pll_n / 4U;
}
void crm_clocks_freq_get(crm_clocks_freq_type *clocks)
{
    clocks->ahb_freq = system_core_clock;
    clocks->apb1_freq = system_core_clock - (fake_failure == 4U);
    clocks->apb2_freq = system_core_clock;
}
void gpio_bits_reset(gpio_type *gpio, unsigned pins)
{
    gpio->output &= ~pins;
}
void gpio_bits_set(gpio_type *gpio, unsigned pins)
{
    gpio->output |= pins;
}
void gpio_default_para_init(gpio_init_type *pins)
{
    memset(pins, 0, sizeof(*pins));
}
void gpio_init(gpio_type *gpio, gpio_init_type *pins)
{
    if (pins->gpio_mode == GPIO_MODE_MUX)
    {
        assert(power_stage_hw_phases_disabled());
        assert(fake_tmr1.brk == 0U);
        assert((gpio->output & pins->gpio_pins) == 0U);
        ++fake_mux_count;
    }
}
void gpio_pin_mux_config(gpio_type *gpio, unsigned pin, unsigned mux)
{
    (void)gpio;
    (void)pin;
    assert(mux == 1U);
}
void tmr_reset(tmr_type *timer)
{
    memset(timer, 0, sizeof(*timer));
}
void tmr_base_init(tmr_type *timer, unsigned period, unsigned divider)
{
    timer->pr = period;
    timer->div = divider;
}
void tmr_cnt_dir_set(tmr_type *timer, unsigned direction)
{
    (void)timer;
    assert(direction == 2U);
}
void tmr_clock_source_div_set(tmr_type *timer, unsigned divider)
{
    (void)timer;
    assert(divider == 0U);
}
void tmr_period_buffer_enable(tmr_type *timer, unsigned enabled)
{
    (void)timer;
    assert(enabled);
}
void tmr_output_default_para_init(tmr_output_config_type *output)
{
    memset(output, 0, sizeof(*output));
}
void tmr_output_channel_config(tmr_type *timer, unsigned channel, tmr_output_config_type *output)
{
    assert(timer == TMR1 && (channel == 2U || channel == 4U || channel == 6U));
    assert(output->oc_mode == 6U && !output->oc_output_state && !output->occ_output_state);
    assert(!output->oc_idle_state && !output->occ_idle_state);
    fake_timer_channels |= 1U << channel;
}
void tmr_channel_value_set(tmr_type *timer, unsigned channel, unsigned value)
{
    assert(timer == TMR1);
    if (channel == 2U)
    {
        timer->c2dt = value;
    }
    else if (channel == 4U)
    {
        timer->c3dt = value;
    }
    else
    {
        assert(channel == 6U);
        timer->c4dt = value;
    }
}
void tmr_output_channel_buffer_enable(tmr_type *timer, unsigned channel, unsigned enabled)
{
    (void)timer;
    (void)channel;
    assert(enabled);
}
void tmr_brkdt_default_para_init(tmr_brkdt_config_type *config)
{
    memset(config, 0, sizeof(*config));
}
void tmr_brkdt_config(tmr_type *timer, tmr_brkdt_config_type *config)
{
    (void)timer;
    assert(!config->auto_output_enable && config->deadtime == 36U);
}
void tmr_output_enable(tmr_type *timer, unsigned enabled)
{
    assert(fake_primask == 1U);
    if (enabled)
    {
        assert((timer->cctrl & 0x5550U) == 0x5550U);
    }
    timer->brk = enabled;
}
void tmr_flag_clear(tmr_type *timer, unsigned flag)
{
    (void)timer;
    assert(flag == 1U);
}
void tmr_interrupt_enable(tmr_type *timer, unsigned mask, unsigned enabled)
{
    assert(timer == TMR6 && mask == 1U && enabled);
}
void tmr_counter_enable(tmr_type *timer, unsigned enabled)
{
    assert(power_stage_hw_phases_disabled());
    timer->running = enabled;
}
unsigned SysTick_Config(unsigned reload)
{
    assert(reload == 180000U);
    return fake_failure == 5U;
}
void NVIC_SetPriority(unsigned irq, unsigned priority)
{
    (void)irq;
    assert(priority >= 2U);
}
void NVIC_EnableIRQ(unsigned irq)
{
    assert(irq == TMR6_DAC_GLOBAL_IRQn);
}
unsigned __get_PRIMASK(void)
{
    return fake_primask;
}
void __disable_irq(void)
{
    fake_primask = 1U;
}
void __set_PRIMASK(unsigned value)
{
    fake_primask = value;
}

int main(int argc, char **argv)
{
    fake_failure = argc > 1 ? (unsigned)strtoul(argv[1], NULL, 10) : 0U;
    assert(!R40_TimingStart());
    bool initialized = R40_PlatformInit();
    assert((fake_gpioa.output & 0xE00U) == 0U);
    assert((fake_gpiob.output & 0x13U) == 0U);
    assert((fake_gpiob.output & 0x1000U) != 0U);
    if (fake_failure >= 1U && fake_failure <= 4U)
    {
        assert(!initialized && fake_mux_count == 0U);
        assert(!fake_tmr1.running && !fake_tmr6.running);
        puts("PASS clock failure leaves gates low and timers stopped");
        return 0;
    }
    assert(initialized && fake_mux_count == 2U && fake_timer_channels == 0x54U);
    assert(!R40_PlatformInit());
    assert(fake_tmr1.pr == 4500U && fake_tmr1.div == 0U);
    assert(fake_tmr6.pr == 499U && fake_tmr6.div == 179U);
    assert(fake_tmr1.c2dt == 2250U && fake_tmr1.c3dt == 2250U && fake_tmr1.c4dt == 2250U);
    assert(R40_CAN_HZ / (R40_CAN_NOMINAL_SEG1 + R40_CAN_NOMINAL_SEG2) == 1000000U);
    assert(R40_CAN_HZ / (R40_CAN_DATA_SEG1 + R40_CAN_DATA_SEG2) == 5000000U);
    bool started = R40_TimingStart();
    if (fake_failure == 5U)
    {
        assert(!started && !fake_tmr1.running && !fake_tmr6.running);
        puts("PASS SysTick failure leaves timers stopped");
        return 0;
    }
    assert(started && !R40_TimingStart());
    assert(power_stage_hw_phases_disabled() && fake_tmr1.brk == 0U);
    fake_tmr1.cctrl = 0xAU;
    power_stage_hw_start();
    assert(fake_tmr1.brk == 1U && !power_stage_hw_phases_disabled());
    power_stage_hw_stop();
    assert(fake_tmr1.cctrl == 0xAU && fake_tmr1.brk == 0U);
    assert(fake_tmr1.c2dt == 2250U && fake_tmr1.c3dt == 2250U && fake_tmr1.c4dt == 2250U);
    for (unsigned bit = 0U; bit < 16U; ++bit)
    {
        unsigned value = 1U << bit;
        if ((value & (s_main_outputs | s_complement_outputs)) != 0U)
        {
            fake_tmr1.cctrl = value;
            assert(!power_stage_hw_phases_disabled());
            power_stage_hw_stop();
            assert(power_stage_hw_phases_disabled());
        }
    }
    for (unsigned initial = 0U; initial <= 1U; ++initial)
    {
        fake_primask = initial;
        power_stage_hw_start();
        assert(fake_primask == initial);
        power_stage_hw_stop();
        assert(fake_primask == initial);
        unsigned outer = critical_hw_enter(), inner = critical_hw_enter();
        assert(fake_primask == 1U);
        critical_hw_exit(inner);
        assert(fake_primask == 1U);
        critical_hw_exit(outer);
        assert(fake_primask == initial);
    }
    assert(time_hw_now_ms() == 0U);
    for (unsigned i = 0U; i < 2001U; ++i)
    {
        SysTick_Handler();
    }
    assert(time_hw_now_ms() == 2001U);
    for (unsigned mask = 0U; mask < 2U; ++mask)
    {
        fake_primask = mask;
        for (unsigned i = 0U; i <= 4500U; ++i)
        {
            float a = (float)i / 4500.0f;
            motor_hw_pwm_set_duty(a, 0.25f, 0.75f);
            assert(fake_tmr1.c2dt == (uint16_t)(a * 4500.0f));
            assert(fake_tmr1.c3dt == 1125U && fake_tmr1.c4dt == 3375U);
            assert(fake_tmr1.brk == 0U && power_stage_hw_phases_disabled() && fake_primask == mask);
        }
        motor_hw_pwm_set_phase_duty(MOTOR_HW_PWM_PHASE_B, 0.5f);
        assert(fake_tmr1.c2dt == 4500U && fake_tmr1.c3dt == 2250U && fake_tmr1.c4dt == 3375U);
        motor_hw_pwm_set_phase_duty((MotorHwPwmPhase)99, 0.0f);
        assert(fake_tmr1.c3dt == 2250U);
        motor_hw_pwm_force_high_sides();
        assert(fake_tmr1.c2dt == 4500U && fake_tmr1.c3dt == 4500U && fake_tmr1.c4dt == 4500U);
        assert(fake_tmr1.brk == 0U && power_stage_hw_phases_disabled() && fake_tmr1.running);
    }
    puts("PASS startup ordering, real ports, timing, six outputs and nested IRQ state");
    return 0;
}
