/* 验证真实R40启动、关相及时基端口；替身只提供寄存器和SDK调用效果。 */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "at32f45x.h"
#include "bsp_system.h"
#include "bsp_pwm.h"
#include "bsp_config.h"
#include "bsp_internal.h"
FoundationDwt foundation_dwt;
FoundationCoreDebug foundation_core_debug;
void NVIC_SystemReset(void)
{
    abort();
}
void __WFI(void)
{
    abort();
}

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
        assert(BspPwm_ArePhasesDisabled());
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
    if (enabled)
    {
        assert((timer->cctrl & 0x5550U) == 0x5550U);
    }
    timer->brk = enabled ? 0x8000U : 0U;
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
    assert(BspPwm_ArePhasesDisabled());
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

/* 固定边界向量验证合并提交：严格大于256计数，失败零写入，关相/停表沿用预载语义。 */
static void test_pwm_commit_boundaries(void)
{
    const struct
    {
        uint32_t count;
        bool up, down;
    } cases[] = {{0U, true, false},
                 {256U, true, false},
                 {257U, true, true},
                 {4243U, true, true},
                 {4244U, false, true},
                 {4500U, false, true},
                 {4501U, false, false},
                 {UINT32_MAX, false, false}};
    const uint32_t channels[] = {0U, 0x10U, 0x5550U};
    const uint32_t controls[] = {0U, 1U, 0x11U, 0x13U};
    tmr_type saved = fake_tmr1;
    for (unsigned channel = 0U; channel < 3U; ++channel)
    {
        for (unsigned master = 0U; master < 2U; ++master)
        {
            for (unsigned mode = 0U; mode < 4U; ++mode)
            {
                for (unsigned i = 0U; i < sizeof(cases) / sizeof(cases[0]); ++i)
                {
                    fake_tmr1.cctrl = channels[channel] | 1U;
                    fake_tmr1.brk = master ? 0x8000U : 0U;
                    fake_tmr1.ctrl1 = controls[mode];
                    fake_tmr1.cval = cases[i].count;
                    fake_tmr1.pr = 4500U;
                    fake_tmr1.c2dt = 111U;
                    fake_tmr1.c3dt = 222U;
                    fake_tmr1.c4dt = 333U;
                    bool expected = (channel == 0U && master == 0U) || mode == 0U ||
                                    (mode == 1U ? cases[i].up : cases[i].down);
                    assert(McuPwm_CommitThree(12U, 34U, 56U) == expected);
                    assert(fake_tmr1.c2dt == (expected ? 12U : 111U));
                    assert(fake_tmr1.c3dt == (expected ? 34U : 222U));
                    assert(fake_tmr1.c4dt == (expected ? 56U : 333U));
                    assert(fake_tmr1.ctrl1 == controls[mode]);
                    assert(fake_tmr1.cctrl == (channels[channel] | 1U));
                    assert(fake_tmr1.brk == (master ? 0x8000U : 0U));
                    fake_tmr1.c2dt = 111U;
                    fake_tmr1.c3dt = 222U;
                    fake_tmr1.c4dt = 333U;
                    assert(McuPwm_CommitPhase(BSP_PWM_PHASE_B, 78U) == expected);
                    assert(fake_tmr1.c2dt == 111U && fake_tmr1.c4dt == 333U);
                    assert(fake_tmr1.c3dt == (expected ? 78U : 222U));
                    assert(fake_tmr1.ctrl1 == controls[mode]);
                }
            }
        }
    }
    /* 扫描实际16位周期/计数，包含保护预算以下的周期和越过顶部的计数。
     * 预期值独立按原count范围与上/下计数公式计算，覆盖原来无符号回绕拒绝语义。 */
    const uint32_t periods[] = {0U, 1U, 255U, 256U, 257U, 4250U, 4500U, 65535U};
    uint32_t checked = 0U;
    for (unsigned p = 0U; p < sizeof(periods) / sizeof(periods[0]); ++p)
    {
        uint32_t period = periods[p];
        for (uint32_t count = 0U; count <= period + 1U; ++count)
        {
            for (unsigned channel = 0U; channel < 3U; ++channel)
            {
                for (unsigned master = 0U; master < 2U; ++master)
                {
                    for (unsigned mode = 0U; mode < 4U; ++mode)
                    {
                        uint32_t control = controls[mode];
                        uint32_t remaining = (control & 0x10U) != 0U ? count : period - count;
                        bool expected = (channel == 0U && master == 0U) || (control & 1U) == 0U ||
                                        (count <= period && remaining > 256U);
                        fake_tmr1.cctrl = channels[channel] | 1U;
                        fake_tmr1.brk = master ? 0x8000U : 0U;
                        fake_tmr1.ctrl1 = control;
                        fake_tmr1.cval = count;
                        fake_tmr1.pr = period;
                        fake_tmr1.c2dt = 111U;
                        fake_tmr1.c3dt = 222U;
                        fake_tmr1.c4dt = 333U;
                        assert(McuPwm_CommitThree(12U, 34U, 56U) == expected);
                        assert(fake_tmr1.c2dt == (expected ? 12U : 111U));
                        assert(fake_tmr1.c3dt == (expected ? 34U : 222U));
                        assert(fake_tmr1.c4dt == (expected ? 56U : 333U));
                        assert(fake_tmr1.ctrl1 == control);
                        assert(fake_tmr1.cctrl == (channels[channel] | 1U));
                        assert(fake_tmr1.brk == (master ? 0x8000U : 0U));
                        fake_tmr1.c2dt = 111U;
                        fake_tmr1.c3dt = 222U;
                        fake_tmr1.c4dt = 333U;
                        assert(McuPwm_CommitPhase(BSP_PWM_PHASE_B, 78U) == expected);
                        assert(fake_tmr1.c2dt == 111U && fake_tmr1.c4dt == 333U);
                        assert(fake_tmr1.c3dt == (expected ? 78U : 222U));
                        assert(fake_tmr1.ctrl1 == control);
                        ++checked;
                    }
                }
            }
        }
    }
    fake_tmr1 = saved;
    printf("PASS AT32 PWM: 192 boundary vectors and %u period/count/output cases, timeout zero "
           "writes\n",
           checked);
}

int main(int argc, char **argv)
{
    fake_failure = argc > 1 ? (unsigned)strtoul(argv[1], NULL, 10) : 0U;
    assert(!McuSystem_Start());
    bool initialized = McuSystem_Init();
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
    assert(!McuSystem_Init());
    assert(fake_tmr1.pr == 4500U && fake_tmr1.div == 0U);
    assert(fake_tmr6.pr == 499U && fake_tmr6.div == 179U);
    assert(fake_tmr1.c2dt == 2250U && fake_tmr1.c3dt == 2250U && fake_tmr1.c4dt == 2250U);
    assert(R40_CAN_HZ / (R40_CAN_NOMINAL_SEG1 + R40_CAN_NOMINAL_SEG2) == 1000000U);
    assert(R40_CAN_HZ / (R40_CAN_DATA_SEG1 + R40_CAN_DATA_SEG2) == 5000000U);
    bool started = McuSystem_Start();
    if (fake_failure == 5U)
    {
        assert(!started && !fake_tmr1.running && !fake_tmr6.running);
        puts("PASS SysTick failure leaves timers stopped");
        return 0;
    }
    assert(started && !McuSystem_Start());
    assert(BspPwm_ArePhasesDisabled() && fake_tmr1.brk == 0U);
    fake_tmr1.cctrl = 0xAU;
    BspPwm_Enable();
    assert(fake_tmr1.brk == 0x8000U && !BspPwm_ArePhasesDisabled());
    BspPwm_Disable();
    assert(fake_tmr1.cctrl == 0xAU && fake_tmr1.brk == 0U);
    assert(fake_tmr1.c2dt == 2250U && fake_tmr1.c3dt == 2250U && fake_tmr1.c4dt == 2250U);
    for (unsigned bit = 0U; bit < 16U; ++bit)
    {
        unsigned value = 1U << bit;
        if ((value & (s_main_outputs | s_complement_outputs)) != 0U)
        {
            fake_tmr1.cctrl = value;
            assert(!BspPwm_ArePhasesDisabled());
            BspPwm_Disable();
            assert(BspPwm_ArePhasesDisabled());
        }
    }
    for (unsigned initial = 0U; initial <= 1U; ++initial)
    {
        fake_primask = initial;
        BspPwm_Enable();
        assert(fake_primask == initial);
        BspPwm_Disable();
        assert(fake_primask == initial);
        unsigned outer = BspSystem_EnterCritical(), inner = BspSystem_EnterCritical();
        assert(fake_primask == 1U);
        BspSystem_ExitCritical(inner);
        assert(fake_primask == 1U);
        BspSystem_ExitCritical(outer);
        assert(fake_primask == initial);
    }
    for (unsigned initial = 0U; initial < 2U; ++initial)
    {
        fake_primask = initial;
        uint32_t outer = BspSystem_EnterControlCritical();
        uint32_t inner = BspSystem_EnterControlCritical();
        assert(outer == 0U && inner == 0x20U && fake_primask == initial);
        BspSystem_ExitControlCritical(inner);
        BspSystem_ExitControlCritical(outer);
        uint32_t restored = BspSystem_EnterControlCritical();
        assert(restored == 0U && fake_primask == initial);
        BspSystem_ExitControlCritical(restored);
        volatile uint32_t word = 7U;
        assert(!BspSystem_CompareExchange32(&word, 3U, 9U) && word == 7U);
        assert(BspSystem_CompareExchange32(&word, 7U, 9U) && word == 9U);
        assert(fake_primask == initial);
    }
    assert(BspSystem_TimeMs() == 0U);
    for (unsigned i = 0U; i < 2001U; ++i)
    {
        SysTick_Handler();
    }
    assert(BspSystem_TimeMs() == 2001U);
    uint32_t us_origin = BspSystem_TimeUs();
    uint64_t elapsed_cycles = 0U;
    for (unsigned n = 0U; n < 600U; n++)
    {
        uint32_t advance = n < 100U ? 179U : 4000000000U;
        foundation_dwt.CYCCNT += advance;
        elapsed_cycles += advance;
        fake_primask = n & 1U;
        assert(BspSystem_TimeUs() == us_origin + (uint32_t)(elapsed_cycles / 180U));
        assert(fake_primask == (n & 1U));
    }
    for (unsigned mask = 0U; mask < 2U; ++mask)
    {
        fake_primask = mask;
        for (unsigned i = 0U; i <= 4500U; ++i)
        {
            float a = (float)i / 4500.0f;
            BspPwm_SetDuty(a, 0.25f, 0.75f);
            assert(fake_tmr1.c2dt == (uint16_t)(a * 4500.0f));
            assert(fake_tmr1.c3dt == 1125U && fake_tmr1.c4dt == 3375U);
            assert(fake_tmr1.brk == 0U && BspPwm_ArePhasesDisabled() && fake_primask == mask);
        }
        BspPwm_SetPhaseDuty(BSP_PWM_PHASE_B, 0.5f);
        assert(fake_tmr1.c2dt == 4500U && fake_tmr1.c3dt == 2250U && fake_tmr1.c4dt == 3375U);
        BspPwm_SetPhaseDuty((BspPwmPhase)99, 0.0f);
        assert(fake_tmr1.c3dt == 2250U);
        assert(BspPwm_SetDuty(1.0f, 1.0f, 1.0f) == BSP_OK);
        assert(fake_tmr1.c2dt == 4500U && fake_tmr1.c3dt == 4500U && fake_tmr1.c4dt == 4500U);
        assert(fake_tmr1.brk == 0U && BspPwm_ArePhasesDisabled() && fake_tmr1.running);
    }
    test_pwm_commit_boundaries();
    puts("PASS startup ordering, real ports, timing, six outputs and nested IRQ state");
    return 0;
}
