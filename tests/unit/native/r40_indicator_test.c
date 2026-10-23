/* 直接执行新板指示器端口，检查上电电平、闪烁优先级、诊断和禁止其他GPIO。 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "at32f45x.h"
#include "indicator_hw.h"
#include "r40_indicator.h"
gpio_type fake_led_gpio;
static unsigned s_key, s_writes;
static bool s_off = true;
uint32_t critical_hw_enter(void)
{
    unsigned old = s_key;
    s_key = 1U;
    return old;
}
void critical_hw_exit(uint32_t key)
{
    s_key = key;
}
bool power_stage_hw_phases_disabled(void)
{
    return s_off;
}
void crm_periph_clock_enable(unsigned clock, unsigned enabled)
{
    assert(clock == CRM_GPIOC_PERIPH_CLOCK && enabled);
}
void gpio_default_para_init(gpio_init_type *pins)
{
    memset(pins, 0, sizeof(*pins));
}
void gpio_init(gpio_type *gpio, gpio_init_type *pins)
{
    assert(gpio == GPIOC && pins->gpio_pins == 0x6000U && pins->gpio_mode == GPIO_MODE_OUTPUT);
    assert((gpio->odt & 0x6000U) == 0x6000U);
}
void gpio_bits_set(gpio_type *gpio, uint16_t pins)
{
    assert(gpio == GPIOC && !(pins & ~0x6000U));
    gpio->odt |= pins;
    ++s_writes;
}
void gpio_bits_reset(gpio_type *gpio, uint16_t pins)
{
    assert(gpio == GPIOC && !(pins & ~0x6000U));
    gpio->odt &= ~pins;
    ++s_writes;
}
int main(void)
{
    R40_IndicatorInit();
    assert((fake_led_gpio.odt & 0x6000U) == 0x6000U);
    for (unsigned p = 1; p <= 4; ++p)
    {
        assert(R40_IndicatorDiagnostic(p));
        R40_IndicatorTick();
        assert((fake_led_gpio.odt & 0x6000U) ==
               (0x6000U ^ ((p & 1U) ? 0x2000U : 0U) ^ ((p & 2U) ? 0x4000U : 0U)));
    }
    assert(!R40_IndicatorDiagnostic(5));
    s_off = false;
    assert(!R40_IndicatorDiagnostic(1));
    s_off = true;
    assert(R40_IndicatorDiagnostic(0));
    indicator_hw_set_led(true, 2);
    indicator_hw_led_task();
    R40_IndicatorTick();
    assert((fake_led_gpio.odt & 0x6000U) == 0x6000U);
    indicator_hw_led_task();
    R40_IndicatorTick();
    assert((fake_led_gpio.odt & 0x6000U) == 0x4000U);
    indicator_hw_set_color(INDICATOR_HW_COLOR_BLUE);
    R40_IndicatorTick();
    assert((fake_led_gpio.odt & 0x6000U) == 0x4000U);
    indicator_hw_set_led(false, 1);
    indicator_hw_led_task();
    indicator_hw_led_task();
    R40_IndicatorTick();
    assert((fake_led_gpio.odt & 0x6000U) == 0x2000U);
    indicator_hw_set_led(false, 0);
    indicator_hw_led_task();
    for (unsigned color = 0; color <= 7; ++color)
    {
        unsigned on = 0, off = 0;
        for (unsigned n = 0; n < 2000; ++n)
        {
            if (n % 50U == 0)
                indicator_hw_set_color((IndicatorHwColor)color);
            R40_IndicatorTick();
            if ((fake_led_gpio.odt & 0x6000U) == 0x6000U)
                ++off;
            else
                ++on;
        }
        if (color == 0)
            assert(on == 0);
        else
            assert(on && off);
    }
    assert(!s_key && s_writes);
    puts("PASS R40 LED: active-low startup, code priority, all colors, breathing, off-only "
         "diagnostics, isolated GPIO");
    return 0;
}
