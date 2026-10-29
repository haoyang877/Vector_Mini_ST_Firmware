/* 直接执行新板指示器端口，检查上电电平、闪烁优先级、诊断和禁止其他GPIO。 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "at32f45x.h"
#include "bsp_led.h"
#include "bsp_internal.h"
#include "control_config.h"
gpio_type fake_led_gpio;
static unsigned s_key, s_writes;
uint32_t BspSystem_EnterCritical(void)
{
    unsigned old = s_key;
    s_key = 1U;
    return old;
}
void BspSystem_ExitCritical(uint32_t key)
{
    s_key = key;
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
static void display_tick(void)
{
    for (unsigned i = 0; i < SUPERVISOR_FREQ / 5U; i++)
    {
        Led_SupervisorTick();
    }
}
static void tick_1ms(void)
{
    Led_SupervisorTick();
    Led_SupervisorTick();
    McuLed_Tick1ms();
}
int main(void)
{
    Led_Init();
    assert((fake_led_gpio.odt & 0x6000U) == 0x6000U);
    BspLed_SetCode(BSP_LED_FAULT, 2);
    display_tick();
    tick_1ms();
    assert((fake_led_gpio.odt & 0x6000U) == 0x6000U);
    display_tick();
    tick_1ms();
    assert((fake_led_gpio.odt & 0x6000U) == 0x4000U);
    BspLed_SetColor(BSP_LED_BLUE);
    tick_1ms();
    assert((fake_led_gpio.odt & 0x6000U) == 0x4000U);
    BspLed_SetCode(BSP_LED_MODE, 1);
    display_tick();
    display_tick();
    tick_1ms();
    assert((fake_led_gpio.odt & 0x6000U) == 0x2000U);
    BspLed_SetCode(BSP_LED_MODE, 0);
    display_tick();
    for (unsigned color = 0; color <= 7; ++color)
    {
        BspLed_SetColor((BspLedColor)color);
        for (unsigned settle = 0U; settle < 50U; ++settle)
        {
            tick_1ms();
        }
        unsigned on = 0, off = 0;
        for (unsigned n = 0; n < 2000; ++n)
        {
            if (n % 50U == 0)
                BspLed_SetColor((BspLedColor)color);
            tick_1ms();
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
    puts("PASS R40 LED: active-low startup, code priority, all colors, breathing, isolated GPIO");
    return 0;
}
