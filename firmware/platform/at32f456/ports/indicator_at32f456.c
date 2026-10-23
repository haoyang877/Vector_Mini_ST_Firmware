#include "indicator_hw.h"
#include "r40_indicator.h"

#include "at32f45x.h"
#include "critical_hw.h"
#include "power_stage_hw.h"

/* 原理图LED8/LED7：3V3--510欧--LED--PC13/PC14，低电平点亮。
 * PB5/PB6双色灯与外部接口共用且PB5有对地0欧，未经实装确认不得驱动。 */
#define R40_LED_RED GPIO_PINS_13
#define R40_LED_BLUE GPIO_PINS_14
static bool s_initialized, s_error, s_previous_error, s_blink_on;
static uint8_t s_blinks, s_previous_blinks, s_step, s_pwm_step;
static uint16_t s_color_pins;
static int s_brightness;
static bool s_falling;
static uint32_t s_diagnostic;

bool R40_IndicatorDiagnostic(uint32_t pattern)
{
    if (pattern > 4U || !power_stage_hw_phases_disabled())
    {
        return false;
    }
    uint32_t key = critical_hw_enter();
    s_diagnostic = pattern;
    critical_hw_exit(key);
    return true;
}

void R40_IndicatorInit(void)
{
    crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);
    gpio_bits_set(GPIOC, R40_LED_RED | R40_LED_BLUE);
    gpio_init_type pins;
    gpio_default_para_init(&pins);
    pins.gpio_pins = R40_LED_RED | R40_LED_BLUE;
    pins.gpio_mode = GPIO_MODE_OUTPUT;
    pins.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    pins.gpio_pull = GPIO_PULL_NONE;
    pins.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
    gpio_init(GPIOC, &pins);
    s_initialized = true;
}

void indicator_hw_set_led(bool error_channel, uint8_t blink_num)
{
    uint32_t key = critical_hw_enter();
    s_error = error_channel;
    s_blinks = blink_num;
    critical_hw_exit(key);
}

void indicator_hw_led_task(void)
{
    uint32_t key = critical_hw_enter();
    if (s_error != s_previous_error || s_blinks != s_previous_blinks)
    {
        s_step = 0U;
        s_blink_on = false;
    }
    if (s_step < 2U * s_blinks)
    {
        s_blink_on = (s_step & 1U) != 0U;
    }
    else if (s_step < 25U)
    {
        s_blink_on = false;
    }
    else
    {
        s_step = 0U;
    }
    ++s_step;
    s_previous_error = s_error;
    s_previous_blinks = s_blinks;
    critical_hw_exit(key);
}

void indicator_hw_set_color(IndicatorHwColor color)
{
    uint16_t pins;
    switch (color)
    {
    case INDICATOR_HW_COLOR_RED:
    case INDICATOR_HW_COLOR_YELLOW:
        pins = R40_LED_RED;
        break;
    case INDICATOR_HW_COLOR_GREEN:
    case INDICATOR_HW_COLOR_BLUE:
    case INDICATOR_HW_COLOR_CYAN:
        pins = R40_LED_BLUE;
        break;
    case INDICATOR_HW_COLOR_PURPLE:
    case INDICATOR_HW_COLOR_WHITE:
        pins = R40_LED_RED | R40_LED_BLUE;
        break;
    default:
        pins = 0U;
        break;
    }
    uint32_t key = critical_hw_enter();
    s_color_pins = pins;
    /* 沿用原20Hz呼吸调度的4步进及100..-40包络。 */
    s_brightness += s_falling ? -4 : 4;
    if (s_brightness >= 100)
    {
        s_brightness = 100;
        s_falling = true;
    }
    else if (s_brightness <= -40)
    {
        s_brightness = -40;
        s_falling = false;
    }
    critical_hw_exit(key);
}

void R40_IndicatorTick(void)
{
    if (!s_initialized)
    {
        return;
    }
    uint32_t key = critical_hw_enter();
    uint16_t active = 0U;
    if (s_diagnostic != 0U)
    {
        active =
            ((s_diagnostic & 1U) ? R40_LED_RED : 0U) | ((s_diagnostic & 2U) ? R40_LED_BLUE : 0U);
    }
    else if (s_blinks != 0U)
    {
        active = s_blink_on ? (s_error ? R40_LED_RED : R40_LED_BLUE) : 0U;
    }
    else if (s_brightness > s_pwm_step)
    {
        active = s_color_pins;
    }
    s_pwm_step = (uint8_t)((s_pwm_step + 1U) % 100U);
    gpio_bits_set(GPIOC, (R40_LED_RED | R40_LED_BLUE) & (uint16_t)~active);
    gpio_bits_reset(GPIOC, active);
    critical_hw_exit(key);
}
