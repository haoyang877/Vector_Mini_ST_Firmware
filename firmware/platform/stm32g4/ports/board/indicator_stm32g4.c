#include "indicator_hw.h"

#include "led.h"
#include "rgb.h"

/* 指示器端口：把平台无关的指示语义映射到板上 LED/RGB 驱动。
 * 模式→颜色、错误→闪烁码属于 app 策略；此处只做语义到驱动的转换。 */

void indicator_hw_set_led(bool error_channel, uint8_t blink_num)
{
    LED_SetState(error_channel, blink_num);
}

void indicator_hw_led_task(void)
{
    LED_Task();
}

void indicator_hw_set_color(IndicatorHwColor color)
{
    switch (color)
    {
    case INDICATOR_HW_COLOR_RED:
        Rgb_UpdateBreathingColor(RGB_COLOR_RED);
        break;
    case INDICATOR_HW_COLOR_GREEN:
        Rgb_UpdateBreathingColor(RGB_COLOR_GREEN);
        break;
    case INDICATOR_HW_COLOR_BLUE:
        Rgb_UpdateBreathingColor(RGB_COLOR_BLUE);
        break;
    case INDICATOR_HW_COLOR_YELLOW:
        Rgb_UpdateBreathingColor(RGB_COLOR_YELLOW);
        break;
    case INDICATOR_HW_COLOR_PURPLE:
        Rgb_UpdateBreathingColor(RGB_COLOR_PURPLE);
        break;
    case INDICATOR_HW_COLOR_CYAN:
        Rgb_UpdateBreathingColor(RGB_COLOR_CYAN);
        break;
    case INDICATOR_HW_COLOR_WHITE:
        Rgb_UpdateBreathingColor(RGB_COLOR_WHITE);
        break;
    case INDICATOR_HW_COLOR_NULL:
    default:
        Rgb_UpdateBreathingColor(RGB_COLOR_NONE);
        break;
    }
}
