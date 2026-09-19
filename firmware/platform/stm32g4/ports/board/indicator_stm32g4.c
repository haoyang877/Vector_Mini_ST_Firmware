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
        Set_RGB_BreathingColor(RED);
        break;
    case INDICATOR_HW_COLOR_GREEN:
        Set_RGB_BreathingColor(GREEN);
        break;
    case INDICATOR_HW_COLOR_BLUE:
        Set_RGB_BreathingColor(BLUE);
        break;
    case INDICATOR_HW_COLOR_YELLOW:
        Set_RGB_BreathingColor(YELLOW);
        break;
    case INDICATOR_HW_COLOR_PURPLE:
        Set_RGB_BreathingColor(PURPLE);
        break;
    case INDICATOR_HW_COLOR_CYAN:
        Set_RGB_BreathingColor(CYAN);
        break;
    case INDICATOR_HW_COLOR_WHITE:
        Set_RGB_BreathingColor(WHITE);
        break;
    case INDICATOR_HW_COLOR_NULL:
    default:
        Set_RGB_BreathingColor(COLOR_NULL);
        break;
    }
}
