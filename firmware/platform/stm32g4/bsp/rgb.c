/* 板级 s_rgb_color 指示灯；发送缓冲在 DMA 传输完成前必须保持有效。 */
#include "rgb.h"

#include "tim.h"

/*2d array of data to be sent by DMA*/
static uint32_t s_pixel_buffer[LED_NUM + 1][24];

static RgbColor s_rgb_color;
static uint8_t s_brightness_direction = 0;
static int s_brightness = 0;

void Rgb_SetColor(uint8_t led_index, RgbColor *color)
{
    uint8_t i;
    /*avoid overflow*/
    if (led_index > LED_NUM)
    {
        return;
    }

    /*green*/
    for (i = 0; i < 8; i++)
    {
        s_pixel_buffer[led_index][i] = ((255 - color->green & (1 << (7 - i))) ? (CODE_1) : CODE_0);
    }
    /*red*/
    for (i = 8; i < 16; i++)
    {
        s_pixel_buffer[led_index][i] = ((255 - color->red & (1 << (15 - i))) ? (CODE_1) : CODE_0);
    }
    /*blue*/
    for (i = 16; i < 24; i++)
    {
        s_pixel_buffer[led_index][i] = ((255 - color->blue & (1 << (23 - i))) ? (CODE_1) : CODE_0);
    }
}

void Rgb_ResetBuffer(void)
{
    uint8_t i;
    for (i = 0; i < 24; i++)
    {
        s_pixel_buffer[LED_NUM][i] = 0;
    }
}

void Rgb_SendBuffer(void)
{
    HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_3, (uint32_t *)s_pixel_buffer, (LED_NUM + 1) * 24);
}

void Rgb_WriteColor(uint16_t led_num, RgbColor *color)
{
    uint16_t i;

    for (i = 0; i < led_num; i++)
    {
        Rgb_SetColor(i, color);
    }
    Rgb_ResetBuffer();
    Rgb_SendBuffer();
}

void Rgb_UpdateBreathingColor(RgbColorId color_type)
{
    if (s_brightness_direction == 0)
    {
        s_brightness += 4;
        if (s_brightness >= 100)
        {
            s_brightness = 100;
            s_brightness_direction = 1;
        }
    }
    else if (s_brightness_direction == 1)
    {
        s_brightness -= 4;
        if (s_brightness <= -40)
        {
            s_brightness = -40;
            s_brightness_direction = 0;
        }
    }

    switch (color_type)
    {
    case RGB_COLOR_NONE:
        s_rgb_color.red = 0;
        s_rgb_color.green = 0;
        s_rgb_color.blue = 0;
        break;

    case RGB_COLOR_RED:
        s_rgb_color.red = s_brightness >= 0 ? s_brightness : 0;
        s_rgb_color.green = 0;
        s_rgb_color.blue = 0;
        break;

    case RGB_COLOR_GREEN:
        s_rgb_color.red = 0;
        s_rgb_color.green = s_brightness >= 0 ? s_brightness : 0;
        s_rgb_color.blue = 0;
        break;

    case RGB_COLOR_BLUE:
        s_rgb_color.red = 0;
        s_rgb_color.green = 0;
        s_rgb_color.blue = s_brightness >= 0 ? s_brightness : 0;
        break;

    case RGB_COLOR_YELLOW:
        s_rgb_color.red = s_brightness >= 0 ? s_brightness : 0;
        s_rgb_color.green = s_brightness >= 0 ? s_brightness : 0;
        s_rgb_color.blue = 0;
        break;

    case RGB_COLOR_PURPLE:
        s_rgb_color.red = s_brightness >= 0 ? s_brightness : 0;
        s_rgb_color.green = 0;
        s_rgb_color.blue = s_brightness >= 0 ? s_brightness : 0;
        break;

    case RGB_COLOR_CYAN:
        s_rgb_color.red = 0;
        s_rgb_color.green = s_brightness >= 0 ? s_brightness : 0;
        s_rgb_color.blue = s_brightness >= 0 ? s_brightness : 0;
        break;

    case RGB_COLOR_WHITE:
        s_rgb_color.red = s_brightness >= 0 ? s_brightness : 0;
        s_rgb_color.green = s_brightness >= 0 ? s_brightness : 0;
        s_rgb_color.blue = s_brightness >= 0 ? s_brightness : 0;
        break;

    default:
        break;
    }

    Rgb_WriteColor(1, &s_rgb_color);
}
