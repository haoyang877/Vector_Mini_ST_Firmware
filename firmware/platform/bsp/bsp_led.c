/* 闪烁和呼吸只有一个节拍所有者；设置函数不推进相位，MCU仅输出已算好的颜色/亮灭。 */
#include "bsp_led.h"
#include "bsp_internal.h"
#include "application_config.h"

static bool s_initialized, s_blink_on, s_falling;
/* 单字发布通道和码，监督拍读取一次，避免观察到混合版本。 */
static volatile uint32_t s_code_request, s_color_request;
static BspLedCodeChannel s_previous_channel;
static uint8_t s_previous_code;
static uint16_t s_blink_step, s_code_divider, s_color_divider;
static int16_t s_brightness;

static void render_color(void)
{
    BspLedColor color = (BspLedColor)s_color_request;
    uint8_t brightness = s_brightness > 0 ? (uint8_t)s_brightness : 0U;
#if BSP_LED_SHARED_OUTPUTS
    uint32_t request = s_code_request;
    if ((request & 0xffU) != 0U)
    {
        color = (request >> 8U) == BSP_LED_FAULT ? BSP_LED_RED : BSP_LED_BLUE;
        brightness = s_blink_on ? 100U : 0U;
    }
#endif
    McuLed_WriteColor(color, brightness);
}

void Led_Init(void)
{
    if (!BSP_ENABLED_LED)
    {
        return;
    }
    McuLed_Init();
    s_initialized = true;
}

BspResult BspLed_SetCode(BspLedCodeChannel channel, uint8_t blink_count)
{
    if (channel != BSP_LED_MODE && channel != BSP_LED_FAULT)
    {
        return BSP_INVALID_ARGUMENT;
    }
    if (!BSP_ENABLED_LED)
    {
        return BSP_NOT_SUPPORTED;
    }
    if (!s_initialized)
    {
        return BSP_NOT_READY;
    }
    s_code_request = ((uint32_t)channel << 8U) | blink_count;
    return BSP_OK;
}

BspResult BspLed_SetColor(BspLedColor color)
{
    if ((unsigned)color > BSP_LED_WHITE)
    {
        return BSP_INVALID_ARGUMENT;
    }
    if (!BSP_ENABLED_LED)
    {
        return BSP_NOT_SUPPORTED;
    }
    if (!s_initialized)
    {
        return BSP_NOT_READY;
    }
    s_color_request = (uint32_t)color;
    return BSP_OK;
}

void Led_SupervisorTick(void)
{
    if (!s_initialized)
    {
        return;
    }
    if (++s_code_divider >= SUPERVISOR_FREQ / 5U)
    {
        uint32_t request = s_code_request;
        BspLedCodeChannel channel = (BspLedCodeChannel)(request >> 8U);
        uint8_t code = (uint8_t)request;
        s_code_divider = 0U;
        if (channel != s_previous_channel || code != s_previous_code)
        {
            s_blink_step = 0U;
            s_blink_on = false;
        }
        if (s_blink_step < 2U * code)
        {
            s_blink_on = (s_blink_step & 1U) != 0U;
        }
        else if (s_blink_step < 25U)
        {
            s_blink_on = false;
        }
        else
        {
            s_blink_step = 0U;
        }
        ++s_blink_step;
        s_previous_channel = channel;
        s_previous_code = code;
#if BSP_LED_SHARED_OUTPUTS
        render_color();
#else
        McuLed_WriteCode(channel, code != 0U && s_blink_on);
#endif
    }
    if (++s_color_divider >= SUPERVISOR_FREQ / 20U)
    {
        s_color_divider = 0U;
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
        render_color();
    }
}
