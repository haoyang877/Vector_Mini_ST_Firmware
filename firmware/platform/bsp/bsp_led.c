/* 闪烁和呼吸只有一个节拍所有者；设置函数不推进相位，MCU仅输出已算好的颜色/亮灭。 */
#include "bsp_led.h"
#include "bsp_system.h"
#include "bsp_internal.h"
#include "control_config.h"

static bool s_initialized, s_blink_on, s_falling;
static BspLedCodeChannel s_channel, s_previous_channel;
static BspLedColor s_color;
static uint8_t s_code, s_previous_code;
static uint16_t s_blink_step, s_code_divider, s_color_divider;
static int16_t s_brightness;

static void render_color(void)
{
    BspLedColor color = s_color;
    uint8_t brightness = s_brightness > 0 ? (uint8_t)s_brightness : 0U;
#if BSP_LED_SHARED_OUTPUTS
    if (s_code != 0U)
    {
        color = s_channel == BSP_LED_FAULT ? BSP_LED_RED : BSP_LED_BLUE;
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
    uint32_t key = BspSystem_EnterCritical();
    s_channel = channel;
    s_code = blink_count;
    BspSystem_ExitCritical(key);
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
    uint32_t key = BspSystem_EnterCritical();
    s_color = color;
    BspSystem_ExitCritical(key);
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
        s_code_divider = 0U;
        if (s_channel != s_previous_channel || s_code != s_previous_code)
        {
            s_blink_step = 0U;
            s_blink_on = false;
        }
        if (s_blink_step < 2U * s_code)
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
        s_previous_channel = s_channel;
        s_previous_code = s_code;
#if BSP_LED_SHARED_OUTPUTS
        render_color();
#else
        McuLed_WriteCode(s_channel, s_code != 0U && s_blink_on);
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
