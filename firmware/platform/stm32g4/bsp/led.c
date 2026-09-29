/* 板级指示灯驱动；引脚与电平约定由本 BSP 定义。 */
#include "led.h"

static LedState s_led;

void LED_SetState(bool mode_or_error, uint8_t blink_num)
{
    s_led.mode_or_error = mode_or_error;
    s_led.blink_num = blink_num;
}

static void update_gpio(void)
{
    LED_R(s_led.on_or_off[1]);
    LED_G(s_led.on_or_off[0]);
}

void LED_Task(void)
{
    /*type of mode or error changed, reset all LEDs*/
    if (s_led.mode_or_error_last != s_led.mode_or_error || s_led.blink_num_last != s_led.blink_num)
    {
        s_led.on_or_off[0] = 0;
        s_led.on_or_off[1] = 0;
        s_led.cnt = 0;
    }

    /*mode instruction:green light*/
    if (s_led.mode_or_error == 0)
    {
        /*toggle*/
        if (s_led.cnt < 2 * s_led.blink_num)
        {
            s_led.on_or_off[0] = s_led.cnt % 2;
        }
        /*turn off*/
        else if (s_led.cnt < 25)
        {
            s_led.on_or_off[0] = 0;
        }
        /*clear cnt*/
        else
        {
            s_led.cnt = 0;
        }
    }
    /*error instruction:red light*/
    else
    {
        /*toggle*/
        if (s_led.cnt < 2 * s_led.blink_num)
        {
            s_led.on_or_off[1] = s_led.cnt % 2;
        }
        /*turn off*/
        else if (s_led.cnt < 25)
        {
            s_led.on_or_off[1] = 0;
        }
        /*clear cnt*/
        else
        {
            s_led.cnt = 0;
        }
    }

    s_led.cnt++;
    s_led.mode_or_error_last = s_led.mode_or_error;
    s_led.blink_num_last = s_led.blink_num;
    update_gpio();
}
