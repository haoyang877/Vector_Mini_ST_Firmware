#ifndef STM32_INDICATOR_FAKE_MAIN_H
#define STM32_INDICATOR_FAKE_MAIN_H
/* 指示灯GPIO寄存器替身；只记录写入值，不访问设备。 */
#include <stdint.h>
typedef struct
{
    uint32_t BSRR;
} IndicatorGpio;
extern IndicatorGpio g_indicator_red, g_indicator_green;
#define LED_R_GPIO_Port (&g_indicator_red)
#define LED_G_GPIO_Port (&g_indicator_green)
#define LED_R_Pin (1U << 15U)
#define LED_G_Pin (1U << 10U)
#endif
