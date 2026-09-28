#ifndef __LED_H__
#define __LED_H__

#include "main.h"
#include <stdbool.h>

/*LED switch macro 0:off 1:on*/
#define LED_R(a)                                                                                   \
    if (a)                                                                                         \
    {                                                                                              \
        LED_R_GPIO_Port->BSRR = (uint32_t)LED_R_Pin << 16U;                                        \
    }                                                                                              \
    else                                                                                           \
    {                                                                                              \
        LED_R_GPIO_Port->BSRR = LED_R_Pin;                                                         \
    }

#define LED_G(a)                                                                                   \
    if (a)                                                                                         \
    {                                                                                              \
        LED_G_GPIO_Port->BSRR = (uint32_t)LED_G_Pin << 16U;                                        \
    }                                                                                              \
    else                                                                                           \
    {                                                                                              \
        LED_G_GPIO_Port->BSRR = LED_G_Pin;                                                         \
    }

#define TEST(a)                                                                                    \
    if (a)                                                                                         \
    {                                                                                              \
        TEST_GPIO_Port->BSRR = (uint32_t)TEST_Pin << 16U;                                          \
    }                                                                                              \
    else                                                                                           \
    {                                                                                              \
        TEST_GPIO_Port->BSRR = TEST_Pin;                                                           \
    }

#define LED_R_TOGGLE                                                                               \
    {                                                                                              \
        LED_R_GPIO_Port->ODR ^= LED_R_Pin;                                                         \
    }
#define LED_G_TOGGLE                                                                               \
    {                                                                                              \
        LED_G_GPIO_Port->ODR ^= LED_G_Pin;                                                         \
    }

typedef struct
{
    bool mode_or_error;
    bool mode_or_error_last;
    bool on_or_off[2];
    uint8_t blink_num;
    uint8_t blink_num_last;
    uint8_t cnt;
} LED_TypeDef;

/**
 * @brief 设置下次指示灯任务使用的模式或故障信息。
 * @param mode_or_error false 使用绿色模式灯，true 使用红色故障灯。
 * @param blink_num 一个闪烁周期内的闪烁次数。
 * @note 板级指示状态由单一调度所有者访问，不使能电机。
 */
void LED_SetState(bool mode_or_error, uint8_t blink_num);
/**
 * @brief 推进指示灯闪烁计数并写板级 GPIO。
 * @note 按既有 200 ms 节拍串行调用，不得调用于电机快速环。
 */
void LED_Task(void);
#endif