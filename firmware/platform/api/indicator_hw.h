#ifndef INDICATOR_HW_H
#define INDICATOR_HW_H

#include <stdbool.h>
#include <stdint.h>

/** 板载指示颜色；由平台端口映射到具体 LED/RGB 驱动值。 */
typedef enum
{
    INDICATOR_HW_COLOR_NULL = 0,
    INDICATOR_HW_COLOR_RED,
    INDICATOR_HW_COLOR_GREEN,
    INDICATOR_HW_COLOR_BLUE,
    INDICATOR_HW_COLOR_YELLOW,
    INDICATOR_HW_COLOR_PURPLE,
    INDICATOR_HW_COLOR_CYAN,
    INDICATOR_HW_COLOR_WHITE
} IndicatorHwColor;

/**
 * @brief 设置 LED 闪烁语义（模式号或故障号）。
 * @param error_channel true 表示闪烁码取自故障编号，false 表示取自模式编号。
 * @param blink_num 闪烁码；闪烁状态机由 1 kHz 调度推进。
 * @note 只更新指示状态，不阻塞、不访问电机路径。
 */
void indicator_hw_set_led(bool error_channel, uint8_t blink_num);

/**
 * @brief 推进 LED 闪烁状态机一次。
 * @note 由 1 kHz 调度按既有分频调用；保持既有非阻塞语义。
 */
void indicator_hw_led_task(void);

/**
 * @brief 设置 RGB 呼吸颜色。
 * @param color 目标颜色，取 IndicatorHwColor。
 * @note 由 1 kHz 调度按既有分频调用；刷新时序与迁移前一致。
 */
void indicator_hw_set_color(IndicatorHwColor color);

#endif
