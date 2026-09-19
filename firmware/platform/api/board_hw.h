#ifndef BOARD_HW_H
#define BOARD_HW_H

#include <stdbool.h>

/**
 * @brief 在 CubeMX 外设初始化之后执行板级启动序列。
 * @param motor_phases_enabled true 时启动三相互补 PWM；false 时保持相输出关闭，
 *        仅保留采样触发与监督时钟。
 * @note 调用方（app）负责启动编排与条件判定；本函数内部按固定顺序执行延迟基准、
 *       ADC 校准、条件 PWM、CH4 采样触发、注入采样、JEOS 切换与 TIM7 监督时钟。
 *       顺序属于行为契约，禁止重排；非阻塞、无动态分配，不得从 ISR 调用。
 */
void board_hw_start(bool motor_phases_enabled);

#endif
