#ifndef __DELAY_H
#define __DELAY_H

#include "main.h"

/**
 * @brief 配置忙等待延迟的时钟换算系数。
 * @param sysclk 系统时钟频率，单位 MHz。
 * @note 初始化时调用；后续延迟要求 SysTick 正常运行。
 */
void delay_init(uint16_t sysclk);
/**
 * @brief 通过 SysTick 忙等待指定毫秒数。
 * @param nms 等待时间，单位 ms。
 * @note 阻塞调用；时钟必须已配置，不可进入电机快速中断。
 */
void delay_ms(uint16_t nms);
/**
 * @brief 通过 SysTick 忙等待指定微秒数。
 * @param nus 等待时间，单位 μs；与时钟相乘不得溢出 uint32_t。
 * @note 阻塞调用；SysTick 必须持续计数，不可进入电机快速中断。
 */
void delay_us(uint32_t nus);

#endif
