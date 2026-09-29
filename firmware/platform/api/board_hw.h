#ifndef BOARD_HW_H
#define BOARD_HW_H

#include <stdbool.h>
#include <stdint.h>

#define BOARD_HW_UID_BYTES 12U

/**
 * @brief 在 CubeMX 外设初始化之后执行板级启动序列。
 * @param motor_phases_enabled true 时启动三相互补 PWM；false 时保持相输出关闭，
 *        仅保留采样触发与监督时钟。
 * @note 调用方（app）负责启动编排与条件判定；本函数内部按固定顺序执行延迟基准、
 *       ADC 校准、条件 PWM、CH4 采样触发、注入采样、JEOS 切换与 TIM7 监督时钟。
 *       顺序属于行为契约，禁止重排；非阻塞、无动态分配，不得从 ISR 调用。
 */
void board_hw_start(bool motor_phases_enabled);

/**
 * @brief 读取芯片实际唯一标识，按三个小端32位字组成12字节值。
 * @param uid 调用方提供的12字节输出缓冲，返回后不保留。
 * @return 非空输出且读取完成返回true；空指针返回false。
 * @note 前台启动组合层调用，有界只读，不分配内存，不代表板卡硬件版本。
 */
bool board_hw_read_uid(uint8_t uid[BOARD_HW_UID_BYTES]);
/**
 * @brief 只读复制独立生产身份记录；解释与CRC校验由组合层委托共享契约执行。
 * @param record 调用方拥有的64B输出。
 * @return 输出非空且读取完成返回true；不代表身份有效。
 * @note 前台启动调用；不更改电机参数、CAN ID或生产权限。
 */
bool board_hw_read_identity(uint8_t record[64]);

#endif
