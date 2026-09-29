#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include <stdint.h>

/**
 * @brief 启动组合：装载参数、初始化应用状态并执行板级启动序列。
 * @note 只做编排；外设初始化位于 platform 的板级启动契约内，本模块不持有硬件句柄。
 */
void Board_Init(void);

/**
 * @brief 按实际构建目标推进启动交接监督。
 * @param now_ms 单调毫秒时钟。
 * @note 前台在通信Service之后调用；独立APP无Loader能力时不执行启动记录或看门狗操作。
 */
void Board_ServiceBoot(uint32_t now_ms);

#endif
