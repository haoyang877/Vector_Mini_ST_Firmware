#ifndef __BOARD_CONFIG_H__
#define __BOARD_CONFIG_H__

/**
 * @brief 启动组合：装载参数、初始化应用状态并执行板级启动序列。
 * @note 只做编排；外设初始化位于 platform 的板级启动契约内，本模块不持有硬件句柄。
 */
void Board_Init(void);

#endif
