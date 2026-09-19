#ifndef CRITICAL_HW_H
#define CRITICAL_HW_H

#include <stdint.h>

/* 临界区契约：屏蔽可屏蔽中断并返回进入前的中断屏蔽状态。
 * 只提供成对的 enter/exit，不阻塞、不分配、不嵌套计时；恢复必须使用同一标记。 */

/**
 * @brief 进入临界区：关闭可屏蔽中断并返回进入前的中断屏蔽状态。
 * @return 进入前的中断屏蔽标记，必须原样传给 critical_hw_exit()。
 * @note 临界区必须尽量短，不得包含 Flash、日志、协议编码或阻塞等待。
 */
uint32_t critical_hw_enter(void);

/**
 * @brief 退出临界区并恢复进入前的中断屏蔽状态。
 * @param state critical_hw_enter() 的返回值。
 * @note 标记不得跨函数或跨任务保存；本契约不隐含内存屏障语义。
 */
void critical_hw_exit(uint32_t state);

#endif
