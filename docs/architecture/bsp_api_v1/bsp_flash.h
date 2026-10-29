/**
 * @file bsp_flash.h
 * @brief 固定应用记录区的受控读写。
 * @note 固定区硬件存储事务由BSP完整持有；记录格式和业务独占属于应用。
 */
#ifndef BSP_FLASH_H
#define BSP_FLASH_H

#include <stddef.h>
#include "bsp.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief 从固定应用记录区起点读取字节。
     * @param data 至少length字节可写缓冲，零长度允许NULL。
     * @param length 读取字节数，不超过记录区容量。
     * @return 完成为BSP_OK；参数/容量非法为INVALID_ARGUMENT；未初始化为NOT_READY；
     *   未提供为NOT_SUPPORTED；读失败为IO_ERROR。
     * @note 仅前台，与写操作由同一记录所有者串行化；不公开任意地址，不解释记录。
     * @note 失败后缓冲不可作为有效记录；已初始化且启用时零长度为成功空操作。
     */
    BspResult BspFlash_Read(void *data, size_t length);

    /**
     * @brief 由BSP独占完整擦写过程并回读验证。
     * @param data 调用期间稳定的源记录，不得与可能擦除的目标区域重叠；零长度允许NULL。
     * @param length 记录字节数，不大于配置的可写容量；地址加长度不得溢出。
     * @return 成功为BSP_OK；非法输入/源区重叠为INVALID_ARGUMENT；未关相或ISR调用为REJECTED；
     *   未初始化为NOT_READY；未提供为NOT_SUPPORTED；硬件/恢复失败为IO_ERROR。
     * @note 仅停机前台及应用配置独占阶段；
     *   BSP保存原中断状态，复核关相并在完整操作期间保持原屏蔽策略。
     * @note BSP拥有采样暂停、擦写、回读、允许时恢复采样及最终中断状态恢复；
     *   调用方不用临界区包裹本函数。
     * @note 阻塞可能包含同bank取指停顿，软件轮询次数有上限但不构成墙钟时间保证，不依赖IRQ计时超
     *   时。
     * @note 硬件超时或状态不明则锁存写禁止直到复位，保留关相，不盲目恢复采样或重试；
     *   沿用原看门狗兜底，不主动喂狗。
     * @note 提前验证全部输入和源区后才擦除；失败可损坏记录，无掉电原子性；零长度不擦除。
     */
    BspResult BspFlash_Write(const void *data, size_t length);

#ifdef __cplusplus
}
#endif

#endif /* BSP_FLASH_H */
