#ifndef FLASH_H
#define FLASH_H

#include "main.h"
#include <stdbool.h>

/* 板载Flash参数区字节读写；地址/容量归shared target，记录布局归参数服务。 */

/**
 * @brief 读取板载参数区字节记录。
 * @param record 调用方缓冲。
 * @param size_bytes 读取字节数，1..TARGET_PARAM_BYTES。
 * @return 范围和指针有效返回 true。
 */
bool flash_read_param(void *record, uint32_t size_bytes);
/**
 * @brief 擦写并回读校验板载参数区记录。
 * @param record 调用方稳定缓冲。
 * @param size_bytes 记录字节数，1..TARGET_RAW_BYTES，仅写当前原始参数记录。
 * @return 数字关相、擦写及回读均成功返回true；任一等待或操作失败返回false。
 * @note 前台调用，调用方屏蔽中断；不理解参数字段，不覆盖保留页或身份页。
 *       软件轮询有次数上限，无IRQ时基；不保证单bank取指停顿的墙钟期限。
 *       忙超时不再改Flash寄存器，并锁存拒绝后续写入直到复位；保持关相。
 */
bool flash_write_param(const void *record, uint32_t size_bytes);
#endif
