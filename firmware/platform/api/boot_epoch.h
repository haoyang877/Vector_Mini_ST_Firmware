#ifndef BOOT_EPOCH_H
#define BOOT_EPOCH_H

#include <stdint.h>

/** @brief 本次程序启动身份的结果；失败保持零值，初始化后不重试或改变身份。 */
typedef enum
{
    BOOT_EPOCH_UNINITIALIZED,
    BOOT_EPOCH_READY,
    BOOT_EPOCH_OUTPUT_ENABLED,
    BOOT_EPOCH_READ_FAILED,
    BOOT_EPOCH_UNKNOWN_RECORD,
    BOOT_EPOCH_WRITE_FAILED,
    BOOT_EPOCH_EXHAUSTED
} BootEpochStatus;

/**
 * @brief 在关相启动前台分配并持久确认本次程序启动的非零身份。
 * @return 已完整写入和读回返回 READY；任何失败均不可用，同次调用返回原结果。
 * @note 仅组合层在采样中断和通信启动前调用；可能擦写独占页，不在 ISR 调用。
 *       不自动修复未知或撕裂记录；全片擦除或旧固件破坏记录后唯一性不再有保证。
 */
BootEpochStatus BootEpoch_Init(void);
/**
 * @brief 读取本次启动已经持久确认的稳定身份。
 * @return READY 时为非零 u64；未初始化、失败或计数耗尽均为零。
 * @note 初始化完成后仅只读，无 I/O；不得由时间、UID、主机随机数补造。
 */
uint64_t BootEpoch_Value(void);
/**
 * @brief 读取本次启动身份的固定初始化结果。
 * @return 未初始化或实际失败原因，READY 仅表示本次身份已持久确认。
 * @note 无 I/O，无副作用；不表示 APP 镜像已经试运行确认。
 */
BootEpochStatus BootEpoch_Status(void);

#endif
