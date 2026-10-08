#ifndef BOOT_EPOCH_FLASH_H
#define BOOT_EPOCH_FLASH_H

#include <stdbool.h>
#include <stdint.h>
#include "vector_mini_st_g431.h"

/* layout5 内页所有权 v1：RAW前三页、保留两页、epoch两页、生产身份一页。 */
#define BOOT_EPOCH_RECORD_BYTES 32U
#define BOOT_EPOCH_RECORD_MAGIC UINT32_C(0x31455042)
#define BOOT_EPOCH_RECORD_VERSION 1U
#define BOOT_EPOCH_RECORD_COMMIT UINT32_C(0x314D4F43)

/**
 * @brief 读取 epoch 专属两页内的字节，遇到既存或新 ECC 错误即失败。
 * @param address 专属页内起始地址。
 * @param record 调用方缓冲，返回后不保留。
 * @param size_bytes 非零读取字节数，不得越过专属区。
 * @return 范围及读取均正常返回 true；失败缓冲无效。
 * @note 只在关相初始化调用；ECCD 沿既有 NMI 停止启动，不跨异常恢复。
 */
bool BootEpochFlash_Read(uint32_t address, void *record, uint32_t size_bytes);
/**
 * @brief 对 epoch 专属区追加一条记录，复用8字节编程与回读能力。
 * @param address 8字节对齐且完整记录位于专属区的地址。
 * @param record 稳定的32字节记录，所有权属于调用方。
 * @return 输出关闭、区域空白、编程和回读均成功返回 true。
 * @note 启动前台调用；调用方已扫描所有页并确认版本和连续计数，不允许覆盖旧记录。
 */
bool BootEpochFlash_Program(uint32_t address, const void *record);
/**
 * @brief 擦除 epoch 专属区内一页，其他页一律拒绝。
 * @param address 专属区的2KiB页首地址。
 * @return 输出关闭、擦除和整页空白读回确认成功返回 true。
 * @note 调用方须证明是本版本旧页且最新高水位保留在另一页；未知非空页不得调用。
 */
bool BootEpochFlash_Erase(uint32_t address);

#endif
