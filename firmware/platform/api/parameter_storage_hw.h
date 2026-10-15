#ifndef PARAMETER_STORAGE_HW_H
#define PARAMETER_STORAGE_HW_H

#include <stdint.h>

/** @brief 既有参数记录的固定字节数，不改变Flash格式或存储地址。 */
#define PARAMETER_STORAGE_RECORD_BYTES 2384U

/** @brief 参数区字节写入结果；不含应用运行模式或故障策略。 */
typedef enum
{
    PARAMETER_STORAGE_WRITTEN = 0,
    PARAMETER_STORAGE_UNCHANGED,
    PARAMETER_STORAGE_UNLOCK_FAILED,
    PARAMETER_STORAGE_ERASE_FAILED,
    PARAMETER_STORAGE_PROGRAM_FAILED,
    PARAMETER_STORAGE_VERIFY_FAILED,
    PARAMETER_STORAGE_LOCK_FAILED
} ParameterStorageResult;

/**
 * @brief 进入参数事务临界区，屏蔽中断并保存原状态。
 * @return 调用前中断状态，只能原样交给parameter_storage_hw_exit。
 * @note 可嵌套；覆盖停机确认、快照和擦写，调用者须成对退出。
 */
uint32_t parameter_storage_hw_enter(void);

/**
 * @brief 恢复进入参数事务前的中断状态。
 * @param state 同一调用层级enter返回的原状态。
 */
void parameter_storage_hw_exit(uint32_t state);

/**
 * @brief 将固定参数区原始字节读入调用者缓冲。
 * @param record 可写且至少有PARAMETER_STORAGE_RECORD_BYTES字节的有效缓冲。
 * @note 不解析或迁移记录；由应用保证与写入互斥。
 */
void parameter_storage_hw_read(void *record);

/**
 * @brief 保存固定大小原始记录；相同时跳过，写入后完整回读校验。
 * @param record 至少有PARAMETER_STORAGE_RECORD_BYTES字节的只读有效缓冲。
 * @return 字节写入结果；只有WRITTEN和UNCHANGED表示成功。
 * @note 调用者须持有参数事务临界区并确保电机控制未启动或已关相。
 *       不保留缓冲指针，不修改应用状态；遇错终止，解锁后均尝试锁回。
 */
ParameterStorageResult parameter_storage_hw_write(const void *record);

#endif
