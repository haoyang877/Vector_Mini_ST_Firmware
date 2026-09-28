#ifndef PARAM_STORE_H
#define PARAM_STORE_H
#include <stdbool.h>
#include <stdint.h>
/**
 * @brief 读取非易失参数区的完整字节记录。
 * @param record 调用方独占缓冲。
 * @param size_bytes 读取字节数，必须在参数区容量内。
 * @return 读取成功返回 true；失败不提供有效记录。
 * @note 仅启动前台调用；不应用参数、不写回迁移。
 */
bool param_store_read(void *record, uint32_t size_bytes);
/**
 * @brief 写入并回读验证非易失参数记录。
 * @param record 调用方稳定缓冲，端口不保留地址。
 * @param size_bytes 记录字节数。
 * @return 关相、擦写及回读均成功返回 true；失败效果未知。
 * @note 仅停机前台调用，调用方保持中断屏蔽及配置独占。
 */
bool param_store_write(const void *record, uint32_t size_bytes);
#endif
