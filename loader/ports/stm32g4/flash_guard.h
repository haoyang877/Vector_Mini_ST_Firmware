#ifndef LOADER_FLASH_GUARD_H
#define LOADER_FLASH_GUARD_H
#include <stdbool.h>
#include <stdint.h>
/**
 * @brief 通过独占 ECC 守卫读取主 Flash。
 * @param address 主 Flash 内的字节地址。
 * @param destination 输出缓冲区；失败时全部内容不可消费。
 * @param bytes 非零读取长度，单位字节。
 * @return 所有读取均无 ECC 错误返回 true。
 * @note 仅前台；原有未知 ECC 标志不清除，失败页在成功整页擦除前保持拒读。
 */
bool LoaderFlash_Read(uint32_t address, void *destination, uint32_t bytes);
/**
 * @brief 只处理当前受保护数据读取产生的可归属 ECC NMI。
 * @param basic_frame 原始 MSP/PSP 指向的 R0..xPSR 八字基本帧。
 * @param exc_return 原始异常返回值。
 * @return 确认归属并使读取失败返回 true；其他原因交上层停机。
 * @note M4 扩展浮点帧的基本帧仍从 SP 开始；不得额外偏移18字或改返回PC。
 */
bool LoaderFlash_HandleNmi(const uint32_t basic_frame[8], uint32_t exc_return);
/**
 * @brief 成功整页擦除后失效缓存并解除该页拒读标记。
 * @param page_address 2KiB 对齐页地址。
 * @return 无未知错误且完成缓存失效返回 true。
 * @note 只能在物理擦除成功后调用；仍须读回验证全FF。
 */
bool LoaderFlash_EraseCompleted(uint32_t page_address);
#endif
