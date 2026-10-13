#ifndef BOOT_HW_H
#define BOOT_HW_H
#include <stdint.h>
/** @brief APP最早期设置VTOR和板级安全输出。
 * @note 在HAL_Init及中断启用前调用。
 */
void boot_hw_early_init(void);
/** @brief 读取MCU96位UID。
 * @param out 原始12字节。
 */
void boot_hw_uid(uint8_t out[12]);
/** @brief 写一次受CRC保护的Mailbox并软复位。
 * @param node 已确认当前节点。
 * @param token 交接关联标识。
 * @note 组合层必须先确认停机、关相和回复发送。
 */
void boot_hw_reset_to_loader(uint32_t node, uint32_t token);
/** @brief 完整16 KiB参数区只读CRC。
 * @return CRC-32/ISO-HDLC。
 */
uint32_t boot_hw_parameters_crc(void);
#endif
