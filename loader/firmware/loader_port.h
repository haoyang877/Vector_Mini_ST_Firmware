#ifndef LOADER_PORT_H
#define LOADER_PORT_H
#include "boot_contract.h"

/** @brief 擦除记录并确认失效。
 * @return 操作成功为true。
 * @note 前台调用，禁止擦写参数区。
 */
bool LoaderPort_EraseRecord(void);
/** @brief 擦APP页8～55。
 * @return 擦除并检查成功为true。
 * @note 前台同步操作，主机停止等待。
 */
bool LoaderPort_EraseApp(void);
/** @brief 写一个不跨页的APP块并回读。
 * @param offset APP相对字节偏移。
 * @param data 只读块。
 * @param size 8～40字节，8字节对齐。
 * @return 写入及回读一致为true。
 */
bool LoaderPort_Program(uint32_t offset, const uint8_t *data, uint32_t size);
/** @brief 返回APP只读视图。
 * @return APP首字节地址，仅只读。
 */
const uint8_t *LoaderPort_App(void);
/** @brief 读取48字节记录。
 * @param out 输出记录。
 */
void LoaderPort_ReadRecord(uint8_t out[BOOT_RECORD_BYTES]);
/** @brief 先写前40字节、后写8字节commit，禁止重复编程。
 * @param raw 完整48字节记录。
 * @return 编程及回读一致为true。
 */
bool LoaderPort_Commit(const uint8_t raw[BOOT_RECORD_BYTES]);
/** @brief 读取MCU96位UID的原始字节。
 * @param out 输出12字节。
 */
void LoaderPort_Uid(uint8_t out[12]);
/** @brief 读取完整16 KiB受保护参数区CRC，不修改Flash。
 * @return CRC-32/ISO-HDLC。
 */
uint32_t LoaderPort_ParamCrc(void);
#endif
