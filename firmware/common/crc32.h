#ifndef CRC32_H
#define CRC32_H
#include <stdint.h>

/* 全项目共用的标准反射 CRC-32（多项式 0xEDB88320），与 Python zlib.crc32 结果一致。
 * 参数记录与齿槽表的完整性校验都调用本实现；禁止在模块内再写副本。 */

/**
 * @brief 计算一段内存的标准 CRC-32。
 * @param data 只读输入缓冲区；bytes 为 0 时允许为 NULL。
 * @param bytes 参与计算的字节数。
 * @param seed 初始种子；内部按位取反后作为初值，seed=0 等价于标准初值 0xFFFFFFFF。
 * @return 32 位校验值。
 * @note 前台调用；逐字节循环（2048 字节表约数千周期），禁止放入 20 kHz 电流环。
 */
uint32_t Crc32_Compute(const void *data, uint32_t bytes, uint32_t seed);
#endif
