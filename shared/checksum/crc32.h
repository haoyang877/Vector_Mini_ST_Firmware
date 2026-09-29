#ifndef CRC32_H
#define CRC32_H
#include <stdint.h>
/**
 * @brief 计算标准反射 CRC32，采用多项式 0xEDB88320，结果与 zlib.crc32 相同。
 * @param data 至少 bytes 字节的只读数据；bytes 为零时允许空指针。
 * @param bytes 数据长度，单位字节。
 * @param seed 上一块已完成取反的 CRC；首块传零，允许按块续算。
 * @return 当前数据块处理后的 CRC32，包含最终取反；零字节返回 seed。
 * @note 无分配、硬件或共享状态；逐字节执行八次位更新，不在电机快速 ISR 调用。
 */
uint32_t Crc32_Compute(const void *data, uint32_t bytes, uint32_t seed);
#endif
