#ifndef YG_PROTOCOL_BYTE_ORDER_H
#define YG_PROTOCOL_BYTE_ORDER_H
#include <stdint.h>
/* 固定宽度字节转换无共享状态，允许未对齐地址；容量由编解码调用方验证。 */

/**
 * @brief 从小端字节读取无符号 16 位值。
 * @param data 非空只读缓冲区，至少 2 字节，允许未对齐地址。
 * @return 解码后的无符号整数。
 * @note 不检查容量，无硬件、分配或共享状态，可在 ISR 调用。
 */
static inline uint16_t YgProtocol_ReadU16Le(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

/**
 * @brief 从小端字节读取无符号 32 位值。
 * @param data 非空只读缓冲区，至少 4 字节，允许未对齐地址。
 * @return 解码后的无符号整数。
 * @note 不检查容量，无硬件、分配或共享状态，可在 ISR 调用。
 */
static inline uint32_t YgProtocol_ReadU32Le(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8U) | ((uint32_t)data[2] << 16U) |
           ((uint32_t)data[3] << 24U);
}

/**
 * @brief 从小端字节读取无符号 64 位值。
 * @param data 非空只读缓冲区，至少 8 字节，允许未对齐地址。
 * @return 解码后的无符号整数。
 * @note 不检查容量，无硬件、分配或共享状态，可在 ISR 调用。
 */
static inline uint64_t YgProtocol_ReadU64Le(const uint8_t *data)
{
    return (uint64_t)YgProtocol_ReadU32Le(data) |
           ((uint64_t)YgProtocol_ReadU32Le(data + 4U) << 32U);
}

/**
 * @brief 将无符号 16 位值写为小端字节。
 * @param data 非空可写缓冲区，至少 2 字节，允许未对齐地址。
 * @param value 待写入的无符号整数。
 * @note 按地址递增写入，无容量检查、硬件、分配或共享状态，可在 ISR 调用。
 */
static inline void YgProtocol_WriteU16Le(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value);
    data[1] = (uint8_t)(value >> 8U);
}

/**
 * @brief 将无符号 32 位值写为小端字节。
 * @param data 非空可写缓冲区，至少 4 字节，允许未对齐地址。
 * @param value 待写入的无符号整数。
 * @note 按地址递增写入，无容量检查、硬件、分配或共享状态，可在 ISR 调用。
 */
static inline void YgProtocol_WriteU32Le(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)(value);
    data[1] = (uint8_t)(value >> 8U);
    data[2] = (uint8_t)(value >> 16U);
    data[3] = (uint8_t)(value >> 24U);
}

/**
 * @brief 将无符号 64 位值写为小端字节。
 * @param data 非空可写缓冲区，至少 8 字节，允许未对齐地址。
 * @param value 待写入的无符号整数。
 * @note 按地址递增写入，无容量检查、硬件、分配或共享状态，可在 ISR 调用。
 */
static inline void YgProtocol_WriteU64Le(uint8_t *data, uint64_t value)
{
    YgProtocol_WriteU32Le(data, (uint32_t)value);
    YgProtocol_WriteU32Le(data + 4U, (uint32_t)(value >> 32U));
}
#endif
