#ifndef YG_PROTOCOL_CRC_H
#define YG_PROTOCOL_CRC_H

#include <stddef.h>
#include <stdint.h>

/**
 * @brief 计算公司协议帧头 CRC8。
 * @param data 待校验字节；长度为零时可以为 NULL。
 * @param length 待校验字节数。
 * @return CRC8 结果。
 * @note 使用 poly=0x9B、init=0x00、xorout=0x00、非反射参数。
 */
uint8_t yg_protocol_crc8(const uint8_t *data, size_t length);

/**
 * @brief 计算公司协议帧尾 CRC16。
 * @param data 待校验字节；长度为零时可以为 NULL。
 * @param length 待校验字节数。
 * @return CRC16 结果。
 * @note 使用 poly=0xBAAD、init=0xFFFF、xorout=0x0000、非反射参数。
 */
uint16_t yg_protocol_crc16(const uint8_t *data, size_t length);

#endif
