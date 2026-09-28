/* 协议核心的纯 CRC 实现；不依赖 MCU、HAL 或可变全局状态。 */
#include "yg_protocol_crc.h"

uint8_t yg_protocol_crc8(const uint8_t *data, size_t length)
{
    uint8_t crc = 0x00U;
    size_t index;

    for (index = 0U; index < length; ++index)
    {
        uint8_t bit;

        crc ^= data[index];
        for (bit = 0U; bit < 8U; ++bit)
        {
            crc = (crc & 0x80U) != 0U ? (uint8_t)((crc << 1U) ^ 0x9BU) : (uint8_t)(crc << 1U);
        }
    }

    return crc;
}

uint16_t yg_protocol_crc16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFFU;
    size_t index;

    for (index = 0U; index < length; ++index)
    {
        uint8_t bit;

        crc ^= (uint16_t)data[index] << 8U;
        for (bit = 0U; bit < 8U; ++bit)
        {
            crc = (crc & 0x8000U) != 0U ? (uint16_t)((crc << 1U) ^ 0xBAADU) : (uint16_t)(crc << 1U);
        }
    }

    return crc;
}
