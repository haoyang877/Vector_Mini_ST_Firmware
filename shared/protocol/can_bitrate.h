#ifndef CAN_BITRATE_H
#define CAN_BITRATE_H

#include <stdint.h>

/* 0x71 的高/低半字分别是仲裁/数据 kbit/s。档位号也用于 schema14，0 为现场缺省值。 */
#define CAN_BITRATE_PARAMETER_ID 0x71U
#define CAN_BITRATE_PROFILE_COUNT 12U
#define CAN_BITRATE_RECORD_V1 0x43420100U
#define CAN_BITRATE_RECORD_OFFSET 2168U
#define CAN_BITRATE_RECORD_CRC_OFFSET 2140U
#define CAN_BITRATE_BASE_RECORD_BYTES 2288U

/**
 * @brief 将受支持的内部档位转换为参数 0x71 的线上值。
 * @param profile 有效档位 0..11，前六档仲裁为 500 kbit/s。
 * @return 高低半字分别为仲裁和数据速率，单位 kbit/s。
 */
static inline uint32_t CanBitrate_Value(uint8_t profile)
{
    static const uint16_t data_kbps[] = {2000U, 500U, 1000U, 3000U, 4000U, 5000U};
    return ((profile < 6U ? 500UL : 1000UL) << 16U) | data_kbps[profile % 6U];
}

/**
 * @brief 同时匹配仲裁和数据段，拒绝未登记的组合。
 * @param value 完整的 u32 参数值。
 * @return 匹配的档位 0..11，非法值返回 UINT8_MAX。
 */
static inline uint8_t CanBitrate_Profile(uint32_t value)
{
    for (uint8_t i = 0U; i < CAN_BITRATE_PROFILE_COUNT; ++i)
    {
        if (CanBitrate_Value(i) == value)
        {
            return i;
        }
    }
    return UINT8_MAX;
}

/**
 * @brief 读取 schema14 的带标记档位，兼容旧保留槽。
 * @param word 基础记录偏移 2168 的小端 u32，记录 CRC 由调用者校验。
 * @return 合法档位；未知标记或档位回退到缺省档 0。
 */
static inline uint8_t CanBitrate_FromRecord(uint32_t word)
{
    uint8_t profile = (uint8_t)word;
    return (word & 0xFFFFFF00UL) == CAN_BITRATE_RECORD_V1 && profile < CAN_BITRATE_PROFILE_COUNT
               ? profile
               : 0U;
}
#endif
