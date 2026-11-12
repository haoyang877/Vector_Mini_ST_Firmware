#ifndef AT32F456_CAN_TIMING_H
#define AT32F456_CAN_TIMING_H
#include "can_bitrate.h"

/* APP 和 Loader 共用实际 TQ 数值；各端口用自己的 SDK 入口写寄存器。 */
typedef struct
{
    uint16_t nominal_seg1;
    uint8_t nominal_seg2, data_seg1, data_seg2, divider, ssp_offset;
} At32CanTiming;

/**
 * @brief 为 180 MHz 的 AT32F456 CAN 控制器选择分频及段长。
 * @param profile 已校验的速率档位 0..11。
 * @return SDK 输入值，两段的同步跳转宽度均取各自 SEG2。
 */
static inline At32CanTiming At32Can_BitTiming(uint8_t profile)
{
    uint32_t value = CanBitrate_Value(profile);
    uint32_t nominal_kbps = value >> 16U;
    uint32_t data_kbps = value & 0xFFFFU;
    uint8_t divider = data_kbps <= 1000U ? 5U : (data_kbps <= 3000U ? 2U : 1U);
    uint32_t tq_clock_khz = 180000U / divider;
    uint32_t nominal_tq = tq_clock_khz / nominal_kbps;
    uint32_t data_tq = tq_clock_khz / data_kbps;
    /* 采样点取最接近 75% 的 TQ；恰好半个 TQ 时选较晚的采样点。 */
    uint8_t nominal_seg2 = (uint8_t)((nominal_tq + 1U) / 4U);
    uint8_t data_seg2 = (uint8_t)((data_tq + 1U) / 4U);
    uint8_t data_seg1 = (uint8_t)(data_tq - data_seg2);
    return (At32CanTiming){(uint16_t)(nominal_tq - nominal_seg2),
                           nominal_seg2,
                           data_seg1,
                           data_seg2,
                           divider,
                           divider <= 2U ? (uint8_t)(data_seg1 + 1U) : 0U};
}
#endif
