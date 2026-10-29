#ifndef ENCODER_PROTOCOL_H
#define ENCODER_PROTOCOL_H

#include "bsp_encoder.h"
#include "bsp_config.h"
#include "encoder_spi.h"
/** @brief 真实器件解码结果；仅BSP内部使用，不混入控制样本。 */
typedef struct
{
    uint16_t angle_counts, frame_word, safety_word;
    uint32_t valid_mask;
    bool crc_checked;
} EncoderDecodedSample;

typedef struct
{
    BspEncoderType type;
    EncoderSpiRequest sample;
    EncoderSpiRequest config[BSP_ENCODER_CONFIG_WORDS];
    uint8_t config_bytes;
} EncoderProtocol;
/* 编译时只链接一个协议描述，协议文件不访问外设。 */
extern const EncoderProtocol g_encoder_protocol;
/**
 * @brief 解析已完整接收的器件响应并归一化为单圈角度。
 * @param data 完整响应字节，由所选器件规定长度。
 * @param out 调用方输出缓冲；失败不发布有效结果。
 * @return 归一化样本状态；仅OK时角度有效。
 * @note BSP内部接口；保持原调用上下文，不保存调用方缓冲指针。
 */
BspResult EncoderProtocol_Decode(const uint8_t *data, EncoderDecodedSample *out);
#endif
