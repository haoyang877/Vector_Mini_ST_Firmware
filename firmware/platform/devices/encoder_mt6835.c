#include "encoder_protocol.h"

#if BSP_ENCODER_DEVICE_TYPE == 3U
/* 器件协议只描述命令、帧校验及角度归一化。 */
const EncoderProtocol g_encoder_protocol = {
    BSP_ENCODER_MT6835,
    {0xA003U, 4U},
    {{0x3007U, 1U}, {0x3008U, 1U}, {0x3009U, 1U}, {0x300AU, 1U}, {0x300DU, 1U}, {0x3011U, 1U}},
    1U};
static uint8_t frame_crc(const uint8_t data[3])
{
    /* CRC-8/0x07，初值0，无反射、无末尾异或。半字节余式由多项式推导。 */
    static const uint8_t s_remainder[16] = {0x00,
                                            0x07,
                                            0x0E,
                                            0x09,
                                            0x1C,
                                            0x1B,
                                            0x12,
                                            0x15,
                                            0x38,
                                            0x3F,
                                            0x36,
                                            0x31,
                                            0x24,
                                            0x23,
                                            0x2A,
                                            0x2D};
    uint8_t crc = 0U;
    for (uint8_t i = 0U; i < 3U; ++i)
    {
        crc ^= data[i];
        crc = (uint8_t)((crc << 4) ^ s_remainder[crc >> 4]);
        crc = (uint8_t)((crc << 4) ^ s_remainder[crc >> 4]);
    }
    return crc;
}
BspResult EncoderProtocol_Decode(const uint8_t *data, EncoderDecodedSample *out)
{
    uint16_t angle = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
    *out = (EncoderDecodedSample){angle,
                                  angle,
                                  (uint16_t)(((uint16_t)data[2] << 8) | data[3]),
                                  BSP_ENCODER_DIAG_FRAME_VALID | BSP_ENCODER_DIAG_SAFETY_VALID,
                                  true};
    return frame_crc(data) != data[3] ? BSP_CRC_ERROR
           : (data[2] & 7U) != 0U     ? BSP_SENSOR_FAULT
                                      : BSP_OK;
}

#endif
