#include "encoder_protocol.h"

#if BSP_ENCODER_DEVICE_TYPE == 1U
/* 器件协议只描述命令、帧校验及角度归一化。 */
const EncoderProtocol g_encoder_protocol = {
    BSP_ENCODER_TLE5012B,
    {0x8021U, 2U},
    {{0x8001U, 2U}, {0x8061U, 2U}, {0x8081U, 2U}, {0x8091U, 2U}, {0x8011U, 2U}, {0x8001U, 2U}},
    2U};

BspResult EncoderProtocol_Decode(const uint8_t *data, EncoderDecodedSample *out)
{
    uint16_t word = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
    *out = (EncoderDecodedSample){
        (uint16_t)((word & 0x7FFFU) << 1U), word, 0U, BSP_ENCODER_DIAG_FRAME_VALID, false};
    return BSP_OK;
}

#endif
