#include "encoder_sensor.h"

#include "encoder_spi.h"
#include "hw_conf.h"

#if ENCODER_SENSOR_TYPE == ENCODER_SENSOR_TYPE_TLE5012B

/* TLE5012B driver：SSC 请求帧/读相位语义与位域解码；总线能力由 encoder_spi 端口提供。
 * 未选中该型号时本文件编译为空，避免把其它型号的代码带进镜像。 */

#define TLE5012B_REQUEST_READ_ANGLE 0x8021U
#define TLE5012B_READ_FRAME 0x0000U

void encoder_sensor_init(void)
{
    encoder_spi_init();
}

bool encoder_sensor_begin(void)
{
    return encoder_spi_read_begin(TLE5012B_REQUEST_READ_ANGLE);
}

EncoderSensorStatus encoder_sensor_complete(bool started, EncoderSensorSample *out)
{
    uint16_t word = 0U;

    if (!encoder_spi_read_complete(
            started, TLE5012B_REQUEST_READ_ANGLE, TLE5012B_READ_FRAME, &word))
    {
        out->status = ENCODER_SENSOR_BUS_TIMEOUT;
        return out->status;
    }

    out->frame_word = word;
    out->safety_word = 0U; /* 该型号本轮未读取安全字与 CRC。 */
    out->crc_ok = true;
    out->angle_q15 = (uint16_t)((word & 0x7FFFU) << 1U);
    out->status = ENCODER_SENSOR_OK;
    return out->status;
}

EncoderSensorType encoder_sensor_type(void)
{
    return ENCODER_SENSOR_TYPE_TLE5012B;
}

#endif
