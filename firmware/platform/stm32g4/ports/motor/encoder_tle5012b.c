#include "encoder_sensor.h"

#include "encoder_spi.h"
#include "hw_conf.h"

#if ENCODER_SENSOR_TYPE == ENCODER_SENSOR_TYPE_TLE5012B

/* TLE5012B driver：SSC 请求帧/读相位语义与位域解码；总线能力由 encoder_spi 端口提供。
 * 未选中该型号时本文件编译为空，避免把其它型号的代码带进镜像。 */

#define TLE5012B_REQUEST_READ_ANGLE 0x8021U
#define TLE5012B_READ_FRAME 0x0000U

/* 诊断只读命令字：0x8000 | 寄存器地址 | ND（ND = 数据字数-1，单字读为 1）。
 * 地址取自 TLE5012B 数据手册寄存器映射：STAT=0x00、MOD_1=0x06、MOD_2=0x08、MOD_3=0x09。 */
#define TLE5012B_REQUEST_READ_STAT 0x8001U
#define TLE5012B_REQUEST_READ_ACSTAT 0x8011U
#define TLE5012B_REQUEST_READ_MOD_1 0x8061U
#define TLE5012B_REQUEST_READ_MOD_2 0x8081U
#define TLE5012B_REQUEST_READ_MOD_3 0x8091U

/* 配置字缓存：下标 0..5 = STAT/MOD_1/MOD_2/MOD_3/ACSTAT/STAT(复读)；仅诊断，控制路径不消费。 */
static volatile uint16_t tle5012b_config_words[ENCODER_SENSOR_CONFIG_WORDS];

/* 只读采集（末位复读 STAT 用于观察 bit15 读后自清）；任一读取失败时该字保留旧值。 */
void encoder_sensor_capture_config(void)
{
    static const uint16_t requests[ENCODER_SENSOR_CONFIG_WORDS] = {
        TLE5012B_REQUEST_READ_STAT,
        TLE5012B_REQUEST_READ_MOD_1,
        TLE5012B_REQUEST_READ_MOD_2,
        TLE5012B_REQUEST_READ_MOD_3,
        TLE5012B_REQUEST_READ_ACSTAT,
        TLE5012B_REQUEST_READ_STAT,
    };
    uint16_t word;
    uint8_t index;

    for (index = 0U; index < ENCODER_SENSOR_CONFIG_WORDS; index++)
    {
        if (encoder_spi_read_register(requests[index], &word))
        {
            tle5012b_config_words[index] = word;
        }
    }
}

/* 读取已采集的配置字；越界返回 0。 */
uint16_t encoder_sensor_config_word(uint8_t index)
{
    return index < ENCODER_SENSOR_CONFIG_WORDS ? tle5012b_config_words[index] : 0U;
}

void encoder_sensor_init(void)
{
    encoder_spi_init();
    encoder_sensor_capture_config();
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
