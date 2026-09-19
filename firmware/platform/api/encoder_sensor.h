#ifndef ENCODER_SENSOR_H
#define ENCODER_SENSOR_H

#include <stdbool.h>
#include <stdint.h>

/* 编码器传感器功能通道：每个型号一份 driver 实现同一组接口。
 * 通道只输出归一化样本（Q15 单圈角度 + 状态 + 诊断字），不暴露帧格式、位宽或总线细节；
 * 板级通过 hw_conf.h 的 ENCODER_SENSOR_TYPE 选择唯一 driver，未选中的 driver 编译为空。 */

/** 传感器型号标识，用于遥测与诊断。MT6701/MT6835 为预留位，当前仅 TLE5012B 提供 driver。 */
typedef enum
{
    ENCODER_SENSOR_TYPE_NONE = 0,
    ENCODER_SENSOR_TYPE_TLE5012B = 1,
    ENCODER_SENSOR_TYPE_MT6701 = 2,
    ENCODER_SENSOR_TYPE_MT6835 = 3
} EncoderSensorType;

/** 采样状态；driver 负责把型号特有的错误归类到这里。 */
typedef enum
{
    ENCODER_SENSOR_OK = 0,
    ENCODER_SENSOR_BUS_TIMEOUT = 1,
    ENCODER_SENSOR_FRAME_ERROR = 2,
    ENCODER_SENSOR_SENSOR_FAULT = 3
} EncoderSensorStatus;

/** 归一化采样：单圈角度 Q15、状态与原始诊断字。 */
typedef struct
{
    EncoderSensorStatus status; /**< 采样状态；仅 OK 时角度有效。 */
    uint16_t angle_q15;         /**< 单圈角度，Q15（0..65535）。 */
    uint16_t frame_word;        /**< 原始数据帧；仅诊断。 */
    uint16_t safety_word;       /**< 状态/安全字；无该字段的型号为 0。 */
    bool crc_ok;                /**< 帧校验结果；无校验的型号恒 true。 */
} EncoderSensorSample;

/**
 * @brief 初始化当前型号的传感器与总线。
 * @note 上电初始化调用一次；失败按既有致命错误路径处理。
 */
void encoder_sensor_init(void);

/**
 * @brief 非阻塞发起一次采样。
 * @return 已发起返回 true；型号不支持异步或总线未就绪返回 false，
 *         此时调用方仍须调用 encoder_sensor_complete(false, ...)。
 * @note 20 kHz 快速环调用；两次调用之间禁止访问传感器总线。
 */
bool encoder_sensor_begin(void);

/**
 * @brief 完成一次采样并填充归一化结果。
 * @param started encoder_sensor_begin() 的返回值；false 时驱动自行发起同步读取。
 * @param out 输出样本；仅状态为 OK 时角度与诊断字有效。
 * @return 本次采样状态。
 * @note 20 kHz 快速环调用；无论成败都释放总线。
 */
EncoderSensorStatus encoder_sensor_complete(bool started, EncoderSensorSample *out);

/**
 * @brief 读取当前链接的传感器型号。
 * @return 型号标识；未选择任何 driver 时为 ENCODER_SENSOR_TYPE_NONE。
 */
EncoderSensorType encoder_sensor_type(void);

#endif
