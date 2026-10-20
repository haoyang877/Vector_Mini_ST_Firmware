#ifndef ENCODER_CONFIG_H
#define ENCODER_CONFIG_H

#include <stdint.h>

#define ENCODER_OFFSET_LUT_SIZE 1024U
#define ENCODER_OFFSET_LUT_BITS 10U
#define ENC_CALIB_LINEARIZED (1U << 0)
#define ENC_CALIB_ELECTRICAL_ZERO (1U << 1)
#define ENC_CALIB_MECHANICAL_ZERO (1U << 2)
#define ENC_CALIB_ZERO_POS ENC_CALIB_ELECTRICAL_ZERO
#define ENC_CALIB_ALL (ENC_CALIB_LINEARIZED | ENC_CALIB_ELECTRICAL_ZERO)

/** @brief 设备装配的编码器标定；只由参数装载与标定提交修改，运行反馈只读。 */
typedef struct
{
    uint16_t electrical_zero_q15;
    uint16_t mechanical_zero_q15;
    int16_t linearization_lut_q15[ENCODER_OFFSET_LUT_SIZE];
    uint8_t calib_flag;
    uint8_t reverse;
} EncoderCalibration;

#endif
