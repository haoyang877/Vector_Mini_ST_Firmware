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

/**
 * @brief 用电机编码器 LUT 线性化方向校正后的单圈原码。
 * @param calibration 已发布的 1024 点 LUT，全零表等价于原码。
 * @param raw_q15 方向校正后的单圈计数，65536 counts/转。
 * @return 线性化后的单圈计数。
 */
static inline uint16_t Encoder_Linearize(const EncoderCalibration *calibration, uint16_t raw_q15)
{
    uint16_t lut_index = raw_q15 >> 6;
    uint16_t fraction = raw_q15 & 0x003FU;
    int32_t correction_a = calibration->linearization_lut_q15[lut_index];
    int32_t correction_b =
        calibration->linearization_lut_q15[(lut_index + 1U) & (ENCODER_OFFSET_LUT_SIZE - 1U)];
    int32_t correction = correction_a + (((correction_b - correction_a) * fraction) >> 6);

    return (uint16_t)((int32_t)raw_q15 - correction);
}

#endif
