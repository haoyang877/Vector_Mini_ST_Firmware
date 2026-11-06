#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "output_encoder.h"
#include "parameter_record.h"
#include "control_math.h"
#include "crc32.h"

static float wrap_test(float x)
{
    x = fmodf(x, _2PI);
    return x < 0 ? x + _2PI : x;
}

static uint16_t q15(float rad)
{
    return (uint16_t)lroundf(wrap_test(rad) * (65536.0f / _2PI));
}

static void near_angle(float a, float b, float tolerance)
{
    assert(fabsf(remainderf(a - b, _2PI)) < tolerance);
}

static void roundtrip(OutputEncoder *encoder)
{
    MotorAxisRecord record = {0};
    MotorConfiguration config;
#if BSP_BOARD == 2
    assert(Param_CreateAxis(3, 0, 0, 2, NULL, &config));
#else
    Param_Return_Default(&config);
#endif
    Param_Upload(&record.motor, &config);
    Param_StoreOutputEncoder(&record, encoder);
    OutputEncoder reboot;
    OutputEncoder_Init(&reboot, encoder->params.gear_ratio);
    assert(Param_RestoreOutputEncoder(&record, &reboot));
    assert(!OutputEncoder_IsReady(&reboot));
    assert(memcmp(&reboot.params, &encoder->params, sizeof(reboot.params)) == 0);
    record.motor.record_crc32 ^= 1;
    assert(!Param_RestoreOutputEncoder(&record, &reboot));
    record.motor.record_crc32 ^= 1;
    record.output.params.phase_rad += .1f;
    assert(!Param_RestoreOutputEncoder(&record, &reboot));
    Param_StoreOutputEncoder(&record, encoder);
    OutputEncoder_Init(&reboot, encoder->params.gear_ratio + 1);
    assert(!Param_RestoreOutputEncoder(&record, &reboot));
    memset(&record.output, 0xff, sizeof(record.output));
    assert(!Param_RestoreOutputEncoder(&record, &reboot));
}

static void direct_drive(void)
{
    OutputEncoder encoder;
    OutputEncoder_Init(&encoder, 1);
    assert(!OutputEncoder_IsReady(&encoder));
    OutputEncoder_Locate(&encoder, q15(6.2f), NAN);
    assert(OutputEncoder_IsReady(&encoder));
    near_angle(OutputEncoder_GetPos(&encoder), 6.2f, .0001f);
    for (unsigned i = 1; i <= 2000; ++i)
    {
        float position = 6.2f + (float)i * .01f;
        OutputEncoder_Update(&encoder, q15(position));
        assert(fabsf(OutputEncoder_GetPos(&encoder) - position) < .001f);
    }
    OutputEncoder_SetZero(&encoder);
    assert(OutputEncoder_GetPos(&encoder) == 0);
    roundtrip(&encoder);
    OutputEncoder reboot;
    OutputEncoder_Init(&reboot, 1);
    reboot.params = encoder.params;
    OutputEncoder_Locate(&reboot, q15(26.2f), NAN);
    near_angle(OutputEncoder_GetPos(&reboot), 0, .001f);
    OutputEncoder_Update(&reboot, q15(26.1f));
    near_angle(OutputEncoder_GetPos(&reboot), -.1f, .001f);
}

static void geared_drive(float ratio, int direction, float phase)
{
    EncoderCalibration motor = {0};
    /* 配对采样不使用旧 LUT；结束时按本次新表校正，包括跨零的插值。 */
    for (unsigned i = 0; i < ENCODER_OFFSET_LUT_SIZE; ++i)
    {
        motor.linearization_lut_q15[i] = (int16_t)(500 + 120 * sinf(_2PI * i / 1024));
    }
    LinearHall hall = {.sin_off_counts = 1900,
                       .cos_off_counts = 2000,
                       .sin_amp = 1750,
                       .cos_amp = 1800,
                       .calibration_success = true};
    OutputEncoderCalib cal;
    OutputEncoder encoder;
    OutputEncoder_Init(&encoder, ratio);
    OutputEncoder_CalibBegin(&cal, ratio);
    for (unsigned i = 0; cal.count < OUTPUT_ENCODER_CAL_SAMPLES && i < 100000; ++i)
    {
        int32_t raw_counts = 12345 + (int32_t)i * 91;
        uint16_t corrected = Encoder_Linearize(&motor, (uint16_t)raw_counts);
        int16_t correction = (int16_t)((uint16_t)raw_counts - corrected);
        float motor_pos = (float)(raw_counts - correction) * (_2PI / 65536.0f);
        float h = (motor_pos - phase) / ((float)direction * ratio);
        OutputEncoder_CalibSample(&cal,
                                  (uint16_t)raw_counts,
                                  (uint16_t)lroundf(1900 - 1750 * sinf(h)),
                                  (uint16_t)lroundf(2000 + 1800 * cosf(h)));
    }
    assert(cal.count == OUTPUT_ENCODER_CAL_SAMPLES);
    assert(OutputEncoder_CalibEnd(&encoder, &cal, &hall, &motor));
    assert(encoder.params.direction == direction);
    near_angle(encoder.params.phase_rad, phase, .003f);
    assert(encoder.params.status == OUTPUT_ENCODER_CAL_OK);
    roundtrip(&encoder);
    /* 所有电机圈与 Hall 的 0/2π 边界，加入粗定位噪声也应选中同一圈。 */
    for (unsigned i = 0; i < 2000; ++i)
    {
        float output = _2PI * (float)i / 2000;
        float m = wrap_test((float)direction * ratio * output + phase);
        float h = wrap_test(output + .015f * cosf(3 * output));
        OutputEncoder_Locate(&encoder, q15(m), h);
        near_angle(OutputEncoder_GetPos(&encoder), output, .001f);
    }
    float start = 2.0f;
    OutputEncoder_Locate(&encoder, q15((float)direction * ratio * start + phase), start);
    for (unsigned i = 1; i < 10000; ++i)
    {
        float output = start - .002f * (float)i;
        OutputEncoder_Update(&encoder, q15((float)direction * ratio * output + phase));
        assert(fabsf(OutputEncoder_GetPos(&encoder) - output) < .004f);
    }
    OutputEncoder_SetZero(&encoder);
    assert(encoder.position_rad == 0);
    roundtrip(&encoder);
    /* 错误减速比及没有覆盖整圈的采样不能保存成功状态。 */
    OutputEncoder wrong;
    OutputEncoder_Init(&wrong, ratio + 1);
    assert(!OutputEncoder_CalibEnd(&wrong, &cal, &hall, &motor));
    assert(wrong.params.status == OUTPUT_ENCODER_CAL_RELATION);
    OutputEncoder_CalibBegin(&cal, ratio);
    assert(!OutputEncoder_CalibEnd(&encoder, &cal, &hall, &motor));
    assert(encoder.params.status == OUTPUT_ENCODER_CAL_NO_SAMPLES && !encoder.ready);
    hall.calibration_success = false;
    assert(!OutputEncoder_CalibEnd(&encoder, &cal, &hall, &motor));
    assert(encoder.params.status == OUTPUT_ENCODER_CAL_HALL);
}

int main(void)
{
    direct_drive();
    geared_drive(8, 1, 2.9f);
    geared_drive(8, -1, -2.9f);
    geared_drive(12, -1, .1f);
    geared_drive(3, 1, -.7f);
    assert(sizeof(MotorAxisRecord) == 2320 && sizeof(InterfaceParam) == 2288);
    puts("PASS output encoder: direct/geared, direction, wraps, new LUT calibration, zero, Flash "
         "restore");
    return 0;
}
