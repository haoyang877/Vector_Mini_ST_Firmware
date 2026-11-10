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
    Param_StoreOutputEncoder(&record, encoder, NULL);
    OutputEncoder reboot;
    OutputEncoder_Init(&reboot, encoder->params.gear_ratio);
    assert(Param_RestoreOutputEncoder(&record, &reboot, NULL));
    assert(!OutputEncoder_IsReady(&reboot));
    assert(memcmp(&reboot.params, &encoder->params, sizeof(reboot.params)) == 0);
    record.motor.record_crc32 ^= 1;
    assert(!Param_RestoreOutputEncoder(&record, &reboot, NULL));
    record.motor.record_crc32 ^= 1;
    record.output.params.phase_rad += .1f;
    assert(!Param_RestoreOutputEncoder(&record, &reboot, NULL));
    Param_StoreOutputEncoder(&record, encoder, NULL);
    OutputEncoder_Init(&reboot, encoder->params.gear_ratio + 1);
    assert(!Param_RestoreOutputEncoder(&record, &reboot, NULL));
    memset(&record.output, 0xff, sizeof(record.output));
    assert(!Param_RestoreOutputEncoder(&record, &reboot, NULL));
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
    if (ratio == 40.0f)
    {
        /* 连续几组 Hall 原码相同，窗内外的样本仍给出严格单调的分数码均值。 */
        for (unsigned i = 500U; i < 508U; ++i)
        {
            cal.samples[i].sin_counts = cal.samples[503].sin_counts;
            cal.samples[i].cos_counts = cal.samples[503].cos_counts;
        }
        assert(OutputEncoder_CalibEnd(&encoder, &cal, &hall, &motor));
    }
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

static void nonlinear_hall(int direction, int rotation, float phase, bool skew, bool ripple)
{
    EncoderCalibration motor = {0};
    for (unsigned i = 0; i < ENCODER_OFFSET_LUT_SIZE; ++i)
    {
        motor.linearization_lut_q15[i] = (int16_t)(500 + 120 * sinf(_2PI * i / 1024));
    }
    LinearHall hall = {.sin_off_counts = 1900,
                       .cos_off_counts = 2000,
                       .sin_amp = 1750,
                       .cos_amp = 1800,
                       .calibration_success = true};
    OutputEncoder encoder;
    OutputEncoderCalib cal;
    OutputEncoder_Init(&encoder, 8);
    OutputEncoder_CalibBegin(&cal, 8);
    for (unsigned i = 0; cal.count < OUTPUT_ENCODER_CAL_SAMPLES && i < 10000; ++i)
    {
        int32_t raw = 65400 + rotation * (int32_t)(i * 64);
        int32_t corrected =
            raw + (int16_t)(Encoder_Linearize(&motor, (uint16_t)raw) - (uint16_t)raw);
        float truth = direction * (float)corrected * (_2PI / (8 * 65536)) + phase;
        float angle = truth + (skew ? 0 : .06f * sinf(2 * truth) + .025f * cosf(3 * truth));
        OutputEncoder_CalibSample(
            &cal,
            (uint16_t)raw,
            (uint16_t)lroundf(1900 - 1750 * sinf(angle + (skew ? .21f : 0))),
            (uint16_t)lroundf(2000 + 1800 * cosf(angle) + (ripple ? 7 * cosf(112 * truth) : 0)));
    }
    assert(OutputEncoder_CalibEnd(&encoder, &cal, &hall, &motor));
    assert(hall.lut_status == LINEAR_HALL_LUT_OK);
    assert(encoder.params.direction == direction);
    assert(LinearHall_LutIsMonotonic(hall.lut_q15));
    double sum = 0;
    float minimum = 10, maximum = -10, raw_minimum = 10, raw_maximum = -10;
    for (unsigned i = 0; i < 8192; ++i)
    {
        float truth = _2PI * ((float)i + .37f) / 8192;
        float angle = truth + (skew ? 0 : .06f * sinf(2 * truth) + .025f * cosf(3 * truth));
        LinearHall_Update(&hall,
                          roundf(1900 - 1750 * sinf(angle + (skew ? .21f : 0))),
                          roundf(2000 + 1800 * cosf(angle) + (ripple ? 7 * cosf(112 * truth) : 0)));
        float error = remainderf(hall.val.angle_rad - truth, _2PI);
        float raw_error = remainderf(hall.val.angle_raw_rad - truth, _2PI);
        sum += error;
        minimum = fminf(minimum, error);
        maximum = fmaxf(maximum, error);
        raw_minimum = fminf(raw_minimum, raw_error);
        raw_maximum = fmaxf(raw_maximum, raw_error);
        assert(hall.val.angle_rad >= 0 && hall.val.angle_rad < _2PI);
    }
    float mean = (float)(sum / 8192);
    assert(fmaxf(maximum - mean, mean - minimum) < .003f);
    assert(maximum - minimum < (raw_maximum - raw_minimum) * .05f);
    LinearHall_Update(&hall, 1900 + .001f, 3800);
    float left = hall.val.angle_rad;
    LinearHall_Update(&hall, 1900 - .001f, 3800);
    near_angle(left, hall.val.angle_rad, .00001f);

    /* 冷启动恢复幅值和扩展，禁止损坏的表改变已经恢复的对象。 */
    MotorAxisRecord record = {0};
    Param_StoreLinearHall(&record.motor, &hall);
    Param_StoreOutputEncoder(&record, &encoder, &hall);
    LinearHall restored;
    LinearHall_Init(&restored);
    Param_RestoreLinearHall(&record.motor, &restored);
    OutputEncoder reboot;
    OutputEncoder_Init(&reboot, 8);
    assert(Param_RestoreOutputEncoder(&record, &reboot, &restored));
    assert(restored.lut_status == LINEAR_HALL_LUT_OK);
    assert(!memcmp(restored.lut_q15, hall.lut_q15, sizeof hall.lut_q15));
    record.output.hall_lut_q15[40] ^= 1;
    assert(!Param_RestoreOutputEncoder(&record, &reboot, &restored));
    memset(record.output.hall_lut_q15, 0, sizeof record.output.hall_lut_q15);
    record.output.hall_lut_q15[LINEAR_HALL_LUT_SIZE - 1U] = 64;
    record.output.crc32 = Crc32_Compute(&record.output, offsetof(OutputEncoderRecord, crc32), 0);
    assert(!Param_RestoreOutputEncoder(&record, &reboot, &restored));

    /* 缺覆盖与原曲线折返失败；失败后的角度仍能输出，旧表不能残留。 */
    cal.count = 10;
    assert(!OutputEncoder_CalibEnd(&encoder, &cal, &hall, &motor));
    assert(hall.lut_status == LINEAR_HALL_LUT_COVERAGE);
    for (unsigned i = 0; i < LINEAR_HALL_LUT_SIZE; ++i)
    {
        assert(hall.lut_q15[i] == 0);
    }
    cal.count = OUTPUT_ENCODER_CAL_SAMPLES;
    cal.samples[50].sin_counts = cal.samples[10].sin_counts;
    cal.samples[50].cos_counts = cal.samples[10].cos_counts;
    assert(!OutputEncoder_CalibEnd(&encoder, &cal, &hall, &motor));
    assert(hall.lut_status == LINEAR_HALL_LUT_MAPPING);
    LinearHall_Update(&hall, 1000, 3000);
    assert(hall.val.angle_rad == hall.val.angle_raw_rad);
}

static void paired_interval_mean(void)
{
    OutputEncoderCalib cal;
    OutputEncoder_CalibBegin(&cal, 8);
    cal.step_counts = 4;
    OutputEncoder_CalibSample(&cal, 65534, 100, 1000);
    for (unsigned i = 1; i <= 4; ++i)
    {
        OutputEncoder_CalibSample(
            &cal, (uint16_t)(65534 + i), (uint16_t)(100 * i), (uint16_t)(1000 * i));
    }
    assert(cal.count == 2 && cal.samples[1].motor_counts == 65537);
    assert(cal.samples[1].sin_counts == 250 && cal.samples[1].cos_counts == 2500);
    OutputEncoder_CalibSample(&cal, 3, 4000, 4000);
    OutputEncoder_CalibBegin(&cal, 8);
    OutputEncoder_CalibSample(&cal, 7, 123, 456);
    assert(cal.samples[0].motor_counts == 7 && cal.samples[0].sin_counts == 123 &&
           cal.samples[0].cos_counts == 456);
}

int main(void)
{
    direct_drive();
    paired_interval_mean();
    geared_drive(8, 1, 2.9f);
    geared_drive(8, -1, -2.9f);
    geared_drive(40, 1, 2.9f);
    geared_drive(40, -1, -2.9f);
    geared_drive(12, -1, .1f);
    geared_drive(3, 1, -.7f);
    for (int direction = -1; direction <= 1; direction += 2)
    {
        for (int rotation = -1; rotation <= 1; rotation += 2)
        {
            nonlinear_hall(direction, rotation, .23f, false, false);
            nonlinear_hall(direction, rotation, .23f, false, true);
            nonlinear_hall(direction, rotation, 6.27f, true, false);
        }
    }
    assert(sizeof(MotorAxisRecord) == 4372 && sizeof(InterfaceParam) == 2288);
    puts("PASS output encoder: direct/geared, direction, wraps, new LUT calibration, zero, Flash "
         "restore");
    return 0;
}
