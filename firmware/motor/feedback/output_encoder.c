#include "output_encoder.h"
#include "control_math.h"
#include <math.h>
#include <string.h>

static float wrap(float angle)
{
    angle = fmodf(angle, _2PI);
    return angle < 0.0f ? angle + _2PI : angle;
}

static float delta(float angle)
{
    return wrap(angle + _PI) - _PI;
}

void OutputEncoder_Init(OutputEncoder *encoder, float gear_ratio)
{
    *encoder =
        (OutputEncoder){.params = {.gear_ratio = gear_ratio,
                                   .direction = 1,
                                   .status = gear_ratio == 1.0f ? OUTPUT_ENCODER_CAL_OK : 0U}};
}

void OutputEncoder_Locate(OutputEncoder *encoder, uint16_t motor_q15, float hall_rad)
{
    const OutputEncoderParams *p = &encoder->params;
    float motor_rad = (float)motor_q15 * (_2PI / 65536.0f);
    float turns = 0.0f;
    if (p->gear_ratio != 1.0f)
    {
        turns = roundf(((float)p->direction * p->gear_ratio * hall_rad + p->phase_rad - motor_rad) /
                       _2PI);
    }
    encoder->position_rad =
        wrap((float)p->direction * (motor_rad + turns * _2PI - p->phase_rad) / p->gear_ratio -
             p->zero_rad);
    encoder->initial_position_rad = encoder->position_rad;
    encoder->motor_delta_q15 = 0;
    encoder->previous_motor_q15 = motor_q15;
    encoder->ready = true;
}

void OutputEncoder_Update(OutputEncoder *encoder, uint16_t motor_q15)
{
    encoder->motor_delta_q15 += (int16_t)(motor_q15 - encoder->previous_motor_q15);
    encoder->previous_motor_q15 = motor_q15;
    encoder->position_rad = encoder->initial_position_rad +
                            (float)encoder->motor_delta_q15 * (float)encoder->params.direction *
                                (_2PI / 65536.0f) / encoder->params.gear_ratio;
}

float OutputEncoder_GetPos(const OutputEncoder *encoder)
{
    return encoder->position_rad;
}

bool OutputEncoder_IsReady(const OutputEncoder *encoder)
{
    return encoder->ready;
}

void OutputEncoder_SetZero(OutputEncoder *encoder)
{
    encoder->params.zero_rad = wrap(encoder->params.zero_rad + encoder->position_rad);
    encoder->params.status |= OUTPUT_ENCODER_ZERO_SET;
    encoder->position_rad = encoder->initial_position_rad = 0.0f;
    encoder->motor_delta_q15 = 0;
}

void OutputEncoder_CalibBegin(OutputEncoderCalib *cal, float gear_ratio)
{
    /* 不清零整个样本数组，count 决定可读范围。 */
    cal->count = 0U;
    cal->motor_sum_counts = 0;
    cal->sin_sum_counts = cal->cos_sum_counts = cal->sample_count = 0U;
    cal->step_counts =
        (int32_t)(gear_ratio * 65536.0f * 1.01f / (float)(OUTPUT_ENCODER_CAL_SAMPLES - 1U));
}

void OutputEncoder_CalibSample(OutputEncoderCalib *cal,
                               uint16_t motor_q15,
                               uint16_t sin_counts,
                               uint16_t cos_counts)
{
    if (cal->count == OUTPUT_ENCODER_CAL_SAMPLES)
    {
        return;
    }
    if (cal->count == 0U)
    {
        cal->motor_counts = cal->first_counts = motor_q15;
    }
    else
    {
        cal->motor_counts += (int16_t)(motor_q15 - cal->previous_q15);
    }
    cal->previous_q15 = motor_q15;
    cal->motor_sum_counts += cal->motor_counts;
    cal->sin_sum_counts += sin_counts;
    cal->cos_sum_counts += cos_counts;
    ++cal->sample_count;
    if (fabsf((float)(cal->motor_counts - cal->first_counts)) >=
        (float)cal->count * (float)cal->step_counts)
    {
        uint32_t n = cal->sample_count;
        cal->samples[cal->count++] =
            (OutputEncoderCalibSample){(int32_t)lroundf((float)cal->motor_sum_counts / (float)n),
                                       (uint16_t)((cal->sin_sum_counts + n / 2U) / n),
                                       (uint16_t)((cal->cos_sum_counts + n / 2U) / n)};
        cal->motor_sum_counts = 0;
        cal->sin_sum_counts = cal->cos_sum_counts = cal->sample_count = 0U;
    }
}

static float sample_phase(const OutputEncoderCalibSample *sample,
                          LinearHall *hall,
                          const EncoderCalibration *motor,
                          float ratio,
                          int32_t direction)
{
    LinearHall_Update(hall, sample->sin_counts, sample->cos_counts);
    float motor_rad =
        (float)Encoder_Linearize(motor, (uint16_t)sample->motor_counts) * (_2PI / 65536.0f);
    return motor_rad - (float)direction * ratio * hall->val.angle_rad;
}

static int32_t corrected_counts(const OutputEncoderCalibSample *sample,
                                const EncoderCalibration *motor)
{
    uint16_t raw = (uint16_t)sample->motor_counts;
    return sample->motor_counts + (int16_t)(Encoder_Linearize(motor, raw) - raw);
}

/* 同一窗口处理三路；仅建表使用，保留同一有效采样时刻。 */
static OutputEncoderCalibSample filtered_sample(const OutputEncoderCalib *cal, unsigned i)
{
    static const unsigned weights[9] = {1U, 8U, 28U, 56U, 70U, 56U, 28U, 8U, 1U};
    float motor_sum = 0.0f;
    uint32_t sin_sum = 0U, cos_sum = 0U;
    for (unsigned k = 0U; k < 9U; ++k)
    {
        const OutputEncoderCalibSample *sample = &cal->samples[i + k - 4U];
        motor_sum += (float)sample->motor_counts * (float)weights[k];
        sin_sum += sample->sin_counts * weights[k];
        cos_sum += sample->cos_counts * weights[k];
    }
    return (OutputEncoderCalibSample){(int32_t)lroundf(motor_sum / 256.0f),
                                      (uint16_t)((sin_sum + 128U) / 256U),
                                      (uint16_t)((cos_sum + 128U) / 256U)};
}

static uint32_t build_hall_lut(const OutputEncoderCalib *cal,
                               LinearHall *hall,
                               const EncoderCalibration *motor,
                               float ratio,
                               int32_t direction,
                               float travel)
{
    uint8_t covered[LINEAR_HALL_LUT_SIZE / 8U] = {0};
    unsigned count = 0U;
    int32_t sum = 0;
    float first_angle = 0.0f, previous_angle = 0.0f, unwrapped = 0.0f, previous_error = 0.0f;
    float older_error = 0.0f, filtered_angle = 0.0f, filtered_error = 0.0f;
    OutputEncoderCalibSample first_sample = filtered_sample(cal, 4U);
    int32_t first_motor = corrected_counts(&first_sample, motor);
    const float step = _2PI / (float)LINEAR_HALL_LUT_SIZE;
    for (unsigned i = 0U; i < OUTPUT_ENCODER_CAL_SAMPLES - 8U; ++i)
    {
        OutputEncoderCalibSample filtered_sample_value = filtered_sample(cal, i + 4U);
        const OutputEncoderCalibSample *sample = &filtered_sample_value;
        LinearHall_Update(hall, sample->sin_counts, sample->cos_counts);
        float angle = hall->val.angle_raw_rad;
        if (i == 0U)
        {
            first_angle = previous_angle = unwrapped = angle;
            continue;
        }
        float span = delta(angle - previous_angle);
        if (span * travel <= 0.0f)
        {
            return LINEAR_HALL_LUT_MAPPING;
        }
        float end = unwrapped + span;
        float reference = (float)direction *
                          (float)(corrected_counts(sample, motor) - first_motor) *
                          (_2PI / 65536.0f) / ratio;
        float error = reference - (end - first_angle);
        /* 先抑制配对点间的高频误差，再从 2048 点重采样至 1024 点。 */
        if (i >= 2U)
        {
            float filtered = (older_error + 2.0f * previous_error + error) * 0.25f;
            if (i > 2U)
            {
                int begin_node = (int)ceilf(fminf(filtered_angle, unwrapped) / step);
                int end_node = (int)floorf(fmaxf(filtered_angle, unwrapped) / step);
                for (int node = begin_node; node <= end_node; ++node)
                {
                    float fraction =
                        ((float)node * step - filtered_angle) / (unwrapped - filtered_angle);
                    unsigned index = (unsigned)node & (LINEAR_HALL_LUT_SIZE - 1U);
                    uint8_t bit = (uint8_t)(1U << (index & 7U));
                    if ((covered[index >> 3U] & bit) == 0U)
                    {
                        long correction =
                            lroundf((filtered_error + fraction * (filtered - filtered_error)) *
                                    (65536.0f / _2PI));
                        if (correction < INT16_MIN || correction > INT16_MAX)
                        {
                            return LINEAR_HALL_LUT_MAPPING;
                        }
                        hall->lut_q15[index] = (int16_t)correction;
                        covered[index >> 3U] |= bit;
                        sum += (int32_t)correction;
                        ++count;
                    }
                }
            }
            filtered_angle = unwrapped;
            filtered_error = filtered;
        }
        previous_angle = angle;
        older_error = previous_error;
        previous_error = error;
        unwrapped = end;
    }
    if (count != LINEAR_HALL_LUT_SIZE)
    {
        return LINEAR_HALL_LUT_COVERAGE;
    }
    int32_t mean = (int32_t)lroundf((float)sum / (float)LINEAR_HALL_LUT_SIZE);
    for (unsigned i = 0U; i < LINEAR_HALL_LUT_SIZE; ++i)
    {
        int32_t correction = hall->lut_q15[i] - mean;
        if (correction < INT16_MIN || correction > INT16_MAX)
        {
            return LINEAR_HALL_LUT_MAPPING;
        }
        hall->lut_q15[i] = (int16_t)correction;
    }
    return LinearHall_LutIsMonotonic(hall->lut_q15) ? LINEAR_HALL_LUT_OK : LINEAR_HALL_LUT_MAPPING;
}

bool OutputEncoder_CalibEnd(OutputEncoder *encoder,
                            const OutputEncoderCalib *cal,
                            LinearHall *hall,
                            const EncoderCalibration *motor)
{
    encoder->ready = false;
    memset(hall->lut_q15, 0, sizeof(hall->lut_q15));
    hall->lut_status = 0U;
    if (encoder->params.gear_ratio < 1.0f ||
        encoder->params.gear_ratio != floorf(encoder->params.gear_ratio))
    {
        encoder->params.status = OUTPUT_ENCODER_CAL_RELATION;
        return false;
    }
    if (cal->count != OUTPUT_ENCODER_CAL_SAMPLES || !hall->calibration_success)
    {
        hall->lut_status = hall->calibration_success ? LINEAR_HALL_LUT_COVERAGE : 0U;
        encoder->params.status =
            !hall->calibration_success ? OUTPUT_ENCODER_CAL_HALL : OUTPUT_ENCODER_CAL_NO_SAMPLES;
        return false;
    }
    LinearHall corrected = *hall;
    float travel = 0.0f, previous = 0.0f;
    for (uint16_t i = 0U; i < OUTPUT_ENCODER_CAL_SAMPLES; ++i)
    {
        LinearHall_Update(&corrected, cal->samples[i].sin_counts, cal->samples[i].cos_counts);
        if (i != 0U)
        {
            travel += delta(corrected.val.angle_rad - previous);
        }
        previous = corrected.val.angle_rad;
    }
    float motor_travel =
        (float)(corrected_counts(&cal->samples[OUTPUT_ENCODER_CAL_SAMPLES - 1U], motor) -
                corrected_counts(&cal->samples[0], motor)) *
        (_2PI / 65536.0f);
    int32_t direction = travel * motor_travel >= 0.0f ? 1 : -1;
    float ratio = encoder->params.gear_ratio;
    if (fabsf(travel) < 6.0f || fabsf(travel - (float)direction * motor_travel / ratio) > 0.15f)
    {
        hall->lut_status = LINEAR_HALL_LUT_MAPPING;
        encoder->params.status = OUTPUT_ENCODER_CAL_RELATION;
        return false;
    }
    hall->lut_status = build_hall_lut(cal, hall, motor, ratio, direction, travel);
    if (hall->lut_status != LINEAR_HALL_LUT_OK)
    {
        memset(hall->lut_q15, 0, sizeof(hall->lut_q15));
        encoder->params.status = OUTPUT_ENCODER_CAL_RELATION;
        return false;
    }
    corrected = *hall;
    float sine = 0.0f, cosine = 0.0f;
    for (uint16_t i = 0U; i < OUTPUT_ENCODER_CAL_SAMPLES; ++i)
    {
        float phase = sample_phase(&cal->samples[i], &corrected, motor, ratio, direction);
        sine += sinf(phase);
        cosine += cosf(phase);
    }
    float phase = atan2f(sine, cosine);
    float peak_error = 0.0f;
    for (uint16_t i = 0U; i < OUTPUT_ENCODER_CAL_SAMPLES; ++i)
    {
        float error = sample_phase(&cal->samples[i], &corrected, motor, ratio, direction) - phase;
        peak_error = fmaxf(peak_error, fabsf(delta(error)));
    }
    /* 辨圈边界为 π 电机轴 rad；保留一半余量，且不能把错误减速比拟合成有效关系。 */
    if (!isfinite(peak_error) || peak_error >= _PI * 0.5f)
    {
        memset(hall->lut_q15, 0, sizeof(hall->lut_q15));
        hall->lut_status = LINEAR_HALL_LUT_MAPPING;
        encoder->params.status = OUTPUT_ENCODER_CAL_RELATION;
        return false;
    }
    encoder->params = (OutputEncoderParams){ratio, phase, 0.0f, direction, OUTPUT_ENCODER_CAL_OK};
    return true;
}
