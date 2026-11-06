#include "output_encoder.h"
#include "control_math.h"
#include <math.h>

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
    if (fabsf((float)(cal->motor_counts - cal->first_counts)) >=
        (float)cal->count * (float)cal->step_counts)
    {
        cal->samples[cal->count++] =
            (OutputEncoderCalibSample){cal->motor_counts, sin_counts, cos_counts};
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

bool OutputEncoder_CalibEnd(OutputEncoder *encoder,
                            const OutputEncoderCalib *cal,
                            const LinearHall *hall,
                            const EncoderCalibration *motor)
{
    encoder->ready = false;
    if (encoder->params.gear_ratio < 1.0f ||
        encoder->params.gear_ratio != floorf(encoder->params.gear_ratio))
    {
        encoder->params.status = OUTPUT_ENCODER_CAL_RELATION;
        return false;
    }
    if (cal->count != OUTPUT_ENCODER_CAL_SAMPLES || !hall->calibration_success)
    {
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
    float motor_travel = (float)(cal->samples[OUTPUT_ENCODER_CAL_SAMPLES - 1U].motor_counts -
                                 cal->samples[0].motor_counts) *
                         (_2PI / 65536.0f);
    int32_t direction = travel * motor_travel >= 0.0f ? 1 : -1;
    float ratio = encoder->params.gear_ratio;
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
    if (fabsf(travel) < 6.0f || fabsf(travel - (float)direction * motor_travel / ratio) > 0.15f ||
        !isfinite(peak_error) || peak_error >= _PI * 0.5f)
    {
        encoder->params.status = OUTPUT_ENCODER_CAL_RELATION;
        return false;
    }
    encoder->params = (OutputEncoderParams){ratio, phase, 0.0f, direction, OUTPUT_ENCODER_CAL_OK};
    return true;
}
