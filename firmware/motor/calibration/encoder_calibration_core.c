#include "encoder_calibration_core.h"

#include <limits.h>

/* 所有可变状态和表均由任务持有；核心只消费显式采样事实并写计算结果。 */

/* 统一采样/验证的稳定准入与正行程；previous 仍由调用者在帧末写入。 */
static bool begin_sample(EncoderCalibrationWindow *window, const EncoderCalibrationSample *sample)
{
    if (!sample->stable)
    {
        return false;
    }
    float observer_delta = sample->observer_position_rad - window->previous_observer_position_rad;
    if (observer_delta > 0.0f)
    {
        window->valid_electrical_travel_rad += observer_delta;
    }
    return sample->relative_theta_rad >= 0.0f;
}

int16_t EncoderCalibrationCore_Q15Difference(uint16_t target_q15, uint16_t source_q15)
{
    return (int16_t)(uint16_t)(target_q15 - source_q15);
}

uint16_t EncoderCalibrationCore_ApplyLut(const int16_t *candidate, uint16_t directed_q15)
{
    uint16_t lut_index = directed_q15 >> 6;
    uint16_t fraction = directed_q15 & 0x003FU;
    int32_t correction_a = candidate[lut_index];
    int32_t correction_b = candidate[(lut_index + 1U) & (ENCODER_CALIBRATION_LUT_SIZE - 1U)];
    int32_t correction = correction_a + (((correction_b - correction_a) * fraction) >> 6);

    return (uint16_t)((int32_t)directed_q15 - correction);
}

void EncoderCalibrationCore_AddAngleSample(uint16_t angle_q15,
                                           uint16_t anchor_q15,
                                           int64_t *sum_q15,
                                           uint32_t *count)
{
    int32_t unwrapped_q15 = (int32_t)anchor_q15 + (int16_t)(uint16_t)(angle_q15 - anchor_q15);
    *sum_q15 += unwrapped_q15;
    (*count)++;
}

bool EncoderCalibrationCore_MeanQ15(int64_t sum_q15, uint32_t count, uint16_t *mean_q15)
{
    if (count == 0U)
    {
        return false;
    }
    *mean_q15 = (uint16_t)(sum_q15 / (int64_t)count);
    return true;
}

void EncoderCalibrationCore_SampleLut(EncoderCalibrationWorkspace *workspace,
                                      EncoderCalibrationWindow *window,
                                      const EncoderCalibrationSample *sample)
{
    if (begin_sample(window, sample))
    {
        uint16_t lut_index = sample->directed_q15 >> (16U - ENCODER_CALIBRATION_LUT_BITS);
        uint16_t reference_q15 =
            (uint16_t)(sample->relative_theta_rad * ((float)ENCODER_CALIBRATION_Q15_CPR /
                                                     sample->required_electrical_theta_rad));
        int32_t correction_q15 =
            EncoderCalibrationCore_Q15Difference(sample->directed_q15, reference_q15);
        if (workspace->bins.samples[lut_index] != 0U)
        {
            int32_t average_q15 =
                workspace->error_sum[lut_index] / (int32_t)workspace->bins.samples[lut_index];
            while (correction_q15 - average_q15 > ENCODER_CALIBRATION_Q15_HALF_TURN)
            {
                correction_q15 -= (int32_t)ENCODER_CALIBRATION_Q15_CPR;
            }
            while (correction_q15 - average_q15 < -ENCODER_CALIBRATION_Q15_HALF_TURN)
            {
                correction_q15 += (int32_t)ENCODER_CALIBRATION_Q15_CPR;
            }
        }
        if (workspace->bins.samples[lut_index] < UINT16_MAX)
        {
            workspace->error_sum[lut_index] += correction_q15;
            workspace->bins.samples[lut_index]++;
        }
    }
    window->previous_observer_position_rad = sample->observer_position_rad;
}

EncoderCalibrationBuildResult
EncoderCalibrationCore_BuildLutStep(EncoderCalibrationWorkspace *workspace,
                                    EncoderCalibrationBuildState *state,
                                    uint16_t bins_per_cycle,
                                    uint16_t minimum_samples)
{
    /* 四遍只共用游标/每拍上界；每个 bin 的数学与写入顺序保持原样。 */
    for (uint16_t bins_processed = 0U; bins_processed < bins_per_cycle &&
                                       state->candidate_lut_index < ENCODER_CALIBRATION_LUT_SIZE;
         ++bins_processed, ++state->candidate_lut_index)
    {
        switch (state->candidate_lut_stage)
        {
        case 0U:
            if (workspace->bins.samples[state->candidate_lut_index] < minimum_samples)
            {
                return ENCODER_CALIBRATION_BUILD_INVALID;
            }
            break;
        case 1U:
        {
            int32_t correction =
                (int16_t)(workspace->error_sum[state->candidate_lut_index] /
                          (int32_t)workspace->bins.samples[state->candidate_lut_index]);
            if (state->candidate_lut_index > 0U)
            {
                while (correction - state->candidate_lut_previous >
                       ENCODER_CALIBRATION_Q15_HALF_TURN)
                {
                    correction -= (int32_t)ENCODER_CALIBRATION_Q15_CPR;
                }
                while (correction - state->candidate_lut_previous <
                       -ENCODER_CALIBRATION_Q15_HALF_TURN)
                {
                    correction += (int32_t)ENCODER_CALIBRATION_Q15_CPR;
                }
            }
            workspace->error_sum[state->candidate_lut_index] = correction;
            state->candidate_lut_previous = correction;
            break;
        }
        case 2U:
            state->candidate_lut_sum += workspace->error_sum[state->candidate_lut_index];
            break;
        default:
        {
            int32_t correction =
                workspace->error_sum[state->candidate_lut_index] - state->candidate_lut_shift;
            if (correction < INT16_MIN || correction > INT16_MAX)
            {
                return ENCODER_CALIBRATION_BUILD_INVALID;
            }
            workspace->bins.candidate[state->candidate_lut_index] = (int16_t)correction;
            break;
        }
        }
    }
    if (state->candidate_lut_index < ENCODER_CALIBRATION_LUT_SIZE)
    {
        return ENCODER_CALIBRATION_BUILD_RUNNING;
    }
    switch (state->candidate_lut_stage)
    {
    case 0U:
        state->candidate_lut_index = 0U;
        state->candidate_lut_stage = 1U;
        break;
    case 1U:
        state->candidate_lut_index = 0U;
        state->candidate_lut_sum = 0;
        state->candidate_lut_stage = 2U;
        break;
    case 2U:
        state->candidate_lut_sum /= (int64_t)ENCODER_CALIBRATION_LUT_SIZE;
        state->candidate_lut_shift = 0;
        while (state->candidate_lut_sum > INT16_MAX)
        {
            state->candidate_lut_sum -= (int32_t)ENCODER_CALIBRATION_Q15_CPR;
            state->candidate_lut_shift += (int32_t)ENCODER_CALIBRATION_Q15_CPR;
        }
        while (state->candidate_lut_sum < INT16_MIN)
        {
            state->candidate_lut_sum += (int32_t)ENCODER_CALIBRATION_Q15_CPR;
            state->candidate_lut_shift -= (int32_t)ENCODER_CALIBRATION_Q15_CPR;
        }
        state->candidate_lut_index = 0U;
        state->candidate_lut_stage = 3U;
        break;
    default:
        return ENCODER_CALIBRATION_BUILD_COMPLETE;
    }
    return ENCODER_CALIBRATION_BUILD_RUNNING;
}

void EncoderCalibrationCore_VerifyLut(EncoderCalibrationVerifyState *state,
                                      const int16_t *candidate,
                                      const EncoderCalibrationSample *sample)
{
    if (begin_sample(&state->window, sample))
    {
        uint16_t reference_q15 =
            (uint16_t)(sample->relative_theta_rad * ((float)ENCODER_CALIBRATION_Q15_CPR /
                                                     sample->required_electrical_theta_rad));
        uint16_t linearized_q15 = EncoderCalibrationCore_ApplyLut(candidate, sample->directed_q15);
        int16_t residual_q15 = EncoderCalibrationCore_Q15Difference(linearized_q15, reference_q15);
        int32_t residual_abs_q15 = residual_q15 >= 0 ? residual_q15 : -(int32_t)residual_q15;
        state->residual_squared_sum += (uint64_t)((int64_t)residual_q15 * (int64_t)residual_q15);
        if ((uint32_t)residual_abs_q15 > state->residual_peak_abs_q15)
        {
            state->residual_peak_abs_q15 = (uint32_t)residual_abs_q15;
        }
        state->residual_sample_count++;
    }
    state->window.previous_observer_position_rad = sample->observer_position_rad;
}

bool EncoderCalibrationCore_ValidateLut(const EncoderCalibrationVerifyState *state,
                                        uint32_t maximum_rms_q15,
                                        uint32_t maximum_peak_q15)
{
    uint64_t max_rms_squared = (uint64_t)maximum_rms_q15 * (uint64_t)maximum_rms_q15;
    return !(
        state->residual_sample_count == 0U || state->residual_peak_abs_q15 > maximum_peak_q15 ||
        state->residual_squared_sum > (uint64_t)state->residual_sample_count * max_rms_squared);
}
