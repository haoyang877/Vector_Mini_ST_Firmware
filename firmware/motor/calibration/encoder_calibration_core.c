#include "encoder_calibration_core.h"

#include <limits.h>

/* 所有可变状态和表均由任务持有；核心只消费显式采样事实并写计算结果。 */

/* 两个采样窗口只计合格帧的正向增量；前次位置由各调用者在帧末更新。 */
static void add_positive_travel(EncoderCalibrationWindow *window, float observer_position_rad)
{
    float observer_delta = observer_position_rad - window->previous_observer_position_rad;
    if (observer_delta > 0.0f)
    {
        window->valid_electrical_travel_rad += observer_delta;
    }
}

int16_t EncoderCalibrationCore_Q15Difference(uint16_t target_q15, uint16_t source_q15)
{
    int32_t difference = (int32_t)target_q15 - (int32_t)source_q15;

    if (difference > ENCODER_CALIBRATION_Q15_HALF_TURN)
    {
        difference -= (int32_t)ENCODER_CALIBRATION_Q15_CPR;
    }
    else if (difference < -ENCODER_CALIBRATION_Q15_HALF_TURN)
    {
        difference += (int32_t)ENCODER_CALIBRATION_Q15_CPR;
    }

    return (int16_t)difference;
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

void EncoderCalibrationCore_AddOriginSample(uint16_t directed_q15,
                                            uint16_t *anchor_q15,
                                            int64_t *sum_q15,
                                            uint32_t *count)
{
    int32_t unwrapped_q15;
    if (*count == 0U)
    {
        *anchor_q15 = directed_q15;
    }
    unwrapped_q15 =
        (int32_t)*anchor_q15 + EncoderCalibrationCore_Q15Difference(directed_q15, *anchor_q15);
    *sum_q15 += unwrapped_q15;
    (*count)++;
}

void EncoderCalibrationCore_AddZeroSample(uint16_t linearized_q15,
                                          uint16_t anchor_q15,
                                          int64_t *sum_q15,
                                          uint32_t *count)
{
    int32_t unwrapped_q15 = (int32_t)anchor_q15 + (int16_t)(uint16_t)(linearized_q15 - anchor_q15);
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
    if (sample->stable)
    {
        add_positive_travel(window, sample->observer_position_rad);
        if (sample->relative_theta_rad >= 0.0f)
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
    }
    window->previous_observer_position_rad = sample->observer_position_rad;
}

EncoderCalibrationBuildResult
EncoderCalibrationCore_BuildLutStep(EncoderCalibrationWorkspace *workspace,
                                    EncoderCalibrationBuildState *state,
                                    uint16_t bins_per_cycle,
                                    uint16_t minimum_samples)
{
    if (state->candidate_lut_stage == 0U)
    {
        uint16_t bins_processed;

        for (bins_processed = 0U; bins_processed < bins_per_cycle &&
                                  state->candidate_lut_index < ENCODER_CALIBRATION_LUT_SIZE;
             ++bins_processed, ++state->candidate_lut_index)
        {
            if (workspace->bins.samples[state->candidate_lut_index] < minimum_samples)
            {
                return ENCODER_CALIBRATION_BUILD_INVALID;
            }
        }

        if (state->candidate_lut_index >= ENCODER_CALIBRATION_LUT_SIZE)
        {
            state->candidate_lut_index = 0U;
            state->candidate_lut_stage = 1U;
        }
    }
    else if (state->candidate_lut_stage == 1U)
    {
        uint16_t bins_processed;

        for (bins_processed = 0U; bins_processed < bins_per_cycle &&
                                  state->candidate_lut_index < ENCODER_CALIBRATION_LUT_SIZE;
             ++bins_processed, ++state->candidate_lut_index)
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
        }

        if (state->candidate_lut_index >= ENCODER_CALIBRATION_LUT_SIZE)
        {
            state->candidate_lut_index = 0U;
            state->candidate_lut_sum = 0;
            state->candidate_lut_stage = 2U;
        }
    }
    else if (state->candidate_lut_stage == 2U)
    {
        uint16_t bins_processed;

        for (bins_processed = 0U; bins_processed < bins_per_cycle &&
                                  state->candidate_lut_index < ENCODER_CALIBRATION_LUT_SIZE;
             ++bins_processed, ++state->candidate_lut_index)
        {
            state->candidate_lut_sum += workspace->error_sum[state->candidate_lut_index];
        }

        if (state->candidate_lut_index >= ENCODER_CALIBRATION_LUT_SIZE)
        {
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
        }
    }
    else
    {
        uint16_t bins_processed;

        for (bins_processed = 0U; bins_processed < bins_per_cycle &&
                                  state->candidate_lut_index < ENCODER_CALIBRATION_LUT_SIZE;
             ++bins_processed, ++state->candidate_lut_index)
        {
            int32_t correction =
                workspace->error_sum[state->candidate_lut_index] - state->candidate_lut_shift;

            if (correction < INT16_MIN || correction > INT16_MAX)
            {
                return ENCODER_CALIBRATION_BUILD_INVALID;
            }

            workspace->bins.candidate[state->candidate_lut_index] = (int16_t)correction;
        }

        if (state->candidate_lut_index >= ENCODER_CALIBRATION_LUT_SIZE)
        {
            return ENCODER_CALIBRATION_BUILD_COMPLETE;
        }
    }
    return ENCODER_CALIBRATION_BUILD_RUNNING;
}

void EncoderCalibrationCore_VerifyLut(EncoderCalibrationVerifyState *state,
                                      const int16_t *candidate,
                                      const EncoderCalibrationSample *sample)
{
    if (sample->stable)
    {
        add_positive_travel(&state->window, sample->observer_position_rad);
        if (sample->relative_theta_rad >= 0.0f)
        {
            uint16_t reference_q15 =
                (uint16_t)(sample->relative_theta_rad * ((float)ENCODER_CALIBRATION_Q15_CPR /
                                                         sample->required_electrical_theta_rad));
            uint16_t linearized_q15 =
                EncoderCalibrationCore_ApplyLut(candidate, sample->directed_q15);
            int16_t residual_q15 =
                EncoderCalibrationCore_Q15Difference(linearized_q15, reference_q15);
            int32_t residual_abs_q15 = residual_q15 >= 0 ? residual_q15 : -(int32_t)residual_q15;
            state->residual_squared_sum +=
                (uint64_t)((int64_t)residual_q15 * (int64_t)residual_q15);
            if ((uint32_t)residual_abs_q15 > state->residual_peak_abs_q15)
            {
                state->residual_peak_abs_q15 = (uint32_t)residual_abs_q15;
            }
            state->residual_sample_count++;
        }
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
