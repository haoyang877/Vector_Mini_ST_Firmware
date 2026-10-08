#include <limits.h>
#include <stdio.h>
#include <string.h>

#include "encoder_calibration_core.h"

/* 独立核心夹具仅用标定核心目录，不借助 motor/FOC/platform 的头文件或桩。 */
static EncoderCalibrationWorkspace s_workspace;

static int check_angles(void)
{
    const uint16_t anchors[] = {0U, 1U, 32768U, 65535U};
    for (unsigned anchor = 0U; anchor < sizeof(anchors) / sizeof(anchors[0]); ++anchor)
    {
        for (uint32_t angle = 0U; angle < 65536U; ++angle)
        {
            int16_t expected = (int16_t)(uint16_t)(angle - anchors[anchor]);
            if (EncoderCalibrationCore_Q15Difference((uint16_t)angle, anchors[anchor]) != expected)
            {
                return 1;
            }
        }
    }
    uint16_t anchor = 0U, mean = 123U;
    uint32_t count = 0U;
    int64_t sum = 0;
    if (EncoderCalibrationCore_MeanQ15(sum, count, &mean) || mean != 123U)
    {
        return 2;
    }
    EncoderCalibrationCore_AddOriginSample(65530U, &anchor, &sum, &count);
    EncoderCalibrationCore_AddOriginSample(10U, &anchor, &sum, &count);
    if (anchor != 65530U || sum != 131076 || count != 2U ||
        !EncoderCalibrationCore_MeanQ15(sum, count, &mean) || mean != 2U)
    {
        return 3;
    }
    sum = 0;
    count = 0U;
    EncoderCalibrationCore_AddZeroSample(65530U, anchor, &sum, &count);
    EncoderCalibrationCore_AddZeroSample(10U, anchor, &sum, &count);
    if (sum != 131076 || count != 2U || !EncoderCalibrationCore_MeanQ15(sum, count, &mean) ||
        mean != 2U)
    {
        return 4;
    }
    memset(&s_workspace, 0, sizeof(s_workspace));
    s_workspace.bins.candidate[1023] = 64;
    s_workspace.bins.candidate[0] = -64;
    if (EncoderCalibrationCore_ApplyLut(s_workspace.bins.candidate, 65535U) != 61U ||
        EncoderCalibrationCore_ApplyLut(s_workspace.bins.candidate, 0U) != 64U)
    {
        return 5;
    }
    return 0;
}

static int check_sampling(void)
{
    memset(&s_workspace, 0, sizeof(s_workspace));
    EncoderCalibrationWindow window = {0};
    EncoderCalibrationSample sample = {
        .observer_position_rad = 2.0f,
        .relative_theta_rad = 0.0f,
        .required_electrical_theta_rad = 100.0f,
        .directed_q15 = 0U,
        .stable = false,
    };
    EncoderCalibrationCore_SampleLut(&s_workspace, &window, &sample);
    sample.stable = true;
    sample.observer_position_rad = 3.0f;
    EncoderCalibrationCore_SampleLut(&s_workspace, &window, &sample);
    if (window.valid_electrical_travel_rad != 1.0f ||
        window.previous_observer_position_rad != 3.0f || s_workspace.bins.samples[0] != 1U)
    {
        return 6;
    }
    sample.relative_theta_rad = -1.0f;
    sample.observer_position_rad = 4.0f;
    EncoderCalibrationCore_SampleLut(&s_workspace, &window, &sample);
    if (window.valid_electrical_travel_rad != 2.0f || s_workspace.bins.samples[0] != 1U)
    {
        return 7;
    }
    sample.relative_theta_rad = 0.0f;
    sample.observer_position_rad = 3.5f;
    s_workspace.bins.samples[0] = UINT16_MAX;
    s_workspace.error_sum[0] = 7;
    EncoderCalibrationCore_SampleLut(&s_workspace, &window, &sample);
    if (window.valid_electrical_travel_rad != 2.0f || s_workspace.bins.samples[0] != UINT16_MAX ||
        s_workspace.error_sum[0] != 7)
    {
        return 8;
    }
    return 0;
}

static void prepare_constant(int32_t correction)
{
    memset(&s_workspace, 0, sizeof(s_workspace));
    for (unsigned bin = 0U; bin < ENCODER_CALIBRATION_LUT_SIZE; ++bin)
    {
        s_workspace.error_sum[bin] = correction * 16;
        s_workspace.bins.samples[bin] = 16U;
    }
}

static int check_build(void)
{
    prepare_constant(-500);
    EncoderCalibrationBuildState build = {0};
    for (unsigned tick = 0U; tick < 512U; ++tick)
    {
        uint16_t previous_index = build.candidate_lut_index;
        uint8_t previous_stage = build.candidate_lut_stage;
        EncoderCalibrationBuildResult result =
            EncoderCalibrationCore_BuildLutStep(&s_workspace, &build, 8U, 16U);
        if (result != (tick == 511U ? ENCODER_CALIBRATION_BUILD_COMPLETE
                                    : ENCODER_CALIBRATION_BUILD_RUNNING) ||
            (build.candidate_lut_stage == previous_stage &&
             build.candidate_lut_index - previous_index != 8U) ||
            (build.candidate_lut_stage != previous_stage &&
             (build.candidate_lut_stage != previous_stage + 1U || build.candidate_lut_index != 0U)))
        {
            return 9;
        }
    }
    for (unsigned bin = 0U; bin < ENCODER_CALIBRATION_LUT_SIZE; ++bin)
    {
        if (s_workspace.bins.candidate[bin] != -500)
        {
            return 10;
        }
    }
    prepare_constant(0);
    build = (EncoderCalibrationBuildState){0};
    s_workspace.bins.samples[7] = 15U;
    if (EncoderCalibrationCore_BuildLutStep(&s_workspace, &build, 8U, 16U) !=
            ENCODER_CALIBRATION_BUILD_INVALID ||
        build.candidate_lut_index != 7U)
    {
        return 11;
    }
    prepare_constant(-32760);
    s_workspace.error_sum[0] = 32760 * 16;
    build = (EncoderCalibrationBuildState){0};
    for (unsigned tick = 0U; tick < 384U; ++tick)
    {
        if (EncoderCalibrationCore_BuildLutStep(&s_workspace, &build, 8U, 16U) !=
            ENCODER_CALIBRATION_BUILD_RUNNING)
        {
            return 12;
        }
    }
    if (EncoderCalibrationCore_BuildLutStep(&s_workspace, &build, 8U, 16U) !=
            ENCODER_CALIBRATION_BUILD_INVALID ||
        build.candidate_lut_stage != 3U || build.candidate_lut_index != 0U)
    {
        return 13;
    }
    return 0;
}

static int check_verification(void)
{
    memset(&s_workspace, 0, sizeof(s_workspace));
    EncoderCalibrationVerifyState verify = {0};
    EncoderCalibrationSample sample = {
        .observer_position_rad = 1.0f,
        .relative_theta_rad = 0.0f,
        .required_electrical_theta_rad = 100.0f,
        .directed_q15 = 1024U,
        .stable = true,
    };
    if (EncoderCalibrationCore_ValidateLut(&verify, 256U, 1024U))
    {
        return 14;
    }
    EncoderCalibrationCore_VerifyLut(&verify, s_workspace.bins.candidate, &sample);
    if (verify.residual_sample_count != 1U || verify.residual_squared_sum != 1048576U ||
        verify.residual_peak_abs_q15 != 1024U ||
        EncoderCalibrationCore_ValidateLut(&verify, 256U, 1024U) ||
        !EncoderCalibrationCore_ValidateLut(&verify, 1024U, 1024U) ||
        EncoderCalibrationCore_ValidateLut(&verify, 1024U, 1023U))
    {
        return 15;
    }
    sample.stable = false;
    sample.observer_position_rad = 2.0f;
    EncoderCalibrationCore_VerifyLut(&verify, s_workspace.bins.candidate, &sample);
    if (verify.residual_sample_count != 1U || verify.window.valid_electrical_travel_rad != 1.0f ||
        verify.window.previous_observer_position_rad != 2.0f)
    {
        return 16;
    }
    return 0;
}

int main(void)
{
    int result = check_angles();
    if (result == 0)
    {
        result = check_sampling();
    }
    if (result == 0)
    {
        result = check_build();
    }
    if (result == 0)
    {
        result = check_verification();
    }
    if (result != 0)
    {
        printf("core FAIL %d\n", result);
        return result;
    }
    puts("core PASS: 262144 ring differences, means, saturation, 8-bin bounds, build/verify "
         "failures");
    return 0;
}
