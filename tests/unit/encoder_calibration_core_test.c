#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

/* 由完整标定夹具包含，直接验证文件内私有计算函数。 */
static EncoderCalibrationWorkspace s_workspace;

static int check_angles(void)
{
    const uint16_t anchors[] = {0U, 1U, 32768U, 65535U};
    for (unsigned anchor = 0U; anchor < sizeof(anchors) / sizeof(anchors[0]); ++anchor)
    {
        for (uint32_t angle = 0U; angle < 65536U; ++angle)
        {
            int16_t expected = (int16_t)(uint16_t)(angle - anchors[anchor]);
            EncoderCalibrationAngleMean mean = {0};
            add_angle(&mean, anchors[anchor]);
            add_angle(&mean, (uint16_t)angle);
            if (delta_q15((uint16_t)angle, anchors[anchor]) != expected ||
                mean.sum_q15 != 2 * (int32_t)anchors[anchor] + expected || mean.count != 2U ||
                mean.anchor_q15 != anchors[anchor])
            {
                return 1;
            }
        }
    }
    EncoderCalibrationAngleMean mean = {0};
    add_angle(&mean, 65530U);
    add_angle(&mean, 10U);
    if (mean.anchor_q15 != 65530U || mean.sum_q15 != 131076 || mean.count != 2U ||
        mean_q15(&mean) != 2U)
    {
        return 3;
    }
    memset(&s_workspace, 0, sizeof(s_workspace));
    s_workspace.bins.candidate[1023] = 64;
    s_workspace.bins.candidate[0] = -64;
    if (apply_lut(s_workspace.bins.candidate, 65535U) != 61U ||
        apply_lut(s_workspace.bins.candidate, 0U) != 64U)
    {
        return 4;
    }
    return 0;
}

static int check_sampling(void)
{
    memset(&s_workspace, 0, sizeof(s_workspace));
    EncoderCalibrationSamplingState state = {
        .start_elec_rad = 1.0f,
        .previous_q15 = 65500U,
        .q15_per_elec_rad = 64.0f,
        .start_q15 = 65500U,
    };
    EncoderCalibrationSample sample = {
        .elec_pos_rad = 2.0f,
        .directed_q15 = 36U,
        .stable = false,
    };
    sample_lut(&s_workspace, &state, &sample);
    sample.stable = true;
    sample.elec_pos_rad = 3.0f;
    sample.directed_q15 = 100U;
    sample_lut(&s_workspace, &state, &sample);
    if (state.motor_travel_q15 != 136 || state.previous_q15 != 100U ||
        s_workspace.bins.samples[1] != 1U || s_workspace.error_sum[1] != 8)
    {
        return 5;
    }

    /* 负相对角及非数值不能转为无符号参考角。 */
    state.start_elec_rad = 5.0f;
    sample.elec_pos_rad = 4.0f;
    sample_lut(&s_workspace, &state, &sample);
    state.start_elec_rad = NAN;
    sample.elec_pos_rad = 4.5f;
    sample_lut(&s_workspace, &state, &sample);
    if (s_workspace.bins.samples[1] != 1U || s_workspace.error_sum[1] != 8 ||
        state.motor_travel_q15 != 136)
    {
        return 6;
    }

    state.start_elec_rad = 1.0f;
    int32_t travel_before_reverse = state.motor_travel_q15;
    sample.directed_q15 = 90U;
    sample.elec_pos_rad = 4.0f;
    s_workspace.bins.samples[1] = UINT16_MAX;
    s_workspace.error_sum[1] = 7;
    sample_lut(&s_workspace, &state, &sample);
    if (state.motor_travel_q15 != travel_before_reverse - 10 ||
        s_workspace.bins.samples[1] != UINT16_MAX || s_workspace.error_sum[1] != 7)
    {
        return 7;
    }

    /* 非零起点跨越多个整圈；参考角先保留整圈位移再落入16位环。 */
    memset(&s_workspace, 0, sizeof(s_workspace));
    state = (EncoderCalibrationSamplingState){
        .start_elec_rad = 3.0f,
        .previous_q15 = 65530U,
        .q15_per_elec_rad = 1024.0f,
        .start_q15 = 65530U,
    };
    const uint32_t travel_q15[] = {0U, 8U, 65536U, 65544U, 131080U};
    for (unsigned index = 0U; index < sizeof(travel_q15) / sizeof(travel_q15[0]); ++index)
    {
        sample.elec_pos_rad = 3.0f + (float)travel_q15[index] / 1024.0f;
        sample.directed_q15 = (uint16_t)(65530U + travel_q15[index] + 16U);
        sample_lut(&s_workspace, &state, &sample);
    }
    if (s_workspace.bins.samples[0] != 5U || s_workspace.error_sum[0] != 80)
    {
        return 8;
    }
    return 0;
}

static void prepare_constant(int32_t correction)
{
    memset(&s_workspace, 0, sizeof(s_workspace));
    for (unsigned bin = 0U; bin < ENCODER_OFFSET_LUT_SIZE; ++bin)
    {
        uint16_t samples = (uint16_t)(1U + bin % 17U);
        s_workspace.error_sum[bin] = correction * samples;
        s_workspace.bins.samples[bin] = samples;
    }
}

static int check_build(void)
{
    prepare_constant(-500);
    EncoderCalibrationBuildState build = {0};
    const unsigned ticks_per_pass =
        ENCODER_OFFSET_LUT_SIZE / SENSORLESS_ENCODER_CALIB_LUT_BUILD_BINS_PER_CYCLE;
    for (unsigned tick = 0U; tick < 2U * ticks_per_pass; ++tick)
    {
        uint16_t previous_bin = build.bin;
        uint8_t previous_pass = build.pass;
        EncoderCalibrationBuildResult result = build_bins(&s_workspace, &build);
        if (result != (tick + 1U == 2U * ticks_per_pass ? ENCODER_CALIBRATION_BUILD_COMPLETE
                                                        : ENCODER_CALIBRATION_BUILD_RUNNING) ||
            (build.pass == previous_pass &&
             build.bin - previous_bin != SENSORLESS_ENCODER_CALIB_LUT_BUILD_BINS_PER_CYCLE) ||
            (build.pass != previous_pass && (build.pass != previous_pass + 1U || build.bin != 0U)))
        {
            return 9;
        }
    }
    for (unsigned bin = 0U; bin < ENCODER_OFFSET_LUT_SIZE; ++bin)
    {
        if (s_workspace.bins.candidate[bin] != -500)
        {
            return 10;
        }
    }
    prepare_constant(0);
    build = (EncoderCalibrationBuildState){0};
    s_workspace.bins.samples[7] = 0U;
    if (build_bins(&s_workspace, &build) != ENCODER_CALIBRATION_BUILD_INVALID || build.bin != 7U)
    {
        return 11;
    }

    /* 整体移位无法容纳跨越int16边界的曲线时，不发布截断后的跳变。 */
    prepare_constant(-32760);
    s_workspace.error_sum[0] = 32760;
    build = (EncoderCalibrationBuildState){0};
    for (unsigned tick = 0U; tick < ticks_per_pass; ++tick)
    {
        if (build_bins(&s_workspace, &build) != ENCODER_CALIBRATION_BUILD_RUNNING)
        {
            return 12;
        }
    }
    if (build_bins(&s_workspace, &build) != ENCODER_CALIBRATION_BUILD_INVALID || build.pass != 1U ||
        build.bin != 0U)
    {
        return 13;
    }
    return 0;
}

static int check_calibration_core(void)
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
    if (result != 0)
    {
        printf("core FAIL %d\n", result);
        return result;
    }
    puts("core PASS: ring means, wrapped references, saturation, two-pass bounds and build "
         "failures");
    return 0;
}
