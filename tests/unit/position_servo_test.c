/* 基础串级离线检查：独立算术期望、限幅/抗饱和、滤波、复位和拒绝不发布。 */
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <float.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "position_cascade.h"
#include "position_cascade_config.h"

static PositionCascadeConfig config(void)
{
    PositionCascadeConfig value = {0};
    value.update_period_s = 0.001f;
    value.call_divider = 1U;
    value.target_position = 0.2f;
    value.position_error_window = 0.001f;
    value.velocity_filter_hz = 0.0f;
    value.maximum_speed = 0.4f;
    value.speed_limit_rad_s = 0.5f;
    value.position_kp = 2.0f;
    value.position_kd = 0.5f;
    value.speed_kp = 0.25f;
    value.speed_ki = 0.5f;
    value.current_limit_a = 4.0f;
    return value;
}

static void test_direct_control(void)
{
    PositionCascadeConfig value = config();
    PositionCascadeControlOutput output;
    PositionCascade_Reset();
    assert(PositionCascade_GetConfiguration() == NULL);
    assert(!PositionCascade_UpdateTargetControl(0.2f, 0, 0, &output));
    assert(PositionCascade_UpdateControl(&value, 0, 0.1f, &output));
    assert(output.position_reference == 0.2f);
    assert(fabsf(output.speed_reference - 0.35f) < 0.000001f);
    assert(fabsf(output.iq_reference - 0.2505f) < 0.000001f);
    assert(!output.target_reached);
    assert(PositionCascade_UpdateTargetControl(0.3f, 0, 0.1f, &output));
    assert(output.position_reference == 0.3f);
    assert(output.speed_reference == 0.5f);
    assert(fabsf(output.iq_reference - 0.4013f) < 0.000001f);
    value.target_position = 0;
    PositionCascade_Reset();
    assert(PositionCascade_UpdateControl(&value, 0, 0.1f, &output));
    assert(fabsf(output.speed_reference + 0.05f) < 0.000001f);
    assert(fabsf(output.iq_reference + 0.1503f) < 0.000001f);
}

static void test_limits_and_anti_windup(void)
{
    PositionCascadeConfig value = config();
    PositionCascadeControlOutput output;
    value.target_position = 10;
    value.speed_kp = 10;
    PositionCascade_Reset();
    for (unsigned index = 0; index < 1000U; ++index)
    {
        assert(PositionCascade_UpdateControl(&value, 0, 0, &output));
        assert(output.speed_reference == 0.5f);
        assert(output.iq_reference == 4);
    }
    assert(PositionCascade_UpdateTargetControl(-10, 0, 0, &output));
    assert(output.speed_reference == -0.5f && output.iq_reference == -4);
    assert(PositionCascade_UpdateTargetControl(0, 0, 0, &output));
    assert(output.iq_reference == 0 && output.target_reached);

    value = config();
    value.speed_limit_rad_s = 2;
    value.target_position = 10;
    PositionCascade_Reset();
    assert(PositionCascade_UpdateControl(&value, 0, 0, &output));
    assert(fabsf(output.speed_reference - 0.6f) < 0.000001f);
}

static void test_filter_and_reset(void)
{
    PositionCascadeConfig value = config();
    PositionCascadeControlOutput output;
    float ratio = 6.2831853072f * 10.0f * value.update_period_s;
    float alpha = ratio / (1.0f + ratio);
    value.velocity_filter_hz = 10;
    PositionCascade_Reset();
    assert(PositionCascade_UpdateControl(&value, 0, 0, &output));
    assert(PositionCascade_UpdateTargetControl(0.2f, 0, 1, &output));
    assert(fabsf(output.speed_feedback_rad_s - alpha) < 0.000001f);
    value.velocity_filter_hz = 0;
    assert(PositionCascade_UpdateControl(&value, 0, 1, &output));
    assert(output.speed_feedback_rad_s == 1);
    PositionCascade_Reset();
    assert(PositionCascade_GetConfiguration() == NULL);
    value.target_position = 0;
    assert(PositionCascade_UpdateControl(&value, 0, 0, &output));
    assert(output.iq_reference == 0 && output.target_reached);
}

static void test_live_limit_and_divider(void)
{
    PositionCascadeConfig value = config();
    PositionCascadeControlOutput output;
    value.update_period_s = 0.1f;
    value.speed_kp = 0;
    value.speed_ki = 1;
    PositionCascade_Reset();
    for (unsigned index = 0; index < 24U; ++index)
    {
        assert(PositionCascade_UpdateControl(&value, 0, 0, &output));
    }
    assert(output.iq_reference > 3);
    value.call_divider = 10;
    value.current_limit_a = 0.25f;
    assert(PositionCascade_UpdateControl(&value, 0, 0, &output));
    assert(output.iq_reference == 0.25f);
    for (unsigned index = 0; index < 9U; ++index)
    {
        assert(PositionCascade_UpdateTargetControl(0, 0, 0, &output));
        assert(output.iq_reference == 0.25f);
    }
    assert(output.speed_reference == 0);
    PositionCascade_Reset();
    value.call_divider = 1;
    value.target_position = 0;
    assert(PositionCascade_UpdateControl(&value, 0, 0, &output));
    assert(output.iq_reference == 0);
}

static void test_invalid_inputs(void)
{
    static const size_t offsets[] = {offsetof(PositionCascadeConfig, update_period_s),
                                     offsetof(PositionCascadeConfig, target_position),
                                     offsetof(PositionCascadeConfig, position_error_window),
                                     offsetof(PositionCascadeConfig, velocity_filter_hz),
                                     offsetof(PositionCascadeConfig, maximum_speed),
                                     offsetof(PositionCascadeConfig, speed_limit_rad_s),
                                     offsetof(PositionCascadeConfig, position_kp),
                                     offsetof(PositionCascadeConfig, position_kd),
                                     offsetof(PositionCascadeConfig, speed_kp),
                                     offsetof(PositionCascadeConfig, speed_ki),
                                     offsetof(PositionCascadeConfig, current_limit_a)};
    PositionCascadeConfig value = config();
    PositionCascadeControlOutput output = {1, 2, 3, 4, true};
    unsigned char before[sizeof(output)];
    memcpy(before, &output, sizeof(output));
    PositionCascade_Reset();
    for (size_t index = 0; index < sizeof(offsets) / sizeof(offsets[0]); ++index)
    {
        PositionCascadeConfig invalid = value;
        float bad = NAN;
        memcpy((unsigned char *)&invalid + offsets[index], &bad, sizeof(bad));
        assert(!PositionCascade_UpdateControl(&invalid, 0, 0, &output));
        assert(memcmp(before, &output, sizeof(output)) == 0);
    }
    value.call_divider = 0;
    assert(!PositionCascade_UpdateControl(&value, 0, 0, &output));
    value = config();
    assert(!PositionCascade_UpdateControl(NULL, 0, 0, &output));
    assert(!PositionCascade_UpdateControl(&value, NAN, 0, &output));
    assert(!PositionCascade_UpdateControl(&value, 0, INFINITY, &output));
    assert(!PositionCascade_UpdateControl(&value, 0, 0, NULL));
    value.speed_kp = FLT_MAX;
    assert(!PositionCascade_UpdateControl(&value, 0, 0, &output));
    value = config();
    assert(PositionCascade_UpdateControl(&value, 0, 0, &output));
    memcpy(before, &output, sizeof(output));
    assert(!PositionCascade_UpdateTargetControl(NAN, 0, 0, &output));
    assert(!PositionCascade_UpdateTargetControl(FLT_MAX, -FLT_MAX, 0, &output));
    assert(memcmp(before, &output, sizeof(output)) == 0);
}

/* 动态额度只改饱和区间，不能改标称电流所定义的实际 PI 增益。 */
static void test_dynamic_current_limit(void)
{
    PositionCascadeConfig value = config();
    PositionCascadeControlOutput output;
    PositionCascade_Reset();
    assert(PositionCascade_UpdateControlLimited(&value, 0, 0.1f, 1.0f, &output));
    assert(fabsf(output.iq_reference - 0.2505f) < 0.000001f);
    assert(PositionCascade_GetConfiguration()->current_limit_a == 4.0f);

    value.update_period_s = 0.1f;
    value.speed_kp = 0;
    value.speed_ki = 1;
    PositionCascade_Reset();
    for (unsigned index = 0; index < 24U; ++index)
    {
        assert(PositionCascade_UpdateControlLimited(&value, 0, 0, 4.0f, &output));
    }
    assert(output.iq_reference > 3);
    value.call_divider = 10;
    assert(PositionCascade_UpdateControlLimited(&value, 0, 0, 0.25f, &output));
    assert(output.iq_reference == 0.25f);
    /* 分频等待拍也立即归零；恢复额度后不释放已撤销的支撑积分。 */
    value.target_position = 0;
    assert(PositionCascade_UpdateControlLimited(&value, 0, 0, 0, &output));
    assert(output.iq_reference == 0);
    for (unsigned index = 0; index < 10U; ++index)
    {
        assert(PositionCascade_UpdateControlLimited(&value, 0, 0, 0.25f, &output));
        assert(output.iq_reference == 0);
    }
    value = config();
    value.target_position = 10;
    value.speed_kp = 10;
    PositionCascade_Reset();
    for (unsigned index = 0; index < 1000U; ++index)
    {
        assert(PositionCascade_UpdateControlLimited(&value, 0, 0, 0.25f, &output));
        assert(output.iq_reference == 0.25f);
    }
    value.target_position = 0;
    assert(PositionCascade_UpdateControlLimited(&value, 0, 0, 0.25f, &output));
    assert(output.iq_reference == 0);
}

static void test_dynamic_limit_rejection_and_legacy_equivalence(void)
{
    PositionCascadeConfig value = config();
    PositionCascadeControlOutput expected[8], output;
    PositionCascade_Reset();
    for (unsigned index = 0; index < 8U; ++index)
    {
        value.target_position = index < 4U ? 0.2f : -0.2f;
        assert(PositionCascade_UpdateControl(&value, 0, 0.1f, &expected[index]));
    }
    PositionCascade_Reset();
    for (unsigned index = 0; index < 8U; ++index)
    {
        value.target_position = index < 4U ? 0.2f : -0.2f;
        assert(PositionCascade_UpdateControlLimited(&value, 0, 0.1f, 4.0f, &output));
        assert(output.position_reference == expected[index].position_reference);
        assert(output.speed_reference == expected[index].speed_reference);
        assert(output.speed_feedback_rad_s == expected[index].speed_feedback_rad_s);
        assert(output.iq_reference == expected[index].iq_reference);
        assert(output.target_reached == expected[index].target_reached);
    }

    /* 非法额度必须保留配置、积分、滤波和分频位置；随后序列与无拒绝路径相等。 */
    value = config();
    value.call_divider = 3;
    value.velocity_filter_hz = 10;
    PositionCascade_Reset();
    for (unsigned index = 0; index < 3U; ++index)
    {
        assert(PositionCascade_UpdateControlLimited(&value, 0, 0.1f, 1.0f, &expected[index]));
    }
    PositionCascade_Reset();
    assert(PositionCascade_UpdateControlLimited(&value, 0, 0.1f, 1.0f, &output));
    PositionCascadeControlOutput before = output;
    PositionCascadeConfig stored = *PositionCascade_GetConfiguration();
    PositionCascadeConfig changed = value;
    changed.target_position = 10;
    const float invalid[] = {-1.0f, NAN, INFINITY};
    for (unsigned index = 0; index < sizeof(invalid) / sizeof(invalid[0]); ++index)
    {
        assert(!PositionCascade_UpdateControlLimited(&changed, 0, 1.0f, invalid[index], &output));
        assert(memcmp(&before, &output, sizeof(output)) == 0);
        assert(memcmp(&stored, PositionCascade_GetConfiguration(), sizeof(stored)) == 0);
    }
    for (unsigned index = 1U; index < 3U; ++index)
    {
        assert(PositionCascade_UpdateControlLimited(&value, 0, 0.1f, 1.0f, &output));
        assert(output.speed_reference == expected[index].speed_reference);
        assert(output.speed_feedback_rad_s == expected[index].speed_feedback_rad_s);
        assert(output.iq_reference == expected[index].iq_reference);
    }
}

int main(void)
{
    test_direct_control();
    test_limits_and_anti_windup();
    test_filter_and_reset();
    test_live_limit_and_divider();
    test_invalid_inputs();
    test_dynamic_current_limit();
    test_dynamic_limit_rejection_and_legacy_equivalence();
    puts("PASS basic cascade: direct target/Kd/PI scaling, limits, anti-windup, filter, reset, "
         "live limit, dynamic budget/nominal gains, invalid-input rejection");
    return 0;
}
