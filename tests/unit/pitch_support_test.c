#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "position_cascade.h"
#include "position_cascade_config.h"

#define DEG_TO_RAD 0.0174532925199433f

/* 纯控制器测试：位置与速度由测试注入，不访问硬件。 */
static PositionCascadeConfig_TypeDef support_config(void)
{
    PositionCascadeConfig_TypeDef c;
    memset(&c, 0, sizeof(c));
    c.update_period_s = 0.0005f;
    c.call_divider = 1U;
    c.position_error_window = POSITION_SERVO_DAMPED_ERROR_WINDOW_RAD;
    c.hold_enter_position = POSITION_SERVO_HOLD_ENTER_POSITION_RAD;
    c.hold_exit_position = POSITION_SERVO_HOLD_EXIT_POSITION_RAD;
    c.velocity_filter_hz = 5.0f;
    c.acceleration = 0.3f;
    c.deceleration = 0.3f;
    c.maximum_speed = 0.2f;
    c.speed_limit = 1.0f;
    c.jerk_limit = 1.5f;
    c.position_kp = 10.0f;
    c.position_kd = 1.0f;
    c.speed_kp = 0.06f;
    c.speed_ki = 0.6f;
    c.current_limit = 7.0f;
    c.passive_hold_enabled = true;
    c.friction_breakaway_ratio = 1.0f;
    c.friction_attack_slew_rate = 2.0f;
    c.friction_fast_release_slew_rate = 15.0f;
    c.friction_release_slew_rate = 15.0f;
    c.friction_reference_speed = 0.03f;
    c.friction_stop_speed = 0.02f;
    c.friction_move_speed = 0.05f;
    c.friction_breakaway_distance = 0.003f;
    c.friction_stuck_time = 0.05f;
    c.support_current_limit_a = 0.5f;
    c.support_adjust_rate_a_per_s = 0.5f;
    c.support_release_rate_a_per_s = 2.0f;
    return c;
}

static PositionCascadeOutput_TypeDef step(PositionCascadeConfig_TypeDef *c,
    float position_rad, float speed_rad_s)
{
    PositionCascadeOutput_TypeDef output;
    float available_a = c->current_limit;
    float support_limit_a = c->support_current_limit_a;
    if (c->thermal_limit_active && c->thermal_output_limit_a < available_a)
    {
        available_a = c->thermal_output_limit_a;
    }
    if (available_a < support_limit_a)
    {
        support_limit_a = available_a;
    }
    assert(PositionCascade_Update(c, position_rad, speed_rad_s, &output));
    assert(isfinite(output.iq_reference));
    assert(isfinite(output.support_current_a));
    assert(fabsf(output.iq_reference) <= available_a + 0.00001f);
    assert(fabsf(output.support_current_a) <= support_limit_a + 0.00001f);
    return output;
}

static PositionCascadeTelemetry_TypeDef telemetry(void)
{
    PositionCascadeTelemetry_TypeDef result;
    assert(PositionCascade_GetTelemetry(&result));
    return result;
}

static PositionCascadeOutput_TypeDef hold(PositionCascadeConfig_TypeDef *c,
    float position_rad, unsigned int ticks)
{
    PositionCascadeOutput_TypeDef output = {0};
    unsigned int i;
    for (i = 0U; i < ticks; ++i)
    {
        output = step(c, position_rad, 0.0f);
    }
    return output;
}

static PositionCascadeOutput_TypeDef drift(PositionCascadeConfig_TypeDef *c,
    float start_deg, float end_deg, unsigned int ticks)
{
    PositionCascadeOutput_TypeDef output = {0};
    float speed = (end_deg - start_deg) * DEG_TO_RAD /
        ((float)ticks * c->update_period_s);
    unsigned int i;
    for (i = 1U; i <= ticks; ++i)
    {
        float position = (start_deg + (end_deg - start_deg) *
            (float)i / (float)ticks) * DEG_TO_RAD;
        output = step(c, position, speed);
    }
    return output;
}

static void test_residual_error_and_noise_need_no_support(void)
{
    PositionCascadeConfig_TypeDef c = support_config();
    PositionCascadeOutput_TypeDef output;
    unsigned int i;
    PositionCascade_Reset();
    /* 固定残差允许靠阻尼保持，不能因为不在精确到位窗口而不断积分。 */
    hold(&c, 0.2f * DEG_TO_RAD, 1000U);
    for (i = 0U; i < 8000U; ++i)
    {
        float noise = 0.025f * sinf((float)i * 0.113f);
        output = step(&c, (0.2f + noise) * DEG_TO_RAD, 0.0f);
        assert(output.support_current_a == 0.0f);
        assert(output.hold_current == 0.0f);
        assert(output.iq_reference == 0.0f);
    }
    assert(telemetry().passive_hold);
}

static void test_quantized_capture_boundary_does_not_hold_motion_current(void)
{
    int sign;
    /* 复现端点量化：均值约0.2393°，但高码为0.2502504°，每10ms打断旧raw门限。 */
    for (sign = -1; sign <= 1; sign += 2)
    {
        PositionCascadeConfig_TypeDef c = support_config();
        PositionCascadeOutput_TypeDef output = {0};
        bool captured = false;
        unsigned int i;
        PositionCascade_Reset();
        step(&c, (float)sign * 0.3f * DEG_TO_RAD, 0.0f);
        for (i = 0U; i < 6000U; ++i)
        {
            float quantized_deg = i % 20U < 10U ? 0.2502504472f : 0.2282785824f;
            float measured_speed = 0.0028f * sinf((float)i * 0.01f);
            output = step(&c, (float)sign * quantized_deg * DEG_TO_RAD, measured_speed);
            captured = captured || telemetry().passive_hold;
        }
        assert(captured);
        assert(telemetry().passive_hold);
        assert(output.hold_current == 0.0f);
        assert(output.iq_reference == 0.0f);
        assert(output.support_current_a == 0.0f);
    }
    /* raw余量不能把真实均值超过0.25°的停滞误判为已捕获。 */
    {
        PositionCascadeConfig_TypeDef c = support_config();
        unsigned int i;
        PositionCascade_Reset();
        for (i = 0U; i < 3000U; ++i)
        {
            step(&c, 0.27f * DEG_TO_RAD, 0.0f);
            assert(!telemetry().passive_hold);
        }
    }
}

static void test_drift_builds_opposing_support_then_freezes(void)
{
    int direction;
    for (direction = -1; direction <= 1; direction += 2)
    {
        PositionCascadeConfig_TypeDef c = support_config();
        PositionCascadeOutput_TypeDef output;
        float frozen_a;
        unsigned int i;
        PositionCascade_Reset();
        hold(&c, 0.0f, 600U);
        output = drift(&c, 0.0f, (float)direction * 0.2f, 2000U);
        assert((float)direction * output.support_current_a < -0.001f);
        output = hold(&c, (float)direction * 0.2f * DEG_TO_RAD, 600U);
        frozen_a = output.support_current_a;
        assert(!telemetry().support_adjusting);
        assert((float)direction * frozen_a < -0.001f);
        /* 固定反馈允许一次细调；测试仅验证状态机，不证明真实负载能减至零。 */
        for (i = 0U; i < 6000U; ++i)
        {
            float noise = 0.025f * sinf((float)i * 0.097f);
            output = step(&c, ((float)direction * 0.2f + noise) * DEG_TO_RAD, 0.0f);
            assert(fabsf(output.support_current_a) <= fabsf(frozen_a) + 0.00001f);
            frozen_a = output.support_current_a;
            assert(output.hold_current == 0.0f);
        }
        assert(!telemetry().support_refining);
        assert(output.support_current_a == 0.0f);
        output = hold(&c, (float)direction * 0.2f * DEG_TO_RAD, 1500U);
        assert(output.support_current_a == frozen_a);
    }
}

static void test_support_and_thermal_limits(void)
{
    PositionCascadeConfig_TypeDef c = support_config();
    PositionCascadeOutput_TypeDef output;
    c.support_current_limit_a = 0.05f;
    PositionCascade_Reset();
    hold(&c, 0.0f, 600U);
    output = drift(&c, 0.0f, 0.35f, 4000U);
    assert(output.support_current_a < -0.049f);
    assert(telemetry().support_limited);
    c.thermal_limit_active = true;
    c.thermal_output_limit_a = 0.02f;
    output = hold(&c, 0.35f * DEG_TO_RAD, 1U);
    assert(fabsf(output.support_current_a) <= 0.02001f);
    c.thermal_output_limit_a = 0.0f;
    output = hold(&c, 0.35f * DEG_TO_RAD, 20U);
    assert(output.support_current_a == 0.0f);
    assert(output.iq_reference == 0.0f);
}

static void test_nonzero_support_does_not_block_external_recovery(void)
{
    PositionCascadeConfig_TypeDef c = support_config();
    PositionCascadeOutput_TypeDef output;
    unsigned int i;
    bool recovered = false;
    PositionCascade_Reset();
    hold(&c, 0.0f, 600U);
    drift(&c, 0.0f, 0.3f, 2000U);
    output = hold(&c, 0.3f * DEG_TO_RAD, 1500U);
    assert(output.support_current_a < -0.001f);
    for (i = 0U; i < 1000U; ++i)
    {
        step(&c, 1.5f * DEG_TO_RAD, 0.0f);
        if (!telemetry().passive_hold)
        {
            recovered = true;
            break;
        }
    }
    assert(recovered);
}

static void test_disabled_and_reset(void)
{
    PositionCascadeConfig_TypeDef c = support_config();
    PositionCascadeOutput_TypeDef output;
    PositionCascade_Reset();
    hold(&c, 0.0f, 600U);
    drift(&c, 0.0f, -0.3f, 2000U);
    output = hold(&c, -0.3f * DEG_TO_RAD, 1500U);
    assert(output.support_current_a > 0.001f);
    PositionCascade_Reset();
    output = hold(&c, 0.0f, 600U);
    assert(output.support_current_a == 0.0f);
    c.support_current_limit_a = 0.0f;
    PositionCascade_Reset();
    hold(&c, 0.0f, 600U);
    output = drift(&c, 0.0f, 0.4f, 2000U);
    output = hold(&c, 0.4f * DEG_TO_RAD, 1000U);
    assert(output.support_current_a == 0.0f);
    assert(output.iq_reference == 0.0f);
    assert(!telemetry().support_adjusting);
}

static void test_motion_current_is_not_copied_into_support(void)
{
    PositionCascadeConfig_TypeDef c = support_config();
    PositionCascadeOutput_TypeDef output = {0};
    float peak_a = 0.0f;
    unsigned int i;
    bool docked = false;
    c.target_position = 2.0f * DEG_TO_RAD;
    PositionCascade_Reset();
    for (i = 0U; i < 2000U; ++i)
    {
        output = step(&c, 0.0f, 0.0f);
        if (fabsf(output.iq_reference) > peak_a)
        {
            peak_a = fabsf(output.iq_reference);
        }
    }
    assert(peak_a > 0.01f);
    for (i = 0U; i < 20000U; ++i)
    {
        output = step(&c, c.target_position, 0.0f);
        if (telemetry().passive_hold)
        {
            assert(output.support_current_a == 0.0f);
            docked = true;
            break;
        }
    }
    assert(docked);
    output = hold(&c, c.target_position, 2000U);
    assert(output.support_current_a == 0.0f);
    assert(output.iq_reference == 0.0f);
    assert(output.hold_current == 0.0f);
}

static PositionCascadeOutput_TypeDef enter_hold_with_remaining_current(
    PositionCascadeConfig_TypeDef *c)
{
    PositionCascadeOutput_TypeDef output = {0};
    unsigned int i;
    bool docked = false;
    c->target_position = 2.0f * DEG_TO_RAD;
    /* 慢卸流仅用于把“漂移开始时的实际电流”与从零缓升明确区分。 */
    c->support_release_rate_a_per_s = 0.01f;
    PositionCascade_Reset();
    for (i = 0U; i < 2000U; ++i)
    {
        step(c, 0.0f, 0.0f);
    }
    for (i = 0U; i < 20000U; ++i)
    {
        output = step(c, c->target_position, 0.0f);
        if (telemetry().passive_hold)
        {
            docked = true;
            break;
        }
    }
    assert(docked);
    assert(fabsf(output.iq_reference) > 0.03f);
    assert(output.support_current_a == 0.0f);
    assert(output.hold_current == 0.0f);
    return output;
}

static void test_unloading_retains_current_only_after_opposing_drift(void)
{
    PositionCascadeConfig_TypeDef c = support_config();
    PositionCascadeOutput_TypeDef output = enter_hold_with_remaining_current(&c);
    float initial_a = output.iq_reference;
    float drift_sign = initial_a > 0.0f ? -1.0f : 1.0f;
    float expected_min_a = fminf(fabsf(initial_a) - 0.01f, c.support_current_limit_a);
    output = drift(&c, 2.0f, 2.0f + drift_sign * 0.12f, 400U);
    output = hold(&c, (2.0f + drift_sign * 0.12f) * DEG_TO_RAD, 800U);
    assert(telemetry().passive_hold);
    assert(output.support_current_a * initial_a > 0.0f);
    /* 若仍从零按0.01A/s补起，这0.6s中至多0.006A，无法通过此断言。 */
    assert(fabsf(output.support_current_a) >= expected_min_a - 0.01f);
    assert(fabsf(output.support_current_a) <= c.support_current_limit_a + 0.00001f);
    assert(output.hold_current == 0.0f);
}

static void test_unloading_does_not_keep_current_pushing_with_drift(void)
{
    PositionCascadeConfig_TypeDef c = support_config();
    PositionCascadeOutput_TypeDef output = enter_hold_with_remaining_current(&c);
    float initial_a = output.iq_reference;
    float drift_sign = initial_a > 0.0f ? 1.0f : -1.0f;
    output = drift(&c, 2.0f, 2.0f + drift_sign * 0.12f, 400U);
    output = hold(&c, (2.0f + drift_sign * 0.12f) * DEG_TO_RAD, 800U);
    assert(telemetry().passive_hold);
    assert(fabsf(output.support_current_a) < 0.02f);
    assert(output.support_current_a * initial_a <= 0.0f);
    assert(output.hold_current == 0.0f);
}

static void test_total_error_guard_recovers_before_one_degree(void)
{
    int sign;
    for (sign = -1; sign <= 1; sign += 2)
    {
        PositionCascadeConfig_TypeDef c = support_config();
        PositionCascadeOutput_TypeDef output;
        bool recovered = false;
        unsigned int i;
        PositionCascade_Reset();
        /* 已有0.2°捕获残差不能再叠加任意漂移，0.45°应启动受控回位。 */
        output = hold(&c, (float)sign * 0.2f * DEG_TO_RAD, 1000U);
        assert(telemetry().passive_hold && output.iq_reference == 0.0f);
        for (i = 0U; i < 1000U; ++i)
        {
            output = step(&c, (float)sign * 0.45f * DEG_TO_RAD, 0.0f);
            if (!telemetry().passive_hold)
            {
                recovered = true;
                break;
            }
        }
        assert(recovered);
        assert(output.support_current_a == 0.0f);
    }
}

static void test_initial_capture_leaves_error_budget_for_support(void)
{
    int sign;
    for (sign = -1; sign <= 1; sign += 2)
    {
        PositionCascadeConfig_TypeDef c = support_config();
        PositionCascade_Reset();
        step(&c, (float)sign * 0.3f * DEG_TO_RAD, 0.0f);
        assert(!telemetry().passive_hold);
    }
}

static void test_reverse_drift_reduces_existing_support(void)
{
    PositionCascadeConfig_TypeDef c = support_config();
    PositionCascadeOutput_TypeDef output;
    float previous_a;
    PositionCascade_Reset();
    hold(&c, 0.0f, 600U);
    drift(&c, 0.0f, 0.3f, 2000U);
    output = hold(&c, 0.3f * DEG_TO_RAD, 800U);
    previous_a = output.support_current_a;
    assert(previous_a < -0.1f);
    drift(&c, 0.3f, 0.1f, 600U);
    output = hold(&c, 0.1f * DEG_TO_RAD, 800U);
    assert(output.support_current_a > previous_a + 0.01f);
    assert(fabsf(output.support_current_a) < fabsf(previous_a));
    assert(!telemetry().support_adjusting);
}

static void test_new_target_transfers_applied_current(void)
{
    PositionCascadeConfig_TypeDef c = support_config();
    PositionCascadeOutput_TypeDef output;
    float previous_a;
    PositionCascade_Reset();
    hold(&c, 0.0f, 600U);
    drift(&c, 0.0f, 0.3f, 2000U);
    output = hold(&c, 0.3f * DEG_TO_RAD, 1500U);
    previous_a = output.iq_reference;
    assert(previous_a < -0.1f);
    c.target_position = -0.2f * DEG_TO_RAD;
    output = step(&c, 0.3f * DEG_TO_RAD, 0.0f);
    assert(!telemetry().passive_hold);
    assert(output.support_current_a == 0.0f);
    /* 新运动可以改变正常PI，但第一拍不应把已有实际支撑突变为零。 */
    assert(fabsf(output.iq_reference - previous_a) < 0.1f);
    assert(fabsf(output.iq_reference) > 0.05f);
}

static void test_invalid_support_configuration(void)
{
    const float bad_limits[] = {-1.0f, NAN, INFINITY};
    const float bad_rates[] = {0.0f, -1.0f, NAN, INFINITY, 1001.0f};
    PositionCascadeConfig_TypeDef c;
    PositionCascadeOutput_TypeDef output;
    unsigned int i;
    for (i = 0U; i < sizeof(bad_limits) / sizeof(bad_limits[0]); ++i)
    {
        PositionCascade_Reset();
        c = support_config();
        c.support_current_limit_a = bad_limits[i];
        assert(!PositionCascade_Update(&c, 0.0f, 0.0f, &output));
    }
    for (i = 0U; i < sizeof(bad_rates) / sizeof(bad_rates[0]); ++i)
    {
        PositionCascade_Reset();
        c = support_config();
        c.support_adjust_rate_a_per_s = bad_rates[i];
        assert(!PositionCascade_Update(&c, 0.0f, 0.0f, &output));
        PositionCascade_Reset();
        c = support_config();
        c.support_release_rate_a_per_s = bad_rates[i];
        assert(!PositionCascade_Update(&c, 0.0f, 0.0f, &output));
    }
    /* 关闭候选时，未使用的新斜率字段不改变旧调用方有效性。 */
    PositionCascade_Reset();
    c = support_config();
    c.support_current_limit_a = 0.0f;
    c.support_adjust_rate_a_per_s = NAN;
    c.support_release_rate_a_per_s = NAN;
    assert(PositionCascade_Update(&c, 0.0f, 0.0f, &output));
}

static float support_after_one_second_drift(float period_s)
{
    PositionCascadeConfig_TypeDef c = support_config();
    PositionCascadeOutput_TypeDef output;
    c.update_period_s = period_s;
    PositionCascade_Reset();
    hold(&c, 0.0f, (unsigned int)(0.3f / period_s + 0.5f));
    drift(&c, 0.0f, 0.3f, (unsigned int)(1.0f / period_s + 0.5f));
    output = hold(&c, 0.3f * DEG_TO_RAD, (unsigned int)(0.75f / period_s + 0.5f));
    assert(!telemetry().support_adjusting);
    return output.support_current_a;
}

static void test_support_time_units_are_independent_of_update_frequency(void)
{
    float at_1khz = support_after_one_second_drift(0.001f);
    float at_2khz = support_after_one_second_drift(0.0005f);
    assert(at_1khz < -0.1f && at_2khz < -0.1f);
    assert(fabsf(at_1khz - at_2khz) < 0.01f);
}

/* 返回第一档细调；previous_safe_a仍为刚进入细调前已稳定的粗支撑。 */
static PositionCascadeOutput_TypeDef begin_refinement(PositionCascadeConfig_TypeDef *c,
    float *previous_safe_a)
{
    PositionCascadeOutput_TypeDef output;
    unsigned int i;
    PositionCascade_Reset();
    hold(c, 0.0f, 600U);
    output = drift(c, 0.0f, 0.15f, 2000U);
    for (i = 0U; i < 4000U; ++i)
    {
        *previous_safe_a = output.support_current_a;
        output = step(c, 0.15f * DEG_TO_RAD, 0.0f);
        if (telemetry().support_refining)
        {
            assert(*previous_safe_a < -0.3f);
            assert(fabsf(output.support_current_a -
                (*previous_safe_a + POSITION_SERVO_SUPPORT_REFINE_STEP_A)) < 0.00001f);
            return output;
        }
    }
    assert(0 && "refinement did not start after stable capture");
    return output;
}

static void test_refinement_waits_for_applied_current_and_can_reach_zero(void)
{
    PositionCascadeConfig_TypeDef c = support_config();
    PositionCascadeOutput_TypeDef output;
    float safe_a;
    float target_a;
    unsigned int reached_tick = UINT_MAX;
    unsigned int drops = 1U;
    unsigned int i;
    output = begin_refinement(&c, &safe_a);
    target_a = output.support_current_a;
    for (i = 0U; i < 16000U; ++i)
    {
        output = step(&c, 0.15f * DEG_TO_RAD, 0.0f);
        if (fabsf(output.support_current_a - target_a) > 0.00001f)
        {
            assert(reached_tick != UINT_MAX);
            assert((float)(i - reached_tick) * c.update_period_s >=
                POSITION_SERVO_SUPPORT_REFINE_DWELL_TIME_S - 2.0f * c.update_period_s);
            assert(fabsf(output.support_current_a) < fabsf(target_a));
            target_a = output.support_current_a;
            reached_tick = UINT_MAX;
            ++drops;
        }
        if (reached_tick == UINT_MAX && fabsf(output.iq_reference - target_a) <= 0.001f)
        {
            reached_tick = i;
        }
        if (!telemetry().support_refining)
        {
            break;
        }
    }
    assert(i < 16000U && drops >= 2U);
    assert(output.support_current_a == 0.0f && output.iq_reference == 0.0f);
    output = hold(&c, 0.15f * DEG_TO_RAD, 6000U);
    assert(!telemetry().support_refining && output.support_current_a == 0.0f);
}

static void test_refinement_drift_restores_previous_stable_step_once(void)
{
    PositionCascadeConfig_TypeDef c = support_config();
    PositionCascadeOutput_TypeDef output;
    float initial_safe_a;
    float first_step_a;
    unsigned int i;
    bool second_step = false;
    output = begin_refinement(&c, &initial_safe_a);
    first_step_a = output.support_current_a;
    for (i = 0U; i < 4000U; ++i)
    {
        output = step(&c, 0.15f * DEG_TO_RAD, 0.0f);
        if (fabsf(output.support_current_a - first_step_a) > 0.00001f)
        {
            second_step = true;
            break;
        }
    }
    assert(second_step);
    /* 0.22°仍在捕获范围内，若消耗标志丢失会再次开始细调。 */
    for (i = 0U; i < 2000U; ++i)
    {
        output = step(&c, 0.22f * DEG_TO_RAD, 0.0f);
        if (!telemetry().support_refining)
        {
            break;
        }
    }
    assert(i < 2000U);
    assert(fabsf(output.support_current_a - first_step_a) < 0.00001f);
    for (i = 0U; i < 10000U; ++i)
    {
        output = step(&c, 0.22f * DEG_TO_RAD, 0.0f);
        assert(!telemetry().support_refining);
        assert(fabsf(output.support_current_a - first_step_a) < 0.00001f);
    }
}

static void test_refinement_motion_with_support_never_restores_larger_push(void)
{
    const float displacements_deg[] = {0.07f, 0.04f};
    unsigned int scenario;
    /* 分别触发整次0.06°护栏和单档0.025°护栏，覆盖两个退出入口。 */
    for (scenario = 0U; scenario < sizeof(displacements_deg) / sizeof(displacements_deg[0]); ++scenario)
    {
        PositionCascadeConfig_TypeDef c = support_config();
        PositionCascadeOutput_TypeDef output;
        float previous_safe_a;
        float reduced_a;
        float position_deg;
        unsigned int i;
        output = begin_refinement(&c, &previous_safe_a);
        reduced_a = output.support_current_a;
        assert(previous_safe_a < reduced_a && reduced_a < 0.0f);
        position_deg = 0.15f - displacements_deg[scenario];
        for (i = 0U; i < 2500U; ++i)
        {
            output = step(&c, position_deg * DEG_TO_RAD, 0.0f);
            assert(fabsf(output.support_current_a) <= fabsf(reduced_a) + 0.00001f);
            if (!telemetry().support_refining)
            {
                break;
            }
        }
        assert(i < 2500U);
        assert(fabsf(output.support_current_a - reduced_a) < 0.00001f);
        for (i = 0U; i < 4000U; ++i)
        {
            output = step(&c, position_deg * DEG_TO_RAD, 0.0f);
            assert(!telemetry().support_refining);
            assert(fabsf(output.support_current_a) <= fabsf(reduced_a) + 0.00001f);
        }
    }
}

static void test_refinement_thermal_reduction_does_not_restore_old_current(void)
{
    PositionCascadeConfig_TypeDef c = support_config();
    PositionCascadeOutput_TypeDef output;
    float previous_safe_a;
    begin_refinement(&c, &previous_safe_a);
    c.thermal_limit_active = true;
    c.thermal_output_limit_a = 0.02f;
    output = hold(&c, 0.15f * DEG_TO_RAD, 1U);
    assert(fabsf(output.support_current_a) <= 0.02001f);
    assert(!telemetry().support_refining);
    c.thermal_limit_active = false;
    output = hold(&c, 0.15f * DEG_TO_RAD, 6000U);
    assert(fabsf(output.support_current_a) <= 0.02001f);
    assert(!telemetry().support_refining);
}

static void test_new_target_clears_refinement_and_allows_new_attempt(void)
{
    PositionCascadeConfig_TypeDef c = support_config();
    PositionCascadeOutput_TypeDef output;
    float previous_safe_a;
    unsigned int i;
    bool restarted = false;
    begin_refinement(&c, &previous_safe_a);
    c.target_position = -2.0f * DEG_TO_RAD;
    output = step(&c, 0.15f * DEG_TO_RAD, 0.0f);
    assert(!telemetry().support_refining && output.support_current_a == 0.0f);
    assert(!telemetry().passive_hold);
    hold(&c, c.target_position, 10000U);
    drift(&c, -2.0f, -1.85f, 2000U);
    for (i = 0U; i < 4000U; ++i)
    {
        step(&c, -1.85f * DEG_TO_RAD, 0.0f);
        if (telemetry().support_refining)
        {
            restarted = true;
            break;
        }
    }
    assert(restarted);
}

/* 静止位置是独立证据：测速偏置不能使轴永久滞留在运动电流。 */
static void test_stationary_position_can_capture_with_velocity_noise(void)
{
    const float periods[] = {0.0005f, 0.001f};
    unsigned int rate;
    for (rate = 0U; rate < sizeof(periods) / sizeof(periods[0]); ++rate)
    {
        PositionCascadeConfig_TypeDef c = support_config();
        PositionCascadeOutput_TypeDef output = {0};
        unsigned int i;
        bool captured = false;
        c.update_period_s = periods[rate];
        PositionCascade_Reset();
        for (i = 0U; i < (unsigned int)(4.0f / c.update_period_s); ++i)
        {
            float noise_rad = 0.012f * DEG_TO_RAD * sinf((float)i * 0.1f);
            output = step(&c, 0.2f * DEG_TO_RAD + noise_rad, 0.01f);
            if (telemetry().passive_hold)
            {
                assert((float)i * c.update_period_s >= 0.30f - c.update_period_s);
                captured = true;
                break;
            }
        }
        assert(captured);
        for (i = 0U; i < (unsigned int)(1.0f / c.update_period_s); ++i)
        {
            output = step(&c, 0.2f * DEG_TO_RAD, 0.01f);
        }
        assert(output.iq_reference == 0.0f && output.hold_current == 0.0f);
    }
}

static void test_moving_position_does_not_use_stationary_fallback(void)
{
    PositionCascadeConfig_TypeDef c = support_config();
    unsigned int i;
    PositionCascade_Reset();
    step(&c, 0.24f * DEG_TO_RAD, 0.01f);
    for (i = 1U; i < 900U; ++i)
    {
        float position_deg = 0.24f - (float)i * c.update_period_s;
        step(&c, position_deg * DEG_TO_RAD, 0.01f);
        assert(!telemetry().passive_hold);
    }
}

static void test_hold_proportional_correction_is_bounded_without_integral(void)
{
    int sign;
    for (sign = -1; sign <= 1; sign += 2)
    {
        PositionCascadeConfig_TypeDef c = support_config();
        PositionCascadeOutput_TypeDef output;
        float settled_support;
        unsigned int i;
        c.support_current_limit_a = 3.0f;
        PositionCascade_Reset();
        hold(&c, (float)sign * 0.2f * DEG_TO_RAD, 1000U);
        output = hold(&c, (float)sign * 0.32f * DEG_TO_RAD, 2000U);
        settled_support = output.support_current_a;
        for (i = 0U; i < 10000U; ++i)
        {
            output = step(&c, (float)sign * 0.32f * DEG_TO_RAD, 0.0f);
            assert(output.hold_current == 0.0f);
            assert(fabsf(output.iq_reference) <= 3.00001f);
            assert(fabsf(output.support_current_a - settled_support) < 0.00001f);
            assert(!telemetry().support_refining);
        }
        assert((float)sign * output.feedback_current < -0.20f);
        assert(fabsf(output.feedback_current) < 0.30f);
    }
}

static void test_saturated_outward_drift_recovers_before_normal_guard(void)
{
    int sign;
    for (sign = -1; sign <= 1; sign += 2)
    {
        PositionCascadeConfig_TypeDef c = support_config();
        unsigned int i;
        bool recovered = false;
        c.support_current_limit_a = 0.05f;
        PositionCascade_Reset();
        hold(&c, 0.0f, 600U);
        drift(&c, 0.0f, (float)sign * 0.3f, 1000U);
        for (i = 0U; i < 600U; ++i)
        {
            step(&c, (float)sign * 0.37f * DEG_TO_RAD, 0.0f);
            if (!telemetry().passive_hold)
            {
                recovered = true;
                break;
            }
        }
        assert(recovered);
    }
}

static void test_correction_deadband_noise_does_not_accumulate_current(void)
{
    PositionCascadeConfig_TypeDef c = support_config();
    PositionCascadeOutput_TypeDef output;
    unsigned int i;
    PositionCascade_Reset();
    hold(&c, 0.2f * DEG_TO_RAD, 1000U);
    for (i = 0U; i < 12000U; ++i)
    {
        float position_deg = 0.25f + 0.025f * sinf((float)i * 0.113f);
        output = step(&c, position_deg * DEG_TO_RAD, 0.0f);
        assert(output.hold_current == 0.0f);
        assert(output.support_current_a == 0.0f);
        assert(fabsf(output.iq_reference) < 0.10f);
    }
}

int main(void)
{
    test_correction_deadband_noise_does_not_accumulate_current();
    test_stationary_position_can_capture_with_velocity_noise();
    test_moving_position_does_not_use_stationary_fallback();
    test_hold_proportional_correction_is_bounded_without_integral();
    test_saturated_outward_drift_recovers_before_normal_guard();
    test_residual_error_and_noise_need_no_support();
    test_quantized_capture_boundary_does_not_hold_motion_current();
    test_drift_builds_opposing_support_then_freezes();
    test_support_and_thermal_limits();
    test_nonzero_support_does_not_block_external_recovery();
    test_disabled_and_reset();
    test_motion_current_is_not_copied_into_support();
    test_unloading_retains_current_only_after_opposing_drift();
    test_unloading_does_not_keep_current_pushing_with_drift();
    test_total_error_guard_recovers_before_one_degree();
    test_initial_capture_leaves_error_budget_for_support();
    test_reverse_drift_reduces_existing_support();
    test_new_target_transfers_applied_current();
    test_invalid_support_configuration();
    test_support_time_units_are_independent_of_update_frequency();
    test_refinement_waits_for_applied_current_and_can_reach_zero();
    test_refinement_drift_restores_previous_stable_step_once();
    test_refinement_motion_with_support_never_restores_larger_push();
    test_refinement_thermal_reduction_does_not_restore_old_current();
    test_new_target_clears_refinement_and_allows_new_attempt();
    puts("Pitch support tests passed.");
    return 0;
}
