/* 单轴 OTG 的采样、在线反向、约束、实例独立性及拒绝路径。 */
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <float.h>
#include <string.h>
#include "position_otg.h"

/* 主回归以 4 kHz 运行；换目标场景用毫秒定义，保留原来的物理时刻。 */
enum
{
    TICKS_PER_MS = 4
};
static const float dt = 0.001f / TICKS_PER_MS;
static const PositionOtgLimits limits = {1.0f, 2.0f, 10.0f};

static void set_target(PositionOtg *otg, float target)
{
    PositionOtgReference before = otg->reference;
    assert(PositionOtg_SetTarget(otg, target, limits));
    assert(!memcmp(&before, &otg->reference, sizeof(before)));
}

static void step(PositionOtg *otg)
{
    PositionOtgReference p = otg->reference;
    assert(PositionOtg_Sample(otg, dt));
    PositionOtgReference q = otg->reference;
    assert(fabsf(q.speed_rad_s) <= limits.speed_rad_s);
    assert(fabsf(q.acceleration_rad_s2) <= limits.acceleration_rad_s2);
    assert(fabsf(q.acceleration_rad_s2 - p.acceleration_rad_s2) <= limits.jerk_rad_s3 * dt + 6e-7f);
    /* 积分余项直接受 jerk 上限约束；抓出换段或到达终点时的隐含跳变。 */
    double dv = (double)q.speed_rad_s - p.speed_rad_s - p.acceleration_rad_s2 * dt;
    double dp = (double)q.position_rad - p.position_rad - p.speed_rad_s * dt;
    assert(fabs(dv) <= 0.5 * limits.jerk_rad_s3 * dt * dt + 2e-7);
    assert(fabs(dp) <= 0.5 * limits.acceleration_rad_s2 * dt * dt + 5e-7);
}

static void finish(PositionOtg *otg)
{
    for (unsigned i = 0; i < 30000U && !otg->finished; ++i)
    {
        step(otg);
    }
    assert(otg->finished && otg->reference.position_rad == otg->target_rad);
    assert(otg->reference.speed_rad_s == 0 && otg->reference.acceleration_rad_s2 == 0);
    step(otg);
}

/* 正反方向均检查加速、匀加速、巡航、减速中的反向接续。 */
static void reversal(float dir, unsigned elapsed_ms)
{
    PositionOtg otg;
    assert(PositionOtg_Reset(&otg, (PositionOtgReference){0}));
    set_target(&otg, dir * 3.0f);
    for (unsigned i = 0; i < elapsed_ms * TICKS_PER_MS; ++i)
    {
        step(&otg);
    }
    assert(dir * otg.reference.speed_rad_s > 0);
    if (elapsed_ms == 50U || elapsed_ms == 350U)
    {
        assert(dir * otg.reference.acceleration_rad_s2 > 0.4f);
    }
    if (elapsed_ms == 1000U)
    {
        assert(otg.reference.speed_rad_s == dir && otg.reference.acceleration_rad_s2 == 0);
    }
    if (elapsed_ms == 3250U)
    {
        assert(dir * otg.reference.acceleration_rad_s2 < -0.4f);
    }
    set_target(&otg, -dir);
    step(&otg);
    assert(dir * otg.reference.speed_rad_s > 0);
    finish(&otg);
}

static void check_profiles(void)
{
    PositionOtg otg;
    assert(PositionOtg_Reset(&otg, (PositionOtgReference){0}));
    set_target(&otg, 3);
    for (unsigned i = 0; i < 100U * TICKS_PER_MS; ++i)
    {
        step(&otg);
    }
    assert(fabsf(otg.reference.acceleration_rad_s2 - 1.0f) < 1e-6f);
    assert(fabsf(otg.reference.speed_rad_s - 0.05f) < 1e-6f);
    assert(fabsf(otg.reference.position_rad - 1.0f / 600.0f) < 1e-7f);
    finish(&otg);
    set_target(&otg, 3);
    assert(otg.finished);
    set_target(&otg, 2.999f);
    float peak = 0;
    while (!otg.finished)
    {
        step(&otg);
        peak = fmaxf(peak, fabsf(otg.reference.speed_rad_s));
    }
    assert(peak > 0 && peak < 0.05f);
    set_target(&otg, -1);
    assert(PositionOtg_Sample(&otg, 30) && otg.finished);
    assert(otg.reference.position_rad == -1);
}

static void check_interleaved_instances(void)
{
    PositionOtg a, b, baseline;
    assert(PositionOtg_Reset(&a, (PositionOtgReference){0}));
    assert(PositionOtg_Reset(&b, (PositionOtgReference){1, 0.2f, -0.5f}));
    baseline = a;
    set_target(&a, 2);
    set_target(&baseline, 2);
    for (unsigned i = 0; i < 3000U * TICKS_PER_MS; ++i)
    {
        if (i < 1000U * TICKS_PER_MS)
        {
            set_target(&b, i % 5U ? -0.3f : 1.5f);
        }
        set_target(&a, 2);
        step(&a);
        step(&b);
        step(&baseline);
        assert(!memcmp(&a.reference, &baseline.reference, sizeof(a.reference)));
    }
    assert(a.finished && b.finished);
}

static void check_short_reversal_and_errors(void)
{
    PositionOtg otg = {0};
    assert(!PositionOtg_Sample(&otg, dt));
    assert(PositionOtg_Reset(&otg, (PositionOtgReference){0, 0.8f, 1.0f}));
    set_target(&otg, 0.01f);
    bool passed = false, returned = false;
    for (unsigned i = 0; i < 3000U * TICKS_PER_MS; ++i)
    {
        step(&otg);
        passed |= otg.reference.position_rad > 0.01f;
        returned |= otg.reference.speed_rad_s < 0;
    }
    assert(passed && returned && otg.finished && otg.reference.position_rad == 0.01f);
    assert(PositionOtg_Reset(&otg, (PositionOtgReference){0, 0.99f, 1.0f}));
    assert(!PositionOtg_SetTarget(&otg, 1, limits));
    assert(PositionOtg_Reset(&otg, (PositionOtgReference){0}));
    set_target(&otg, 1);
    step(&otg);
    PositionOtg saved = otg;
    assert(PositionOtg_SetTarget(&otg, saved.target_rad, limits));
    assert(!memcmp(&saved, &otg, sizeof(saved)));
    PositionOtgLimits invalid = limits;
    invalid.acceleration_rad_s2 = INFINITY;
    assert(!PositionOtg_SetTarget(&otg, saved.target_rad, invalid));
    assert(!PositionOtg_SetTarget(&otg, NAN, limits));
    assert(!PositionOtg_SetTarget(&otg, 1, (PositionOtgLimits){1, 2, 0}));
    assert(!PositionOtg_Sample(&otg, -dt));
    assert(!PositionOtg_Reset(&otg, (PositionOtgReference){0, NAN, 0}));
    assert(!memcmp(&saved, &otg, sizeof(saved)));
}

/* 独立按双精度积分整张段表，检查段间接续及两个采样时刻之间的速度极值。 */
static void check_segment_limits(const PositionOtg *otg)
{
    double p = otg->reference.position_rad;
    double v = otg->reference.speed_rad_s;
    double a = otg->reference.acceleration_rad_s2;
    assert(otg->segment_count <= 7);
    for (unsigned i = 0; i < otg->segment_count; ++i)
    {
        const PositionOtgSegment *segment = &otg->segments[i];
        const double t = (double)segment->duration.seconds + segment->duration.remainder_s,
                     j = segment->jerk_rad_s3;
        assert(fabs(p - segment->start.position_rad) <= 5e-7);
        assert(fabs(v - segment->start.speed_rad_s) <= 2e-7);
        assert(fabs(a - segment->start.acceleration_rad_s2) <= 5e-7);
        assert(t > 0 && fabs(j) <= limits.jerk_rad_s3);
        if (j != 0 && -a / j > 0 && -a / j < t)
        {
            assert(fabs(v - a * a / (2 * j)) <= limits.speed_rad_s + 3e-7);
        }
        p += t * (v + t * (0.5 * a + t * j / 6.0));
        v += t * (a + 0.5 * t * j);
        a += t * j;
        assert(fabs(v) <= limits.speed_rad_s + 3e-7);
        assert(fabs(a) <= limits.acceleration_rad_s2 + 5e-7);
    }
    assert(fabs(p - otg->target_rad) < 5e-7 && fabs(v) < 3e-7 && fabs(a) < 5e-7);
}

static float random_signed(uint32_t *seed)
{
    *seed = *seed * 1664525U + 1013904223U;
    return (float)(*seed >> 8) / 8388608.0f - 1.0f;
}

static void check_random_states(void)
{
    PositionOtg otg;
    uint32_t seed = 20261105U;
    for (unsigned i = 0; i < 2000U; ++i)
    {
        float p = random_signed(&seed), v = 0.79f * random_signed(&seed);
        float a = 2.0f * random_signed(&seed);
        assert(PositionOtg_Reset(&otg, (PositionOtgReference){p, v, a}));
        set_target(&otg, 3.0f * random_signed(&seed));
        check_segment_limits(&otg);
        for (unsigned k = 0; k < 5; ++k)
        {
            step(&otg);
            set_target(&otg, 3.0f * random_signed(&seed));
            check_segment_limits(&otg);
        }
    }
}

/* 相同物理时刻比较 2 kHz 一拍与 4 kHz 两拍；使用台架实读的限值和电机侧角度。 */
static void check_sample_period_equivalence(unsigned retarget_ms, float speed_limit)
{
    const float acceleration_limit = 0.785398163f;
    const PositionOtgLimits axis_limits = {
        speed_limit, acceleration_limit, acceleration_limit / 0.1f};
    const float target_rad = 20.943951f; /* 8:1 减速比，输出端 150°。 */
    PositionOtg reference, candidate;
    assert(PositionOtg_Reset(&reference, (PositionOtgReference){0}));
    assert(PositionOtg_Reset(&candidate, (PositionOtgReference){0}));
    float peak_speed = 0.0f;
    for (unsigned tick = 0; tick < 120000U; ++tick)
    {
        float target = retarget_ms && tick >= 2U * retarget_ms ? -target_rad : target_rad;
        assert(PositionOtg_SetTarget(&reference, target, axis_limits));
        assert(PositionOtg_SetTarget(&candidate, target, axis_limits));
        assert(PositionOtg_Sample(&reference, 0.0005f));
        for (unsigned half = 0; half < 2U; ++half)
        {
            float previous_acceleration = candidate.reference.acceleration_rad_s2;
            assert(PositionOtg_Sample(&candidate, dt));
            peak_speed = fmaxf(peak_speed, fabsf(candidate.reference.speed_rad_s));
            assert(fabsf(candidate.reference.speed_rad_s) <= axis_limits.speed_rad_s);
            assert(fabsf(candidate.reference.acceleration_rad_s2) <=
                   axis_limits.acceleration_rad_s2);
            assert(fabsf(candidate.reference.acceleration_rad_s2 - previous_acceleration) <=
                   axis_limits.jerk_rad_s3 * dt + 5e-7f);
        }
        assert(fabsf(reference.reference.position_rad - candidate.reference.position_rad) < 4e-6f);
        assert(fabsf(reference.reference.speed_rad_s - candidate.reference.speed_rad_s) < 2e-6f);
        assert(fabsf(reference.reference.acceleration_rad_s2 -
                     candidate.reference.acceleration_rad_s2) < 3e-6f);
        assert(reference.finished == candidate.finished);
        if (candidate.finished)
        {
            assert(candidate.reference.position_rad == target);
            assert(candidate.reference.speed_rad_s == 0 &&
                   candidate.reference.acceleration_rad_s2 == 0);
            printf("PASS 2/4 kHz same-time trajectory: retarget %u ms, elapsed %.4f s\n",
                   retarget_ms,
                   (tick + 1U) * 0.0005);
            return;
        }
    }
    assert(0 && "axis did not finish within 60 s");
}

/* 同一运动改用不同数量级的角度单位，段时长和采样时刻应保持等价。 */
static void check_unit_scaling(void)
{
    PositionOtg base;
    assert(PositionOtg_Reset(&base, (PositionOtgReference){0, 0.25f, 0.5f}));
    set_target(&base, 3.0f);
    for (int exponent = -100; exponent <= 100; exponent += 200)
    {
        float factor = scalbnf(1.0f, exponent);
        PositionOtg scaled, expected = base;
        PositionOtgLimits scaled_limits = {factor, 2.0f * factor, 10.0f * factor};
        assert(
            PositionOtg_Reset(&scaled, (PositionOtgReference){0, 0.25f * factor, 0.5f * factor}));
        assert(PositionOtg_SetTarget(&scaled, 3.0f * factor, scaled_limits));
        assert(expected.segment_count == scaled.segment_count);
        for (unsigned tick = 0; tick < 20000U && !expected.finished; ++tick)
        {
            assert(PositionOtg_Sample(&expected, dt) && PositionOtg_Sample(&scaled, dt));
            assert(expected.finished == scaled.finished);
            assert(fabsf(expected.reference.position_rad -
                         scalbnf(scaled.reference.position_rad, -exponent)) < 5e-7f);
        }
        assert(expected.finished && scaled.finished);
    }
}

static void check_long_cruise_clock(void)
{
    PositionOtg ticks, jump;
    assert(PositionOtg_Reset(&ticks, (PositionOtgReference){0}));
    set_target(&ticks, 32768.0f);
    assert(PositionOtg_Sample(&ticks, 16384.0f));
    jump = ticks;
    for (unsigned tick = 0; tick < 4000U; ++tick)
    {
        assert(PositionOtg_Sample(&ticks, dt));
    }
    assert(PositionOtg_Sample(&jump, 1.0f));
    assert(ticks.reference.position_rad == jump.reference.position_rad);
    assert(PositionOtg_Sample(&ticks, FLT_MAX) && ticks.finished);
    assert(ticks.reference.position_rad == 32768.0f);
}

/* 回零制动时，坐标换算的末位变化仍是有效目标。对正反两个方向逐拍重建，
 * 检查从原参考接续、有界时间到达新目标及每段 jerk/加速度/速度约束。 */
static void check_retarget_near_stop(float travel_rad, float target_rad, PositionOtgLimits limits)
{
    PositionOtg path;
    assert(PositionOtg_Reset(&path, (PositionOtgReference){travel_rad, 0.0f, 0.0f}));
    assert(PositionOtg_SetTarget(&path, 0.0f, limits));
    unsigned braking_ticks = 0U;
    for (unsigned tick = 0U; tick < 16000U && !path.finished; ++tick)
    {
        assert(PositionOtg_Sample(&path, dt));
        if (path.reference.speed_rad_s * path.reference.acceleration_rad_s2 >= 0.0f)
        {
            continue;
        }
        ++braking_ticks;
        PositionOtg next = path;
        assert(PositionOtg_SetTarget(&next, target_rad, limits));
        assert(memcmp(&next.reference, &path.reference, sizeof(next.reference)) == 0);
        assert(next.segment_count <= 7U);
        double duration = 0.0;
        double integrated_position = path.reference.position_rad;
        for (unsigned i = 0U; i < next.segment_count; ++i)
        {
            const PositionOtgSegment *segment = &next.segments[i];
            double t = (double)segment->duration.seconds + segment->duration.remainder_s;
            double jerk = segment->jerk_rad_s3;
            double accel = segment->start.acceleration_rad_s2;
            double speed = segment->start.speed_rad_s;
            assert(t > 0.0 && fabs(jerk) <= limits.jerk_rad_s3);
            assert(fabs(integrated_position - segment->start.position_rad) <=
                   8.0 * FLT_EPSILON * fabsf(travel_rad));
            integrated_position += t * (speed + t * (0.5 * accel + t * jerk / 6.0));
            assert(fabs(accel + jerk * t) <= limits.acceleration_rad_s2 * (1.0 + 4e-6));
            assert(fabs(speed + t * (accel + 0.5 * jerk * t)) <= limits.speed_rad_s * (1.0 + 4e-6));
            duration += t;
        }
        assert(duration < 1.0);
        assert(fabs(integrated_position - target_rad) <= 8.0 * FLT_EPSILON * fabsf(travel_rad));
        assert(PositionOtg_Sample(&next, 1.0f) && next.finished);
        assert(next.reference.position_rad == target_rad && next.reference.speed_rad_s == 0.0f &&
               next.reference.acceleration_rad_s2 == 0.0f);
    }
    assert(path.finished && braking_ticks > 0U);
}

/* 1 号样机 Yaw 的 Type120 制动记录。十六进制浮点常量精确保留七个 binary32 输入。 */
static void check_recorded_yaw_stop(void)
{
    const PositionOtgReference initial = {-0x1.a77920p+2f, 0x1.2c8006p+4f, -0x1.25bff0p+6f};
    const float target_rad = -0x1.1d9aa2p+2f;
    const PositionOtgLimits yaw_limits = {22.0f, 0x1.4f1a6cp+6f, 0x1.a2e106p+9f};
    PositionOtg otg;
    assert(PositionOtg_Reset(&otg, initial));
    assert(PositionOtg_SetTarget(&otg, target_rad, yaw_limits));
    assert(memcmp(&otg.reference, &initial, sizeof(initial)) == 0);
    assert(PositionOtg_Sample(&otg, 1.0f) && otg.finished);
    assert(otg.reference.position_rad == target_rad && otg.reference.speed_rad_s == 0.0f &&
           otg.reference.acceleration_rad_s2 == 0.0f);
}

int main(void)
{
    check_recorded_yaw_stop();
    const unsigned times[] = {50U, 350U, 1000U, 3250U};
    for (unsigned i = 0; i < sizeof(times) / sizeof(times[0]); ++i)
    {
        reversal(1, times[i]);
        reversal(-1, times[i]);
    }
    check_sample_period_equivalence(0U, 0.785398163f);
    check_sample_period_equivalence(550U, 0.785398163f);
    check_sample_period_equivalence(3500U, 0.785398163f);
    check_sample_period_equivalence(0U, 3.14159265f);
    check_sample_period_equivalence(3500U, 3.14159265f);
    check_unit_scaling();
    check_long_cruise_clock();
    const PositionOtgLimits roll = {0.38f, 3.14159265f, 3.14159265f / 0.1f};
    const float yaw_accel = (6.283185307f / 3.0f) * 40.0f;
    const PositionOtgLimits yaw = {0.55f * 40.0f, yaw_accel, yaw_accel / 0.1f};
    for (int sign = -1; sign <= 1; sign += 2)
    {
        check_retarget_near_stop(sign * 0.262f, sign * 1e-9f, roll);
        check_retarget_near_stop(sign * 1.396f * 40.0f, sign * 4e-8f, yaw);
    }
    check_random_states();
    check_profiles();
    check_interleaved_instances();
    check_short_reversal_and_errors();
    puts("PASS position OTG: signed reversals, p/v/a continuity, v/a/jerk limits, retargeting, "
         "independent instances");
    return 0;
}
