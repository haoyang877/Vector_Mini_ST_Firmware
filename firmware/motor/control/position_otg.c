#include "position_otg.h"

#include <float.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

/* 目标平台和原生夹具使用 IEEE binary32。 */
typedef char PositionOtgRequiresBinary32[(sizeof(float) == sizeof(uint32_t) && FLT_RADIX == 2 &&
                                          FLT_MANT_DIG == 24 && FLT_MAX_EXP == 128)
                                             ? 1
                                             : -1];

/* 从当前参考的相对位移开始建表，逐段保留累加丢失的距离。 */
typedef struct
{
    float displacement_rad, speed_rad_s, acceleration_rad_s2, displacement_tail_rad;
} RampState;

/* TwoSum：取本次单精度加法舍入的余量，供后续段继续累计。 */
static float sum_error(float a, float b, float sum)
{
    float b_part = sum - a;
    return (a - (sum - b_part)) + (b - b_part);
}

static PositionOtgTime normalize_time(float seconds, float remainder_s)
{
    float rounded_s = seconds + remainder_s;
    return (PositionOtgTime){rounded_s, sum_error(seconds, remainder_s, rounded_s)};
}

static PositionOtgTime subtract_time(PositionOtgTime left, PositionOtgTime right)
{
    float seconds = left.seconds - right.seconds;
    return normalize_time(seconds,
                          sum_error(left.seconds, -right.seconds, seconds) +
                              (left.remainder_s - right.remainder_s));
}

static void append_segment(
    RampState *state, PositionOtg *plan, float jerk_rad_s3, float duration_s, float duration_tail_s)
{
    PositionOtgTime duration = normalize_time(duration_s, duration_tail_s);
    duration_s = duration.seconds;
    duration_tail_s = duration.remainder_s;
    if (duration_s <= 0.0f)
    {
        return;
    }
    if (plan != NULL)
    {
        PositionOtgSegment *segment = &plan->segments[plan->segment_count++];
        segment->start = (PositionOtgReference){
            (plan->reference.position_rad + state->displacement_rad) + state->displacement_tail_rad,
            state->speed_rad_s,
            state->acceleration_rad_s2};
        segment->jerk_rad_s3 = jerk_rad_s3;
        segment->duration = duration;
    }
    float mean_acceleration = fmaf(0.5f * duration_s, jerk_rad_s3, state->acceleration_rad_s2);
    float end_acceleration = fmaf(duration_s, jerk_rad_s3, state->acceleration_rad_s2);
    float end_speed = fmaf(duration_s, mean_acceleration, state->speed_rad_s);
    float displacement =
        duration_s * fmaf(duration_s,
                          fmaf(duration_s, jerk_rad_s3 / 6.0f, 0.5f * state->acceleration_rad_s2),
                          state->speed_rad_s);
    float displacement_sum = state->displacement_rad + displacement;
    state->displacement_tail_rad +=
        sum_error(state->displacement_rad, displacement, displacement_sum) +
        duration_tail_s * end_speed;
    state->displacement_rad = displacement_sum;
    state->speed_rad_s = fmaf(duration_tail_s, end_acceleration, end_speed);
    state->acceleration_rad_s2 = fmaf(duration_tail_s, jerk_rad_s3, end_acceleration);
}

/* 用至多三段把速度变为 end_speed_rad_s、加速度变为零。先判断消除当前加速度
 * 后的速度，再决定 jerk 方向；因此反向或制动途中均可直接接续。 */
static void
ramp_to_speed(RampState *state, PositionOtg *plan, float end_speed_rad_s, PositionOtgLimits limits)
{
    float jerk_rad_s3 = limits.jerk_rad_s3;
    float start_acceleration = state->acceleration_rad_s2;
    float speed_change = end_speed_rad_s - state->speed_rad_s;
    float speed_change_to_zero_accel =
        start_acceleration * fabsf(start_acceleration) / (2.0f * jerk_rad_s3);
    float direction = speed_change >= speed_change_to_zero_accel ? 1.0f : -1.0f;
    float jerk = direction * jerk_rad_s3;
    /* 两个 jerk 段要求 peak² = jerk * Δv + 起始加速度平方 / 2。保存积与和的舍入余量，
     * 否则后续巡航会把边界上微小的速度误差累积成位置跳变。 */
    float jerk_speed = jerk * speed_change;
    float half_start_sq = 0.5f * start_acceleration * start_acceleration;
    float required_sq = jerk_speed + half_start_sq;
    float required_tail = sum_error(jerk_speed, half_start_sq, required_sq) +
                          fmaf(jerk, speed_change, -jerk_speed) +
                          fmaf(0.5f * start_acceleration, start_acceleration, -half_start_sq) +
                          jerk * sum_error(end_speed_rad_s, -state->speed_rad_s, speed_change);
    float peak_acceleration =
        fminf(limits.acceleration_rad_s2, sqrtf(fmaxf(0.0f, required_sq + required_tail)));
    float peak_sq = peak_acceleration * peak_acceleration;
    float excess = required_sq - peak_sq;
    float excess_tail = sum_error(required_sq, -peak_sq, excess) + required_tail -
                        fmaf(peak_acceleration, peak_acceleration, -peak_sq);
    bool has_plateau =
        peak_acceleration == limits.acceleration_rad_s2 && excess + excess_tail > 0.0f;
    /* 达到加速度上限时余量由恒加速段承担；三角形斜坡则修正开方舍入。 */
    float peak_tail = peak_acceleration > 0.0f && !has_plateau
                          ? (excess + excess_tail) / (2.0f * peak_acceleration)
                          : 0.0f;
    float peak = direction * peak_acceleration;
    float delta = peak - start_acceleration;
    float delta_tail = sum_error(peak, -start_acceleration, delta);
    float rise_s = delta / jerk;
    float rise_tail_s = (fmaf(-rise_s, jerk, delta) + delta_tail) / jerk;
    append_segment(state, plan, jerk, rise_s, rise_tail_s + peak_tail / jerk_rad_s3);
    state->acceleration_rad_s2 = peak + direction * peak_tail;
    if (has_plateau)
    {
        float slope = jerk_rad_s3 * peak_acceleration;
        float hold_s = excess / slope;
        float hold_tail_s = (fmaf(-hold_s, slope, excess) + excess_tail -
                             hold_s * fmaf(jerk_rad_s3, peak_acceleration, -slope)) /
                            slope;
        append_segment(state, plan, 0.0f, hold_s, hold_tail_s);
    }
    float fall_s = peak_acceleration / jerk_rad_s3;
    float fall_tail_s = fmaf(-fall_s, jerk_rad_s3, peak_acceleration) / jerk_rad_s3;
    append_segment(state, plan, -jerk, fall_s, fall_tail_s + peak_tail / jerk_rad_s3);
    state->speed_rad_s = end_speed_rad_s;
    state->acceleration_rad_s2 = 0.0f;
}

static float ramp_displacement(RampState initial_state, float speed_rad_s, PositionOtgLimits limits)
{
    /* 从恒速到静止的速度斜坡关于时间中点对称，面积为半个速度乘持续时间。 */
    float magnitude = fabsf(speed_rad_s);
    float accel = limits.acceleration_rad_s2;
    float jerk_rad_s3 = limits.jerk_rad_s3;
    float duration_s = magnitude * jerk_rad_s3 <= accel * accel
                           ? 2.0f * sqrtf(magnitude / jerk_rad_s3)
                           : magnitude / accel + accel / jerk_rad_s3;
    float braking_displacement_rad = 0.5f * speed_rad_s * duration_s;
    if (initial_state.speed_rad_s == 0.0f && initial_state.acceleration_rad_s2 == 0.0f)
    {
        return 2.0f * braking_displacement_rad;
    }
    ramp_to_speed(&initial_state, NULL, speed_rad_s, limits);
    return (initial_state.displacement_rad + braking_displacement_rad) +
           initial_state.displacement_tail_rad;
}

/* 正 IEEE binary32 的位序就是数值序。对可表示的巡航速度二分最多 31 次，
 * 极短行程也具有相对精度；余量由恒速段精确补齐，无收敛等待。 */
static float
select_cruise_speed(RampState initial_state, float displacement_rad, PositionOtgLimits limits)
{
    float stopping_displacement = ramp_displacement(initial_state, 0.0f, limits);
    if (displacement_rad == stopping_displacement)
    {
        return 0.0f;
    }
    float direction = displacement_rad > stopping_displacement ? 1.0f : -1.0f;
    float speed_at_limit = direction * limits.speed_rad_s;
    if (direction * (displacement_rad - ramp_displacement(initial_state, speed_at_limit, limits)) >=
        0.0f)
    {
        return speed_at_limit;
    }
    uint32_t low_bits = 0U, high_bits;
    memcpy(&high_bits, &limits.speed_rad_s, sizeof(high_bits));
    while (high_bits - low_bits > 1U)
    {
        uint32_t mid_bits = low_bits + (high_bits - low_bits) / 2U;
        float speed_rad_s;
        memcpy(&speed_rad_s, &mid_bits, sizeof(speed_rad_s));
        float remaining =
            direction *
            (displacement_rad - ramp_displacement(initial_state, direction * speed_rad_s, limits));
        if (remaining >= 0.0f)
        {
            /* 已有足够接近的可行速度，剩余距离用巡航补齐，节省重规划计算。 */
            if (remaining <= 0.001f * fabsf(displacement_rad - stopping_displacement))
            {
                return direction * speed_rad_s;
            }
            low_bits = mid_bits;
        }
        else
        {
            high_bits = mid_bits;
        }
    }
    float speed_rad_s;
    memcpy(&speed_rad_s, &low_bits, sizeof(speed_rad_s));
    return direction * speed_rad_s;
}

bool PositionOtg_Reset(PositionOtg *otg, PositionOtgReference initial)
{
    if (otg == NULL || !isfinite(initial.position_rad) || !isfinite(initial.speed_rad_s) ||
        !isfinite(initial.acceleration_rad_s2))
    {
        return false;
    }
    *otg = (PositionOtg){.reference = initial, .initialized = true};
    return true;
}

static bool build_segments(PositionOtg *plan, float target_rad, PositionOtgLimits limits)
{
    RampState ramp = {0.0f, plan->reference.speed_rad_s, plan->reference.acceleration_rad_s2, 0.0f};
    float speed_at_zero_accel_rad_s =
        ramp.speed_rad_s +
        0.5f * (ramp.acceleration_rad_s2 / limits.jerk_rad_s3) * fabsf(ramp.acceleration_rad_s2);
    if (fabsf(ramp.acceleration_rad_s2) > limits.acceleration_rad_s2 ||
        fabsf(ramp.speed_rad_s) > limits.speed_rad_s || !isfinite(speed_at_zero_accel_rad_s) ||
        fabsf(speed_at_zero_accel_rad_s) > limits.speed_rad_s * (1.0f + 4.0f * FLT_EPSILON))
    {
        return false;
    }
    float displacement_rad = target_rad - plan->reference.position_rad;
    if (!isfinite(displacement_rad))
    {
        return false;
    }
    float cruise_speed_rad_s = select_cruise_speed(ramp, displacement_rad, limits);
    ramp_to_speed(&ramp, plan, cruise_speed_rad_s, limits);
    /* 用相同的实际段时长试算刹停终点，剩余距离才是巡航段的位移。 */
    RampState stopped = ramp;
    ramp_to_speed(&stopped, NULL, 0.0f, limits);
    float cruise_displacement_rad =
        (displacement_rad - stopped.displacement_rad) +
        (sum_error(target_rad, -plan->reference.position_rad, displacement_rad) -
         stopped.displacement_tail_rad);
    if (!isfinite(cruise_displacement_rad))
    {
        return false;
    }
    if (cruise_speed_rad_s == 0.0f)
    {
        /* 当前刹停点已舍入到请求的位置时，无需补巡航段。 */
        float stop_position = (plan->reference.position_rad + stopped.displacement_rad) +
                              stopped.displacement_tail_rad;
        return cruise_displacement_rad == 0.0f || stop_position == target_rad;
    }

    float cruise_s = cruise_displacement_rad / cruise_speed_rad_s;
    float cruise_tail_s =
        fmaf(-cruise_s, cruise_speed_rad_s, cruise_displacement_rad) / cruise_speed_rad_s;
    append_segment(&ramp, plan, 0.0f, cruise_s, cruise_tail_s);
    ramp_to_speed(&ramp, plan, 0.0f, limits);
    return true;
}

static PositionOtgReference scale_reference(PositionOtgReference reference, int exponent)
{
    return (PositionOtgReference){scalbnf(reference.position_rad, exponent),
                                  scalbnf(reference.speed_rad_s, exponent),
                                  scalbnf(reference.acceleration_rad_s2, exponent)};
}

bool PositionOtg_SetTarget(PositionOtg *otg, float target_rad, PositionOtgLimits limits)
{
    if (otg == NULL || !otg->initialized)
    {
        return false;
    }
    /* 已接受的目标和限值经过完整校验，普通拍只需比较是否有变化。 */
    if (otg->has_plan && otg->target_rad == target_rad &&
        otg->limits.speed_rad_s == limits.speed_rad_s &&
        otg->limits.acceleration_rad_s2 == limits.acceleration_rad_s2 &&
        otg->limits.jerk_rad_s3 == limits.jerk_rad_s3)
    {
        return true;
    }
    if (!isfinite(target_rad) || !isfinite(limits.speed_rad_s) || limits.speed_rad_s <= 0.0f ||
        !isfinite(limits.acceleration_rad_s2) || limits.acceleration_rad_s2 <= 0.0f ||
        !isfinite(limits.jerk_rad_s3) || limits.jerk_rad_s3 <= 0.0f)
    {
        return false;
    }
    PositionOtg plan = {.reference = otg->reference,
                        .initialized = true,
                        .has_plan = true,
                        .limits = limits,
                        .target_rad = target_rad};
    /* 全部角度量同时乘 2 的整数次幂不改变时间或轨迹形状。只在平方可能
     * 上溢或下溢的量级换单位，普通电机参数直接使用 rad。 */
    int exponent = 0;
    float largest =
        fmaxf(limits.speed_rad_s, fmaxf(limits.acceleration_rad_s2, limits.jerk_rad_s3));
    float scaled_target = target_rad;
    if (largest > 0x1p48f || largest < 0x1p-48f ||
        !isfinite(target_rad - otg->reference.position_rad))
    {
        (void)frexpf(largest, &exponent);
        if (largest > 0x1p48f)
        {
            exponent -= 48;
        }
        else if (largest < 0x1p-48f)
        {
            exponent += 48;
        }
        else
        {
            exponent = 1;
        }
        if (!isfinite(target_rad - otg->reference.position_rad) && exponent < 1)
        {
            exponent = 1;
        }
        scaled_target = scalbnf(target_rad, -exponent);
        plan.reference = scale_reference(plan.reference, -exponent);
        limits.speed_rad_s = scalbnf(limits.speed_rad_s, -exponent);
        limits.acceleration_rad_s2 = scalbnf(limits.acceleration_rad_s2, -exponent);
        limits.jerk_rad_s3 = scalbnf(limits.jerk_rad_s3, -exponent);
        if (!isfinite(scaled_target) || !isfinite(plan.reference.position_rad) ||
            !(limits.speed_rad_s > 0.0f && limits.acceleration_rad_s2 > 0.0f &&
              limits.jerk_rad_s3 > 0.0f))
        {
            return false;
        }
    }
    if (!build_segments(&plan, scaled_target, limits))
    {
        return false;
    }
    for (unsigned i = 0; i < plan.segment_count; ++i)
    {
        PositionOtgSegment *segment = &plan.segments[i];
        if (exponent != 0)
        {
            segment->start = scale_reference(segment->start, exponent);
            segment->jerk_rad_s3 = scalbnf(segment->jerk_rad_s3, exponent);
        }
        if (!isfinite(segment->start.position_rad) || !isfinite(segment->start.speed_rad_s) ||
            !isfinite(segment->start.acceleration_rad_s2) || !isfinite(segment->jerk_rad_s3) ||
            !isfinite(segment->duration.seconds) || !isfinite(segment->duration.remainder_s))
        {
            return false;
        }
    }
    if (plan.segment_count == 0U &&
        (target_rad != otg->reference.position_rad || otg->reference.speed_rad_s != 0.0f ||
         otg->reference.acceleration_rad_s2 != 0.0f))
    {
        return false;
    }
    plan.reference = otg->reference;
    plan.finished = plan.segment_count == 0U;
    *otg = plan;
    return true;
}

bool PositionOtg_Sample(PositionOtg *otg, float dt_s)
{
    if (otg == NULL || !otg->has_plan || !isfinite(dt_s) || dt_s <= 0.0f)
    {
        return false;
    }
    if (otg->finished)
    {
        return true;
    }
    PositionOtgTime step = {dt_s, 0.0f};
    PositionOtgTime elapsed = otg->segment_elapsed;
    unsigned segment_index = otg->segment_index;
    while (segment_index < otg->segment_count)
    {
        /* 先比较本段剩余时间，大步长跨多段时也不需要相加两个接近 FLT_MAX 的时间。 */
        PositionOtgTime remaining = subtract_time(otg->segments[segment_index].duration, elapsed);
        if (step.seconds < remaining.seconds ||
            (step.seconds == remaining.seconds && step.remainder_s < remaining.remainder_s))
        {
            float seconds = elapsed.seconds + step.seconds;
            elapsed = normalize_time(seconds,
                                     sum_error(elapsed.seconds, step.seconds, seconds) +
                                         (elapsed.remainder_s + step.remainder_s));
            break;
        }
        step = subtract_time(step, remaining);
        elapsed = (PositionOtgTime){0};
        ++segment_index;
    }
    PositionOtgReference next = {otg->target_rad, 0.0f, 0.0f};
    bool finished = segment_index == otg->segment_count;
    if (!finished)
    {
        const PositionOtgSegment *segment = &otg->segments[segment_index];
        const PositionOtgReference *start = &segment->start;
        float time_s = elapsed.seconds;
        float delta_accel_rad_s2 = segment->jerk_rad_s3 * time_s;
        next.position_rad =
            start->position_rad +
            time_s * (start->speed_rad_s +
                      time_s * (0.5f * start->acceleration_rad_s2 + delta_accel_rad_s2 / 6.0f));
        next.speed_rad_s =
            start->speed_rad_s + time_s * (start->acceleration_rad_s2 + 0.5f * delta_accel_rad_s2);
        next.acceleration_rad_s2 = start->acceleration_rad_s2 + delta_accel_rad_s2;
        if (!isfinite(next.position_rad) || !isfinite(next.speed_rad_s) ||
            !isfinite(next.acceleration_rad_s2))
        {
            return false;
        }
        /* 解析曲线已限幅，消除单精度段首系数求值产生的末位误差。 */
        next.speed_rad_s =
            fminf(otg->limits.speed_rad_s, fmaxf(-otg->limits.speed_rad_s, next.speed_rad_s));
        next.acceleration_rad_s2 =
            fminf(otg->limits.acceleration_rad_s2,
                  fmaxf(-otg->limits.acceleration_rad_s2, next.acceleration_rad_s2));
    }
    otg->reference = next;
    otg->segment_elapsed = elapsed;
    otg->segment_index = segment_index;
    otg->finished = finished;
    return true;
}
