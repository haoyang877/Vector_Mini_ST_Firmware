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

/* 建表使用相对位移和较宽的中间值；普通拍只用单精度求三次多项式。 */
typedef struct
{
    double displacement_rad, speed_rad_s, acceleration_rad_s2;
} RampState;

static void
append_segment(RampState *state, PositionOtg *plan, double jerk_rad_s3, double duration_s)
{
    if (duration_s <= 0.0)
    {
        return;
    }
    if (plan != NULL)
    {
        PositionOtgSegment *segment = &plan->segments[plan->segment_count++];
        segment->start =
            (PositionOtgReference){(float)(plan->reference.position_rad + state->displacement_rad),
                                   (float)state->speed_rad_s,
                                   (float)state->acceleration_rad_s2};
        segment->jerk_rad_s3 = (float)jerk_rad_s3;
        segment->duration_s = duration_s;
    }
    state->displacement_rad +=
        duration_s * (state->speed_rad_s + duration_s * (0.5 * state->acceleration_rad_s2 +
                                                         duration_s * jerk_rad_s3 / 6.0));
    state->speed_rad_s +=
        duration_s * (state->acceleration_rad_s2 + 0.5 * duration_s * jerk_rad_s3);
    state->acceleration_rad_s2 += duration_s * jerk_rad_s3;
}

/* 用至多三段把速度变为 end_speed_rad_s、加速度变为零。先判断消除当前加速度
 * 后的速度，再决定 jerk 方向；因此反向或制动途中均可直接接续。 */
static void
ramp_to_speed(RampState *state, PositionOtg *plan, double end_speed_rad_s, PositionOtgLimits limits)
{
    double jerk_rad_s3 = limits.jerk_rad_s3;
    double start_acceleration = state->acceleration_rad_s2;
    double speed_change = end_speed_rad_s - state->speed_rad_s;
    double speed_change_to_zero_accel =
        start_acceleration * fabs(start_acceleration) / (2.0 * jerk_rad_s3);
    double direction = speed_change >= speed_change_to_zero_accel ? 1.0 : -1.0;
    double peak_accel_squared = fmax(0.0,
                                     direction * jerk_rad_s3 * speed_change +
                                         0.5 * start_acceleration * start_acceleration);
    double peak_acceleration = fmin(limits.acceleration_rad_s2, sqrt(peak_accel_squared));
    append_segment(state,
                   plan,
                   direction * jerk_rad_s3,
                   (peak_acceleration - direction * start_acceleration) / jerk_rad_s3);
    if (peak_accel_squared > (double)limits.acceleration_rad_s2 * limits.acceleration_rad_s2)
    {
        append_segment(state,
                       plan,
                       0.0,
                       (peak_accel_squared - peak_acceleration * peak_acceleration) /
                           (jerk_rad_s3 * peak_acceleration));
    }
    append_segment(state, plan, -direction * jerk_rad_s3, peak_acceleration / jerk_rad_s3);
    state->speed_rad_s = end_speed_rad_s;
    state->acceleration_rad_s2 = 0.0;
}

static double
ramp_displacement(RampState initial_state, double speed_rad_s, PositionOtgLimits limits)
{
    /* 从恒速到静止的速度斜坡关于时间中点对称，面积为半个速度乘持续时间。 */
    double magnitude = fabs(speed_rad_s);
    double accel = limits.acceleration_rad_s2;
    double jerk_rad_s3 = limits.jerk_rad_s3;
    double duration_s = magnitude * jerk_rad_s3 <= accel * accel
                            ? 2.0 * sqrt(magnitude / jerk_rad_s3)
                            : magnitude / accel + accel / jerk_rad_s3;
    double braking_displacement_rad = 0.5 * speed_rad_s * duration_s;
    if (initial_state.speed_rad_s == 0.0 && initial_state.acceleration_rad_s2 == 0.0)
    {
        return 2.0 * braking_displacement_rad;
    }
    ramp_to_speed(&initial_state, NULL, speed_rad_s, limits);
    return initial_state.displacement_rad + braking_displacement_rad;
}

/* 正 IEEE binary32 的位序就是数值序。对可表示的巡航速度二分最多 31 次，
 * 极短行程也具有相对精度；余量由恒速段精确补齐，无收敛等待。 */
static double
select_cruise_speed(RampState initial_state, double displacement_rad, PositionOtgLimits limits)
{
    double stopping_displacement = ramp_displacement(initial_state, 0.0, limits);
    if (displacement_rad == stopping_displacement)
    {
        return 0.0;
    }
    double direction = displacement_rad > stopping_displacement ? 1.0 : -1.0;
    double speed_at_limit = direction * limits.speed_rad_s;
    if (direction * (displacement_rad - ramp_displacement(initial_state, speed_at_limit, limits)) >=
        0.0)
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
        double remaining =
            direction *
            (displacement_rad - ramp_displacement(initial_state, direction * speed_rad_s, limits));
        if (remaining >= 0.0)
        {
            /* 已有足够接近的可行速度，剩余距离用巡航补齐，节省重规划计算。 */
            if (remaining <= 0.001 * fabs(displacement_rad - stopping_displacement))
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
    RampState ramp = {0.0, otg->reference.speed_rad_s, otg->reference.acceleration_rad_s2};
    double speed_at_zero_accel_rad_s = ramp.speed_rad_s + ramp.acceleration_rad_s2 *
                                                              fabs(ramp.acceleration_rad_s2) /
                                                              (2.0 * (double)limits.jerk_rad_s3);
    if (fabs(ramp.acceleration_rad_s2) > limits.acceleration_rad_s2 ||
        fabs(ramp.speed_rad_s) > limits.speed_rad_s ||
        fabs(speed_at_zero_accel_rad_s) >
            (double)limits.speed_rad_s * (1.0 + 4.0 * (double)FLT_EPSILON))
    {
        return false;
    }
    PositionOtg plan = {.reference = otg->reference,
                        .initialized = true,
                        .has_plan = true,
                        .limits = limits,
                        .target_rad = target_rad};
    double displacement_rad = (double)target_rad - plan.reference.position_rad;
    double cruise_speed_rad_s = select_cruise_speed(ramp, displacement_rad, limits);
    double cruise_displacement_rad =
        displacement_rad - ramp_displacement(ramp, cruise_speed_rad_s, limits);
    if (cruise_speed_rad_s == 0.0 && cruise_displacement_rad != 0.0)
    {
        return false;
    }
    ramp_to_speed(&ramp, &plan, cruise_speed_rad_s, limits);
    if (cruise_speed_rad_s != 0.0)
    {
        append_segment(&ramp, &plan, 0.0, cruise_displacement_rad / cruise_speed_rad_s);
    }
    ramp_to_speed(&ramp, &plan, 0.0, limits);
    for (unsigned i = 0; i < plan.segment_count; ++i)
    {
        if (!isfinite(plan.segments[i].start.position_rad) ||
            !isfinite(plan.segments[i].duration_s) || plan.segments[i].duration_s > FLT_MAX)
        {
            return false;
        }
    }
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
    double elapsed_s = otg->segment_elapsed_s + dt_s;
    unsigned segment_index = otg->segment_index;
    while (segment_index < otg->segment_count &&
           elapsed_s >= otg->segments[segment_index].duration_s)
    {
        elapsed_s -= otg->segments[segment_index++].duration_s;
    }
    PositionOtgReference next = {otg->target_rad, 0.0f, 0.0f};
    bool finished = segment_index == otg->segment_count;
    if (!finished)
    {
        const PositionOtgSegment *segment = &otg->segments[segment_index];
        const PositionOtgReference *start = &segment->start;
        float time_s = (float)elapsed_s;
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
    otg->segment_elapsed_s = elapsed_s;
    otg->segment_index = segment_index;
    otg->finished = finished;
    return true;
}
