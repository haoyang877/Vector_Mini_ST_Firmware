/* 冻结 5665ff87 的纯整形核心；状态由 2 kHz 外环独占，无硬件访问。 */
#include "speed_feedforward.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

void SpeedFeedforward_Default(SpeedFeedforwardConfig *config)
{
    config->enabled = false;
    config->gain_a_per_rad_s2 = 0.028f;
    config->acceleration_limit_rad_s2 = 20.0f;
    config->jerk_limit_rad_s3 = 200.0f;
    config->current_limit_a = 2.0f;
}

bool SpeedFeedforward_IsValid(const SpeedFeedforwardConfig *config)
{
    return config != NULL && isfinite(config->gain_a_per_rad_s2) &&
           config->gain_a_per_rad_s2 >= 0.0f && config->gain_a_per_rad_s2 <= 0.2f &&
           isfinite(config->acceleration_limit_rad_s2) &&
           config->acceleration_limit_rad_s2 >= 0.1f &&
           config->acceleration_limit_rad_s2 <= 100.0f && isfinite(config->jerk_limit_rad_s3) &&
           config->jerk_limit_rad_s3 >= 1.0f && config->jerk_limit_rad_s3 <= 2000.0f &&
           isfinite(config->current_limit_a) && config->current_limit_a >= 0.0f &&
           config->current_limit_a <= 10.0f;
}

void SpeedFeedforward_Reset(SpeedFeedforwardState *state)
{
    memset(state, 0, sizeof(*state));
}

bool SpeedFeedforward_Update(SpeedFeedforwardState *state,
                             const SpeedFeedforwardConfig *config,
                             float target_rad_s,
                             float initial_speed_rad_s,
                             float dt_s,
                             float total_current_limit_a)
{
    SpeedFeedforwardState next;
    float error_rad_s, omega_per_s, jerk_rad_s3, current_limit_a;
    if (state == NULL || !SpeedFeedforward_IsValid(config) || !config->enabled ||
        !isfinite(target_rad_s) || !isfinite(initial_speed_rad_s) || !isfinite(dt_s) ||
        dt_s <= 0.0f || dt_s > 0.001f || !isfinite(total_current_limit_a) ||
        total_current_limit_a <= 0.0f)
    {
        return false;
    }
    next = *state;
    if (!next.initialized)
    {
        SpeedFeedforward_Reset(&next);
        next.speed_rad_s = initial_speed_rad_s;
        next.initialized = true;
    }
    error_rad_s = target_rad_s - next.speed_rad_s;
    /* 临界阻尼整形；时间尺度由 jerk/加速度决定，上限保证离散计算稳定。 */
    omega_per_s =
        fminf(100.0f, config->jerk_limit_rad_s3 / (2.0f * config->acceleration_limit_rad_s2));
    jerk_rad_s3 =
        omega_per_s * omega_per_s * error_rad_s - 2.0f * omega_per_s * next.acceleration_rad_s2;
    if (!isfinite(jerk_rad_s3))
    {
        return false;
    }
    jerk_rad_s3 = fminf(fmaxf(jerk_rad_s3, -config->jerk_limit_rad_s3), config->jerk_limit_rad_s3);
    next.acceleration_rad_s2 = fminf(
        fmaxf(next.acceleration_rad_s2 + jerk_rad_s3 * dt_s, -config->acceleration_limit_rad_s2),
        config->acceleration_limit_rad_s2);
    next.speed_rad_s += next.acceleration_rad_s2 * dt_s;
    if (fabsf(error_rad_s) < 0.000001f &&
        fabsf(next.acceleration_rad_s2) <= config->jerk_limit_rad_s3 * dt_s)
    {
        next.speed_rad_s = target_rad_s;
        next.acceleration_rad_s2 = 0.0f;
    }
    current_limit_a = fminf(config->current_limit_a, 0.5f * total_current_limit_a);
    next.current_a =
        fminf(fmaxf(config->gain_a_per_rad_s2 * next.acceleration_rad_s2, -current_limit_a),
              current_limit_a);
    if (!isfinite(next.speed_rad_s) || !isfinite(next.acceleration_rad_s2) ||
        !isfinite(next.current_a))
    {
        return false;
    }
    *state = next;
    return true;
}
