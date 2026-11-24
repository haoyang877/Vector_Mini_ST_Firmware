#include "speed_friction_feedforward.h"

#include <math.h>

/* 17 点分段线性 tanh，最大绝对误差小于 0.006；无超越函数或无界循环。 */
static float friction_tanh(float value)
{
    static const float s_lut[17] = {0.0f,
                                    .2449186624f,
                                    .4621171573f,
                                    .6351489524f,
                                    .7615941560f,
                                    .8482836400f,
                                    .9051482536f,
                                    .9413755385f,
                                    .9640275801f,
                                    .9780261147f,
                                    .9866142982f,
                                    .9918597246f,
                                    .9950547537f,
                                    .9969976355f,
                                    .9981778976f,
                                    .9988944427f,
                                    .9993292997f};
    float magnitude = fabsf(value);
    if (magnitude >= 4.0f)
    {
        return value < 0.0f ? -s_lut[16] : s_lut[16];
    }
    magnitude *= 4.0f;
    unsigned index = (unsigned)magnitude;
    float output = s_lut[index] + (magnitude - (float)index) * (s_lut[index + 1U] - s_lut[index]);
    return value < 0.0f ? -output : output;
}

float SpeedFrictionFeedforward_Update(const SpeedFrictionFeedforwardConfig *config,
                                      float previous_a,
                                      float target_rad_s,
                                      float sample_period_s,
                                      float current_limit_a)
{
    if (!isfinite(previous_a) || !isfinite(target_rad_s) || !isfinite(sample_period_s) ||
        sample_period_s <= 0.0f || sample_period_s > .001f || !isfinite(current_limit_a) ||
        current_limit_a < 0.0f)
    {
        return NAN;
    }
    float bound_a = fminf(config->limit_a, current_limit_a * .5f);
    float raw_a = config->scale *
                  (config->coulomb_a * friction_tanh(target_rad_s / config->zero_width_rad_s) +
                   config->viscous_a_per_rad_s * target_rad_s);
    if (!isfinite(raw_a))
    {
        return NAN;
    }
    raw_a = fminf(fmaxf(raw_a, -bound_a), bound_a);
    /* 与原实现相同的 Pade 近似，默认周期下 alpha 误差小于 3e-8。 */
    float alpha = sample_period_s / (config->time_constant_s + .5f * sample_period_s);
    float step_a = config->slew_a_per_s * sample_period_s;
    float delta_a = fminf(fmaxf(alpha * (raw_a - previous_a), -step_a), step_a);
    return fminf(fmaxf(previous_a + delta_a, -bound_a), bound_a);
}
