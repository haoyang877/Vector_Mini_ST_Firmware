/** @file
 * @brief 固定窗口热计算；无分配、无阻塞、不推断绝对绕组温度。
 */
#include "motor_thermal.h"
#include <stddef.h>
#include <string.h>

bool MotorThermal_ConfigValid(const MotorThermalConfig *c)
{
    /* 所有浮点量都有有限上下界；有序比较也会拒绝NaN及Inf。 */
    return c != NULL && c->continuous_vector_a >= 0.01f &&
        c->continuous_phase_rms_a >= 0.001f &&
        c->peak_a >= c->continuous_vector_a && c->peak_a <= 100.0f &&
        c->continuous_phase_rms_a <= c->peak_a &&
        c->heating_tau_s >= 0.1f && c->heating_tau_s <= 86400.0f &&
        c->cooling_tau_s >= c->heating_tau_s && c->cooling_tau_s <= 86400.0f &&
        c->update_period_s > 0.0f && c->update_period_s <= 0.001f &&
        c->samples_per_update > 0U && c->samples_per_update <= 1000U &&
        c->clear_heat >= 0.0f && c->clear_heat < c->derating_exit_heat &&
        c->derating_exit_heat < c->derating_enter_heat &&
        c->derating_enter_heat < c->trip_heat && c->trip_heat <= 1.0f;
}

void MotorThermal_Invalidate(MotorThermalState *state)
{
    if (state != NULL)
    {
        state->valid = false;
        state->fault_latched = true;
        state->available_a = 0.0f;
    }
}

static void update_limit(MotorThermalState *state)
{
    float sustainable_a;
    float weight;
    if (!state->valid || state->fault_latched)
    {
        state->available_a = 0.0f;
        return;
    }
    if (state->max_heat >= state->config.derating_enter_heat)
    {
        state->derating = true;
    }
    else if (state->max_heat <= state->config.derating_exit_heat)
    {
        state->derating = false;
    }
    if (!state->derating)
    {
        state->available_a = state->config.peak_a;
        return;
    }
    /* 逐相热点主导时采用相RMS作为保守向量边界，不依赖未知电角度。 */
    sustainable_a = state->config.continuous_vector_a;
    if (state->heat[1] >= state->heat[0] || state->heat[2] >= state->heat[0] ||
        state->heat[3] >= state->heat[0])
    {
        /* 配置已校验为有限正值，无需通用 fminf 的分类和库调用。 */
        if (state->config.continuous_phase_rms_a < sustainable_a)
        {
            sustainable_a = state->config.continuous_phase_rms_a;
        }
    }
    weight = (state->max_heat - state->config.derating_enter_heat) /
        (state->config.trip_heat - state->config.derating_enter_heat);
    /* NaN 与旧 fmaxf(0, NaN) 一样取0；保留零值符号及两端饱和。 */
    if (weight < 0.0f || weight != weight)
    {
        weight = 0.0f;
    }
    else if (weight > 1.0f)
    {
        weight = 1.0f;
    }
    state->available_a = state->config.peak_a + weight * (sustainable_a - state->config.peak_a);
}

bool MotorThermal_Reconfigure(MotorThermalState *state,
                              const MotorThermalConfig *config,
                              float heating_alpha,
                              float cooling_alpha)
{
    float heat[4];
    float max_heat = 0.0f;
    unsigned i;
    if (state == NULL || !state->valid || !MotorThermal_ConfigValid(config) ||
        config->samples_per_update != state->config.samples_per_update ||
        config->update_period_s != state->config.update_period_s || !(heating_alpha > 0.0f && heating_alpha <= 1.0f &&
        cooling_alpha > 0.0f && cooling_alpha <= heating_alpha))
    {
        return false;
    }
    for (i = 0U; i < 4U; ++i)
    {
        float old_a =
            i == 0U ? state->config.continuous_vector_a : state->config.continuous_phase_rms_a;
        float new_a = i == 0U ? config->continuous_vector_a : config->continuous_phase_rms_a;
        float scale = fmaxf(1.0f, (old_a * old_a) / (new_a * new_a));
        heat[i] = state->heat[i] * scale;
        if (!isfinite(heat[i]) || heat[i] < 0.0f)
        {
            return false;
        }
        max_heat = fmaxf(max_heat, heat[i]);
    }
    state->config = *config;
    state->heating_alpha = heating_alpha;
    state->cooling_alpha = cooling_alpha;
    memcpy(state->heat, heat, sizeof(heat));
    state->max_heat = max_heat;
    state->fault_latched = state->fault_latched || max_heat >= config->trip_heat;
    update_limit(state);
    return true;
}

bool MotorThermal_Init(MotorThermalState *state,
                       const MotorThermalConfig *config,
                       float initial_heat)
{
    unsigned i;
    if (state == NULL)
    {
        return false;
    }
    memset(state, 0, sizeof(*state));
    if (!MotorThermal_ConfigValid(config) || !isfinite(initial_heat) || initial_heat < 0.0f)
    {
        MotorThermal_Invalidate(state);
        return false;
    }
    state->config = *config;
    state->heating_alpha = MotorThermal_Alpha(config->update_period_s, config->heating_tau_s);
    state->cooling_alpha = MotorThermal_Alpha(config->update_period_s, config->cooling_tau_s);
    for (i = 0U; i < 4U; ++i)
    {
        state->heat[i] = initial_heat;
    }
    state->max_heat = initial_heat;
    state->valid = true;
    state->fault_latched = initial_heat >= config->trip_heat;
    update_limit(state);
    return true;
}

bool MotorThermal_Accumulate(MotorThermalState *state, float ia_a, float ib_a, float ic_a)
{
    if (state == NULL || !state->valid)
    {
        return false;
    }
    if (state->sample_count >= state->config.samples_per_update)
    {
        MotorThermal_Invalidate(state);
        return false;
    }
    if (!(fabsf(ia_a) <= 100.0f))
    {
        MotorThermal_Invalidate(state);
        return false;
    }
    state->square_sum_a2[0] += ia_a * ia_a;
    if (!(fabsf(ib_a) <= 100.0f))
    {
        MotorThermal_Invalidate(state);
        return false;
    }
    state->square_sum_a2[1] += ib_a * ib_a;
    if (!(fabsf(ic_a) <= 100.0f))
    {
        MotorThermal_Invalidate(state);
        return false;
    }
    state->square_sum_a2[2] += ic_a * ic_a;
    ++state->sample_count;
    return true;
}

/* 四次固定通道调用在 ARMCC -O3 下内联，避免 q 数组落栈；异常保留已更新通道。 */
static inline bool update_heat(MotorThermalState *state, unsigned channel, float q, float *max_heat)
{
    float heat = state->heat[channel];
    float alpha = q >= heat ? state->heating_alpha : state->cooling_alpha;
    heat += alpha * (q - heat);
    state->heat[channel] = heat;
    if (!isfinite(heat) || heat < 0.0f)
    {
        return false;
    }
    if (heat > *max_heat)
    {
        *max_heat = heat;
    }
    return true;
}

bool MotorThermal_Update(MotorThermalState *state)
{
    float q0, q1, q2, q3;
    float phase_denominator;
    float samples;
    float max_heat = 0.0f;
    if (state == NULL || !state->valid || state->sample_count != state->config.samples_per_update)
    {
        MotorThermal_Invalidate(state);
        return false;
    }
    samples = (float)state->sample_count;
    phase_denominator =
        samples * state->config.continuous_phase_rms_a * state->config.continuous_phase_rms_a;
    q0 = (2.0f / 3.0f) *
           (state->square_sum_a2[0] + state->square_sum_a2[1] + state->square_sum_a2[2]) /
           (samples * state->config.continuous_vector_a * state->config.continuous_vector_a);
    q1 = state->square_sum_a2[0] / phase_denominator;
    q2 = state->square_sum_a2[1] / phase_denominator;
    q3 = state->square_sum_a2[2] / phase_denominator;
    if (!update_heat(state, 0U, q0, &max_heat) ||
        !update_heat(state, 1U, q1, &max_heat) ||
        !update_heat(state, 2U, q2, &max_heat) ||
        !update_heat(state, 3U, q3, &max_heat))
    {
        state->max_heat = max_heat;
        MotorThermal_Invalidate(state);
        return false;
    }
    state->max_heat = max_heat;
    memset(state->square_sum_a2, 0, sizeof(state->square_sum_a2));
    state->sample_count = 0U;
    if (state->max_heat >= state->config.trip_heat)
    {
        state->fault_latched = true;
    }
    update_limit(state);
    return true;
}

bool MotorThermal_ClearFault(MotorThermalState *state)
{
    if (state == NULL || !state->valid || state->max_heat > state->config.clear_heat)
    {
        return false;
    }
    state->fault_latched = false;
    update_limit(state);
    return true;
}

bool MotorThermal_LimitVector(float limit_a, float *id_a, float *iq_a)
{
    float norm;
    if (id_a == NULL || iq_a == NULL ||
        !(limit_a >= 0.0f && limit_a <= 100.0f &&
          fabsf(*id_a) <= 100.0f && fabsf(*iq_a) <= 100.0f))
    {
        if (id_a != NULL)
        {
            *id_a = 0.0f;
        }
        if (iq_a != NULL)
        {
            *iq_a = 0.0f;
        }
        return false;
    }
    norm = *id_a * *id_a + *iq_a * *iq_a;
    if (norm <= limit_a * limit_a)
    {
        return true;
    }
    norm = sqrtf(norm);
    if (!isfinite(norm))
    {
        *id_a = 0.0f;
        *iq_a = 0.0f;
        return false;
    }
    if (norm > limit_a)
    {
        float scale = limit_a / norm;
        *id_a *= scale;
        *iq_a *= scale;
    }
    return true;
}
