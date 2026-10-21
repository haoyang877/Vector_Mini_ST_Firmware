#include "motor_model.h"
#include <math.h>
#include <stddef.h>

/* 确定性PMSM对象：独立于控制器的dq与机械方程，按子步中点积分。
 * 关相采用低速R/L衰减近似，禁止据此推断高转速回馈与母线安全性。 */

bool MotorModel_Valid(const VirtualMotorConfig *config)
{
    return config != NULL && isfinite(config->phase_resistance_ohm) &&
           config->phase_resistance_ohm > 0.0f && isfinite(config->d_inductance_h) &&
           config->d_inductance_h > 0.0f && isfinite(config->q_inductance_h) &&
           config->q_inductance_h > 0.0f && isfinite(config->flux_wb) && config->flux_wb > 0.0f &&
           config->pole_pairs > 0U && config->pole_pairs <= 1000U &&
           isfinite(config->inertia_kg_m2) && config->inertia_kg_m2 > 0.0f &&
           isfinite(config->damping_nm_per_rad_s) && config->damping_nm_per_rad_s >= 0.0f &&
           isfinite(config->load_torque_nm);
}

static MotorModelState derivative(const VirtualMotorConfig *config,
                                  const MotorModelState *state,
                                  const float voltage_v[3],
                                  bool enabled)
{
    float electrical_angle = (float)fmod(state->angle_rad * config->pole_pairs, 6.283185307179586);
    float sine = sinf(electrical_angle), cosine = cosf(electrical_angle);
    float alpha = voltage_v[0];
    float beta = (voltage_v[1] - voltage_v[2]) / 1.732050808f;
    float vd = alpha * cosine + beta * sine;
    float vq = -alpha * sine + beta * cosine;
    float electrical_velocity = state->velocity_rad_s * (float)config->pole_pairs;
    float torque =
        1.5f * (float)config->pole_pairs *
        (config->flux_wb + (config->d_inductance_h - config->q_inductance_h) * state->id_a) *
        state->iq_a;
    MotorModelState change = {
        .id_a = -config->phase_resistance_ohm * state->id_a / config->d_inductance_h,
        .iq_a = -config->phase_resistance_ohm * state->iq_a / config->q_inductance_h,
        .angle_rad = state->velocity_rad_s,
        .velocity_rad_s = (torque - config->load_torque_nm -
                           config->damping_nm_per_rad_s * state->velocity_rad_s) /
                          config->inertia_kg_m2};
    if (enabled)
    {
        change.id_a += (vd + electrical_velocity * config->q_inductance_h * state->iq_a) /
                       config->d_inductance_h;
        change.iq_a +=
            (vq - electrical_velocity * (config->d_inductance_h * state->id_a + config->flux_wb)) /
            config->q_inductance_h;
    }
    return change;
}

static MotorModelState
add(const MotorModelState *state, const MotorModelState *change, float step_s)
{
    return (MotorModelState){state->id_a + change->id_a * step_s,
                             state->iq_a + change->iq_a * step_s,
                             state->angle_rad + change->angle_rad * step_s,
                             state->velocity_rad_s + change->velocity_rad_s * step_s};
}

bool MotorModel_Step(const VirtualMotorConfig *config,
                     MotorModelState *state,
                     const float voltage_v[3],
                     bool enabled,
                     float step_s)
{
    if (!MotorModel_Valid(config) || state == NULL || voltage_v == NULL || !isfinite(step_s) ||
        step_s <= 0.0f || !isfinite(voltage_v[0]) || !isfinite(voltage_v[1]) ||
        !isfinite(voltage_v[2]))
    {
        return false;
    }
    MotorModelState first = derivative(config, state, voltage_v, enabled);
    MotorModelState midpoint = add(state, &first, step_s * 0.5f);
    MotorModelState second = derivative(config, &midpoint, voltage_v, enabled);
    MotorModelState next = add(state, &second, step_s);
    if (!isfinite(next.id_a) || !isfinite(next.iq_a) || !isfinite(next.angle_rad) ||
        !isfinite(next.velocity_rad_s))
    {
        return false;
    }
    *state = next;
    return true;
}

void MotorModel_PhaseCurrents(const VirtualMotorConfig *config,
                              const MotorModelState *state,
                              float current_a[3])
{
    float angle = (float)fmod(state->angle_rad * config->pole_pairs, 6.283185307179586);
    float alpha = state->id_a * cosf(angle) - state->iq_a * sinf(angle);
    float beta = state->id_a * sinf(angle) + state->iq_a * cosf(angle);
    current_a[0] = alpha;
    current_a[1] = -0.5f * alpha + 0.866025404f * beta;
    current_a[2] = -0.5f * alpha - 0.866025404f * beta;
}
