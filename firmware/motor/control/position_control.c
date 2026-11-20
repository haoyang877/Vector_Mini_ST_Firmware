/* 位置串级跟踪外环给定的位置和速度参考，速度 PI 输出限幅电流。 */
#include "position_control.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "pi_controller.h"
#include "application_config.h"

/* 位置控制内部固定值，不属于共享参数边界。 */
#define POSITION_SERVO_SPEED_CORRECTION_HEADROOM_RATIO 1.50f
#define POSITION_CASCADE_TARGET_REACHED_SPEED_RAD_S 0.03f
#define POSITION_CASCADE_HOLD_EXIT_WINDOW_RATIO 1.5f
#define POSITION_CASCADE_HOLD_EXIT_SPEED_RAD_S 0.08f
#define POSITION_DAMPER_PLAN_BLEND_RAD_S 0.04f
#define POSITION_DAMPER_ENTER_RAD 0.0078539816f   /* 0.45 度 */
#define POSITION_DAMPER_RELEASE_RAD 0.0026179939f /* 0.15 度 */
#define POSITION_DAMPER_FULL_RAD 0.012217305f     /* 0.70 度 */

static float position_cascade_clamp(float value, float limit)
{
    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return -limit;
    }
    return value;
}

static float smooth_fraction(float value)
{
    float x = fminf(1.0f, fmaxf(0.0f, value));
    return x * x * (3.0f - 2.0f * x);
}

static void track_current_budget(PositionCascadeState *state)
{
    PiController *pi = &state->speed_controller;
    pi->output_min = -state->current_budget_a - state->damper_feedforward_a;
    pi->output_max = state->current_budget_a - state->damper_feedforward_a;
    PiController_TrackOutput(pi, fminf(fmaxf(pi->output, pi->output_min), pi->output_max));
}

static void update_damper_feedforward(PositionCascadeState *state,
                                      float error_rad,
                                      float speed_rad_s,
                                      float plan_weight)
{
    const PositionCascadeConfig *config = &state->config;
    const float dt_s = config->update_period_s;
    state->damper_assist_speed_rad_s +=
        dt_s / (0.020f + dt_s) * (speed_rad_s - state->damper_assist_speed_rad_s);
    const bool holding = state->hold_phase == POSITION_CASCADE_HOLD;
    float direction = state->damper_assist_direction;
    if (holding || config->position_kp == 0.0f)
    {
        direction = 0.0f;
    }
    else if (fabsf(error_rad) >= POSITION_DAMPER_ENTER_RAD)
    {
        direction = error_rad > 0.0f ? 1.0f : -1.0f;
    }
    else if (direction * error_rad <= POSITION_DAMPER_RELEASE_RAD)
    {
        direction = 0.0f;
    }
    state->damper_assist_direction = direction;

    /* 巡航补偿跟随规划方向。减速时按剩余误差助推，并提前 30 ms 退坡。 */
    float closing_speed = fmaxf(0.0f, direction * state->damper_assist_speed_rad_s);
    float projected_error = direction * error_rad - 0.030f * closing_speed;
    float correction =
        direction * smooth_fraction((projected_error - POSITION_DAMPER_RELEASE_RAD) /
                                    (POSITION_DAMPER_FULL_RAD - POSITION_DAMPER_RELEASE_RAD));
    float plan_sign = config->reference_speed_rad_s < 0.0f ? -1.0f : 1.0f;
    float overspeed = plan_sign * speed_rad_s - fabsf(config->reference_speed_rad_s);
    float tracking_weight = 1.0f - smooth_fraction(overspeed / 0.07f);
    float target_a = holding
                         ? 0.0f
                         : config->damper_current_a * (plan_sign * plan_weight * tracking_weight +
                                                       (1.0f - plan_weight) * correction);
    target_a = position_cascade_clamp(target_a, state->current_budget_a);
    float previous_a = state->damper_feedforward_a;
    float slew_a_s =
        target_a * previous_a >= 0.0f && fabsf(target_a) > fabsf(previous_a) ? 10.0f : 24.0f;
    state->damper_feedforward_a =
        previous_a + position_cascade_clamp(target_a - previous_a, slew_a_s * dt_s);

    /* 只卸去阻碍静止回位的反向积分，保留支撑和制动电流。 */
    PiController *pi = &state->speed_controller;
    if (config->hold_confirm_ticks != 0U && fabsf(state->damper_assist_speed_rad_s) < 0.025f &&
        direction * error_rad > POSITION_DAMPER_ENTER_RAD && direction * pi->integral_output < 0.0f)
    {
        pi->integral_output += position_cascade_clamp(-pi->integral_output, 3.0f * dt_s);
    }
}

static bool position_cascade_config_valid(const PositionCascadeConfig *config)
{
    return config != NULL && isfinite(config->update_period_s) && config->update_period_s > 0.0f &&
           config->call_divider > 0U && isfinite(config->target_position) &&
           isfinite(config->reference_speed_rad_s) && isfinite(config->position_error_window) &&
           config->position_error_window > 0.0f &&
           config->position_error_window <= POSITION_CASCADE_POSITION_ERROR_WINDOW_MAX_RAD &&
           isfinite(config->velocity_filter_hz) && config->velocity_filter_hz >= 0.0f &&
           config->velocity_filter_hz * config->update_period_s <= 0.5f &&
           isfinite(config->maximum_speed) && config->maximum_speed > 0.0f &&
           isfinite(config->speed_limit_rad_s) &&
           config->speed_limit_rad_s >= config->maximum_speed && isfinite(config->position_kp) &&
           config->position_kp >= 0.0f && config->position_kp <= CASCADE_POSITION_KP_MAX_PER_S &&
           isfinite(config->position_kd) && config->position_kd >= 0.0f &&
           config->position_kd <= CASCADE_POSITION_KD_MAX && isfinite(config->speed_kp) &&
           config->speed_kp >= 0.0f && isfinite(config->speed_ki) && config->speed_ki >= 0.0f &&
           isfinite(config->current_limit_a) && config->current_limit_a > 0.0f &&
           isfinite(config->damper_current_a) && config->damper_current_a >= 0.0f &&
           isfinite(config->speed_kp * config->current_limit_a) &&
           isfinite(config->speed_ki * config->current_limit_a);
}

static bool position_cascade_in_target_window(const PositionCascadeConfig *config,
                                              float position_error_rad,
                                              float speed_rad_s)
{
    return fabsf(position_error_rad) <= config->position_error_window &&
           fabsf(speed_rad_s) <= POSITION_CASCADE_TARGET_REACHED_SPEED_RAD_S;
}

static void position_cascade_reset_hold(PositionCascadeState *state)
{
    state->hold_phase = POSITION_CASCADE_MOVE;
    state->hold_settle_ticks = 0U;
}

static void position_cascade_update_hold(PositionCascadeState *state,
                                         float position_error_rad,
                                         float speed_rad_s)
{
    const PositionCascadeConfig *config = &state->config;
    if (state->hold_phase == POSITION_CASCADE_HOLD)
    {
        if (fabsf(position_error_rad) >
                config->position_error_window * POSITION_CASCADE_HOLD_EXIT_WINDOW_RATIO ||
            fabsf(speed_rad_s) > POSITION_CASCADE_HOLD_EXIT_SPEED_RAD_S)
        {
            state->hold_phase = POSITION_CASCADE_SETTLE;
            state->hold_settle_ticks = 0U;
        }
        return;
    }

    state->hold_phase = POSITION_CASCADE_SETTLE;
    if (!position_cascade_in_target_window(config, position_error_rad, speed_rad_s))
    {
        state->hold_settle_ticks = 0U;
    }
    else if (state->hold_settle_ticks == config->hold_confirm_ticks)
    {
        state->hold_phase = POSITION_CASCADE_HOLD;
    }
    else
    {
        /* 按首个合格样本之后经过的控制拍计时。 */
        ++state->hold_settle_ticks;
    }
}

static bool position_cascade_run(PositionCascadeState *state,
                                 float measured_position_rad,
                                 float measured_speed_rad_s,
                                 PositionCascadeControlOutput *output)
{
    const PositionCascadeConfig *config = &state->config;
    float position_error_rad = config->target_position - measured_position_rad;
    if (!isfinite(position_error_rad))
    {
        return false;
    }
    if (++state->loop_count >= config->call_divider)
    {
        float filtered_speed_rad_s = measured_speed_rad_s;
        float speed_command_rad_s;
        float speed_limit_rad_s;
        float iq_reference_a;
        state->loop_count = 0U;
        if (config->velocity_filter_hz > 0.0f)
        {
            filtered_speed_rad_s =
                state->filtered_speed_rad_s +
                state->velocity_filter_alpha * (measured_speed_rad_s - state->filtered_speed_rad_s);
        }
        if (!isfinite(filtered_speed_rad_s))
        {
            return false;
        }
        float plan_weight = 0.0f;
        float headroom = POSITION_SERVO_SPEED_CORRECTION_HEADROOM_RATIO;
        if (config->damper_current_a > 0.0f)
        {
            plan_weight = smooth_fraction(fabsf(config->reference_speed_rad_s) /
                                          POSITION_DAMPER_PLAN_BLEND_RAD_S);
            headroom = 1.0f;
            if (config->reference_speed_rad_s * position_error_rad > 0.0f)
            {
                headroom += 0.30f * plan_weight;
            }
        }
        speed_limit_rad_s = config->maximum_speed * headroom;
        if (speed_limit_rad_s > config->speed_limit_rad_s)
        {
            speed_limit_rad_s = config->speed_limit_rad_s;
        }
        speed_command_rad_s = position_cascade_clamp(config->reference_speed_rad_s +
                                                         config->position_kp * position_error_rad -
                                                         config->position_kd * filtered_speed_rad_s,
                                                     speed_limit_rad_s);
        if (!isfinite(speed_command_rad_s) || !isfinite(speed_command_rad_s - filtered_speed_rad_s))
        {
            return false;
        }
        if (config->hold_confirm_ticks != 0U)
        {
            position_cascade_update_hold(state, position_error_rad, filtered_speed_rad_s);
        }
        if (config->damper_current_a > 0.0f)
        {
            update_damper_feedforward(state, position_error_rad, filtered_speed_rad_s, plan_weight);
            track_current_budget(state);
        }
        PiController *speed_pi = &state->speed_controller;
        float integral_gain = speed_pi->integral_gain;
        if (state->hold_phase == POSITION_CASCADE_HOLD)
        {
            /* 暂停本拍误差积分，保留支撑积分和 PI 的常规配置。 */
            speed_pi->integral_gain = 0.0f;
        }
        iq_reference_a = PiController_Run(speed_pi, speed_command_rad_s, filtered_speed_rad_s);
        speed_pi->integral_gain = integral_gain;
        if (!isfinite(iq_reference_a))
        {
            return false;
        }
        /* PI 只跟踪扣除前馈之后的电流份额。 */
        PiController_TrackOutput(&state->speed_controller, iq_reference_a);
        state->filtered_speed_rad_s = filtered_speed_rad_s;
    }
    output->position_reference = config->target_position;
    output->speed_reference = state->speed_controller.reference;
    output->speed_feedback_rad_s = state->filtered_speed_rad_s;
    output->iq_reference = position_cascade_clamp(
        state->speed_controller.output + state->damper_feedforward_a, state->current_budget_a);
    output->target_reached =
        position_cascade_in_target_window(config, position_error_rad, state->filtered_speed_rad_s);
    return true;
}

bool PositionCascade_UpdateControl(PositionCascadeState *state,
                                   const PositionCascadeConfig *config,
                                   float measured_position,
                                   float measured_speed,
                                   PositionCascadeControlOutput *output)
{
    return config != NULL &&
           PositionCascade_UpdateControlLimited(
               state, config, measured_position, measured_speed, config->current_limit_a, output);
}

bool PositionCascade_UpdateControlLimited(PositionCascadeState *state,
                                          const PositionCascadeConfig *config,
                                          float measured_position,
                                          float measured_speed,
                                          float available_current_a,
                                          PositionCascadeControlOutput *output)
{
    float alpha;
    if (output == NULL || !isfinite(measured_position) || !isfinite(measured_speed) ||
        !isfinite(available_current_a) || available_current_a < 0.0f ||
        !position_cascade_config_valid(config))
    {
        return false;
    }
    alpha = state->velocity_filter_alpha;
    if (!state->configured || config->velocity_filter_hz != state->config.velocity_filter_hz ||
        config->update_period_s != state->config.update_period_s)
    {
        float ratio = 6.2831853072f * config->velocity_filter_hz * config->update_period_s;
        alpha = ratio / (1.0f + ratio);
        if (!isfinite(alpha))
        {
            return false;
        }
    }
    if (!state->configured)
    {
        state->filtered_speed_rad_s = measured_speed;
    }
    if (state->hold_phase != POSITION_CASCADE_MOVE &&
        (config->hold_confirm_ticks != state->config.hold_confirm_ticks ||
         config->target_position != state->config.target_position))
    {
        position_cascade_reset_hold(state);
    }
    state->config = *config;
    state->velocity_filter_alpha = alpha;
    state->configured = true;
    state->current_budget_a = fminf(available_current_a, config->current_limit_a);
    if (config->damper_current_a == 0.0f)
    {
        state->damper_feedforward_a = 0.0f;
        state->damper_assist_direction = 0.0f;
        state->damper_assist_speed_rad_s = 0.0f;
    }
    state->damper_feedforward_a =
        position_cascade_clamp(state->damper_feedforward_a, state->current_budget_a);
    PiController_Configure(&state->speed_controller,
                           config->speed_kp * config->current_limit_a,
                           config->speed_ki * config->current_limit_a,
                           config->update_period_s,
                           -state->current_budget_a,
                           state->current_budget_a);
    /* 下调电流上限在分频等待拍也立即生效。 */
    track_current_budget(state);
    return position_cascade_run(state, measured_position, measured_speed, output);
}

bool PositionCascade_UpdateTargetControl(PositionCascadeState *state,
                                         float target_position,
                                         float measured_position,
                                         float measured_speed,
                                         PositionCascadeControlOutput *output)
{
    if (!state->configured || output == NULL || !isfinite(target_position) ||
        !isfinite(measured_position) || !isfinite(measured_speed))
    {
        return false;
    }
    if (state->config.hold_confirm_ticks != 0U && target_position != state->config.target_position)
    {
        state->config.hold_confirm_ticks = 0U;
        position_cascade_reset_hold(state);
    }
    state->config.target_position = target_position;
    return position_cascade_run(state, measured_position, measured_speed, output);
}

void PositionCascade_Reset(PositionCascadeState *state)
{
    memset(state, 0, sizeof(*state));
}

const PositionCascadeConfig *PositionCascade_GetConfiguration(const PositionCascadeState *state)
{
    return state->configured ? &state->config : NULL;
}
