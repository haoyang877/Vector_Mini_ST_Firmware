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
        speed_limit_rad_s = config->maximum_speed * POSITION_SERVO_SPEED_CORRECTION_HEADROOM_RATIO;
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
        /* 没有前馈；接受的总输出就是 PI 份额，保留原积分限幅跟踪。 */
        PiController_TrackOutput(&state->speed_controller, iq_reference_a);
        state->filtered_speed_rad_s = filtered_speed_rad_s;
    }
    output->position_reference = config->target_position;
    output->speed_reference = state->speed_controller.reference;
    output->speed_feedback_rad_s = state->filtered_speed_rad_s;
    output->iq_reference = state->speed_controller.output;
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
    float limit_a = fminf(available_current_a, config->current_limit_a);
    PiController_Configure(&state->speed_controller,
                           config->speed_kp * config->current_limit_a,
                           config->speed_ki * config->current_limit_a,
                           config->update_period_s,
                           -limit_a,
                           limit_a);
    /* 下调电流上限即使落在分频等待拍也立即作用，不清掉已有支撑积分。 */
    PiController_TrackOutput(&state->speed_controller,
                             position_cascade_clamp(state->speed_controller.output, limit_a));
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
