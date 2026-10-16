/* 直接位置串级：目标经速度限幅进入 PI；单实例串行，无轨迹、前馈或保持状态机。 */
#include "position_cascade.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "foc_pid.h"
#include "position_cascade_config.h"

typedef struct
{
    PositionCascadeConfig config;
    PiController speed_controller;
    float filtered_speed_rad_s;
    float velocity_filter_alpha;
    uint16_t loop_count;
    bool configured;
} PositionCascadeState;

static PositionCascadeState s_state;

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
           isfinite(config->position_error_window) && config->position_error_window > 0.0f &&
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

static bool position_cascade_run(float measured_position_rad,
                                 float measured_speed_rad_s,
                                 PositionCascadeControlOutput *output)
{
    const PositionCascadeConfig *config = &s_state.config;
    float position_error_rad = config->target_position - measured_position_rad;
    if (!isfinite(position_error_rad))
    {
        return false;
    }
    if (++s_state.loop_count >= config->call_divider)
    {
        float filtered_speed_rad_s = measured_speed_rad_s;
        float speed_command_rad_s;
        float speed_limit_rad_s;
        float iq_reference_a;
        s_state.loop_count = 0U;
        if (config->velocity_filter_hz > 0.0f)
        {
            filtered_speed_rad_s = s_state.filtered_speed_rad_s +
                                   s_state.velocity_filter_alpha *
                                       (measured_speed_rad_s - s_state.filtered_speed_rad_s);
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
        speed_command_rad_s = position_cascade_clamp(config->position_kp * position_error_rad -
                                                         config->position_kd * filtered_speed_rad_s,
                                                     speed_limit_rad_s);
        if (!isfinite(speed_command_rad_s) || !isfinite(speed_command_rad_s - filtered_speed_rad_s))
        {
            return false;
        }
        iq_reference_a =
            PiController_Run(&s_state.speed_controller, speed_command_rad_s, filtered_speed_rad_s);
        if (!isfinite(iq_reference_a))
        {
            return false;
        }
        /* 没有前馈；接受的总输出就是 PI 份额，保留原积分限幅跟踪。 */
        PiController_TrackOutput(&s_state.speed_controller, iq_reference_a);
        s_state.filtered_speed_rad_s = filtered_speed_rad_s;
    }
    output->position_reference = config->target_position;
    output->speed_reference = s_state.speed_controller.reference;
    output->speed_feedback_rad_s = s_state.filtered_speed_rad_s;
    output->iq_reference = s_state.speed_controller.output;
    output->target_reached =
        fabsf(position_error_rad) <= config->position_error_window &&
        fabsf(s_state.filtered_speed_rad_s) <= POSITION_CASCADE_TARGET_REACHED_SPEED_RAD_S;
    return true;
}

bool PositionCascade_UpdateControl(const PositionCascadeConfig *config,
                                   float measured_position,
                                   float measured_speed,
                                   PositionCascadeControlOutput *output)
{
    return config != NULL &&
           PositionCascade_UpdateControlLimited(
               config, measured_position, measured_speed, config->current_limit_a, output);
}

bool PositionCascade_UpdateControlLimited(const PositionCascadeConfig *config,
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
    alpha = s_state.velocity_filter_alpha;
    if (!s_state.configured || config->velocity_filter_hz != s_state.config.velocity_filter_hz ||
        config->update_period_s != s_state.config.update_period_s)
    {
        float ratio = 6.2831853072f * config->velocity_filter_hz * config->update_period_s;
        alpha = ratio / (1.0f + ratio);
        if (!isfinite(alpha))
        {
            return false;
        }
    }
    if (!s_state.configured)
    {
        s_state.filtered_speed_rad_s = measured_speed;
    }
    s_state.config = *config;
    s_state.velocity_filter_alpha = alpha;
    s_state.configured = true;
    float limit_a = fminf(available_current_a, config->current_limit_a);
    PiController_Configure(&s_state.speed_controller,
                           config->speed_kp * config->current_limit_a,
                           config->speed_ki * config->current_limit_a,
                           config->update_period_s,
                           -limit_a,
                           limit_a);
    /* 下调电流上限即使落在分频等待拍也立即作用，不清掉已有支撑积分。 */
    PiController_TrackOutput(&s_state.speed_controller,
                             position_cascade_clamp(s_state.speed_controller.output, limit_a));
    return position_cascade_run(measured_position, measured_speed, output);
}

bool PositionCascade_UpdateTargetControl(float target_position,
                                         float measured_position,
                                         float measured_speed,
                                         PositionCascadeControlOutput *output)
{
    if (!s_state.configured || output == NULL || !isfinite(target_position) ||
        !isfinite(measured_position) || !isfinite(measured_speed))
    {
        return false;
    }
    s_state.config.target_position = target_position;
    return position_cascade_run(measured_position, measured_speed, output);
}

void PositionCascade_Reset(void)
{
    memset(&s_state, 0, sizeof(s_state));
}

const PositionCascadeConfig *PositionCascade_GetConfiguration(void)
{
    return s_state.configured ? &s_state.config : NULL;
}
