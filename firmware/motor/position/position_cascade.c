/* 位置级联控制：单实例串行所有权；校验失败不发布新配置，由调用方处理故障。 */
#include "fast_loop_profile.h"
#include "position_cascade.h"

#include <math.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "foc_pid.h"
#include "position_cascade_config.h"
#include "position_smooth_trajectory.h"

typedef struct
{
    PositionCascadeConfig validated_config;
    bool config_valid;
    uint32_t configured_hold_ticks;
    uint32_t configured_stuck_ticks;
    float configured_velocity_filter_alpha;
    float configured_hold_velocity_filter_alpha;
    PiController speed_controller;
    PositionSmoothTrajectory smooth_trajectory;
    bool smooth_active;
    bool smooth_planning;
    uint16_t loop_count;
    uint32_t hold_counter;
    uint32_t friction_stuck_counter;
    uint32_t position_stuck_counter;
    float position_stuck_anchor;
    float last_target;
    float position_reference;
    float trajectory_speed;
    float trajectory_acceleration;
    float speed_reference;
    float velocity_filtered;
    float control_velocity;
    float iq_reference;
    float feedback_current;
    float acceleration_feedforward_current;
    float friction_feedforward_current;
    float friction_breakaway_start_position;
    int8_t friction_direction;
    PositionServoPhase phase;
    bool friction_breakaway_active;
    bool friction_landing_active;
    bool settle_recovery_active;
    bool target_transition_active;
    bool target_reached;
    bool current_saturated;
    bool settling_in_window;
    bool trajectory_limited;
    bool friction_current_ramping;
    bool stiction_integrating;
    bool initialized;
    bool defer_telemetry;
} PositionCascadeState;

static PositionCascadeState s_state;

static inline float position_cascade_abs(float value)
{
    /* FPU 目标使用单条绝对值指令。 */
    return fabsf(value);
}

static inline float position_cascade_min(float first, float second)
{
    return first <= second ? first : second;
}

static inline float position_cascade_max(float first, float second)
{
    return first >= second ? first : second;
}

static inline float position_cascade_constrain(float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

static uint32_t position_cascade_time_to_ticks(const PositionCascadeConfig *config, float time_s)
{
    float ticks;

    if (time_s <= 0.0f)
    {
        return 0U;
    }
    ticks = time_s / config->update_period_s + 0.5f;
    if (ticks >= (float)UINT32_MAX)
    {
        return UINT32_MAX;
    }
    if (ticks < 1.0f)
    {
        return 1U;
    }
    return (uint32_t)ticks;
}

typedef char PositionConfigOffsetFitsByte[sizeof(PositionCascadeConfig) <= UINT8_MAX ? 1 : -1];

static bool position_cascade_config_is_valid(const PositionCascadeConfig *config)
{
    static const uint8_t finite_offsets[] = {
        offsetof(PositionCascadeConfig, update_period_s),
        offsetof(PositionCascadeConfig, target_position),
        offsetof(PositionCascadeConfig, position_error_window),
        offsetof(PositionCascadeConfig, hold_enter_position),
        offsetof(PositionCascadeConfig, hold_exit_position),
        offsetof(PositionCascadeConfig, velocity_filter_hz),
        offsetof(PositionCascadeConfig, hold_velocity_filter_hz),
        offsetof(PositionCascadeConfig, following_error_limit),
        offsetof(PositionCascadeConfig, stiction_integral_rate),
        offsetof(PositionCascadeConfig, acceleration),
        offsetof(PositionCascadeConfig, deceleration),
        offsetof(PositionCascadeConfig, maximum_speed),
        offsetof(PositionCascadeConfig, speed_limit_rad_s),
        offsetof(PositionCascadeConfig, jerk_limit),
        offsetof(PositionCascadeConfig, position_kp),
        offsetof(PositionCascadeConfig, position_kd),
        offsetof(PositionCascadeConfig, speed_kp),
        offsetof(PositionCascadeConfig, speed_ki),
        offsetof(PositionCascadeConfig, acceleration_feedforward_gain),
        offsetof(PositionCascadeConfig, current_limit_a),
        offsetof(PositionCascadeConfig, friction_coulomb_positive),
        offsetof(PositionCascadeConfig, friction_coulomb_negative),
        offsetof(PositionCascadeConfig, friction_viscous_positive),
        offsetof(PositionCascadeConfig, friction_viscous_negative),
        offsetof(PositionCascadeConfig, friction_breakaway_ratio),
        offsetof(PositionCascadeConfig, friction_attack_slew_rate),
        offsetof(PositionCascadeConfig, friction_fast_release_slew_rate),
        offsetof(PositionCascadeConfig, friction_release_slew_rate),
        offsetof(PositionCascadeConfig, friction_reference_speed),
        offsetof(PositionCascadeConfig, friction_stop_speed),
        offsetof(PositionCascadeConfig, friction_move_speed),
        offsetof(PositionCascadeConfig, friction_breakaway_distance),
        offsetof(PositionCascadeConfig, friction_stuck_time),
    };
    size_t index;
    /* 仅在配置变化时扫描；只读取明确列出的 float，不检查结构填充字节。 */
    if (config == NULL)
    {
        return false;
    }
    for (index = 0; index < sizeof(finite_offsets) / sizeof(finite_offsets[0]); ++index)
    {
        float value;
        memcpy(&value, (const unsigned char *)config + finite_offsets[index], sizeof(value));
        if (!isfinite(value))
        {
            return false;
        }
    }
    return config->update_period_s > 0.0f && config->call_divider > 0U &&
           config->position_error_window > 0.0f &&
           config->hold_enter_position >= config->position_error_window &&
           config->hold_exit_position > config->hold_enter_position &&
           config->velocity_filter_hz >= 0.0f &&
           config->velocity_filter_hz * config->update_period_s <= 0.5f &&
           config->hold_velocity_filter_hz >= 0.0f &&
           config->hold_velocity_filter_hz * config->update_period_s <= 0.5f &&
           config->following_error_limit >= 0.0f && config->stiction_integral_rate >= 0.0f &&
           isfinite(config->stiction_integral_rate * config->update_period_s) &&
           config->acceleration > 0.0f && config->deceleration > 0.0f &&
           config->maximum_speed > 0.0f && config->speed_limit_rad_s > 0.0f &&
           config->maximum_speed <= config->speed_limit_rad_s && config->jerk_limit > 0.0f &&
           config->position_kp >= 0.0f && config->position_kp <= CASCADE_POSITION_KP_MAX_PER_S &&
           config->position_kd >= 0.0f && config->position_kd <= CASCADE_POSITION_KD_MAX &&
           config->speed_kp >= 0.0f && config->speed_ki >= 0.0f &&
           config->acceleration_feedforward_gain >= 0.0f && config->current_limit_a > 0.0f &&
           config->friction_coulomb_positive >= 0.0f && config->friction_coulomb_negative >= 0.0f &&
           config->friction_viscous_positive >= 0.0f && config->friction_viscous_negative >= 0.0f &&
           config->friction_breakaway_ratio >= 1.0f && config->friction_attack_slew_rate > 0.0f &&
           config->friction_fast_release_slew_rate > 0.0f &&
           config->friction_release_slew_rate > 0.0f && config->friction_reference_speed > 0.0f &&
           config->friction_stop_speed >= 0.0f &&
           config->friction_move_speed > config->friction_stop_speed &&
           config->friction_breakaway_distance > 0.0f && config->friction_stuck_time >= 0.0f;
}

static bool position_cascade_check_config(const PositionCascadeConfig *config)
{
    const PositionCascadeConfig *saved = &s_state.validated_config;
    if (config == NULL || !isfinite(config->target_position))
    {
        return false;
    }
    /* 逐项比较调参字段，忽略结构填充与每拍单独校验的目标位置；避免无关变化触发完整校验。 */
    if (s_state.config_valid && config->update_period_s == saved->update_period_s &&
        config->call_divider == saved->call_divider &&
        config->friction_feedforward_enabled == saved->friction_feedforward_enabled &&
        memcmp(&config->position_error_window,
               &saved->position_error_window,
               offsetof(PositionCascadeConfig, friction_feedforward_enabled) -
                   offsetof(PositionCascadeConfig, position_error_window)) == 0 &&
        memcmp(&config->friction_coulomb_positive,
               &saved->friction_coulomb_positive,
               sizeof(*config) - offsetof(PositionCascadeConfig, friction_coulomb_positive)) == 0)
    {
        return true;
    }
    s_state.defer_telemetry = true;
    if (!position_cascade_config_is_valid(config))
    {
        return false;
    }
    /* 完整校验成功后才刷新派生常量，缓存命中时不增加 2 kHz 控制计算。 */
    s_state.configured_hold_ticks =
        position_cascade_time_to_ticks(config, POSITION_SERVO_HOLD_CONFIRM_TIME_S);
    s_state.configured_stuck_ticks =
        position_cascade_time_to_ticks(config, config->friction_stuck_time);
    {
        float ratio = 6.2831853072f * config->velocity_filter_hz * config->update_period_s;
        s_state.configured_velocity_filter_alpha = ratio / (1.0f + ratio);
        ratio = 6.2831853072f * config->hold_velocity_filter_hz * config->update_period_s;
        s_state.configured_hold_velocity_filter_alpha = ratio / (1.0f + ratio);
    }
    memcpy(&s_state.validated_config, config, sizeof(*config));
    s_state.config_valid = true;
    return true;
}

bool PositionCascade_GetTelemetry(PositionCascadeTelemetry *telemetry)
{
    if (telemetry == NULL)
    {
        return false;
    }

    telemetry->position_reference = s_state.position_reference;
    telemetry->trajectory_speed_reference = s_state.trajectory_speed;
    telemetry->speed_command = s_state.speed_reference;
    telemetry->speed_feedback_rad_s = s_state.control_velocity;
    telemetry->acceleration_reference = s_state.trajectory_acceleration;
    telemetry->feedback_current = s_state.feedback_current;
    telemetry->acceleration_feedforward_current = s_state.acceleration_feedforward_current;
    telemetry->friction_feedforward_current = s_state.friction_feedforward_current;
    telemetry->hold_current = s_state.speed_controller.integral_output;
    telemetry->phase = s_state.phase;
    telemetry->target_reached = s_state.target_reached;
    telemetry->current_saturated = s_state.current_saturated;
    telemetry->friction_landing_active = s_state.friction_landing_active;
    telemetry->settle_recovery_active = s_state.settle_recovery_active;
    telemetry->hold_candidate_active =
        s_state.phase == POSITION_SERVO_PHASE_SETTLE && s_state.hold_counter > 0U;
    telemetry->trajectory_limited = s_state.trajectory_limited;
    telemetry->stiction_integrating = s_state.stiction_integrating;
    return s_state.initialized;
}

bool PositionCascade_ShouldDeferTelemetry(void)
{
    return s_state.defer_telemetry;
}

static bool position_cascade_update_online_trajectory(const PositionCascadeConfig *config,
                                                      float measured_position)
{
    float distance = config->target_position - s_state.position_reference;
    float direction;
    float speed_toward_target;
    float acceleration_toward_target;
    float acceleration_transition_time;
    float acceleration_transition_distance;
    float braking_distance_available;
    float braking_speed;
    float desired_speed;
    float speed_error;
    float desired_acceleration;
    float acceleration_step;
    float acceleration_magnitude;
    float previous_speed;
    float following_error;
    float governed_speed;
    float governor_scale;

    s_state.trajectory_limited = false;

    if (position_cascade_abs(distance) <= config->position_error_window &&
        position_cascade_abs(s_state.trajectory_speed) <= POSITION_SERVO_HOLD_ENTER_SPEED_RAD_S &&
        position_cascade_abs(s_state.trajectory_acceleration) <=
            config->jerk_limit * config->update_period_s)
    {
        s_state.position_reference = config->target_position;
        s_state.trajectory_speed = 0.0f;
        s_state.trajectory_acceleration = 0.0f;
        return true;
    }

    direction = distance >= 0.0f ? 1.0f : -1.0f;
    speed_toward_target = direction * s_state.trajectory_speed;
    acceleration_toward_target =
        position_cascade_max(direction * s_state.trajectory_acceleration, 0.0f);
    if (speed_toward_target > 0.0f)
    {
        /* 只为朝目标方向的加速度保留过渡距离；负加速度已在制动，重复计入会产生停走尾段。 */
        acceleration_transition_time =
            (acceleration_toward_target + config->deceleration) / config->jerk_limit;
        acceleration_transition_distance =
            speed_toward_target * acceleration_transition_time + 0.5f * acceleration_toward_target *
                                                                     acceleration_transition_time *
                                                                     acceleration_transition_time;
    }
    else
    {
        acceleration_transition_distance = 0.0f;
    }
    braking_distance_available = position_cascade_max(
        position_cascade_abs(distance) - acceleration_transition_distance, 0.0f);
    braking_speed = sqrtf(2.0f * config->deceleration * braking_distance_available);
    desired_speed = direction * position_cascade_min(config->maximum_speed, braking_speed);
    if (config->following_error_limit > 0.0f)
    {
        /* 柔性限跃度跟随误差调节，参考位置不跳变到反馈。 */
        following_error = direction * (s_state.position_reference - measured_position);
        governor_scale = position_cascade_constrain(
            1.0f - following_error / config->following_error_limit, 0.0f, 1.0f);
        governed_speed = config->maximum_speed * governor_scale;
        if (position_cascade_abs(desired_speed) > governed_speed)
        {
            desired_speed = direction * governed_speed;
            s_state.trajectory_limited = true;
        }
    }
    speed_error = desired_speed - s_state.trajectory_speed;
    {
        bool increasing_speed =
            desired_speed * s_state.trajectory_speed >= 0.0f &&
            position_cascade_abs(desired_speed) > position_cascade_abs(s_state.trajectory_speed);

        acceleration_magnitude = increasing_speed ? config->acceleration : config->deceleration;
        desired_acceleration =
            position_cascade_constrain(speed_error / POSITION_SERVO_ACCEL_RAMP_TIME_S,
                                       -acceleration_magnitude,
                                       acceleration_magnitude);
    }

    acceleration_step = config->jerk_limit * config->update_period_s;
    s_state.trajectory_acceleration +=
        position_cascade_constrain(desired_acceleration - s_state.trajectory_acceleration,
                                   -acceleration_step,
                                   acceleration_step);
    s_state.trajectory_acceleration = position_cascade_constrain(
        s_state.trajectory_acceleration,
        -position_cascade_max(config->acceleration, config->deceleration),
        position_cascade_max(config->acceleration, config->deceleration));
    previous_speed = s_state.trajectory_speed;
    s_state.trajectory_speed += s_state.trajectory_acceleration * config->update_period_s;
    s_state.trajectory_speed = position_cascade_constrain(
        s_state.trajectory_speed, -config->maximum_speed, config->maximum_speed);
    s_state.position_reference +=
        0.5f * (previous_speed + s_state.trajectory_speed) * config->update_period_s;

    return false;
}

static bool position_cascade_update_trajectory(const PositionCascadeConfig *config,
                                               float measured_position)
{
    PositionSmoothTrajectory *s = &s_state.smooth_trajectory;
    PositionSmoothSample sample;
    bool done;
    bool interrupted_preparation =
        s_state.smooth_active && !s->ready && s->target != config->target_position;
    s_state.smooth_planning = false;
    if (s_state.smooth_active &&
        (s->target != config->target_position || s->speed_limit_rad_s != config->maximum_speed ||
         s->acceleration != config->acceleration || s->deceleration != config->deceleration ||
         s->jerk_limit != config->jerk_limit || config->following_error_limit > 0))
    {
        s_state.smooth_active = false;
    }
    /* 运动中换目标保持 q/v/a 并使用在线规划；静止到静止运动使用精确平滑端点。 */
    if (!s_state.smooth_active && !interrupted_preparation && config->following_error_limit == 0 &&
        s_state.trajectory_speed == 0 && s_state.trajectory_acceleration == 0 &&
        position_cascade_abs(config->target_position - s_state.position_reference) >
            config->position_error_window)
    {
        s_state.smooth_active = PositionSmooth_Begin(s,
                                                     s_state.position_reference,
                                                     config->target_position,
                                                     config->maximum_speed,
                                                     config->acceleration,
                                                     config->deceleration,
                                                     config->jerk_limit);
        if (!s_state.smooth_active)
        {
            s->failed = true;
            return false;
        }
    }
    if (!s_state.smooth_active)
    {
        return position_cascade_update_online_trajectory(config, measured_position);
    }
    if (!s->ready)
    {
        s_state.smooth_planning = true;
        FAST_PROFILE_BEGIN(FAST_PROFILE_PLAN_PREPARE);
        (void)PositionSmooth_Prepare(s);
        FAST_PROFILE_END(FAST_PROFILE_PLAN_PREPARE);
        return false;
    }
    done = PositionSmooth_Advance(s, config->update_period_s, &sample);
    s_state.position_reference = sample.position;
    s_state.trajectory_speed = sample.speed;
    s_state.trajectory_acceleration = sample.acceleration;
    s_state.trajectory_limited = false;
    return done;
}

static float position_cascade_friction_magnitude(const PositionCascadeConfig *config,
                                                 int8_t direction)
{
    float speed = position_cascade_abs(s_state.trajectory_speed);

    if (direction > 0)
    {
        return config->friction_coulomb_positive + config->friction_viscous_positive * speed;
    }
    if (direction < 0)
    {
        return config->friction_coulomb_negative + config->friction_viscous_negative * speed;
    }
    return 0.0f;
}

static void position_cascade_update_friction(const PositionCascadeConfig *config,
                                             float measured_position,
                                             float measured_speed)
{
    float target_current = 0.0f;
    float current_step;
    float current_delta;
    float magnitude;
    float target_error;
    float remaining_distance = 0.0f;
    float speed_toward_target = 0.0f;
    float braking_distance = 0.0f;
    float landing_position;
    float landing_zero_position;
    float landing_span;
    float landing_scale = 1.0f;
    float previous_current = s_state.friction_feedforward_current;
    uint32_t stuck_ticks;
    int8_t desired_direction = 0;
    bool feedback_is_stopped;
    bool request_breakaway = false;
    bool controller_requests_braking = false;
    bool inside_braking_envelope = false;
    bool fast_release = false;
    bool waiting_for_recovery = false;

    if (!config->friction_feedforward_enabled)
    {
        s_state.friction_feedforward_current = 0.0f;
        s_state.friction_direction = 0;
        s_state.friction_stuck_counter = 0U;
        s_state.friction_breakaway_active = false;
        s_state.friction_landing_active = false;
        s_state.settle_recovery_active = false;
        s_state.friction_current_ramping = false;
        return;
    }

    target_error = config->target_position - measured_position;
    landing_position = config->hold_enter_position;
    landing_zero_position = landing_position * POSITION_SERVO_FRICTION_LANDING_ZERO_RATIO;
    feedback_is_stopped = position_cascade_abs(measured_speed) <= config->friction_stop_speed;
    stuck_ticks = s_state.configured_stuck_ticks;

    if (s_state.phase == POSITION_SERVO_PHASE_HOLD || s_state.settling_in_window)
    {
        s_state.friction_stuck_counter = 0U;
        s_state.friction_breakaway_active = false;
        s_state.friction_landing_active = false;
        s_state.settle_recovery_active = false;
        fast_release = true;
    }
    else if (s_state.friction_landing_active)
    {
        /* 着陆衰减期间锁存运动方向。转子在目标窗口外停住时，以有界起步斜坡恢复，
         * 即使参考仍在运动，也不等待轨迹结束。 */
        if (position_cascade_abs(target_error) > config->hold_exit_position && feedback_is_stopped)
        {
            waiting_for_recovery = true;
            if (s_state.friction_stuck_counter < UINT32_MAX)
            {
                s_state.friction_stuck_counter++;
            }
            if (stuck_ticks == 0U || s_state.friction_stuck_counter >= stuck_ticks)
            {
                s_state.friction_landing_active = false;
                s_state.settle_recovery_active = true;
                desired_direction = target_error > 0.0f ? 1 : -1;
                request_breakaway = true;
            }
            else
            {
                desired_direction = s_state.friction_direction;
            }
        }
        else
        {
            s_state.friction_stuck_counter = 0U;
            desired_direction = s_state.friction_direction;
        }
        fast_release = true;
    }
    else if (s_state.phase == POSITION_SERVO_PHASE_MOVE)
    {
        if (position_cascade_abs(s_state.trajectory_speed) >= config->friction_reference_speed)
        {
            desired_direction = s_state.trajectory_speed > 0.0f ? 1 : -1;
        }
        else if (position_cascade_abs(config->target_position - s_state.position_reference) >
                 config->position_error_window)
        {
            desired_direction = config->target_position > s_state.position_reference ? 1 : -1;
        }
        else if (position_cascade_abs(target_error) > config->hold_enter_position)
        {
            /* 参考接近停止但转子仍落后时，保持 MOVE/SETTLE 间的靠近补偿。 */
            desired_direction = target_error > 0.0f ? 1 : -1;
        }
    }
    else if (s_state.phase == POSITION_SERVO_PHASE_SETTLE)
    {
        if (s_state.settle_recovery_active)
        {
            if (position_cascade_abs(target_error) > config->hold_enter_position)
            {
                /* 锁存方向，使越过目标时进入着陆。 */
                desired_direction = s_state.friction_direction != 0
                                        ? s_state.friction_direction
                                        : (target_error > 0.0f ? 1 : -1);
            }
            else
            {
                s_state.settle_recovery_active = false;
                s_state.friction_breakaway_active = false;
                fast_release = true;
            }
        }
        else if (position_cascade_abs(target_error) > config->hold_exit_position &&
                 feedback_is_stopped)
        {
            waiting_for_recovery = true;
            fast_release = true;
            if (s_state.friction_stuck_counter < UINT32_MAX)
            {
                s_state.friction_stuck_counter++;
            }
            if (stuck_ticks == 0U || s_state.friction_stuck_counter >= stuck_ticks)
            {
                s_state.settle_recovery_active = true;
                desired_direction = target_error > 0.0f ? 1 : -1;
                request_breakaway = true;
            }
        }
        else
        {
            s_state.friction_stuck_counter = 0U;
            fast_release = true;
        }
    }

    if (desired_direction != s_state.friction_direction)
    {
        s_state.friction_direction = desired_direction;
        s_state.friction_stuck_counter = 0U;
        s_state.friction_breakaway_active = false;
        s_state.friction_breakaway_start_position = measured_position;
    }
    if (request_breakaway)
    {
        s_state.friction_breakaway_active = true;
        s_state.friction_breakaway_start_position = measured_position;
    }

    if (!waiting_for_recovery && desired_direction != 0 && feedback_is_stopped &&
        !s_state.friction_landing_active)
    {
        if (s_state.friction_stuck_counter < UINT32_MAX)
        {
            s_state.friction_stuck_counter++;
        }
        if (!s_state.friction_breakaway_active &&
            (stuck_ticks == 0U || s_state.friction_stuck_counter >= stuck_ticks))
        {
            s_state.friction_breakaway_active = true;
            s_state.friction_breakaway_start_position = measured_position;
        }
    }
    else if (!waiting_for_recovery &&
             (desired_direction == 0 ||
              (!s_state.friction_breakaway_active &&
               (float)desired_direction * measured_speed >= config->friction_move_speed) ||
              (float)desired_direction *
                      (measured_position - s_state.friction_breakaway_start_position) >=
                  config->friction_breakaway_distance))
    {
        s_state.friction_stuck_counter = 0U;
        s_state.friction_breakaway_active = false;
    }

    if (desired_direction != 0)
    {
        remaining_distance = (float)desired_direction * target_error;
        speed_toward_target = (float)desired_direction * measured_speed;
        if (speed_toward_target > 0.0f)
        {
            braking_distance =
                speed_toward_target * speed_toward_target / (2.0f * config->deceleration);
        }
        inside_braking_envelope = remaining_distance <= landing_position + braking_distance;
        controller_requests_braking = speed_toward_target > config->friction_stop_speed &&
                                      (float)desired_direction * s_state.speed_reference <= 0.0f &&
                                      inside_braking_envelope;

        if (!s_state.friction_landing_active &&
            (remaining_distance <= 0.0f || controller_requests_braking ||
             (speed_toward_target > config->friction_stop_speed && inside_braking_envelope)))
        {
            s_state.friction_landing_active = true;
            s_state.settle_recovery_active = false;
            /* v²/(2a) 属于惯性制动距离。若在整个距离上削减库仑补偿，转子会提前停住并二次起步。 */
            s_state.friction_breakaway_active = false;
            s_state.friction_stuck_counter = 0U;
            fast_release = true;
        }

        magnitude = position_cascade_friction_magnitude(config, desired_direction);
        if (s_state.friction_breakaway_active)
        {
            magnitude *= config->friction_breakaway_ratio;
        }
        if (s_state.friction_landing_active)
        {
            /* 进入位置捕获窗口后才卸除动摩擦补偿；保持原电流斜率限制，由速度阻尼制动。 */
            landing_span = landing_position - landing_zero_position;
            if (remaining_distance <= landing_zero_position || landing_span <= 0.0f)
            {
                landing_scale = 0.0f;
            }
            else
            {
                landing_scale = position_cascade_constrain(
                    (remaining_distance - landing_zero_position) / landing_span, 0.0f, 1.0f);
            }
            magnitude *= landing_scale;
            fast_release = true;
        }
        target_current = desired_direction > 0 ? magnitude : -magnitude;
    }

    /* 方向反转时先卸载到零，再施加起步补偿。 */
    if (target_current * s_state.friction_feedforward_current < 0.0f)
    {
        target_current = 0.0f;
        fast_release = true;
    }
    current_delta = target_current - s_state.friction_feedforward_current;
    if (position_cascade_abs(target_current) >
        position_cascade_abs(s_state.friction_feedforward_current))
    {
        current_step = (s_state.settle_recovery_active ? config->friction_release_slew_rate
                                                       : config->friction_attack_slew_rate) *
                       config->update_period_s;
    }
    else if (fast_release)
    {
        float release_rate = config->friction_fast_release_slew_rate;
        /* 捕获窗口内缓释靠近转矩，避免在保持切换时产生电流跳变。 */
        if ((s_state.settling_in_window ||
             (s_state.friction_landing_active &&
              position_cascade_abs(target_error) <= config->hold_enter_position)) &&
            s_state.friction_feedforward_current * target_error > 0.0f)
        {
            release_rate = position_cascade_min(
                release_rate, POSITION_SERVO_FRICTION_CAPTURE_RELEASE_SLEW_A_PER_S);
        }
        current_step = release_rate * config->update_period_s;
    }
    else
    {
        current_step = config->friction_release_slew_rate * config->update_period_s;
    }
    s_state.friction_feedforward_current +=
        position_cascade_constrain(current_delta, -current_step, current_step);
    s_state.friction_feedforward_current = position_cascade_constrain(
        s_state.friction_feedforward_current, -config->current_limit_a, config->current_limit_a);
    if (!isfinite(s_state.friction_feedforward_current))
    {
        s_state.friction_feedforward_current = 0.0f;
    }
    s_state.friction_current_ramping = position_cascade_abs(s_state.friction_feedforward_current) >
                                       position_cascade_abs(previous_current) + 0.000001f;

    if (s_state.phase == POSITION_SERVO_PHASE_SETTLE && s_state.friction_landing_active &&
        landing_scale <= 0.0f &&
        position_cascade_abs(s_state.friction_feedforward_current) <= current_step)
    {
        s_state.friction_landing_active = false;
        s_state.settle_recovery_active = false;
        s_state.friction_direction = 0;
        s_state.friction_stuck_counter = 0U;
    }
}

static void position_cascade_update_integral_transport(const PositionCascadeConfig *config,
                                                       float target_error,
                                                       float speed_error)
{
    float integral_magnitude;
    float unload_step;
    float proportional_step;
    float maximum_step;
    bool opposes_position_correction;
    bool opposes_speed_correction;

    if (!s_state.target_transition_active)
    {
        return;
    }
    opposes_position_correction =
        position_cascade_abs(target_error) > config->hold_enter_position &&
        s_state.speed_controller.integral_output * target_error < 0.0f;
    opposes_speed_correction = position_cascade_abs(speed_error) > config->friction_stop_speed &&
                               s_state.speed_controller.integral_output * speed_error < 0.0f;
    /* 只有积分电流同时阻碍位置与速度纠正时才衰减；仅制动速度误差不清除负载支撑电流。 */
    if (!opposes_position_correction || !opposes_speed_correction)
    {
        return;
    }

    integral_magnitude = position_cascade_abs(s_state.speed_controller.integral_output);
    proportional_step = integral_magnitude * POSITION_SERVO_INTEGRAL_OPPOSING_DECAY_RATE_PER_S *
                        config->update_period_s;
    maximum_step = POSITION_SERVO_INTEGRAL_OPPOSING_MAX_SLEW_A_PER_S * config->update_period_s;
    unload_step = position_cascade_min(integral_magnitude,
                                       position_cascade_min(proportional_step, maximum_step));
    if (s_state.speed_controller.integral_output > 0.0f)
    {
        s_state.speed_controller.integral_output -= unload_step;
    }
    else
    {
        s_state.speed_controller.integral_output += unload_step;
    }

    if (position_cascade_abs(s_state.speed_controller.integral_output) <=
        POSITION_SERVO_INTEGRAL_ZERO_THRESHOLD_A)
    {
        s_state.speed_controller.integral_output = 0.0f;
    }
}

static void position_cascade_update_stiction_integral(const PositionCascadeConfig *config,
                                                      float measured_position,
                                                      float measured_speed,
                                                      float feedforward_current,
                                                      float proportional_gain)
{
    float error = s_state.position_reference - measured_position;
    float target_error = config->target_position - measured_position;
    float current_candidate;
    float increment;
    uint32_t ticks = s_state.configured_stuck_ticks;

    s_state.stiction_integrating = false;
    /* 同时核对参考和目标误差，不向轨迹前方、纠正反向或已确认的保持窗口内继续推力。 */
    if (config->stiction_integral_rate == 0.0f || s_state.settling_in_window ||
        s_state.phase == POSITION_SERVO_PHASE_HOLD ||
        position_cascade_abs(error) <= config->hold_enter_position ||
        position_cascade_abs(target_error) <= config->hold_enter_position ||
        error * target_error <= 0.0f ||
        position_cascade_abs(measured_speed) > config->friction_stop_speed ||
        (s_state.position_stuck_counter > 0U &&
         position_cascade_abs(measured_position - s_state.position_stuck_anchor) >=
             config->friction_breakaway_distance))
    {
        s_state.position_stuck_counter = 0U;
        return;
    }
    if (s_state.position_stuck_counter == 0U)
    {
        s_state.position_stuck_anchor = measured_position;
    }
    if (s_state.position_stuck_counter < UINT32_MAX)
    {
        s_state.position_stuck_counter++;
    }
    if (s_state.position_stuck_counter < ticks || s_state.friction_current_ramping)
    {
        return;
    }

    /* 用现有 PI 学习残余静态负载，不另建累积前馈；恢复运动后由 PI 和反转迁移管理电流。
     * 可用裕量同时扣除前馈和比例项。 */
    increment =
        (error > 0.0f ? 1.0f : -1.0f) * config->stiction_integral_rate * config->update_period_s;
    current_candidate = s_state.speed_controller.integral_output + feedforward_current +
                        proportional_gain * (s_state.speed_reference - measured_speed);
    if (increment > 0.0f)
    {
        increment = position_cascade_max(
            0.0f, position_cascade_min(increment, config->current_limit_a - current_candidate));
    }
    else
    {
        increment = position_cascade_min(
            0.0f, position_cascade_max(increment, -config->current_limit_a - current_candidate));
    }
    s_state.speed_controller.integral_output =
        position_cascade_constrain(s_state.speed_controller.integral_output + increment,
                                   -config->current_limit_a,
                                   config->current_limit_a);
    s_state.stiction_integrating = increment != 0.0f;
}

void PositionCascade_Reset(void)
{
    memset(&s_state, 0, sizeof(s_state));
    PiController_Reset(&s_state.speed_controller);
}

/* 将 2 kHz 控制的寄存器保存栈帧排除在中间快拍之外。 */
#if defined(__GNUC__) || defined(__clang__) || defined(__CC_ARM)
#define POSITION_CONTROL_NOINLINE __attribute__((noinline))
#else
#define POSITION_CONTROL_NOINLINE
#endif
static POSITION_CONTROL_NOINLINE bool position_cascade_run_control(
    const PositionCascadeConfig *config, float measured_position, float measured_speed)
{
    float position_error;
    float velocity_error;
    float feedforward_current;
    float feedforward_candidate;
    float current_candidate;
    float applied_feedback_current;
    float hold_exit_position;
    float speed_kp_current_domain;
    float speed_ki_current_domain;
    float speed_command_limit;
    uint32_t hold_ticks;
    bool trajectory_done;
    bool was_holding = s_state.phase == POSITION_SERVO_PHASE_HOLD;

    s_state.loop_count = 0U;
    s_state.defer_telemetry = true;
    if (config->velocity_filter_hz > 0.0f)
    {
        s_state.velocity_filtered +=
            s_state.configured_velocity_filter_alpha * (measured_speed - s_state.velocity_filtered);
    }
    else
    {
        s_state.velocity_filtered = measured_speed;
    }
    measured_speed = s_state.velocity_filtered;
    if (!was_holding)
    {
        s_state.control_velocity = measured_speed;
    }

    FAST_PROFILE_BEGIN(FAST_PROFILE_TRAJECTORY);
    trajectory_done = position_cascade_update_trajectory(config, measured_position);
    FAST_PROFILE_END(FAST_PROFILE_TRAJECTORY);
    if (s_state.smooth_trajectory.failed || !isfinite(s_state.position_reference) ||
        !isfinite(s_state.trajectory_speed) || !isfinite(s_state.trajectory_acceleration))
    {
        return false;
    }
    /* 规划拆成有界拍，静止参考保持已有支撑电流；规划启动前不施加起步补偿。 */
    if (s_state.smooth_planning)
    {
        return true;
    }
    if (s_state.phase == POSITION_SERVO_PHASE_MOVE && trajectory_done)
    {
        s_state.phase = POSITION_SERVO_PHASE_SETTLE;
    }

    hold_exit_position = config->hold_exit_position;
    /* 确认 HOLD 前先平缓前馈，并在噪声中保留此锁存。 */
    if (position_cascade_abs(config->target_position - s_state.position_reference) >
            hold_exit_position ||
        position_cascade_abs(config->target_position - measured_position) > hold_exit_position)
    {
        s_state.settling_in_window = false;
    }
    else if (position_cascade_abs(config->target_position - measured_position) <=
                 config->hold_enter_position &&
             position_cascade_abs(config->target_position - s_state.position_reference) <=
                 config->hold_enter_position)
    {
        s_state.settling_in_window = true;
    }
    if (s_state.phase == POSITION_SERVO_PHASE_HOLD)
    {
        if (position_cascade_abs(config->target_position - measured_position) >
                hold_exit_position ||
            position_cascade_abs(measured_speed) > POSITION_SERVO_HOLD_EXIT_SPEED_RAD_S)
        {
            s_state.phase = POSITION_SERVO_PHASE_SETTLE;
            s_state.target_reached = false;
            s_state.hold_counter = 0U;
            /* 保持丢失时开启新纠正过程，即使命令未变，也重新允许有界反向积分迁移；保留有效负载支撑。 */
            s_state.target_transition_active = true;
        }
    }
    else if (trajectory_done)
    {
        hold_ticks = s_state.configured_hold_ticks;
        if ((s_state.hold_counter == 0U && s_state.settling_in_window &&
             position_cascade_abs(config->target_position - measured_position) <=
                 hold_exit_position &&
             position_cascade_abs(measured_speed) <= POSITION_SERVO_HOLD_ENTER_SPEED_RAD_S &&
             position_cascade_abs(s_state.friction_feedforward_current) <=
                 config->friction_release_slew_rate * config->update_period_s) ||
            (s_state.hold_counter > 0U &&
             position_cascade_abs(config->target_position - measured_position) <=
                 hold_exit_position &&
             position_cascade_abs(measured_speed) <= POSITION_SERVO_HOLD_EXIT_SPEED_RAD_S &&
             position_cascade_abs(s_state.friction_feedforward_current) <=
                 config->friction_release_slew_rate * config->update_period_s))
        {
            if (s_state.hold_counter < hold_ticks)
            {
                s_state.hold_counter++;
            }
            if (s_state.hold_counter >= hold_ticks)
            {
                s_state.phase = POSITION_SERVO_PHASE_HOLD;
                s_state.target_reached = true;
                s_state.target_transition_active = false;
                s_state.settle_recovery_active = false;
                s_state.hold_counter = 0U;
            }
        }
        else
        {
            s_state.hold_counter = 0U;
        }
    }
    else
    {
        s_state.hold_counter = 0U;
    }

    /* HOLD 资格使用原始速度；进入时初始化滤波，退出或换目标时旁路。
     * 只滤波阻尼与比例速度路径，不设位置死区，也不删除负载支撑电流。 */
    if (s_state.phase == POSITION_SERVO_PHASE_HOLD && was_holding &&
        config->hold_velocity_filter_hz > 0.0f)
    {
        s_state.control_velocity += s_state.configured_hold_velocity_filter_alpha *
                                    (measured_speed - s_state.control_velocity);
    }
    else
    {
        s_state.control_velocity = measured_speed;
    }
    position_error = s_state.position_reference - measured_position;
    velocity_error = s_state.trajectory_speed - s_state.control_velocity;
    s_state.speed_reference = s_state.trajectory_speed + config->position_kp * position_error +
                              config->position_kd * velocity_error;
    /* 落后轴允许略高于轨迹巡航的追赶速度；独立限制该裕量并遵守电机速度上限。 */
    speed_command_limit = position_cascade_min(config->speed_limit_rad_s,
                                               config->maximum_speed *
                                                   POSITION_SERVO_SPEED_CORRECTION_HEADROOM_RATIO);
    s_state.speed_reference = position_cascade_constrain(
        s_state.speed_reference, -speed_command_limit, speed_command_limit);

    if (!s_state.settling_in_window)
    {
        position_cascade_update_integral_transport(config,
                                                   config->target_position - measured_position,
                                                   s_state.speed_reference - measured_speed);
    }
    FAST_PROFILE_BEGIN(FAST_PROFILE_FRICTION);
    position_cascade_update_friction(config, measured_position, measured_speed);
    FAST_PROFILE_END(FAST_PROFILE_FRICTION);
    s_state.acceleration_feedforward_current = position_cascade_constrain(
        config->acceleration_feedforward_gain * s_state.trajectory_acceleration,
        -config->current_limit_a,
        config->current_limit_a);
    feedforward_candidate =
        s_state.acceleration_feedforward_current + s_state.friction_feedforward_current;
    feedforward_current = position_cascade_constrain(
        feedforward_candidate, -config->current_limit_a, config->current_limit_a);

    /* PI 使用安培域计算，保留原速度增益调参语义。 */
    speed_kp_current_domain = config->speed_kp * config->current_limit_a;
    speed_ki_current_domain = config->speed_ki * config->current_limit_a;
    position_cascade_update_stiction_integral(
        config, measured_position, measured_speed, feedforward_current, speed_kp_current_domain);
    /* 仅前馈建立期间暂停同方向积分；达到模型值后仍允许修正残余负载。 */
    if ((s_state.friction_current_ramping &&
         s_state.friction_feedforward_current * (s_state.speed_reference - measured_speed) >
             0.0f) ||
        s_state.phase == POSITION_SERVO_PHASE_HOLD ||
        (s_state.settling_in_window &&
         position_cascade_abs(measured_speed) <= POSITION_SERVO_HOLD_ENTER_SPEED_RAD_S))
    {
        speed_ki_current_domain = 0.0f;
    }
    FAST_PROFILE_BEGIN(FAST_PROFILE_SPEED_PI);
    PiController_Configure(&s_state.speed_controller,
                           speed_kp_current_domain,
                           speed_ki_current_domain,
                           config->update_period_s,
                           -config->current_limit_a,
                           config->current_limit_a);
    s_state.feedback_current = PiController_Run(
        &s_state.speed_controller, s_state.speed_reference, s_state.control_velocity);
    current_candidate = s_state.feedback_current + feedforward_current;
    if (!isfinite(current_candidate))
    {
        FAST_PROFILE_END(FAST_PROFILE_SPEED_PI);
        return false;
    }
    s_state.current_saturated = feedforward_candidate > config->current_limit_a ||
                                feedforward_candidate < -config->current_limit_a ||
                                current_candidate > config->current_limit_a ||
                                current_candidate < -config->current_limit_a;
    s_state.iq_reference = position_cascade_constrain(
        current_candidate, -config->current_limit_a, config->current_limit_a);

    /* 总电流饱和纳入速度 PI 的抗饱和反馈。 */
    applied_feedback_current =
        position_cascade_constrain(s_state.iq_reference - feedforward_current,
                                   -config->current_limit_a,
                                   config->current_limit_a);
    PiController_TrackOutput(&s_state.speed_controller, applied_feedback_current);
    s_state.feedback_current = s_state.speed_controller.output;
    FAST_PROFILE_END(FAST_PROFILE_SPEED_PI);
    return true;
}

static bool position_cascade_run_validated(const PositionCascadeConfig *config,
                                           float measured_position,
                                           float measured_speed)
{
    if (!s_state.initialized)
    {
        /* 静态初始化或 Reset 已清状态；保留刚校验的配置，避免模式进入时再次复位。 */
        s_state.initialized = true;
        s_state.last_target = config->target_position;
        s_state.position_reference = measured_position;
        s_state.velocity_filtered = measured_speed;
        s_state.control_velocity = measured_speed;
        s_state.friction_breakaway_start_position = measured_position;
        s_state.target_transition_active =
            position_cascade_abs(config->target_position - measured_position) >
            config->position_error_window;
        s_state.phase = s_state.target_transition_active ? POSITION_SERVO_PHASE_MOVE
                                                         : POSITION_SERVO_PHASE_SETTLE;
    }
    else if (s_state.last_target != config->target_position)
    {
        s_state.last_target = config->target_position;
        s_state.phase = POSITION_SERVO_PHASE_MOVE;
        s_state.target_reached = false;
        s_state.hold_counter = 0U;
        s_state.friction_stuck_counter = 0U;
        s_state.friction_breakaway_active = false;
        s_state.friction_landing_active = false;
        s_state.settle_recovery_active = false;
        s_state.target_transition_active = true;
        s_state.settling_in_window = false;
        s_state.position_stuck_counter = 0U;
        s_state.stiction_integrating = false;
    }

    if (++s_state.loop_count < config->call_divider)
    {
        return true;
    }
    return position_cascade_run_control(config, measured_position, measured_speed);
}

const PositionCascadeConfig *PositionCascade_GetConfiguration(void)
{
    return s_state.config_valid ? &s_state.validated_config : NULL;
}

static void position_cascade_copy_control_output(PositionCascadeControlOutput *output)
{
    output->position_reference = s_state.position_reference;
    output->speed_reference = s_state.speed_reference;
    output->speed_feedback_rad_s = s_state.control_velocity;
    output->iq_reference = s_state.iq_reference;
    output->target_reached = s_state.target_reached;
}

bool PositionCascade_UpdateControl(const PositionCascadeConfig *config,
                                   float measured_position,
                                   float measured_speed,
                                   PositionCascadeControlOutput *output)
{
    s_state.defer_telemetry = false;
    if (output == NULL || !isfinite(measured_position) || !isfinite(measured_speed) ||
        !position_cascade_check_config(config))
    {
        return false;
    }
    if (!position_cascade_run_validated(config, measured_position, measured_speed))
    {
        return false;
    }
    position_cascade_copy_control_output(output);
    return true;
}

bool PositionCascade_UpdateTargetControl(float target_position,
                                         float measured_position,
                                         float measured_speed,
                                         PositionCascadeControlOutput *output)
{
    s_state.defer_telemetry = false;
    if (!s_state.config_valid || output == NULL || !isfinite(target_position) ||
        !isfinite(measured_position) || !isfinite(measured_speed))
    {
        return false;
    }
    s_state.validated_config.target_position = target_position;
    if (!position_cascade_run_validated(
            &s_state.validated_config, measured_position, measured_speed))
    {
        return false;
    }
    position_cascade_copy_control_output(output);
    return true;
}
