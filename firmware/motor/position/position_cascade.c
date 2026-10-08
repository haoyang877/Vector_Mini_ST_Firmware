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

/* 闭环核心只拥有 PI 与最终电流；参考/反馈结果直接借用 PI 字段，不重复缓存。 */
typedef struct
{
    PiController speed_controller;
    float iq_reference_a;
    bool current_saturated;
} PositionCascadeCoreState;

typedef struct
{
    float position_kp_per_s;
    float position_kd;
    float maximum_speed_rad_s;
    float speed_limit_rad_s;
} PositionCascadeCoreSpeedConfig;

typedef struct
{
    float position_reference_rad;
    float trajectory_speed_rad_s;
    float measured_position_rad;
    float control_velocity_rad_s;
} PositionCascadeCoreMotion;

typedef struct
{
    float speed_reference_rad_s;
    float control_velocity_rad_s;
    float proportional_gain_a_per_rad_s;
    float integral_gain_a_per_rad;
    float feedforward_candidate_a;
    float feedforward_current_a;
} PositionCascadeCoreCurrent;

typedef struct
{
    float speed_reference_rad_s;
    float feedback_current_a;
    float integral_current_a;
    float iq_reference_a;
    bool current_saturated;
} PositionCascadeCoreOutput;

typedef struct
{
    float unload_step_a;
    bool apply;
} PositionCascadeIntegralTransport;

typedef struct
{
    float increment_a;
    bool apply;
} PositionCascadeStaticIntegral;

/* 摩擦状态只由本模块的复位、事件及更新入口写入，不借用 PI 或伺服状态。 */
typedef struct
{
    float current_a;
    float breakaway_start_position_rad;
    uint32_t stuck_counter;
    int8_t direction;
    bool breakaway_active;
    bool landing_active;
    bool recovery_active;
    bool current_ramping;
} PositionCascadeFrictionState;

/* 只读摩擦调参：rad、rad/s、s 和 A；不包含闭环增益或 PI 状态。 */
typedef struct
{
    float current_limit_a;
    float deceleration_rad_s2;
    float attack_slew_a_per_s;
    float breakaway_distance_rad;
    float breakaway_ratio;
    float coulomb_negative_a;
    float coulomb_positive_a;
    float fast_release_slew_a_per_s;
    bool enabled;
    float move_speed_rad_s;
    float reference_speed_rad_s;
    float release_slew_a_per_s;
    float stop_speed_rad_s;
    float viscous_negative_a_per_rad_s;
    float viscous_positive_a_per_rad_s;
    float capture_enter_rad;
    float capture_exit_rad;
    float reference_finish_window_rad;
    float update_period_s;
} PositionCascadeFrictionConfig;

typedef struct
{
    float target_position_rad;
    float position_reference_rad;
    float trajectory_speed_rad_s;
    float measured_position_rad;
    float measured_speed_rad_s;
    uint32_t stuck_ticks;
    bool holding;
    bool moving;
    bool settling;
    bool capture_active;
} PositionCascadeFrictionInput;

typedef struct
{
    float current_a;
    bool landing_active;
    bool recovery_active;
    bool current_ramping;
} PositionCascadeFrictionOutput;

typedef struct
{
    PositionCascadeConfig validated_config;
    bool config_valid;
    uint32_t configured_hold_ticks;
    uint32_t configured_stuck_ticks;
    float configured_velocity_filter_alpha;
    float configured_hold_velocity_filter_alpha;
    PositionSmoothTrajectory smooth_trajectory;
    bool smooth_active;
    bool smooth_planning;
    uint16_t loop_count;
    uint32_t hold_counter;
    uint32_t position_stuck_counter;
    float position_stuck_anchor;
    float last_target;
    float position_reference;
    float trajectory_speed;
    float trajectory_acceleration;
    float velocity_filtered;
    float control_velocity;
    float acceleration_feedforward_current;
    PositionServoPhase phase;
    bool target_transition_active;
    bool target_reached;
    bool settling_in_window;
    bool trajectory_limited;
    bool stiction_integrating;
    bool initialized;
    bool defer_telemetry;
    PositionCascadeCoreState core;
    PositionCascadeFrictionState friction;
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

/* 输入均来自已校验配置；核心不认识目标、轨迹阶段、摩擦模型或硬件。 */
static void position_cascade_core_reset(PositionCascadeCoreState *state)
{
    memset(state, 0, sizeof(*state));
}

static PositionCascadeCoreOutput position_cascade_core_output(const PositionCascadeCoreState *state)
{
    PositionCascadeCoreOutput output;
    output.speed_reference_rad_s = state->speed_controller.reference;
    output.feedback_current_a = state->speed_controller.output;
    output.integral_current_a = state->speed_controller.integral_output;
    output.iq_reference_a = state->iq_reference_a;
    output.current_saturated = state->current_saturated;
    return output;
}

static float position_cascade_core_speed(const PositionCascadeCoreSpeedConfig *config,
                                         const PositionCascadeCoreMotion *input)
{
    float position_error = input->position_reference_rad - input->measured_position_rad;
    float velocity_error = input->trajectory_speed_rad_s - input->control_velocity_rad_s;
    float speed_reference = input->trajectory_speed_rad_s +
                            config->position_kp_per_s * position_error +
                            config->position_kd * velocity_error;
    float speed_command_limit = position_cascade_min(
        config->speed_limit_rad_s,
        config->maximum_speed_rad_s * POSITION_SERVO_SPEED_CORRECTION_HEADROOM_RATIO);
    return position_cascade_constrain(speed_reference, -speed_command_limit, speed_command_limit);
}

static void position_cascade_core_apply_transport(PositionCascadeCoreState *state,
                                                  PositionCascadeIntegralTransport request)
{
    if (!request.apply)
    {
        return;
    }
    if (state->speed_controller.integral_output > 0.0f)
    {
        state->speed_controller.integral_output -= request.unload_step_a;
    }
    else
    {
        state->speed_controller.integral_output += request.unload_step_a;
    }
    if (position_cascade_abs(state->speed_controller.integral_output) <=
        POSITION_SERVO_INTEGRAL_ZERO_THRESHOLD_A)
    {
        state->speed_controller.integral_output = 0.0f;
    }
}

static void position_cascade_core_apply_static_integral(PositionCascadeCoreState *state,
                                                        PositionCascadeStaticIntegral request,
                                                        float current_limit_a)
{
    if (!request.apply)
    {
        return;
    }
    state->speed_controller.integral_output =
        position_cascade_constrain(state->speed_controller.integral_output + request.increment_a,
                                   -current_limit_a,
                                   current_limit_a);
}

/* 参数对象与状态不重叠；只组合安培域反馈与显式前馈。
 * 跟踪限幅后的反馈份额，失败不覆盖旧 Iq/饱和标志。 */
static bool position_cascade_core_run_current(PositionCascadeCoreState *restrict state,
                                              const PositionCascadeCoreCurrent *restrict input,
                                              float update_period_s,
                                              float current_limit_a)
{
    float feedback_current;
    float current_candidate;
    float applied_feedback_current;
    PiController_Configure(&state->speed_controller,
                           input->proportional_gain_a_per_rad_s,
                           input->integral_gain_a_per_rad,
                           update_period_s,
                           -current_limit_a,
                           current_limit_a);
    feedback_current = PiController_Run(
        &state->speed_controller, input->speed_reference_rad_s, input->control_velocity_rad_s);
    current_candidate = feedback_current + input->feedforward_current_a;
    if (!isfinite(current_candidate))
    {
        return false;
    }
    state->current_saturated = input->feedforward_candidate_a > current_limit_a ||
                               input->feedforward_candidate_a < -current_limit_a ||
                               current_candidate > current_limit_a ||
                               current_candidate < -current_limit_a;
    state->iq_reference_a =
        position_cascade_constrain(current_candidate, -current_limit_a, current_limit_a);
    applied_feedback_current = position_cascade_constrain(
        state->iq_reference_a - input->feedforward_current_a, -current_limit_a, current_limit_a);
    PiController_TrackOutput(&state->speed_controller, applied_feedback_current);
    return true;
}

static void position_cascade_friction_reset(PositionCascadeFrictionState *state)
{
    memset(state, 0, sizeof(*state));
}

static void position_cascade_friction_initialize(PositionCascadeFrictionState *state,
                                                 float measured_position_rad)
{
    state->breakaway_start_position_rad = measured_position_rad;
}

static void position_cascade_friction_target_changed(PositionCascadeFrictionState *state)
{
    state->stuck_counter = 0U;
    state->breakaway_active = false;
    state->landing_active = false;
    state->recovery_active = false;
}

static void position_cascade_friction_hold_entered(PositionCascadeFrictionState *state)
{
    state->recovery_active = false;
}

static PositionCascadeFrictionOutput
position_cascade_friction_output(const PositionCascadeFrictionState *state)
{
    PositionCascadeFrictionOutput output;
    output.current_a = state->current_a;
    output.landing_active = state->landing_active;
    output.recovery_active = state->recovery_active;
    output.current_ramping = state->current_ramping;
    return output;
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
    static const struct
    {
        uint8_t offset;
        bool strictly_positive;
    } field_rules[] = {
        {offsetof(PositionCascadeConfig, update_period_s), true},
        {offsetof(PositionCascadeConfig, position_error_window), true},
        {offsetof(PositionCascadeConfig, hold_enter_position), false},
        {offsetof(PositionCascadeConfig, hold_exit_position), false},
        {offsetof(PositionCascadeConfig, velocity_filter_hz), false},
        {offsetof(PositionCascadeConfig, hold_velocity_filter_hz), false},
        {offsetof(PositionCascadeConfig, following_error_limit), false},
        {offsetof(PositionCascadeConfig, stiction_integral_rate), false},
        {offsetof(PositionCascadeConfig, acceleration), true},
        {offsetof(PositionCascadeConfig, deceleration), true},
        {offsetof(PositionCascadeConfig, maximum_speed), true},
        {offsetof(PositionCascadeConfig, speed_limit_rad_s), true},
        {offsetof(PositionCascadeConfig, jerk_limit), true},
        {offsetof(PositionCascadeConfig, position_kp), false},
        {offsetof(PositionCascadeConfig, position_kd), false},
        {offsetof(PositionCascadeConfig, speed_kp), false},
        {offsetof(PositionCascadeConfig, speed_ki), false},
        {offsetof(PositionCascadeConfig, acceleration_feedforward_gain), false},
        {offsetof(PositionCascadeConfig, current_limit_a), true},
        {offsetof(PositionCascadeConfig, friction_coulomb_positive), false},
        {offsetof(PositionCascadeConfig, friction_coulomb_negative), false},
        {offsetof(PositionCascadeConfig, friction_viscous_positive), false},
        {offsetof(PositionCascadeConfig, friction_viscous_negative), false},
        {offsetof(PositionCascadeConfig, friction_breakaway_ratio), false},
        {offsetof(PositionCascadeConfig, friction_attack_slew_rate), true},
        {offsetof(PositionCascadeConfig, friction_fast_release_slew_rate), true},
        {offsetof(PositionCascadeConfig, friction_release_slew_rate), true},
        {offsetof(PositionCascadeConfig, friction_reference_speed), true},
        {offsetof(PositionCascadeConfig, friction_stop_speed), false},
        {offsetof(PositionCascadeConfig, friction_move_speed), false},
        {offsetof(PositionCascadeConfig, friction_breakaway_distance), true},
        {offsetof(PositionCascadeConfig, friction_stuck_time), false},
    };
    size_t index;
    /* 调参字段均要求有限且非负；严格正值同时拒绝 +0/-0。
     * 仅在配置变化时扫描明确列出的 float，不检查结构填充字节。 */
    if (config == NULL)
    {
        return false;
    }
    for (index = 0; index < sizeof(field_rules) / sizeof(field_rules[0]); ++index)
    {
        float value;
        memcpy(&value, (const unsigned char *)config + field_rules[index].offset, sizeof(value));
        if (!isfinite(value) || value < 0.0f ||
            (field_rules[index].strictly_positive && value == 0.0f))
        {
            return false;
        }
    }
    /* 目标允许负值；字段间关系、上界和派生乘积仍显式检查。 */
    return isfinite(config->target_position) && config->call_divider > 0U &&
           config->hold_enter_position >= config->position_error_window &&
           config->hold_exit_position > config->hold_enter_position &&
           config->velocity_filter_hz * config->update_period_s <= 0.5f &&
           config->hold_velocity_filter_hz * config->update_period_s <= 0.5f &&
           isfinite(config->stiction_integral_rate * config->update_period_s) &&
           config->maximum_speed <= config->speed_limit_rad_s &&
           config->position_kp <= CASCADE_POSITION_KP_MAX_PER_S &&
           config->position_kd <= CASCADE_POSITION_KD_MAX &&
           config->friction_breakaway_ratio >= 1.0f &&
           config->friction_move_speed > config->friction_stop_speed;
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

    PositionCascadeCoreOutput core = position_cascade_core_output(&s_state.core);
    PositionCascadeFrictionOutput friction = position_cascade_friction_output(&s_state.friction);

    telemetry->position_reference = s_state.position_reference;
    telemetry->trajectory_speed_reference = s_state.trajectory_speed;
    telemetry->speed_command = core.speed_reference_rad_s;
    telemetry->speed_feedback_rad_s = s_state.control_velocity;
    telemetry->acceleration_reference = s_state.trajectory_acceleration;
    telemetry->feedback_current = core.feedback_current_a;
    telemetry->acceleration_feedforward_current = s_state.acceleration_feedforward_current;
    telemetry->friction_feedforward_current = friction.current_a;
    telemetry->hold_current = core.integral_current_a;
    telemetry->phase = s_state.phase;
    telemetry->target_reached = s_state.target_reached;
    telemetry->current_saturated = core.current_saturated;
    telemetry->friction_landing_active = friction.landing_active;
    telemetry->settle_recovery_active = friction.recovery_active;
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

static float position_cascade_friction_magnitude(const PositionCascadeFrictionConfig *config,
                                                 float trajectory_speed_rad_s,
                                                 int8_t direction)
{
    float speed = position_cascade_abs(trajectory_speed_rad_s);

    if (direction > 0)
    {
        return config->coulomb_positive_a + config->viscous_positive_a_per_rad_s * speed;
    }
    if (direction < 0)
    {
        return config->coulomb_negative_a + config->viscous_negative_a_per_rad_s * speed;
    }
    return 0.0f;
}

/* 状态与窄输入不重叠，输入只在当前控制拍借用；内联消除栈复制，状态边界保持独立。 */
#if defined(__CC_ARM)
#define POSITION_CASCADE_FRICTION_INLINE __forceinline
#elif defined(__GNUC__) || defined(__clang__)
#define POSITION_CASCADE_FRICTION_INLINE inline __attribute__((always_inline))
#else
#define POSITION_CASCADE_FRICTION_INLINE inline
#endif
static POSITION_CASCADE_FRICTION_INLINE void
position_cascade_friction_update(PositionCascadeFrictionState *restrict state,
                                 const PositionCascadeFrictionConfig *restrict config,
                                 const PositionCascadeFrictionInput *restrict input)
{
    float measured_position = input->measured_position_rad;
    float measured_speed = input->measured_speed_rad_s;
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
    float previous_current = state->current_a;
    uint32_t stuck_ticks;
    int8_t desired_direction = 0;
    bool feedback_is_stopped;
    bool request_breakaway = false;
    bool inside_braking_envelope = false;
    bool fast_release = false;
    bool waiting_for_recovery = false;

    if (!config->enabled)
    {
        state->current_a = 0.0f;
        state->direction = 0;
        state->stuck_counter = 0U;
        state->breakaway_active = false;
        state->landing_active = false;
        state->recovery_active = false;
        state->current_ramping = false;
        return;
    }

    target_error = input->target_position_rad - measured_position;
    landing_position = config->capture_enter_rad;
    landing_zero_position = landing_position * POSITION_SERVO_FRICTION_LANDING_ZERO_RATIO;
    feedback_is_stopped = position_cascade_abs(measured_speed) <= config->stop_speed_rad_s;
    stuck_ticks = input->stuck_ticks;

    if (input->holding || input->capture_active)
    {
        state->stuck_counter = 0U;
        state->breakaway_active = false;
        state->landing_active = false;
        state->recovery_active = false;
        fast_release = true;
    }
    else if (state->landing_active)
    {
        /* 着陆衰减期间锁存运动方向。转子在目标窗口外停住时，以有界起步斜坡恢复，
         * 即使参考仍在运动，也不等待轨迹结束。 */
        if (position_cascade_abs(target_error) > config->capture_exit_rad && feedback_is_stopped)
        {
            waiting_for_recovery = true;
            if (state->stuck_counter < UINT32_MAX)
            {
                state->stuck_counter++;
            }
            if (stuck_ticks == 0U || state->stuck_counter >= stuck_ticks)
            {
                state->landing_active = false;
                state->recovery_active = true;
                desired_direction = target_error > 0.0f ? 1 : -1;
                request_breakaway = true;
            }
            else
            {
                desired_direction = state->direction;
            }
        }
        else
        {
            state->stuck_counter = 0U;
            desired_direction = state->direction;
        }
        fast_release = true;
    }
    else if (input->moving)
    {
        if (position_cascade_abs(input->trajectory_speed_rad_s) >= config->reference_speed_rad_s)
        {
            desired_direction = input->trajectory_speed_rad_s > 0.0f ? 1 : -1;
        }
        else if (position_cascade_abs(input->target_position_rad - input->position_reference_rad) >
                 config->reference_finish_window_rad)
        {
            desired_direction = input->target_position_rad > input->position_reference_rad ? 1 : -1;
        }
        else if (position_cascade_abs(target_error) > config->capture_enter_rad)
        {
            /* 参考接近停止但转子仍落后时，保持 MOVE/SETTLE 间的靠近补偿。 */
            desired_direction = target_error > 0.0f ? 1 : -1;
        }
    }
    else if (input->settling)
    {
        if (state->recovery_active)
        {
            if (position_cascade_abs(target_error) > config->capture_enter_rad)
            {
                /* 锁存方向，使越过目标时进入着陆。 */
                desired_direction =
                    state->direction != 0 ? state->direction : (target_error > 0.0f ? 1 : -1);
            }
            else
            {
                state->recovery_active = false;
                state->breakaway_active = false;
                fast_release = true;
            }
        }
        else if (position_cascade_abs(target_error) > config->capture_exit_rad &&
                 feedback_is_stopped)
        {
            waiting_for_recovery = true;
            fast_release = true;
            if (state->stuck_counter < UINT32_MAX)
            {
                state->stuck_counter++;
            }
            if (stuck_ticks == 0U || state->stuck_counter >= stuck_ticks)
            {
                state->recovery_active = true;
                desired_direction = target_error > 0.0f ? 1 : -1;
                request_breakaway = true;
            }
        }
        else
        {
            state->stuck_counter = 0U;
            fast_release = true;
        }
    }

    if (desired_direction != state->direction)
    {
        state->direction = desired_direction;
        state->stuck_counter = 0U;
        state->breakaway_active = false;
        state->breakaway_start_position_rad = measured_position;
    }
    if (request_breakaway)
    {
        state->breakaway_active = true;
        state->breakaway_start_position_rad = measured_position;
    }

    if (!waiting_for_recovery && desired_direction != 0 && feedback_is_stopped &&
        !state->landing_active)
    {
        if (state->stuck_counter < UINT32_MAX)
        {
            state->stuck_counter++;
        }
        if (!state->breakaway_active && (stuck_ticks == 0U || state->stuck_counter >= stuck_ticks))
        {
            state->breakaway_active = true;
            state->breakaway_start_position_rad = measured_position;
        }
    }
    else if (!waiting_for_recovery &&
             (desired_direction == 0 ||
              (!state->breakaway_active &&
               (float)desired_direction * measured_speed >= config->move_speed_rad_s) ||
              (float)desired_direction *
                      (measured_position - state->breakaway_start_position_rad) >=
                  config->breakaway_distance_rad))
    {
        state->stuck_counter = 0U;
        state->breakaway_active = false;
    }

    if (desired_direction != 0)
    {
        remaining_distance = (float)desired_direction * target_error;
        speed_toward_target = (float)desired_direction * measured_speed;
        if (speed_toward_target > 0.0f)
        {
            braking_distance =
                speed_toward_target * speed_toward_target / (2.0f * config->deceleration_rad_s2);
        }
        inside_braking_envelope = remaining_distance <= landing_position + braking_distance;
        if (!state->landing_active &&
            (remaining_distance <= 0.0f ||
             (speed_toward_target > config->stop_speed_rad_s && inside_braking_envelope)))
        {
            state->landing_active = true;
            state->recovery_active = false;
            /* v²/(2a) 属于惯性制动距离。若在整个距离上削减库仑补偿，转子会提前停住并二次起步。 */
            state->breakaway_active = false;
            state->stuck_counter = 0U;
            fast_release = true;
        }

        magnitude = position_cascade_friction_magnitude(
            config, input->trajectory_speed_rad_s, desired_direction);
        if (state->breakaway_active)
        {
            magnitude *= config->breakaway_ratio;
        }
        if (state->landing_active)
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
    if (target_current * state->current_a < 0.0f)
    {
        target_current = 0.0f;
        fast_release = true;
    }
    current_delta = target_current - state->current_a;
    if (position_cascade_abs(target_current) > position_cascade_abs(state->current_a))
    {
        current_step =
            (state->recovery_active ? config->release_slew_a_per_s : config->attack_slew_a_per_s) *
            config->update_period_s;
    }
    else if (fast_release)
    {
        float release_rate = config->fast_release_slew_a_per_s;
        /* 捕获窗口内缓释靠近转矩，避免在保持切换时产生电流跳变。 */
        if ((input->capture_active ||
             (state->landing_active &&
              position_cascade_abs(target_error) <= config->capture_enter_rad)) &&
            state->current_a * target_error > 0.0f)
        {
            release_rate = position_cascade_min(
                release_rate, POSITION_SERVO_FRICTION_CAPTURE_RELEASE_SLEW_A_PER_S);
        }
        current_step = release_rate * config->update_period_s;
    }
    else
    {
        current_step = config->release_slew_a_per_s * config->update_period_s;
    }
    state->current_a += position_cascade_constrain(current_delta, -current_step, current_step);
    state->current_a = position_cascade_constrain(
        state->current_a, -config->current_limit_a, config->current_limit_a);
    if (!isfinite(state->current_a))
    {
        state->current_a = 0.0f;
    }
    state->current_ramping =
        position_cascade_abs(state->current_a) > position_cascade_abs(previous_current) + 0.000001f;

    if (input->settling && state->landing_active && landing_scale <= 0.0f &&
        position_cascade_abs(state->current_a) <= current_step)
    {
        state->landing_active = false;
        state->recovery_active = false;
        state->direction = 0;
        state->stuck_counter = 0U;
    }
}

static PositionCascadeIntegralTransport
position_cascade_integral_transport_request(const PositionCascadeConfig *config,
                                            float target_error,
                                            float speed_error,
                                            float integral_current_a,
                                            bool target_transition_active)
{
    PositionCascadeIntegralTransport request = {0};
    float integral_magnitude;
    float unload_step;
    float proportional_step;
    float maximum_step;
    bool opposes_position_correction;
    bool opposes_speed_correction;

    if (!target_transition_active)
    {
        return request;
    }
    opposes_position_correction =
        position_cascade_abs(target_error) > config->hold_enter_position &&
        integral_current_a * target_error < 0.0f;
    opposes_speed_correction = position_cascade_abs(speed_error) > config->friction_stop_speed &&
                               integral_current_a * speed_error < 0.0f;
    /* 只有积分电流同时阻碍位置与速度纠正时才衰减；仅制动速度误差不清除负载支撑电流。 */
    if (!opposes_position_correction || !opposes_speed_correction)
    {
        return request;
    }

    integral_magnitude = position_cascade_abs(integral_current_a);
    proportional_step = integral_magnitude * POSITION_SERVO_INTEGRAL_OPPOSING_DECAY_RATE_PER_S *
                        config->update_period_s;
    maximum_step = POSITION_SERVO_INTEGRAL_OPPOSING_MAX_SLEW_A_PER_S * config->update_period_s;
    unload_step = position_cascade_min(integral_magnitude,
                                       position_cascade_min(proportional_step, maximum_step));
    request.unload_step_a = unload_step;
    request.apply = true;
    return request;
}

static PositionCascadeStaticIntegral
position_cascade_static_integral_request(const PositionCascadeConfig *config,
                                         float measured_position,
                                         float measured_speed,
                                         float feedforward_current,
                                         float proportional_gain,
                                         float integral_current_a,
                                         float speed_reference_rad_s,
                                         bool friction_current_ramping)
{
    PositionCascadeStaticIntegral request = {0};
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
        return request;
    }
    if (s_state.position_stuck_counter == 0U)
    {
        s_state.position_stuck_anchor = measured_position;
    }
    if (s_state.position_stuck_counter < UINT32_MAX)
    {
        s_state.position_stuck_counter++;
    }
    if (s_state.position_stuck_counter < ticks || friction_current_ramping)
    {
        return request;
    }

    /* 用现有 PI 学习残余静态负载，不另建累积前馈；恢复运动后由 PI 和反转迁移管理电流。
     * 可用裕量同时扣除前馈和比例项。 */
    increment =
        (error > 0.0f ? 1.0f : -1.0f) * config->stiction_integral_rate * config->update_period_s;
    current_candidate = integral_current_a + feedforward_current +
                        proportional_gain * (speed_reference_rad_s - measured_speed);
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
    request.increment_a = increment;
    request.apply = true;
    s_state.stiction_integrating = increment != 0.0f;
    return request;
}

void PositionCascade_Reset(void)
{
    /* 配置/调度区在 core 前；子对象由各自所有者复位，不重复清零 PI。 */
    memset(&s_state, 0, offsetof(PositionCascadeState, core));
    position_cascade_core_reset(&s_state.core);
    position_cascade_friction_reset(&s_state.friction);
}

/* 将 2 kHz 控制的寄存器保存栈帧排除在中间快拍之外。 */
#if defined(__GNUC__) || defined(__clang__) || defined(__CC_ARM)
#define POSITION_CONTROL_NOINLINE __attribute__((noinline))
#else
#define POSITION_CONTROL_NOINLINE
#endif
static POSITION_CONTROL_NOINLINE bool position_cascade_run_control(
    const PositionCascadeConfig *restrict config, float measured_position, float measured_speed)
{
    float speed_reference;
    float feedforward_current;
    float feedforward_candidate;
    float hold_exit_position;
    float speed_kp_current_domain;
    float speed_ki_current_domain;
    PositionCascadeFrictionOutput friction = position_cascade_friction_output(&s_state.friction);
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
        if (position_cascade_abs(config->target_position - measured_position) <=
                hold_exit_position &&
            position_cascade_abs(friction.current_a) <=
                config->friction_release_slew_rate * config->update_period_s &&
            ((s_state.hold_counter == 0U && s_state.settling_in_window &&
              position_cascade_abs(measured_speed) <= POSITION_SERVO_HOLD_ENTER_SPEED_RAD_S) ||
             (s_state.hold_counter > 0U &&
              position_cascade_abs(measured_speed) <= POSITION_SERVO_HOLD_EXIT_SPEED_RAD_S)))
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
                position_cascade_friction_hold_entered(&s_state.friction);
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
    {
        PositionCascadeCoreSpeedConfig core_config = {config->position_kp,
                                                      config->position_kd,
                                                      config->maximum_speed,
                                                      config->speed_limit_rad_s};
        PositionCascadeCoreMotion motion = {s_state.position_reference,
                                            s_state.trajectory_speed,
                                            measured_position,
                                            s_state.control_velocity};
        speed_reference = position_cascade_core_speed(&core_config, &motion);
    }

    if (!s_state.settling_in_window)
    {
        PositionCascadeIntegralTransport request = position_cascade_integral_transport_request(
            config,
            config->target_position - measured_position,
            speed_reference - measured_speed,
            position_cascade_core_output(&s_state.core).integral_current_a,
            s_state.target_transition_active);
        position_cascade_core_apply_transport(&s_state.core, request);
    }
    {
        PositionCascadeFrictionConfig friction_config = {config->current_limit_a,
                                                         config->deceleration,
                                                         config->friction_attack_slew_rate,
                                                         config->friction_breakaway_distance,
                                                         config->friction_breakaway_ratio,
                                                         config->friction_coulomb_negative,
                                                         config->friction_coulomb_positive,
                                                         config->friction_fast_release_slew_rate,
                                                         config->friction_feedforward_enabled,
                                                         config->friction_move_speed,
                                                         config->friction_reference_speed,
                                                         config->friction_release_slew_rate,
                                                         config->friction_stop_speed,
                                                         config->friction_viscous_negative,
                                                         config->friction_viscous_positive,
                                                         config->hold_enter_position,
                                                         config->hold_exit_position,
                                                         config->position_error_window,
                                                         config->update_period_s};
        PositionCascadeFrictionInput motion = {config->target_position,
                                               s_state.position_reference,
                                               s_state.trajectory_speed,
                                               measured_position,
                                               measured_speed,
                                               s_state.configured_stuck_ticks,
                                               s_state.phase == POSITION_SERVO_PHASE_HOLD,
                                               s_state.phase == POSITION_SERVO_PHASE_MOVE,
                                               s_state.phase == POSITION_SERVO_PHASE_SETTLE,
                                               s_state.settling_in_window};
        FAST_PROFILE_BEGIN(FAST_PROFILE_FRICTION);
        position_cascade_friction_update(&s_state.friction, &friction_config, &motion);
        FAST_PROFILE_END(FAST_PROFILE_FRICTION);
        friction = position_cascade_friction_output(&s_state.friction);
    }
    s_state.acceleration_feedforward_current = position_cascade_constrain(
        config->acceleration_feedforward_gain * s_state.trajectory_acceleration,
        -config->current_limit_a,
        config->current_limit_a);
    feedforward_candidate = s_state.acceleration_feedforward_current + friction.current_a;
    feedforward_current = position_cascade_constrain(
        feedforward_candidate, -config->current_limit_a, config->current_limit_a);

    /* PI 使用安培域计算，保留原速度增益调参语义。 */
    speed_kp_current_domain = config->speed_kp * config->current_limit_a;
    speed_ki_current_domain = config->speed_ki * config->current_limit_a;
    {
        PositionCascadeStaticIntegral request = position_cascade_static_integral_request(
            config,
            measured_position,
            measured_speed,
            feedforward_current,
            speed_kp_current_domain,
            position_cascade_core_output(&s_state.core).integral_current_a,
            speed_reference,
            friction.current_ramping);
        position_cascade_core_apply_static_integral(
            &s_state.core, request, config->current_limit_a);
    }
    /* 仅前馈建立期间暂停同方向积分；达到模型值后仍允许修正残余负载。 */
    if ((friction.current_ramping &&
         friction.current_a * (speed_reference - measured_speed) > 0.0f) ||
        s_state.phase == POSITION_SERVO_PHASE_HOLD ||
        (s_state.settling_in_window &&
         position_cascade_abs(measured_speed) <= POSITION_SERVO_HOLD_ENTER_SPEED_RAD_S))
    {
        speed_ki_current_domain = 0.0f;
    }
    {
        PositionCascadeCoreCurrent current = {speed_reference,
                                              s_state.control_velocity,
                                              speed_kp_current_domain,
                                              speed_ki_current_domain,
                                              feedforward_candidate,
                                              feedforward_current};
        bool valid;
        FAST_PROFILE_BEGIN(FAST_PROFILE_SPEED_PI);
        valid = position_cascade_core_run_current(
            &s_state.core, &current, config->update_period_s, config->current_limit_a);
        FAST_PROFILE_END(FAST_PROFILE_SPEED_PI);
        return valid;
    }
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
        position_cascade_friction_initialize(&s_state.friction, measured_position);
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
        position_cascade_friction_target_changed(&s_state.friction);
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
    PositionCascadeCoreOutput core = position_cascade_core_output(&s_state.core);
    output->position_reference = s_state.position_reference;
    output->speed_reference = core.speed_reference_rad_s;
    output->speed_feedback_rad_s = s_state.control_velocity;
    output->iq_reference = core.iq_reference_a;
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
