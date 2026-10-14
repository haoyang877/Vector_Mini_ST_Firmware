#include "fast_loop_profile.h"
#include "position_cascade.h"

#include <math.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "foc_pid.h"
#include "position_cascade_config.h"
#include "motor_thermal.h"
#include "position_smooth_trajectory.h"

typedef struct
{
	PositionCascadeConfig_TypeDef validated_config;
	bool config_valid;
	uint32_t configured_hold_ticks;
	uint32_t configured_stuck_ticks;
	float configured_velocity_filter_alpha;
	float configured_hold_velocity_filter_alpha;
	float configured_capture_position_rad;
	float configured_hold_exit_position_rad;
	PI_Controller_TypeDef speed_controller;
	PositionSmoothTrajectory smooth_trajectory;
	bool smooth_active;
	bool smooth_planning;
	uint16_t loop_count;
	uint32_t hold_counter;
	uint32_t passive_counter;
	uint32_t passive_exit_counter;
	uint32_t near_stop_counter;
	float near_progress_error_rad;
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
	float qualification_position_rad;
	float iq_reference;
	float feedback_current;
	float acceleration_feedforward_current;
	float friction_feedforward_current;
	float friction_breakaway_start_position;
	int8_t friction_direction;
	PositionServoPhase_TypeDef phase;
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
	bool settle_friction_quiet;
	bool passive_hold;
	bool passive_recovery_limited;
	bool passive_brake_latched;
	bool near_target_no_integral;
	bool initialized;
	bool defer_telemetry;
} PositionCascadeState_TypeDef;

static PositionCascadeState_TypeDef state;

static inline float PositionCascade_Abs(float value)
{
	/* C99 magnitude operation maps to one instruction on an FPU target. */
	return fabsf(value);
}

static inline float PositionCascade_Min(float first, float second)
{
	return first <= second ? first : second;
}

static inline float PositionCascade_Max(float first, float second)
{
	return first >= second ? first : second;
}

static inline float PositionCascade_Constrain(float value, float minimum, float maximum)
{
	if (value < minimum)
		return minimum;
	if (value > maximum)
		return maximum;
	return value;
}

static uint32_t PositionCascade_TimeToTicks(
	const PositionCascadeConfig_TypeDef *config, float time_s)
{
	float ticks;

	if (time_s <= 0.0f)
		return 0U;
	ticks = time_s / config->update_period_s + 0.5f;
	if (ticks >= (float)UINT32_MAX)
		return UINT32_MAX;
	if (ticks < 1.0f)
		return 1U;
	return (uint32_t)ticks;
}

static float PositionCascade_OutputLimit(const PositionCascadeConfig_TypeDef *config)
{
    return MotorThermal_ResolveLimit(config->current_limit, config->thermal_output_limit_a,
        config->thermal_limit_active);
}

static bool PositionCascade_ConfigIsValid(const PositionCascadeConfig_TypeDef *config)
{
	return config != NULL &&
		isfinite(config->update_period_s) && config->update_period_s > 0.0f &&
		config->call_divider > 0U &&
		isfinite(config->target_position) &&
		isfinite(config->position_error_window) && config->position_error_window > 0.0f &&
		isfinite(config->hold_enter_position) &&
		config->hold_enter_position >= config->position_error_window &&
		isfinite(config->hold_exit_position) &&
		config->hold_exit_position > config->hold_enter_position &&
		isfinite(config->velocity_filter_hz) && config->velocity_filter_hz >= 0.0f &&
		config->velocity_filter_hz * config->update_period_s <= 0.5f &&
		isfinite(config->hold_velocity_filter_hz) &&
		config->hold_velocity_filter_hz >= 0.0f &&
		config->hold_velocity_filter_hz * config->update_period_s <= 0.5f &&
		isfinite(config->following_error_limit) && config->following_error_limit >= 0.0f &&
		isfinite(config->stiction_integral_rate) && config->stiction_integral_rate >= 0.0f &&
		isfinite(config->stiction_integral_rate * config->update_period_s) &&
		isfinite(config->acceleration) && config->acceleration > 0.0f &&
		isfinite(config->deceleration) && config->deceleration > 0.0f &&
		isfinite(config->maximum_speed) && config->maximum_speed > 0.0f &&
		isfinite(config->speed_limit) && config->speed_limit > 0.0f &&
		config->maximum_speed <= config->speed_limit &&
		isfinite(config->jerk_limit) && config->jerk_limit > 0.0f &&
		isfinite(config->position_kp) && config->position_kp >= 0.0f &&
		config->position_kp <= CASCADE_POSITION_KP_MAX_PER_S &&
		isfinite(config->position_kd) && config->position_kd >= 0.0f &&
		config->position_kd <= CASCADE_POSITION_KD_MAX &&
		isfinite(config->speed_kp) && config->speed_kp >= 0.0f &&
		isfinite(config->speed_ki) && config->speed_ki >= 0.0f &&
		isfinite(config->acceleration_feedforward_gain) &&
		config->acceleration_feedforward_gain >= 0.0f &&
		isfinite(config->current_limit) && config->current_limit > 0.0f &&
        (!config->thermal_limit_active || (isfinite(config->thermal_output_limit_a) &&
         config->thermal_output_limit_a >= 0.0f)) &&
		isfinite(config->friction_coulomb_positive) &&
		config->friction_coulomb_positive >= 0.0f &&
		isfinite(config->friction_coulomb_negative) &&
		config->friction_coulomb_negative >= 0.0f &&
		isfinite(config->friction_viscous_positive) &&
		config->friction_viscous_positive >= 0.0f &&
		isfinite(config->friction_viscous_negative) &&
		config->friction_viscous_negative >= 0.0f &&
		isfinite(config->friction_breakaway_ratio) &&
		config->friction_breakaway_ratio >= 1.0f &&
		isfinite(config->friction_attack_slew_rate) &&
		config->friction_attack_slew_rate > 0.0f && config->friction_attack_slew_rate <= 1000.0f &&
		isfinite(config->friction_fast_release_slew_rate) &&
		config->friction_fast_release_slew_rate > 0.0f &&
		isfinite(config->friction_release_slew_rate) &&
		config->friction_release_slew_rate > 0.0f && config->friction_release_slew_rate <= 1000.0f &&
		isfinite(config->friction_reference_speed) &&
		config->friction_reference_speed > 0.0f &&
		isfinite(config->friction_stop_speed) && config->friction_stop_speed >= 0.0f &&
		isfinite(config->friction_move_speed) &&
		config->friction_move_speed > config->friction_stop_speed &&
		isfinite(config->friction_breakaway_distance) &&
		config->friction_breakaway_distance > 0.0f &&
		isfinite(config->friction_stuck_time) && config->friction_stuck_time >= 0.0f;
}

static void PositionCascade_CopyOutput(PositionCascadeOutput_TypeDef *output)
{
	output->position_reference = state.position_reference;
	output->speed_reference = state.speed_reference;
	output->acceleration_reference = state.trajectory_acceleration;
	output->speed_feedback = state.control_velocity;
	output->iq_reference = state.iq_reference;
	output->feedback_current = state.feedback_current;
	output->acceleration_feedforward_current =
		state.acceleration_feedforward_current;
	output->friction_feedforward_current = state.friction_feedforward_current;
	output->hold_current = state.speed_controller.Ui;
	output->phase = state.phase;
	output->target_reached = state.target_reached;
}

static bool PositionCascade_CheckConfig(const PositionCascadeConfig_TypeDef *config)
{
	const PositionCascadeConfig_TypeDef *saved = &state.validated_config;
	if (config == NULL || !isfinite(config->target_position)) return false;
	/* Compare all tuning fields exactly, excluding struct padding and the
	 * frequently changing target (validated above on EVERY fast call).
	 * Padding bytes have unspecified values even after aggregate assignment.
	 * Do not let them or a new target trigger all tuning checks in the IRQ. */
	if (state.config_valid && config->update_period_s == saved->update_period_s &&
		config->call_divider == saved->call_divider &&
		config->friction_feedforward_enabled == saved->friction_feedforward_enabled &&
		config->passive_hold_enabled == saved->passive_hold_enabled &&
        config->thermal_limit_active == saved->thermal_limit_active &&
		memcmp(&config->position_error_window, &saved->position_error_window,
			offsetof(PositionCascadeConfig_TypeDef, thermal_limit_active) -
            offsetof(PositionCascadeConfig_TypeDef, position_error_window)) == 0 &&
		memcmp(&config->friction_coulomb_positive, &saved->friction_coulomb_positive,
			sizeof(*config) - offsetof(PositionCascadeConfig_TypeDef, friction_coulomb_positive)) == 0)
		return true;
	state.defer_telemetry = true;
	if (!PositionCascade_ConfigIsValid(config)) return false;
	/* Configuration-derived constants stay out of the 2 kHz control path.
	 * Refresh only after complete validation, including when tuning changes. */
	state.configured_hold_ticks = PositionCascade_TimeToTicks(config,
		POSITION_SERVO_HOLD_CONFIRM_TIME_S);
	/* 到位精度使用实际误差窗口；保留原 HOLD 滞环比例，避免宽窗口提前冻结。 */
	state.configured_capture_position_rad = config->position_error_window;
	state.configured_hold_exit_position_rad = config->hold_exit_position *
		(config->position_error_window / config->hold_enter_position);
	state.configured_stuck_ticks = PositionCascade_TimeToTicks(config,
		config->friction_stuck_time);
	{
		float ratio = 6.2831853072f * config->velocity_filter_hz * config->update_period_s;
		state.configured_velocity_filter_alpha = ratio / (1.0f + ratio);
		ratio = 6.2831853072f * config->hold_velocity_filter_hz *
			config->update_period_s;
		state.configured_hold_velocity_filter_alpha = ratio / (1.0f + ratio);
	}
	memcpy(&state.validated_config, config, sizeof(*config));
	state.config_valid = true;
	return true;
}

bool PositionCascade_GetTelemetry(PositionCascadeTelemetry_TypeDef *telemetry)
{
	if (telemetry == NULL)
		return false;

	telemetry->position_reference = state.position_reference;
	telemetry->trajectory_speed_reference = state.trajectory_speed;
	telemetry->speed_command = state.speed_reference;
	telemetry->speed_feedback = state.control_velocity;
	telemetry->acceleration_reference = state.trajectory_acceleration;
	telemetry->feedback_current = state.feedback_current;
	telemetry->acceleration_feedforward_current =
		state.acceleration_feedforward_current;
	telemetry->friction_feedforward_current =
		state.friction_feedforward_current;
	telemetry->hold_current = state.speed_controller.Ui;
	telemetry->phase = state.phase;
	telemetry->target_reached = state.target_reached;
	telemetry->current_saturated = state.current_saturated;
	telemetry->friction_landing_active = state.friction_landing_active;
	telemetry->settle_recovery_active = state.settle_recovery_active;
	telemetry->hold_candidate_active =
		state.phase == POSITION_SERVO_PHASE_SETTLE && state.hold_counter > 0U;
	telemetry->trajectory_limited = state.trajectory_limited;
	telemetry->stiction_integrating = state.stiction_integrating;
	telemetry->passive_hold = state.passive_hold;
	return state.initialized;
}

bool PositionCascade_ShouldDeferTelemetry(void)
{
	return state.defer_telemetry;
}

static bool PositionCascade_UpdateOnlineTrajectory(
	const PositionCascadeConfig_TypeDef *config, float measured_position)
{
	float distance = config->target_position - state.position_reference;
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

	state.trajectory_limited = false;

	if (PositionCascade_Abs(distance) <= config->position_error_window &&
		PositionCascade_Abs(state.trajectory_speed) <=
			POSITION_SERVO_HOLD_ENTER_SPEED_RAD_S &&
		PositionCascade_Abs(state.trajectory_acceleration) <=
			config->jerk_limit * config->update_period_s)
	{
		state.position_reference = config->target_position;
		state.trajectory_speed = 0.0f;
		state.trajectory_acceleration = 0.0f;
		return true;
	}

	direction = distance >= 0.0f ? 1.0f : -1.0f;
	speed_toward_target = direction * state.trajectory_speed;
	acceleration_toward_target = PositionCascade_Max(
		direction * state.trajectory_acceleration, 0.0f);
	if (speed_toward_target > 0.0f)
	{
		/*
		 * Reserve transition distance for acceleration toward the target.
		 * Negative acceleration is already braking; counting its magnitude
		 * again creates an artificial distance offset and stop/restart tail.
		 */
		acceleration_transition_time =
			(acceleration_toward_target +
			 config->deceleration) /
			config->jerk_limit;
		acceleration_transition_distance = speed_toward_target *
			acceleration_transition_time +
			0.5f * acceleration_toward_target *
			acceleration_transition_time * acceleration_transition_time;
	}
	else
	{
		acceleration_transition_distance = 0.0f;
	}
	braking_distance_available = PositionCascade_Max(PositionCascade_Abs(distance) -
		acceleration_transition_distance, 0.0f);
	braking_speed = sqrtf(2.0f * config->deceleration *
		braking_distance_available);
	desired_speed = direction * PositionCascade_Min(config->maximum_speed,
		braking_speed);
	if (config->following_error_limit > 0.0f)
	{
		/* A soft, jerk-limited governor; never teleport the reference to feedback. */
		following_error = direction * (state.position_reference - measured_position);
		governor_scale = PositionCascade_Constrain(1.0f - following_error /
			config->following_error_limit, 0.0f, 1.0f);
		governed_speed = config->maximum_speed * governor_scale;
		if (PositionCascade_Abs(desired_speed) > governed_speed)
		{
			desired_speed = direction * governed_speed;
			state.trajectory_limited = true;
		}
	}
	speed_error = desired_speed - state.trajectory_speed;
	{
		bool increasing_speed = desired_speed * state.trajectory_speed >= 0.0f &&
			PositionCascade_Abs(desired_speed) >
			PositionCascade_Abs(state.trajectory_speed);

		acceleration_magnitude = increasing_speed ?
			config->acceleration : config->deceleration;
		desired_acceleration = PositionCascade_Constrain(speed_error /
			POSITION_SERVO_ACCEL_RAMP_TIME_S,
			-acceleration_magnitude, acceleration_magnitude);
	}

	acceleration_step = config->jerk_limit * config->update_period_s;
	state.trajectory_acceleration += PositionCascade_Constrain(desired_acceleration -
		state.trajectory_acceleration, -acceleration_step, acceleration_step);
	state.trajectory_acceleration = PositionCascade_Constrain(
		state.trajectory_acceleration,
		-PositionCascade_Max(config->acceleration, config->deceleration),
		 PositionCascade_Max(config->acceleration, config->deceleration));
	previous_speed = state.trajectory_speed;
	state.trajectory_speed += state.trajectory_acceleration *
		config->update_period_s;
	state.trajectory_speed = PositionCascade_Constrain(state.trajectory_speed,
		-config->maximum_speed, config->maximum_speed);
	state.position_reference += 0.5f * (previous_speed +
		state.trajectory_speed) * config->update_period_s;

	return false;
}

static bool PositionCascade_UpdateTrajectory(
	const PositionCascadeConfig_TypeDef *config, float measured_position)
{
	PositionSmoothTrajectory *s = &state.smooth_trajectory;
	PositionSmoothSample sample;
	bool done;
	bool interrupted_preparation = state.smooth_active && !s->ready &&
		s->target != config->target_position;
	state.smooth_planning = false;
	if (state.smooth_active && (s->target != config->target_position ||
		s->speed_limit != config->maximum_speed ||
		s->acceleration != config->acceleration || s->deceleration != config->deceleration ||
		s->jerk_limit != config->jerk_limit || config->following_error_limit > 0))
		state.smooth_active = false;
	/* Mid-motion retargets retain the current q/v/a and use the existing online
	 * jerk-limited planner. Rest-to-rest moves get exact smooth endpoints. */
	if (!state.smooth_active && !interrupted_preparation && config->following_error_limit == 0 &&
		state.trajectory_speed == 0 && state.trajectory_acceleration == 0 &&
		PositionCascade_Abs(config->target_position-state.position_reference) >
			config->position_error_window)
	{
		state.smooth_active = PositionSmooth_Begin(s, state.position_reference,
			config->target_position, config->maximum_speed, config->acceleration,
			config->deceleration, config->jerk_limit);
		if (!state.smooth_active) { s->failed = true; return false; }
	}
	if (!state.smooth_active)
		return PositionCascade_UpdateOnlineTrajectory(config, measured_position);
	if (!s->ready)
	{
		state.smooth_planning = true;
		FAST_PROFILE_BEGIN(FAST_PROFILE_PLAN_PREPARE);
		(void)PositionSmooth_Prepare(s);
		FAST_PROFILE_END(FAST_PROFILE_PLAN_PREPARE);
		return false;
	}
	done = PositionSmooth_Advance(s, config->update_period_s, &sample);
	state.position_reference = sample.position;
	state.trajectory_speed = sample.speed;
	state.trajectory_acceleration = sample.acceleration;
	state.trajectory_limited = false;
	return done;
}

static float PositionCascade_FrictionMagnitude(
	const PositionCascadeConfig_TypeDef *config, int8_t direction)
{
	float speed = PositionCascade_Abs(state.trajectory_speed);

	if (direction > 0)
		return config->friction_coulomb_positive +
			config->friction_viscous_positive * speed;
	if (direction < 0)
		return config->friction_coulomb_negative +
			config->friction_viscous_negative * speed;
	return 0.0f;
}

static void PositionCascade_UpdateFriction(
	const PositionCascadeConfig_TypeDef *config, float measured_position,
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
	float previous_current = state.friction_feedforward_current;
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
		state.friction_feedforward_current = 0.0f;
		state.friction_direction = 0;
		state.friction_stuck_counter = 0U;
		state.friction_breakaway_active = false;
		state.friction_landing_active = false;
		state.settle_recovery_active = false;
		state.friction_current_ramping = false;
		state.settle_friction_quiet = false;
		state.passive_brake_latched = false;
		return;
	}

	target_error = config->target_position - measured_position;
	/* 停靠资格确认期间保留同向接近补偿；越过目标仍允许立即制动撤力。 */
	if (config->passive_hold_enabled && state.passive_counter > 0U &&
		state.friction_feedforward_current * target_error >= 0.0f)
	{
		state.friction_current_ramping = false;
		return;
	}
	/* 主动保持保留近区锁存；阻尼档由被动停靠统一卸流，避免提前失去接近力矩。 */
	if (config->passive_hold_enabled)
	{
		state.settle_friction_quiet = false;
	}
	else if (state.phase == POSITION_SERVO_PHASE_SETTLE &&
		PositionCascade_Abs(target_error) <= config->hold_enter_position)
	{
		state.settle_friction_quiet = true;
	}
	else if (PositionCascade_Abs(target_error) > config->hold_exit_position)
	{
		state.settle_friction_quiet = false;
	}
	landing_position = state.configured_capture_position_rad;
	if (config->passive_hold_enabled)
	{
		landing_position = PositionCascade_Min(landing_position, POSITION_SERVO_PASSIVE_ENTER_POSITION_RAD);
	}
	landing_zero_position = landing_position * POSITION_SERVO_FRICTION_LANDING_ZERO_RATIO;
	feedback_is_stopped = PositionCascade_Abs(measured_speed) <=
		config->friction_stop_speed;
	stuck_ticks = state.configured_stuck_ticks;

	if (state.phase == POSITION_SERVO_PHASE_HOLD || state.settling_in_window ||
		state.settle_friction_quiet)
	{
		state.friction_stuck_counter = 0U;
		state.friction_breakaway_active = false;
		state.friction_landing_active = false;
		state.settle_recovery_active = false;
		fast_release = true;
	}
	else if (state.friction_landing_active)
	{
		/*
		 * Keep the motion direction latched throughout the landing taper.
		 * If the rotor stops outside the target window, restart with a
		 * bounded breakaway ramp, including while the reference is still moving.
		 * Do not wait for trajectory completion with a stalled rotor.
		 */
		if (PositionCascade_Abs(target_error) > state.configured_capture_position_rad &&
			feedback_is_stopped)
		{
			waiting_for_recovery = true;
			if (state.friction_stuck_counter < UINT32_MAX)
				state.friction_stuck_counter++;
			if (stuck_ticks == 0U ||
				state.friction_stuck_counter >= stuck_ticks)
			{
				state.friction_landing_active = false;
				state.settle_recovery_active = true;
				desired_direction = target_error > 0.0f ? 1 : -1;
				request_breakaway = true;
			}
			else
				desired_direction = state.friction_direction;
		}
		else
		{
			state.friction_stuck_counter = 0U;
			desired_direction = state.friction_direction;
		}
		fast_release = true;
	}
	else if (state.phase == POSITION_SERVO_PHASE_MOVE)
	{
		if (PositionCascade_Abs(state.trajectory_speed) >=
			config->friction_reference_speed)
			desired_direction = state.trajectory_speed > 0.0f ? 1 : -1;
		else if (PositionCascade_Abs(config->target_position -
			state.position_reference) > config->position_error_window)
			desired_direction = config->target_position >
				state.position_reference ? 1 : -1;
		else if (PositionCascade_Abs(target_error) > state.configured_capture_position_rad)
			/* The reference can be nearly stopped while the rotor still lags.
			 * Carry approach compensation through this MOVE/SETTLE gap. */
			desired_direction = target_error > 0.0f ? 1 : -1;
	}
	else if (state.phase == POSITION_SERVO_PHASE_SETTLE)
	{
		if (state.settle_recovery_active)
		{
			if (PositionCascade_Abs(target_error) >
				state.configured_capture_position_rad)
			{
				/* Latch direction so crossing the target starts landing. */
				desired_direction = state.friction_direction != 0 ?
					state.friction_direction :
					(target_error > 0.0f ? 1 : -1);
			}
			else
			{
				state.settle_recovery_active = false;
				state.friction_breakaway_active = false;
				fast_release = true;
			}
		}
		else if (PositionCascade_Abs(target_error) > state.configured_capture_position_rad)
		{
			/* 未捕获仍需接近补偿；PLL 噪声超过静止阈值不能直接撤掉力矩。 */
			desired_direction = target_error > 0.0f ? 1 : -1;
			waiting_for_recovery = feedback_is_stopped;
			if (feedback_is_stopped && state.friction_stuck_counter < UINT32_MAX)
			{
				state.friction_stuck_counter++;
			}
			else if (!feedback_is_stopped)
			{
				state.friction_stuck_counter = 0U;
			}
			if (feedback_is_stopped && (stuck_ticks == 0U ||
				state.friction_stuck_counter >= stuck_ticks))
			{
				state.settle_recovery_active = true;
				desired_direction = target_error > 0.0f ? 1 : -1;
				request_breakaway = true;
			}
		}
		else
		{
			state.friction_stuck_counter = 0U;
			fast_release = true;
		}
	}
	/* 窗口内尚未连续确认时继续按距离卸载，不因一个样本切到零前馈。 */
	if (state.phase == POSITION_SERVO_PHASE_SETTLE && !state.settling_in_window &&
		!state.settle_friction_quiet &&
		PositionCascade_Abs(target_error) <= state.configured_capture_position_rad &&
		desired_direction == 0 && state.friction_direction != 0)
	{
		desired_direction = state.friction_direction;
		state.friction_landing_active = true;
	}

	if (desired_direction != state.friction_direction)
	{
		state.passive_brake_latched = false;
		state.friction_direction = desired_direction;
		state.friction_stuck_counter = 0U;
		state.friction_breakaway_active = false;
		state.friction_breakaway_start_position = measured_position;
	}
	if (request_breakaway)
	{
		state.friction_breakaway_active = true;
		state.friction_breakaway_start_position = measured_position;
	}

	if (!waiting_for_recovery && desired_direction != 0 && feedback_is_stopped &&
		!state.friction_landing_active)
	{
		if (state.friction_stuck_counter < UINT32_MAX)
			state.friction_stuck_counter++;
		if (!state.friction_breakaway_active &&
			(stuck_ticks == 0U || state.friction_stuck_counter >= stuck_ticks))
		{
			state.friction_breakaway_active = true;
			state.friction_breakaway_start_position = measured_position;
		}
	}
	else if (!waiting_for_recovery && (desired_direction == 0 ||
		(!state.friction_breakaway_active &&
		 (float)desired_direction * measured_speed >= config->friction_move_speed) ||
		(float)desired_direction * (measured_position -
			state.friction_breakaway_start_position) >=
			config->friction_breakaway_distance))
	{
		state.friction_stuck_counter = 0U;
		state.friction_breakaway_active = false;
	}

	if (desired_direction != 0)
	{
		remaining_distance = (float)desired_direction * target_error;
		speed_toward_target = (float)desired_direction * measured_speed;
		if (speed_toward_target > 0.0f)
			braking_distance = speed_toward_target * speed_toward_target /
				(2.0f * config->deceleration);
		inside_braking_envelope = remaining_distance <=
			landing_position + braking_distance;
		controller_requests_braking = speed_toward_target >
			config->friction_stop_speed &&
			(float)desired_direction * state.speed_reference <= 0.0f &&
			inside_braking_envelope;

		if (!state.friction_landing_active &&
			(remaining_distance <= 0.0f || controller_requests_braking ||
			 (speed_toward_target > config->friction_stop_speed &&
			  inside_braking_envelope)))
		{
			state.friction_landing_active = true;
			state.settle_recovery_active = false;
			/* v^2/(2a) belongs to inertial braking, not Coulomb friction.
			 * Tapering over that whole distance stalls the rotor before capture,
			 * then breakaway assistance creates a second visible movement. */
			state.friction_breakaway_active = false;
			state.friction_stuck_counter = 0U;
			fast_release = true;
		}

		magnitude = PositionCascade_FrictionMagnitude(config, desired_direction);
		if (!state.friction_landing_active)
		{
			state.passive_brake_latched = false;
		}
		else if (config->passive_hold_enabled && controller_requests_braking &&
			remaining_distance <= landing_position)
		{
			state.passive_brake_latched = true;
		}
		if (state.friction_breakaway_active)
			magnitude *= config->friction_breakaway_ratio;
		if (state.friction_landing_active)
		{
			/* Preserve kinetic-friction compensation until position capture.
			 * Unload inside the entry window; retain the existing current slew
			 * limit. Velocity damping supplies braking without a torque gap. */
			landing_span = landing_position - landing_zero_position;
			if (remaining_distance <= landing_zero_position ||
				landing_span <= 0.0f)
				landing_scale = 0.0f;
			else
				landing_scale = PositionCascade_Constrain(
					(remaining_distance - landing_zero_position) / landing_span,
					0.0f, 1.0f);
			magnitude *= landing_scale;
			fast_release = true;
		}
		/* 仅捕获区内锁存撤力；远处速度纠偏反向不代表到位，否则先停再脱黏。 */
		if (state.passive_brake_latched)
		{
			magnitude = 0.0f;
			fast_release = true;
		}
		target_current = desired_direction > 0 ? magnitude : -magnitude;
	}

	/* A direction reversal always unloads through zero before applying boost. */
	if (target_current * state.friction_feedforward_current < 0.0f)
	{
		target_current = 0.0f;
		fast_release = true;
	}
	current_delta = target_current - state.friction_feedforward_current;
	if (PositionCascade_Abs(target_current) >
		PositionCascade_Abs(state.friction_feedforward_current))
		current_step = config->friction_attack_slew_rate *
			config->update_period_s;
	else if (fast_release)
	{
		float release_rate = config->friction_fast_release_slew_rate;
		/* Ease the final removal of approach torque inside the captured window.
		 * Crossing the target or leaving capture still uses fast unloading. */
		if ((state.settling_in_window || state.settle_friction_quiet ||
			(state.friction_landing_active &&
			 PositionCascade_Abs(target_error) <= state.configured_capture_position_rad)) &&
			state.friction_feedforward_current * target_error > 0.0f)
			/* 单一下降配置不再隐藏限速；旧三斜率调用保持原捕获上限。 */
            release_rate = config->friction_fast_release_slew_rate == config->friction_release_slew_rate ?
                release_rate : PositionCascade_Min(release_rate,
                    POSITION_SERVO_FRICTION_CAPTURE_RELEASE_SLEW_A_PER_S);
		current_step = release_rate * config->update_period_s;
	}
	else
		current_step = config->friction_release_slew_rate *
			config->update_period_s;
	state.friction_feedforward_current += PositionCascade_Constrain(current_delta,
		-current_step, current_step);
	state.friction_feedforward_current = PositionCascade_Constrain(
		state.friction_feedforward_current, -config->current_limit,
		config->current_limit);
	if (!isfinite(state.friction_feedforward_current))
		state.friction_feedforward_current = 0.0f;
	state.friction_current_ramping =
		PositionCascade_Abs(state.friction_feedforward_current) >
		PositionCascade_Abs(previous_current) + 0.000001f;

	if (state.phase == POSITION_SERVO_PHASE_SETTLE &&
		state.friction_landing_active && landing_scale <= 0.0f &&
		PositionCascade_Abs(state.friction_feedforward_current) <= current_step)
	{
		state.friction_landing_active = false;
		state.settle_recovery_active = false;
		state.friction_direction = 0;
		state.friction_stuck_counter = 0U;
	}
}

static void PositionCascade_UpdateIntegralTransport(
	const PositionCascadeConfig_TypeDef *config, float target_error,
	float speed_error)
{
	float integral_magnitude;
	float unload_step;
	float proportional_step;
	float maximum_step;
	bool opposes_position_correction;
	bool opposes_speed_correction;

	if (!state.target_transition_active)
		return;
	opposes_position_correction =
		PositionCascade_Abs(target_error) > state.configured_capture_position_rad &&
		state.speed_controller.Ui * target_error < 0.0f;
	opposes_speed_correction =
		PositionCascade_Abs(speed_error) > config->friction_stop_speed &&
		state.speed_controller.Ui * speed_error < 0.0f;
	/* A braking speed error alone does not invalidate learned load current.
	 * Decay only when it opposes both position and velocity correction, e.g.
	 * a target reversal or overshoot. Otherwise PI owns deceleration. */
	if (!opposes_position_correction || !opposes_speed_correction)
		return;

	integral_magnitude = PositionCascade_Abs(state.speed_controller.Ui);
	proportional_step = integral_magnitude *
		POSITION_SERVO_INTEGRAL_OPPOSING_DECAY_RATE_PER_S *
		config->update_period_s;
	maximum_step = POSITION_SERVO_INTEGRAL_OPPOSING_MAX_SLEW_A_PER_S *
		config->update_period_s;
	unload_step = PositionCascade_Min(integral_magnitude,
		PositionCascade_Min(proportional_step, maximum_step));
	if (state.speed_controller.Ui > 0.0f)
		state.speed_controller.Ui -= unload_step;
	else
		state.speed_controller.Ui += unload_step;

	if (PositionCascade_Abs(state.speed_controller.Ui) <=
		POSITION_SERVO_INTEGRAL_ZERO_THRESHOLD_A)
		state.speed_controller.Ui = 0.0f;
}

static void PositionCascade_UpdateStictionIntegral(
	const PositionCascadeConfig_TypeDef *config, float measured_position,
	float measured_speed, float feedforward_current, float proportional_gain)
{
	float error = state.position_reference - measured_position;
	float target_error = config->target_position - measured_position;
	float current_candidate;
	float increment;
	uint32_t ticks = state.configured_stuck_ticks;

	state.stiction_integrating = false;
	/* Use reference error as well as target error: never push ahead of the
	 * trajectory, against its correction, or through a qualified hold window. */
	if (state.near_target_no_integral || config->stiction_integral_rate == 0.0f || state.settling_in_window ||
		(config->passive_hold_enabled && state.passive_counter > 0U) ||
		state.phase == POSITION_SERVO_PHASE_HOLD ||
		PositionCascade_Abs(error) <= state.configured_capture_position_rad ||
		PositionCascade_Abs(target_error) <= state.configured_capture_position_rad ||
		error * target_error <= 0.0f ||
		PositionCascade_Abs(measured_speed) > config->friction_stop_speed ||
		(state.position_stuck_counter > 0U &&
		 PositionCascade_Abs(measured_position - state.position_stuck_anchor) >=
			config->friction_breakaway_distance))
	{
		state.position_stuck_counter = 0U;
		return;
	}
	if (state.position_stuck_counter == 0U)
		state.position_stuck_anchor = measured_position;
	if (state.position_stuck_counter < UINT32_MAX)
		state.position_stuck_counter++;
	if (state.position_stuck_counter < ticks || state.friction_current_ramping)
		return;

	/* Learn residual static load in the existing PI, not a second accumulating
	 * feedforward. Stop adding once motion resumes; ordinary PI and reversal
	 * transport then own this current. Include FF and P in available headroom. */
	increment = (error > 0.0f ? 1.0f : -1.0f) *
		config->stiction_integral_rate * config->update_period_s;
	current_candidate = state.speed_controller.Ui + feedforward_current +
		proportional_gain * (state.speed_reference - measured_speed);
	if (increment > 0.0f)
		increment = PositionCascade_Max(0.0f, PositionCascade_Min(increment,
			config->current_limit - current_candidate));
	else
		increment = PositionCascade_Min(0.0f, PositionCascade_Max(increment,
			-config->current_limit - current_candidate));
	state.speed_controller.Ui = PositionCascade_Constrain(
		state.speed_controller.Ui + increment, -PositionCascade_OutputLimit(config), PositionCascade_OutputLimit(config));
	state.stiction_integrating = increment != 0.0f;
}

void PositionCascade_Reset(void)
{
	memset(&state, 0, sizeof(state));
	PI_Controller_Reset(&state.speed_controller);
}

/* 被动停靠仅输出卸流剩余量；不运行位置/速度PI，也不保留负载积分。 */
static void PositionCascade_RunPassiveHold(const PositionCascadeConfig_TypeDef *config,
	float measured_speed)
{
	float step_a = POSITION_SERVO_PASSIVE_RELEASE_A_PER_S * config->update_period_s;
	state.iq_reference = PositionCascade_Constrain(state.iq_reference,
		-PositionCascade_OutputLimit(config), PositionCascade_OutputLimit(config));
	state.iq_reference -= PositionCascade_Constrain(state.iq_reference, -step_a, step_a);
	PI_Controller_Reset(&state.speed_controller);
	state.feedback_current = state.iq_reference;
	state.friction_feedforward_current = 0.0f;
	state.acceleration_feedforward_current = 0.0f;
	state.speed_reference = 0.0f;
	state.control_velocity = measured_speed;
	state.stiction_integrating = false;
	state.current_saturated = false;
	state.friction_current_ramping = false;
	state.friction_breakaway_active = false;
	state.friction_landing_active = false;
	state.settle_recovery_active = false;
	state.passive_brake_latched = false;
	if (PositionCascade_Abs(config->target_position - state.qualification_position_rad) <=
		state.configured_capture_position_rad &&
		PositionCascade_Abs(measured_speed) <= POSITION_SERVO_HOLD_ENTER_SPEED_RAD_S)
	{
		if (state.hold_counter < state.configured_hold_ticks)
		{
			++state.hold_counter;
		}
		state.target_reached = state.hold_counter >= state.configured_hold_ticks;
	}
	else
	{
		state.hold_counter = 0U;
		state.target_reached = false;
	}
	state.phase = state.target_reached ? POSITION_SERVO_PHASE_HOLD : POSITION_SERVO_PHASE_SETTLE;
}

/* true表示本拍继续被动停靠；false表示已从实测状态启动普通受控回位。 */
static bool PositionCascade_RunPassiveRecovery(const PositionCascadeConfig_TypeDef *config,
	float measured_position, float measured_speed)
{
	float raw_remaining_rad = PositionCascade_Abs(config->target_position - measured_position);
	float remaining_rad = PositionCascade_Abs(config->target_position - state.qualification_position_rad);
	float speed_limit = PositionCascade_Min(config->maximum_speed, config->speed_limit);
	/* 原始与资格位置都越过1°才连续计时，保持0.5°撤流/1°唤醒滞环。 */
	if (raw_remaining_rad <= POSITION_SERVO_PASSIVE_EXIT_POSITION_RAD ||
		remaining_rad <= POSITION_SERVO_PASSIVE_EXIT_POSITION_RAD || state.iq_reference != 0.0f)
	{
		state.passive_exit_counter = 0U;
		PositionCascade_RunPassiveHold(config, measured_speed);
		return true;
	}
	if (state.passive_exit_counter < state.configured_hold_ticks)
	{
		++state.passive_exit_counter;
		PositionCascade_RunPassiveHold(config, measured_speed);
		return true;
	}
	/* 外推不是新目标：保留原目标，从实际q/v重新规划，不能带入已结束的旧参考。 */
	state.position_reference = measured_position;
	state.qualification_position_rad = measured_position;
	state.trajectory_speed = PositionCascade_Constrain(measured_speed, -speed_limit, speed_limit);
	state.trajectory_acceleration = 0.0f;
	state.control_velocity = measured_speed;
	state.smooth_active = false;
	state.smooth_planning = false;
	state.smooth_trajectory.failed = false;
	state.passive_hold = false;
	state.passive_recovery_limited = false;
	state.passive_counter = 0U;
	state.passive_exit_counter = 0U;
	state.passive_brake_latched = false;
	state.hold_counter = 0U;
	state.target_reached = false;
	state.target_transition_active = true;
	state.settling_in_window = false;
	state.settle_friction_quiet = false;
	state.settle_recovery_active = false;
	state.friction_landing_active = false;
	state.friction_breakaway_active = false;
	state.friction_stuck_counter = 0U;
	state.position_stuck_counter = 0U;
	state.near_target_no_integral = false;
	state.near_stop_counter = 0U;
	state.stiction_integrating = false;
	state.friction_direction = 0;
	state.friction_breakaway_start_position = measured_position;
	state.friction_feedforward_current = 0.0f;
	state.acceleration_feedforward_current = 0.0f;
	PI_Controller_Reset(&state.speed_controller);
	state.phase = POSITION_SERVO_PHASE_MOVE;
	return false;
}

/* Keep the 2 kHz register-save frame off the intermediate fast ticks. */
#if defined(__GNUC__) || defined(__clang__) || defined(__CC_ARM)
#define POSITION_CONTROL_NOINLINE __attribute__((noinline))
#else
#define POSITION_CONTROL_NOINLINE
#endif
static POSITION_CONTROL_NOINLINE bool PositionCascade_RunControl(const PositionCascadeConfig_TypeDef *config,
	float measured_position, float measured_speed)
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
	bool was_holding = state.phase == POSITION_SERVO_PHASE_HOLD;

	state.loop_count = 0U;
	state.defer_telemetry = true;
	/* 仅过滤到位资格，控制位置误差仍用原始反馈；减少编码器量化导致的反复进出。 */
	if (config->velocity_filter_hz > 0.0f)
	{
		state.qualification_position_rad += state.configured_velocity_filter_alpha *
			(measured_position - state.qualification_position_rad);
	}
	else
	{
		state.qualification_position_rad = measured_position;
	}
	if (config->velocity_filter_hz > 0.0f)
	{
		state.velocity_filtered += state.configured_velocity_filter_alpha *
			(measured_speed - state.velocity_filtered);
	}
	else
		state.velocity_filtered = measured_speed;
	measured_speed = state.velocity_filtered;
	if (!was_holding)
		state.control_velocity = measured_speed;

	/* 停靠资格在规划前处理；唤醒当拍即用新的实际起点创建回位参考。 */
	if (state.passive_recovery_limited)
	{
		if (config->passive_hold_enabled)
		{
			if (PositionCascade_RunPassiveRecovery(config, measured_position, measured_speed))
			{
				return true;
			}
		}
		else
		{
			state.passive_recovery_limited = false;
			state.passive_hold = false;
		}
	}
	FAST_PROFILE_BEGIN(FAST_PROFILE_TRAJECTORY);
	trajectory_done = PositionCascade_UpdateTrajectory(config, measured_position);
	FAST_PROFILE_END(FAST_PROFILE_TRAJECTORY);
	if (state.smooth_trajectory.failed || !isfinite(state.position_reference) ||
		!isfinite(state.trajectory_speed) ||
		!isfinite(state.trajectory_acceleration))
		return false;
	/* Planning is split across bounded ticks while a stationary reference keeps
	 * its existing support current. Do not apply breakaway before the plan starts. */
	if (state.smooth_planning)
	{
		return true;
	}
	/* 近目标仅用比例/阻尼与模型前馈；旧积分平滑卸除，禁止再次累积。 */
	if (!config->passive_hold_enabled ||
		PositionCascade_Abs(config->target_position - measured_position) > POSITION_SERVO_NO_INTEGRAL_EXIT_RAD)
	{
		state.near_target_no_integral = false;
	}
	else if (PositionCascade_Abs(config->target_position - measured_position) <= POSITION_SERVO_NO_INTEGRAL_ENTER_RAD &&
		PositionCascade_Abs(config->target_position - state.position_reference) <= POSITION_SERVO_NO_INTEGRAL_ENTER_RAD)
	{
		if (!state.near_target_no_integral)
		{
			state.near_progress_error_rad = PositionCascade_Abs(config->target_position - measured_position);
			state.near_stop_counter = 0U;
		}
		state.near_target_no_integral = true;
	}
	if (config->passive_hold_enabled && !state.passive_hold && trajectory_done &&
		PositionCascade_Abs(config->target_position - state.qualification_position_rad) <=
			POSITION_SERVO_PASSIVE_ENTER_POSITION_RAD &&
		PositionCascade_Abs(config->target_position - measured_position) <=
			POSITION_SERVO_PASSIVE_ENTER_POSITION_RAD &&
		PositionCascade_Abs(measured_speed) <= POSITION_SERVO_PASSIVE_CAPTURE_SPEED_RAD_S)
	{
		if (state.passive_counter < state.configured_hold_ticks)
		{
			++state.passive_counter;
		}
		if (state.passive_counter >= state.configured_hold_ticks)
		{
			state.passive_hold = true;
			state.passive_recovery_limited = true;
			state.near_stop_counter = 0U;
			state.near_progress_error_rad = PositionCascade_Abs(config->target_position - state.qualification_position_rad);
			state.passive_exit_counter = 0U;
			state.target_transition_active = false;
			state.settling_in_window = false;
			state.hold_counter = 0U;
			state.target_reached = false;
			state.position_stuck_counter = 0U;
		}
	}
	else if (!state.passive_hold)
	{
		state.passive_counter = 0U;
	}
	if (state.passive_hold)
	{
		PositionCascade_RunPassiveHold(config, measured_speed);
		return true;
	}
	if (state.phase == POSITION_SERVO_PHASE_MOVE && trajectory_done)
		state.phase = POSITION_SERVO_PHASE_SETTLE;

	/* 确认到位前必须持续满足精度；进入 HOLD 后才启用退出滞环。 */
	hold_exit_position = was_holding ? state.configured_hold_exit_position_rad :
		state.configured_capture_position_rad;
	if (PositionCascade_Abs(config->target_position - state.position_reference) >
		hold_exit_position ||
		PositionCascade_Abs(config->target_position - state.qualification_position_rad) > hold_exit_position ||
		PositionCascade_Abs(config->target_position - measured_position) > config->hold_exit_position)
		state.settling_in_window = false;
	else if (was_holding && PositionCascade_Abs(config->target_position - state.qualification_position_rad) <=
		state.configured_capture_position_rad &&
		PositionCascade_Abs(config->target_position - state.position_reference) <=
		state.configured_capture_position_rad)
		state.settling_in_window = true;
	if (state.phase == POSITION_SERVO_PHASE_HOLD)
	{
		if (PositionCascade_Abs(config->target_position - state.qualification_position_rad) >
			hold_exit_position ||
			PositionCascade_Abs(config->target_position - measured_position) > config->hold_exit_position ||
			PositionCascade_Abs(measured_speed) >
			POSITION_SERVO_HOLD_EXIT_SPEED_RAD_S)
		{
			state.phase = POSITION_SERVO_PHASE_SETTLE;
			state.target_reached = false;
			state.hold_counter = 0U;
			/* A loss of hold starts a new correction episode even when the
			 * command has not changed. Re-enable the existing bounded opposing
			 * integral transport; useful load support still remains untouched. */
			state.target_transition_active = true;
		}
	}
	else if (trajectory_done && !config->passive_hold_enabled)
	{
		hold_ticks = state.configured_hold_ticks;
		if ((state.hold_counter == 0U &&
			 PositionCascade_Abs(config->target_position - state.qualification_position_rad) <=
				hold_exit_position &&
			 PositionCascade_Abs(config->target_position - state.position_reference) <=
				state.configured_capture_position_rad &&
			 PositionCascade_Abs(measured_speed) <=
				POSITION_SERVO_HOLD_ENTER_SPEED_RAD_S) ||
			(state.hold_counter > 0U &&
			 PositionCascade_Abs(config->target_position - state.qualification_position_rad) <=
				hold_exit_position &&
			 PositionCascade_Abs(config->target_position - state.position_reference) <=
				state.configured_capture_position_rad &&
			 PositionCascade_Abs(measured_speed) <=
				POSITION_SERVO_HOLD_EXIT_SPEED_RAD_S))
		{
			if (state.hold_counter < hold_ticks)
				state.hold_counter++;
			if (state.hold_counter >= hold_ticks)
			{
				/* 先连续确认精度，再撤除前馈；单个噪声样本不能卸掉接近力矩。 */
				state.settling_in_window = true;
				if (PositionCascade_Abs(state.friction_feedforward_current) <=
					config->friction_release_slew_rate * config->update_period_s)
				{
					state.phase = POSITION_SERVO_PHASE_HOLD;
					state.target_reached = true;
					state.target_transition_active = false;
					state.settle_recovery_active = false;
					state.hold_counter = 0U;
				}
			}
		}
		else
		{
			state.hold_counter = 0U;
			state.settling_in_window = false;
		}
	}
	else
	{
		state.hold_counter = 0U;
	}

	/* Qualify HOLD using the original feedback above. Seed on entry, bypass on
	 * exit/retarget, and filter only the two damping/proportional speed paths.
	 * Never deadband position or remove the learned load-support current. */
	if (state.phase == POSITION_SERVO_PHASE_HOLD && was_holding &&
		config->hold_velocity_filter_hz > 0.0f)
		state.control_velocity += state.configured_hold_velocity_filter_alpha *
			(measured_speed - state.control_velocity);
	else
		state.control_velocity = measured_speed;
	position_error = state.position_reference - measured_position;
	velocity_error = state.trajectory_speed - state.control_velocity;
	state.speed_reference = state.trajectory_speed +
		config->position_kp * position_error +
		config->position_kd * velocity_error;
	/* A lagging axis needs some speed above trajectory cruise to catch up.
	 * Bound that headroom independently, and always honor the motor limit. */
	speed_command_limit = PositionCascade_Min(config->speed_limit,
		config->maximum_speed * POSITION_SERVO_SPEED_CORRECTION_HEADROOM_RATIO);
	state.speed_reference = PositionCascade_Constrain(state.speed_reference,
		-speed_command_limit, speed_command_limit);

	if (state.near_target_no_integral)
	{
		float remaining_rad = PositionCascade_Abs(config->target_position - measured_position);
		uint32_t near_stuck_ticks = PositionCascade_TimeToTicks(config, POSITION_SERVO_NEAR_STUCK_TIME_S);
		/* 用单向位置推进判定停滞，0.06°大于已知±0.025°测量噪声的峰峰值。
		 * 冻结已有负载补偿不新增积分；噪声或向外回退不能反复续期。 */
		if (state.near_progress_error_rad - remaining_rad >= POSITION_SERVO_NEAR_PROGRESS_RAD)
		{
			state.near_progress_error_rad = remaining_rad;
			state.near_stop_counter = 0U;
		}
		else if (state.near_stop_counter < near_stuck_ticks)
		{
			++state.near_stop_counter;
		}
		if (state.near_stop_counter >= near_stuck_ticks)
		{
			float step_a = POSITION_SERVO_PASSIVE_RELEASE_A_PER_S * config->update_period_s;
			state.speed_controller.Ui -= PositionCascade_Constrain(state.speed_controller.Ui, -step_a, step_a);
		}
		else
		{
			PositionCascade_UpdateIntegralTransport(config,
				config->target_position - measured_position,
				state.speed_reference - measured_speed);
		}
	}
	else
	{
		state.near_stop_counter = 0U;
		if (!state.settling_in_window)
		{
			PositionCascade_UpdateIntegralTransport(config,
				config->target_position - measured_position,
				state.speed_reference - measured_speed);
		}
	}
	FAST_PROFILE_BEGIN(FAST_PROFILE_FRICTION);
	PositionCascade_UpdateFriction(config, measured_position, measured_speed);
	FAST_PROFILE_END(FAST_PROFILE_FRICTION);
	state.acceleration_feedforward_current = PositionCascade_Constrain(
		config->acceleration_feedforward_gain * state.trajectory_acceleration,
		-PositionCascade_OutputLimit(config), PositionCascade_OutputLimit(config));
	feedforward_candidate = state.acceleration_feedforward_current +
		state.friction_feedforward_current;
	feedforward_current = PositionCascade_Constrain(feedforward_candidate,
		-PositionCascade_OutputLimit(config), PositionCascade_OutputLimit(config));

	/* Preserve the legacy speed-gain tuning while executing this PI in amperes. */
	speed_kp_current_domain = config->speed_kp * config->current_limit;
	speed_ki_current_domain = config->speed_ki * config->current_limit;
	PositionCascade_UpdateStictionIntegral(config, measured_position, measured_speed,
		feedforward_current, speed_kp_current_domain);
	/* Pause same-direction accumulation only while FF builds, not indefinitely
	 * after it reaches its model value. Residual load must remain correctable. */
	/* 静摩擦恢复已在同一个 Ui 中按限速累积，避免与速度 Ki 重复叠加。 */
	if (state.near_target_no_integral || state.stiction_integrating ||
		(config->passive_hold_enabled && state.passive_counter > 0U) ||
		(state.friction_current_ramping && state.friction_feedforward_current *
		(state.speed_reference - measured_speed) > 0.0f) ||
		state.phase == POSITION_SERVO_PHASE_HOLD ||
		((state.settling_in_window ||
		  (PositionCascade_Abs(config->target_position - measured_position) <=
			state.configured_capture_position_rad &&
		   PositionCascade_Abs(config->target_position - state.position_reference) <=
			state.configured_capture_position_rad)) &&
		 PositionCascade_Abs(measured_speed) <= POSITION_SERVO_HOLD_ENTER_SPEED_RAD_S))
		speed_ki_current_domain = 0.0f;
	FAST_PROFILE_BEGIN(FAST_PROFILE_SPEED_PI);
	PI_Controller_Configure(&state.speed_controller, speed_kp_current_domain,
		speed_ki_current_domain, config->update_period_s,
		-PositionCascade_OutputLimit(config), PositionCascade_OutputLimit(config));
	state.feedback_current = PI_Controller_Run(&state.speed_controller,
		state.speed_reference, state.control_velocity);
	current_candidate = state.feedback_current + feedforward_current;
	if (!isfinite(current_candidate))
	{
		FAST_PROFILE_END(FAST_PROFILE_SPEED_PI);
		return false;
	}
	state.current_saturated =
		feedforward_candidate > PositionCascade_OutputLimit(config) ||
        feedforward_candidate < -PositionCascade_OutputLimit(config) ||
        current_candidate > PositionCascade_OutputLimit(config) ||
        current_candidate < -PositionCascade_OutputLimit(config);
	state.iq_reference = PositionCascade_Constrain(current_candidate,
		-PositionCascade_OutputLimit(config), PositionCascade_OutputLimit(config));

	/* Total-current saturation is included in the speed-PI anti-windup path. */
	applied_feedback_current = PositionCascade_Constrain(state.iq_reference -
		feedforward_current, -PositionCascade_OutputLimit(config), PositionCascade_OutputLimit(config));
	if (state.near_target_no_integral && !config->thermal_limit_active)
	{
		/* 非热降额近区沿用原积分策略；热限幅必须跟踪实际施加电流。 */
		state.speed_controller.Out = applied_feedback_current;
	}
	else
	{
		PI_Controller_TrackOutput(&state.speed_controller, applied_feedback_current);
	}
	state.feedback_current = state.speed_controller.Out;
	FAST_PROFILE_END(FAST_PROFILE_SPEED_PI);
	return true;
}

static bool PositionCascade_RunValidated(const PositionCascadeConfig_TypeDef *config,
    float measured_position, float measured_speed)
{
	bool target_reset = false;
	if (!state.initialized)
	{
		target_reset = true;
		/* Static initialization / Reset already cleared state. Preserve the
		 * just-validated configuration and avoid a second reset on mode entry. */
		state.initialized = true;
		state.last_target = config->target_position;
		state.position_reference = measured_position;
		state.velocity_filtered = measured_speed;
		state.control_velocity = measured_speed;
		state.qualification_position_rad = measured_position;
		state.friction_breakaway_start_position = measured_position;
		state.target_transition_active =
			PositionCascade_Abs(config->target_position - measured_position) >
			config->position_error_window;
		state.phase = state.target_transition_active ?
			POSITION_SERVO_PHASE_MOVE : POSITION_SERVO_PHASE_SETTLE;
	}
	else if (state.last_target != config->target_position)
	{
		target_reset = true;
		/* 阻尼停靠允许位置残差；静止重启从实测位置规划，避免先补旧目标再换向。
		 * 运动中改目标保留原参考q/v/a，不能重置轨迹造成速度或加速度跳变。 */
		if (config->passive_hold_enabled && state.trajectory_speed == 0.0f &&
			state.trajectory_acceleration == 0.0f &&
			PositionCascade_Abs(state.velocity_filtered) <= POSITION_SERVO_HOLD_ENTER_SPEED_RAD_S &&
			PositionCascade_Abs(measured_speed) <= POSITION_SERVO_HOLD_ENTER_SPEED_RAD_S)
		{
			state.position_reference = measured_position;
			state.smooth_active = false;
		}
		state.last_target = config->target_position;
		state.qualification_position_rad = measured_position;
		state.phase = POSITION_SERVO_PHASE_MOVE;
		state.target_reached = false;
		state.hold_counter = 0U;
		state.friction_stuck_counter = 0U;
		state.friction_breakaway_active = false;
		state.friction_landing_active = false;
		state.settle_recovery_active = false;
		state.target_transition_active = true;
		state.settling_in_window = false;
		state.settle_friction_quiet = false;
		state.passive_hold = false;
		state.passive_recovery_limited = false;
		state.passive_counter = 0U;
		state.passive_exit_counter = 0U;
		state.passive_brake_latched = false;
		state.near_target_no_integral = false;
		state.near_stop_counter = 0U;
		state.position_stuck_counter = 0U;
		state.stiction_integrating = false;
	}
	/* 静止零电流时的新目标已在容差内，不为小角度命令重新建立摩擦力矩。 */
	if (target_reset && config->passive_hold_enabled && state.iq_reference == 0.0f &&
		state.trajectory_speed == 0.0f && state.trajectory_acceleration == 0.0f &&
		PositionCascade_Abs(measured_speed) <= POSITION_SERVO_PASSIVE_CAPTURE_SPEED_RAD_S &&
		PositionCascade_Abs(config->target_position - measured_position) <= POSITION_SERVO_PASSIVE_ENTER_POSITION_RAD)
	{
		state.position_reference = config->target_position;
		state.smooth_active = false;
		state.passive_hold = true;
		state.passive_recovery_limited = true;
	}

	if (++state.loop_count < config->call_divider)
	{
		return true;
	}
    return PositionCascade_RunControl(config, measured_position, measured_speed);
}

bool PositionCascade_Update(const PositionCascadeConfig_TypeDef *config,
	float measured_position, float measured_speed,
	PositionCascadeOutput_TypeDef *output)
{
	state.defer_telemetry = false;
	if (output == NULL || !isfinite(measured_position) ||
		!isfinite(measured_speed) || !PositionCascade_CheckConfig(config))
		return false;
	if (!PositionCascade_RunValidated(config, measured_position, measured_speed)) return false;
	PositionCascade_CopyOutput(output);
	return true;
}

const PositionCascadeConfig_TypeDef *PositionCascade_GetConfiguration(void)
{
	return state.config_valid ? &state.validated_config : NULL;
}

bool PositionCascade_UpdateTarget(float target_position, float measured_position,
	float measured_speed, PositionCascadeOutput_TypeDef *output)
{
	state.defer_telemetry = false;
	if (!state.config_valid || output == NULL || !isfinite(target_position) ||
		!isfinite(measured_position) || !isfinite(measured_speed))
		return false;
	state.validated_config.target_position = target_position;
	if (!PositionCascade_RunValidated(&state.validated_config,
		measured_position, measured_speed)) return false;
	PositionCascade_CopyOutput(output);
	return true;
}

static void PositionCascade_CopyControlOutput(PositionCascadeControlOutput_TypeDef *output)
{
    output->position_reference = state.position_reference;
    output->speed_reference = state.speed_reference;
    output->speed_feedback = state.control_velocity;
    output->iq_reference = state.iq_reference;
    output->target_reached = state.target_reached;
}

bool PositionCascade_UpdateControl(const PositionCascadeConfig_TypeDef *config,
    float measured_position, float measured_speed, PositionCascadeControlOutput_TypeDef *output)
{
    state.defer_telemetry = false;
    if (output == NULL || !isfinite(measured_position) ||
        !isfinite(measured_speed) || !PositionCascade_CheckConfig(config)) return false;
    if (!PositionCascade_RunValidated(config, measured_position, measured_speed)) return false;
    PositionCascade_CopyControlOutput(output);
    return true;
}

bool PositionCascade_UpdateTargetControl(float target_position, float measured_position,
    float measured_speed, PositionCascadeControlOutput_TypeDef *output)
{
    state.defer_telemetry = false;
    if (!state.config_valid || output == NULL || !isfinite(target_position) ||
        !isfinite(measured_position) || !isfinite(measured_speed)) return false;
    state.validated_config.target_position = target_position;
    if (!PositionCascade_RunValidated(&state.validated_config,
        measured_position, measured_speed)) return false;
    PositionCascade_CopyControlOutput(output);
    return true;
}
