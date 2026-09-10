#include "../System/fast_loop_profile.h"
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
	PositionCascadeConfig_TypeDef validated_config;
	bool config_valid;
	uint32_t configured_hold_ticks;
	uint32_t configured_stuck_ticks;
	float configured_velocity_filter_alpha;
	PI_Controller_TypeDef speed_controller;
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
		config->friction_attack_slew_rate > 0.0f &&
		isfinite(config->friction_fast_release_slew_rate) &&
		config->friction_fast_release_slew_rate > 0.0f &&
		isfinite(config->friction_release_slew_rate) &&
		config->friction_release_slew_rate > 0.0f &&
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
	output->speed_feedback = state.velocity_filtered;
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
		memcmp(&config->position_error_window, &saved->position_error_window,
			offsetof(PositionCascadeConfig_TypeDef, friction_feedforward_enabled) -
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
	state.configured_stuck_ticks = PositionCascade_TimeToTicks(config,
		config->friction_stuck_time);
	{
		float ratio = 6.2831853072f * config->velocity_filter_hz * config->update_period_s;
		state.configured_velocity_filter_alpha = ratio / (1.0f + ratio);
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
	telemetry->speed_feedback = state.velocity_filtered;
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
		return;
	}

	target_error = config->target_position - measured_position;
	landing_position = config->hold_enter_position;
	landing_zero_position = landing_position * POSITION_SERVO_FRICTION_LANDING_ZERO_RATIO;
	feedback_is_stopped = PositionCascade_Abs(measured_speed) <=
		config->friction_stop_speed;
	stuck_ticks = state.configured_stuck_ticks;

	if (state.phase == POSITION_SERVO_PHASE_HOLD || state.settling_in_window)
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
		if (PositionCascade_Abs(target_error) > config->hold_exit_position &&
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
		else if (PositionCascade_Abs(target_error) > config->hold_enter_position)
			/* The reference can be nearly stopped while the rotor still lags.
			 * Carry approach compensation through this MOVE/SETTLE gap. */
			desired_direction = target_error > 0.0f ? 1 : -1;
	}
	else if (state.phase == POSITION_SERVO_PHASE_SETTLE)
	{
		if (state.settle_recovery_active)
		{
			if (PositionCascade_Abs(target_error) >
				config->hold_enter_position)
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
		else if (PositionCascade_Abs(target_error) > config->hold_exit_position &&
			feedback_is_stopped)
		{
			waiting_for_recovery = true;
			fast_release = true;
			if (state.friction_stuck_counter < UINT32_MAX)
				state.friction_stuck_counter++;
			if (stuck_ticks == 0U ||
				state.friction_stuck_counter >= stuck_ticks)
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

	if (desired_direction != state.friction_direction)
	{
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
		current_step = (state.settle_recovery_active ?
			config->friction_release_slew_rate : config->friction_attack_slew_rate) *
			config->update_period_s;
	else if (fast_release)
	{
		float release_rate = config->friction_fast_release_slew_rate;
		/* Ease the final removal of approach torque inside the captured window.
		 * Crossing the target or leaving capture still uses fast unloading. */
		if ((state.settling_in_window ||
			(state.friction_landing_active &&
			 PositionCascade_Abs(target_error) <= config->hold_enter_position)) &&
			state.friction_feedforward_current * target_error > 0.0f)
			release_rate = PositionCascade_Min(release_rate,
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
		PositionCascade_Abs(target_error) > config->hold_enter_position &&
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
	if (config->stiction_integral_rate == 0.0f || state.settling_in_window ||
		state.phase == POSITION_SERVO_PHASE_HOLD ||
		PositionCascade_Abs(error) <= config->hold_enter_position ||
		PositionCascade_Abs(target_error) <= config->hold_enter_position ||
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
		state.speed_controller.Ui + increment, -config->current_limit, config->current_limit);
	state.stiction_integrating = increment != 0.0f;
}

void PositionCascade_Reset(void)
{
	memset(&state, 0, sizeof(state));
	PI_Controller_Reset(&state.speed_controller);
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

	state.loop_count = 0U;
	state.defer_telemetry = true;
	if (config->velocity_filter_hz > 0.0f)
	{
		state.velocity_filtered += state.configured_velocity_filter_alpha *
			(measured_speed - state.velocity_filtered);
	}
	else
		state.velocity_filtered = measured_speed;
	measured_speed = state.velocity_filtered;

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
	if (state.phase == POSITION_SERVO_PHASE_MOVE && trajectory_done)
		state.phase = POSITION_SERVO_PHASE_SETTLE;

	hold_exit_position = config->hold_exit_position;
	/* Quiet the feedforward before confirming HOLD; retain this latch through noise. */
	if (PositionCascade_Abs(config->target_position - state.position_reference) >
		hold_exit_position ||
		PositionCascade_Abs(config->target_position - measured_position) > hold_exit_position)
		state.settling_in_window = false;
	else if (PositionCascade_Abs(config->target_position - measured_position) <=
		config->hold_enter_position &&
		PositionCascade_Abs(config->target_position - state.position_reference) <=
		config->hold_enter_position)
		state.settling_in_window = true;
	if (state.phase == POSITION_SERVO_PHASE_HOLD)
	{
		if (PositionCascade_Abs(config->target_position - measured_position) >
			hold_exit_position || PositionCascade_Abs(measured_speed) >
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
	else if (trajectory_done)
	{
		hold_ticks = state.configured_hold_ticks;
		if ((state.hold_counter == 0U && state.settling_in_window &&
			 PositionCascade_Abs(config->target_position - measured_position) <=
				hold_exit_position &&
			 PositionCascade_Abs(measured_speed) <=
				POSITION_SERVO_HOLD_ENTER_SPEED_RAD_S &&
			 PositionCascade_Abs(state.friction_feedforward_current) <=
				config->friction_release_slew_rate * config->update_period_s) ||
			(state.hold_counter > 0U &&
			 PositionCascade_Abs(config->target_position - measured_position) <=
				hold_exit_position &&
			 PositionCascade_Abs(measured_speed) <=
				POSITION_SERVO_HOLD_EXIT_SPEED_RAD_S &&
			 PositionCascade_Abs(state.friction_feedforward_current) <=
				config->friction_release_slew_rate * config->update_period_s))
		{
			if (state.hold_counter < hold_ticks)
				state.hold_counter++;
			if (state.hold_counter >= hold_ticks)
			{
				state.phase = POSITION_SERVO_PHASE_HOLD;
				state.target_reached = true;
				state.target_transition_active = false;
				state.settle_recovery_active = false;
				state.hold_counter = 0U;
			}
		}
		else
			state.hold_counter = 0U;
	}
	else
	{
		state.hold_counter = 0U;
	}

	position_error = state.position_reference - measured_position;
	velocity_error = state.trajectory_speed - measured_speed;
	state.speed_reference = state.trajectory_speed +
		config->position_kp * position_error +
		config->position_kd * velocity_error;
	/* A lagging axis needs some speed above trajectory cruise to catch up.
	 * Bound that headroom independently, and always honor the motor limit. */
	speed_command_limit = PositionCascade_Min(config->speed_limit,
		config->maximum_speed * POSITION_SERVO_SPEED_CORRECTION_HEADROOM_RATIO);
	state.speed_reference = PositionCascade_Constrain(state.speed_reference,
		-speed_command_limit, speed_command_limit);

	if (!state.settling_in_window)
		PositionCascade_UpdateIntegralTransport(config,
			config->target_position - measured_position,
			state.speed_reference - measured_speed);
	FAST_PROFILE_BEGIN(FAST_PROFILE_FRICTION);
	PositionCascade_UpdateFriction(config, measured_position, measured_speed);
	FAST_PROFILE_END(FAST_PROFILE_FRICTION);
	state.acceleration_feedforward_current = PositionCascade_Constrain(
		config->acceleration_feedforward_gain * state.trajectory_acceleration,
		-config->current_limit, config->current_limit);
	feedforward_candidate = state.acceleration_feedforward_current +
		state.friction_feedforward_current;
	feedforward_current = PositionCascade_Constrain(feedforward_candidate,
		-config->current_limit, config->current_limit);

	/* Preserve the legacy speed-gain tuning while executing this PI in amperes. */
	speed_kp_current_domain = config->speed_kp * config->current_limit;
	speed_ki_current_domain = config->speed_ki * config->current_limit;
	PositionCascade_UpdateStictionIntegral(config, measured_position, measured_speed,
		feedforward_current, speed_kp_current_domain);
	/* Pause same-direction accumulation only while FF builds, not indefinitely
	 * after it reaches its model value. Residual load must remain correctable. */
	if ((state.friction_current_ramping && state.friction_feedforward_current *
		(state.speed_reference - measured_speed) > 0.0f) ||
		state.phase == POSITION_SERVO_PHASE_HOLD ||
		(state.settling_in_window && PositionCascade_Abs(measured_speed) <=
		 POSITION_SERVO_HOLD_ENTER_SPEED_RAD_S))
		speed_ki_current_domain = 0.0f;
	FAST_PROFILE_BEGIN(FAST_PROFILE_SPEED_PI);
	PI_Controller_Configure(&state.speed_controller, speed_kp_current_domain,
		speed_ki_current_domain, config->update_period_s,
		-config->current_limit, config->current_limit);
	state.feedback_current = PI_Controller_Run(&state.speed_controller,
		state.speed_reference, measured_speed);
	current_candidate = state.feedback_current + feedforward_current;
	if (!isfinite(current_candidate))
	{
		FAST_PROFILE_END(FAST_PROFILE_SPEED_PI);
		return false;
	}
	state.current_saturated =
		feedforward_candidate > config->current_limit ||
		feedforward_candidate < -config->current_limit ||
		current_candidate > config->current_limit ||
		current_candidate < -config->current_limit;
	state.iq_reference = PositionCascade_Constrain(current_candidate,
		-config->current_limit, config->current_limit);

	/* Total-current saturation is included in the speed-PI anti-windup path. */
	applied_feedback_current = PositionCascade_Constrain(state.iq_reference -
		feedforward_current, -config->current_limit, config->current_limit);
	PI_Controller_TrackOutput(&state.speed_controller, applied_feedback_current);
	state.feedback_current = state.speed_controller.Out;
	FAST_PROFILE_END(FAST_PROFILE_SPEED_PI);
	return true;
}

static bool PositionCascade_RunValidated(const PositionCascadeConfig_TypeDef *config,
    float measured_position, float measured_speed)
{
	if (!state.initialized)
	{
		/* Static initialization / Reset already cleared state. Preserve the
		 * just-validated configuration and avoid a second reset on mode entry. */
		state.initialized = true;
		state.last_target = config->target_position;
		state.position_reference = measured_position;
		state.velocity_filtered = measured_speed;
		state.friction_breakaway_start_position = measured_position;
		state.target_transition_active =
			PositionCascade_Abs(config->target_position - measured_position) >
			config->position_error_window;
		state.phase = state.target_transition_active ?
			POSITION_SERVO_PHASE_MOVE : POSITION_SERVO_PHASE_SETTLE;
	}
	else if (state.last_target != config->target_position)
	{
		state.last_target = config->target_position;
		state.phase = POSITION_SERVO_PHASE_MOVE;
		state.target_reached = false;
		state.hold_counter = 0U;
		state.friction_stuck_counter = 0U;
		state.friction_breakaway_active = false;
		state.friction_landing_active = false;
		state.settle_recovery_active = false;
		state.target_transition_active = true;
		state.settling_in_window = false;
		state.position_stuck_counter = 0U;
		state.stiction_integrating = false;
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
    output->speed_feedback = state.velocity_filtered;
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
