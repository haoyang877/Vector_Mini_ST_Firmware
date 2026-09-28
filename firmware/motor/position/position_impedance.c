#include "position_impedance.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "position_impedance_config.h"
#include "foc_traptraj.h"
#include "hw_conf.h"
#include "utils.h"

typedef struct
{
	bool initialized;
	bool integral_transport_active;
	float integral;
	float velocity_filtered;
	float last_position;
	float last_target;
	float last_acceleration;
	float last_deceleration;
	float last_maximum_speed;
	float position_reference;
	float speed_reference;
	float iq_reference;
	float friction_current;
	float friction_breakaway_start_position;
	float friction_recovery_start_position;
	float friction_recovery_landing_start_distance;
	uint32_t hold_counter;
	uint32_t friction_stuck_counter;
	uint32_t friction_recovery_delay_counter;
	uint32_t friction_recovery_pulse_counter;
	uint32_t friction_recovery_cooldown_counter;
	uint16_t loop_counter;
	int8_t friction_direction;
	int8_t friction_pending_direction;
	PositionImpedanceFrictionState_TypeDef friction_state;
	bool friction_breakaway_active;
	bool target_reached;
} PositionImpedanceState_TypeDef;

static PositionImpedanceState_TypeDef state;

static bool PositionImpedance_ConfigIsValid(const PositionImpedanceConfig_TypeDef *config)
{
	return config != NULL &&
		isfinite(config->target_position) &&
		isfinite(config->position_error_window) && config->position_error_window > 0.0f &&
		isfinite(config->acceleration) && config->acceleration > 0.0f &&
		isfinite(config->deceleration) && config->deceleration > 0.0f &&
		isfinite(config->maximum_speed) && config->maximum_speed > 0.0f &&
		config->maximum_speed <= POSITION_IMPEDANCE_MAX_SPEED_RPS * _2PI &&
		isfinite(config->speed_limit) && config->speed_limit > 0.0f &&
		config->maximum_speed <= config->speed_limit &&
		isfinite(config->kp) && config->kp >= 0.0f &&
		config->kp <= POSITION_IMPEDANCE_KP_MAX_A_PER_RAD &&
		isfinite(config->kd) && config->kd >= 0.0f &&
		config->kd <= POSITION_IMPEDANCE_KD_MAX_A_PER_RAD_S &&
		isfinite(config->ki) && config->ki >= 0.0f &&
		config->ki <= POSITION_IMPEDANCE_KI_MAX_A_PER_RAD_S &&
		isfinite(config->integral_limit) && config->integral_limit >= 0.0f &&
		config->integral_limit <= CURRENT_COMMAND_LIMIT_MAX_A &&
		isfinite(config->output_limit) && config->output_limit > 0.0f &&
		isfinite(config->friction_positive_current) && config->friction_positive_current >= 0.0f &&
		config->friction_positive_current <= CURRENT_COMMAND_LIMIT_MAX_A &&
		isfinite(config->friction_negative_current) && config->friction_negative_current >= 0.0f &&
		config->friction_negative_current <= CURRENT_COMMAND_LIMIT_MAX_A &&
		isfinite(config->breakaway_positive_current) &&
		config->breakaway_positive_current >= config->friction_positive_current &&
		config->breakaway_positive_current <= CURRENT_COMMAND_LIMIT_MAX_A &&
		isfinite(config->breakaway_negative_current) &&
		config->breakaway_negative_current >= config->friction_negative_current &&
		config->breakaway_negative_current <= CURRENT_COMMAND_LIMIT_MAX_A &&
		isfinite(config->friction_attack_slew_rate) && config->friction_attack_slew_rate > 0.0f &&
		isfinite(config->friction_fast_release_slew_rate) &&
		config->friction_fast_release_slew_rate >= config->friction_release_slew_rate &&
		config->friction_fast_release_slew_rate <= config->friction_attack_slew_rate &&
		isfinite(config->friction_release_slew_rate) && config->friction_release_slew_rate > 0.0f &&
		isfinite(config->friction_position_enter) && config->friction_position_enter > 0.0f &&
		isfinite(config->friction_position_exit) &&
		config->friction_position_exit > config->friction_position_enter &&
		isfinite(config->friction_reference_speed) && config->friction_reference_speed > 0.0f &&
		isfinite(config->friction_stop_speed) && config->friction_stop_speed >= 0.0f &&
		isfinite(config->friction_move_speed) &&
		config->friction_move_speed > config->friction_stop_speed &&
		isfinite(config->friction_stuck_time) && config->friction_stuck_time >= 0.0f &&
		isfinite(config->friction_landing_position) &&
		config->friction_landing_position > config->friction_position_exit &&
		isfinite(config->friction_landing_speed) &&
		config->friction_landing_speed > config->friction_reference_speed &&
		isfinite(config->friction_recovery_delay) && config->friction_recovery_delay >= 0.0f &&
		isfinite(config->friction_recovery_pulse_time) &&
		config->friction_recovery_pulse_time > 0.0f &&
		isfinite(config->friction_recovery_cooldown) &&
		config->friction_recovery_cooldown >= 0.0f;
}

static void PositionImpedance_CopyOutput(PositionImpedanceOutput_TypeDef *output)
{
	output->position_reference = state.position_reference;
	output->speed_reference = state.speed_reference;
	output->velocity_feedback = state.velocity_filtered;
	output->iq_reference = state.iq_reference;
	output->friction_current = state.friction_current;
	output->target_reached = state.target_reached;
}

bool PositionImpedance_GetTelemetry(PositionImpedanceTelemetry_TypeDef *telemetry)
{
	if (telemetry == NULL)
		return false;

	telemetry->velocity_feedback = state.velocity_filtered;
	telemetry->friction_current = state.friction_current;
	telemetry->integral_current = state.integral;
	telemetry->friction_state = state.friction_state;
	telemetry->target_reached = state.target_reached;
	return state.initialized;
}

static float PositionImpedance_GetFrictionMagnitude(
	const PositionImpedanceConfig_TypeDef *config, int8_t direction, bool breakaway)
{
	if (direction > 0)
		return breakaway ? config->breakaway_positive_current :
			config->friction_positive_current;
	if (direction < 0)
		return breakaway ? config->breakaway_negative_current :
			config->friction_negative_current;
	return 0.0f;
}

static uint32_t PositionImpedance_TimeToTicks(float time_s)
{
	float ticks;

	if (time_s <= 0.0f)
		return 0U;
	ticks = time_s / Position_Ts + 0.5f;
	if (ticks >= (float)UINT32_MAX)
		return UINT32_MAX;
	if (ticks < 1.0f)
		return 1U;
	return (uint32_t)ticks;
}

static void PositionImpedance_ClearBreakaway(void)
{
	state.friction_breakaway_active = false;
	state.friction_stuck_counter = 0U;
	state.friction_breakaway_start_position = state.last_position;
}

static void PositionImpedance_UpdateFriction(
	const PositionImpedanceConfig_TypeDef *config, float target_error)
{
	float friction_target = 0.0f;
	float friction_attack_step;
	float friction_fast_release_step;
	float friction_release_step;
	float friction_step;
	float recovery_magnitude;
	float recovery_displacement;
	float measured_position;
	float remaining_distance;
	float speed_toward_target;
	float reference_speed_toward_target;
	float tracking_error_toward_target;
	float speed_excess;
	float braking_distance;
	float landing_distance;
	float landing_span;
	float landing_scale;
	float landing_speed_scale;
	float landing_magnitude;
	uint32_t stuck_ticks;
	uint32_t recovery_delay_ticks;
	uint32_t recovery_pulse_ticks;
	uint32_t recovery_cooldown_ticks;
	int8_t desired_direction;
	int8_t previous_direction;
	bool reference_is_moving = fast_abs(state.speed_reference) >
		config->friction_reference_speed;
	bool feedback_is_stopped = fast_abs(state.velocity_filtered) <=
		config->friction_stop_speed;
	bool trajectory_is_finished = fast_abs(config->target_position -
		state.position_reference) <= config->position_error_window;

	if (!config->friction_feedforward_enabled)
	{
		state.friction_state = POSITION_IMPEDANCE_FRICTION_TRACK;
		state.friction_direction = 0;
		state.friction_stuck_counter = 0U;
		state.friction_recovery_delay_counter = 0U;
		state.friction_recovery_pulse_counter = 0U;
		state.friction_recovery_cooldown_counter = 0U;
		state.friction_current = 0.0f;
		state.friction_pending_direction = 0;
		state.friction_recovery_landing_start_distance = 0.0f;
		PositionImpedance_ClearBreakaway();
		return;
	}

	measured_position = config->target_position - target_error;
	friction_attack_step = config->friction_attack_slew_rate * Position_Ts;
	friction_fast_release_step = config->friction_fast_release_slew_rate * Position_Ts;
	friction_release_step = config->friction_release_slew_rate * Position_Ts;
	friction_step = friction_release_step;
	stuck_ticks = PositionImpedance_TimeToTicks(config->friction_stuck_time);
	recovery_delay_ticks = PositionImpedance_TimeToTicks(config->friction_recovery_delay);
	recovery_pulse_ticks = PositionImpedance_TimeToTicks(config->friction_recovery_pulse_time);
	recovery_cooldown_ticks = PositionImpedance_TimeToTicks(config->friction_recovery_cooldown);
	if (state.friction_recovery_cooldown_counter > 0U)
		state.friction_recovery_cooldown_counter--;

	switch (state.friction_state)
	{
	case POSITION_IMPEDANCE_FRICTION_TRACK:
		previous_direction = state.friction_direction;
		desired_direction = state.friction_direction;
		if (reference_is_moving)
			desired_direction = state.speed_reference > 0.0f ? 1 : -1;
		else if (desired_direction == 0 &&
			fast_abs(target_error) >= config->friction_position_exit)
			desired_direction = target_error > 0.0f ? 1 : -1;
		if (desired_direction != previous_direction)
			PositionImpedance_ClearBreakaway();
		state.friction_direction = desired_direction;

		if (desired_direction == 0)
		{
			PositionImpedance_ClearBreakaway();
			break;
		}

		remaining_distance = (float)desired_direction * target_error;
		speed_toward_target = (float)desired_direction * state.velocity_filtered;
		braking_distance = speed_toward_target > 0.0f ?
			(speed_toward_target * speed_toward_target) / (2.0f * config->deceleration) : 0.0f;
		landing_distance = config->friction_landing_position + braking_distance;
		if (remaining_distance <= 0.0f ||
			(speed_toward_target > config->friction_stop_speed &&
			 remaining_distance <= landing_distance) ||
			(fast_abs(target_error) <= config->friction_landing_position &&
			 fast_abs(state.speed_reference) <= config->friction_landing_speed))
		{
			state.friction_state = POSITION_IMPEDANCE_FRICTION_LANDING;
			state.friction_recovery_landing_start_distance = 0.0f;
			state.friction_recovery_delay_counter = 0U;
			PositionImpedance_ClearBreakaway();
			break;
		}

		if (state.friction_breakaway_active)
		{
			if ((float)desired_direction *
				(measured_position - state.friction_breakaway_start_position) >=
				config->friction_position_exit)
				PositionImpedance_ClearBreakaway();
		}
		else if (feedback_is_stopped)
		{
			if (state.friction_stuck_counter < UINT32_MAX)
				state.friction_stuck_counter++;
			if (stuck_ticks == 0U || state.friction_stuck_counter >= stuck_ticks)
			{
				state.friction_breakaway_active = true;
				state.friction_breakaway_start_position = measured_position;
			}
		}
		else if (speed_toward_target >= config->friction_move_speed)
		{
			state.friction_stuck_counter = 0U;
		}

		friction_target = PositionImpedance_GetFrictionMagnitude(config,
			desired_direction, state.friction_breakaway_active);
		friction_target = desired_direction > 0 ? friction_target : -friction_target;
		break;

	case POSITION_IMPEDANCE_FRICTION_LANDING:
		PositionImpedance_ClearBreakaway();
		if (state.target_reached)
		{
			state.friction_state = POSITION_IMPEDANCE_FRICTION_HOLD;
			state.friction_recovery_delay_counter = 0U;
		}
		else
		{
			/*
			 * During a planned deceleration, retain dynamic friction compensation
			 * while feedback is materially behind the trajectory. Taper only when
			 * feedback catches the reference or is faster than it. A successful
			 * recovery uses its own remaining-distance taper below.
			 */
			remaining_distance = (float)state.friction_direction * target_error;
			speed_toward_target = (float)state.friction_direction *
				state.velocity_filtered;
			reference_speed_toward_target = (float)state.friction_direction *
				state.speed_reference;
			tracking_error_toward_target = (float)state.friction_direction *
				(state.position_reference - measured_position);
			speed_excess = speed_toward_target -
				fast_max(reference_speed_toward_target, 0.0f);
			landing_scale = 0.0f;
			landing_span = state.friction_recovery_landing_start_distance -
				config->friction_landing_position;

			if (state.friction_direction != 0 && remaining_distance > 0.0f &&
				landing_span > 0.0f)
			{
				landing_scale = constrain((remaining_distance -
					config->friction_landing_position) / landing_span, 0.0f, 1.0f);
				if (remaining_distance <= config->friction_landing_position)
					state.friction_recovery_landing_start_distance = 0.0f;
			}
			else if (state.friction_direction != 0 && !trajectory_is_finished &&
				remaining_distance > 0.0f)
			{
				if (tracking_error_toward_target > config->friction_landing_position)
				{
					landing_scale = 1.0f;
				}
				else if (tracking_error_toward_target > 0.0f)
				{
					landing_scale = constrain(tracking_error_toward_target /
						config->friction_landing_position, 0.0f, 1.0f);
					if (speed_excess > 0.0f)
					{
						landing_speed_scale = constrain(1.0f - speed_excess /
							POSITION_IMPEDANCE_FRICTION_OVERSPEED_MARGIN_RAD_S,
							0.0f, 1.0f);
						landing_scale = fast_min(landing_scale, landing_speed_scale);
					}
				}
			}

			if (landing_scale > 0.0f)
			{
				landing_magnitude = PositionImpedance_GetFrictionMagnitude(config,
					state.friction_direction, false);
				friction_target = landing_magnitude * landing_scale;
				if (state.friction_direction < 0)
					friction_target = -friction_target;
			}
			if ((landing_span <= 0.0f &&
				 (trajectory_is_finished || tracking_error_toward_target <= 0.0f ||
				  speed_excess >= POSITION_IMPEDANCE_FRICTION_OVERSPEED_MARGIN_RAD_S)) ||
				(state.friction_direction != 0 && remaining_distance <= 0.0f))
				friction_step = friction_fast_release_step;

			if (trajectory_is_finished &&
				fast_abs(state.velocity_filtered) < config->friction_move_speed &&
				fast_abs(target_error) > config->friction_landing_position &&
				state.friction_recovery_cooldown_counter == 0U)
			{
				if (state.friction_recovery_delay_counter < UINT32_MAX)
					state.friction_recovery_delay_counter++;
				if (recovery_delay_ticks == 0U ||
					state.friction_recovery_delay_counter >= recovery_delay_ticks)
				{
					state.friction_state = POSITION_IMPEDANCE_FRICTION_RECOVERY;
					state.friction_direction = target_error > 0.0f ? 1 : -1;
					state.friction_recovery_start_position = measured_position;
					state.friction_recovery_landing_start_distance = 0.0f;
					state.friction_recovery_delay_counter = 0U;
					state.friction_recovery_pulse_counter = 0U;
					/* A stopped correction needs the identified breakaway current. */
					friction_target = PositionImpedance_GetFrictionMagnitude(config,
						state.friction_direction, true);
					if (state.friction_direction < 0)
						friction_target = -friction_target;
				}
			}
			else
			{
				state.friction_recovery_delay_counter = 0U;
			}
		}
		break;

	case POSITION_IMPEDANCE_FRICTION_HOLD:
		PositionImpedance_ClearBreakaway();
		state.friction_recovery_delay_counter = 0U;
		if (!state.target_reached)
		{
			state.friction_state = POSITION_IMPEDANCE_FRICTION_LANDING;
			state.friction_direction = fast_abs(target_error) >
				config->friction_position_enter ? (target_error > 0.0f ? 1 : -1) : 0;
			state.friction_recovery_landing_start_distance = 0.0f;
		}
		break;

	case POSITION_IMPEDANCE_FRICTION_RECOVERY:
		recovery_magnitude = PositionImpedance_GetFrictionMagnitude(config,
			state.friction_direction, true);
		friction_target = state.friction_direction > 0 ?
			recovery_magnitude : -recovery_magnitude;
		recovery_displacement = (float)state.friction_direction *
			(measured_position - state.friction_recovery_start_position);
		if (state.friction_direction == 0 ||
			(float)state.friction_direction * target_error <=
				config->friction_position_enter)
		{
			state.friction_state = POSITION_IMPEDANCE_FRICTION_LANDING;
			state.friction_recovery_landing_start_distance = 0.0f;
			state.friction_recovery_pulse_counter = 0U;
			state.friction_recovery_cooldown_counter = recovery_cooldown_ticks;
			friction_target = 0.0f;
			friction_step = friction_fast_release_step;
		}
		else if (recovery_displacement >= config->friction_position_exit)
		{
			/* Confirm real motion by displacement, then taper through the remaining distance. */
			state.friction_state = POSITION_IMPEDANCE_FRICTION_LANDING;
			state.friction_recovery_landing_start_distance = fast_max(
				(float)state.friction_direction * target_error,
				config->friction_landing_position);
			state.friction_recovery_pulse_counter = 0U;
			state.friction_recovery_cooldown_counter = recovery_cooldown_ticks;
			friction_target = PositionImpedance_GetFrictionMagnitude(config,
				state.friction_direction, false);
			if (state.friction_direction < 0)
				friction_target = -friction_target;
		}
		else if (fast_abs(state.friction_current) >=
			fast_min(recovery_magnitude, config->output_limit) - friction_step)
		{
			if (state.friction_recovery_pulse_counter < UINT32_MAX)
				state.friction_recovery_pulse_counter++;
			if (recovery_pulse_ticks == 0U ||
				state.friction_recovery_pulse_counter >= recovery_pulse_ticks)
			{
				state.friction_state = POSITION_IMPEDANCE_FRICTION_LANDING;
				state.friction_recovery_landing_start_distance = 0.0f;
				state.friction_recovery_pulse_counter = 0U;
				state.friction_recovery_cooldown_counter = recovery_cooldown_ticks;
				friction_target = 0.0f;
				friction_step = friction_fast_release_step;
			}
		}
		break;

	case POSITION_IMPEDANCE_FRICTION_DIRECTION_CHANGE:
		PositionImpedance_ClearBreakaway();
		friction_target = 0.0f;
		if (fast_abs(state.friction_current) <= friction_release_step &&
			(state.friction_pending_direction == 0 ||
			 (float)state.friction_pending_direction * state.integral >=
				-POSITION_IMPEDANCE_INTEGRAL_ZERO_THRESHOLD_A))
		{
			state.friction_current = 0.0f;
			state.friction_state = POSITION_IMPEDANCE_FRICTION_TRACK;
			state.friction_direction = state.friction_pending_direction;
			state.friction_pending_direction = 0;
			state.friction_recovery_landing_start_distance = 0.0f;
			state.friction_breakaway_active = feedback_is_stopped &&
				state.friction_direction != 0;
			state.friction_breakaway_start_position = measured_position;
		}
		break;

	default:
		state.friction_state = POSITION_IMPEDANCE_FRICTION_LANDING;
		state.friction_direction = 0;
		state.friction_pending_direction = 0;
		state.friction_recovery_landing_start_distance = 0.0f;
		PositionImpedance_ClearBreakaway();
		friction_target = 0.0f;
		break;
	}

	/*
	 * A known friction model must establish torque before trajectory error grows.
	 * TRACK uses the fast attack. Catch-up and failed recovery use the intermediate
	 * release rate; ordinary tapering and direction changes remain conservative.
	 */
	if (state.friction_state == POSITION_IMPEDANCE_FRICTION_TRACK &&
		friction_target * state.friction_current >= 0.0f &&
		fast_abs(friction_target) > fast_abs(state.friction_current))
		friction_step = friction_attack_step;
	state.friction_current += constrain(friction_target - state.friction_current,
		-friction_step, friction_step);
	state.friction_current = constrain(state.friction_current,
		-config->output_limit, config->output_limit);
	if (!isfinite(state.friction_current))
		state.friction_current = 0.0f;
}

void PositionImpedance_Reset(void)
{
	memset(&state, 0, sizeof(state));
}

bool PositionImpedance_Update(const PositionImpedanceConfig_TypeDef *config,
	float measured_position, PositionImpedanceOutput_TypeDef *output)
{
	float raw_velocity;
	float velocity_filter_ratio;
	float velocity_filter_alpha;
	float target_error;
	float position_error;
	float velocity_error;
	float proportional_current;
	float damping_current;
	float integral_candidate;
	float current_candidate;
	float integral_limit;
	float desired_motion;
	float integral_transport_distance;
	float integral_decay_ratio;
	float integral_decay_step;
	float hold_exit_position;
	uint32_t hold_ticks;
	int8_t target_direction;
	bool trajectory_is_moving;
	bool trajectory_held_for_friction;
	bool friction_allows_integral;
	bool target_is_unsettled;
	bool integral_opposes_motion;
	bool feedback_follows_motion;
	bool trajectory_needs_update;

	if (output == NULL || !isfinite(measured_position) ||
		!PositionImpedance_ConfigIsValid(config))
		return false;

	if (!state.initialized)
	{
		PositionImpedance_Reset();
		state.initialized = true;
		state.last_position = measured_position;
		state.position_reference = measured_position;
		state.last_target = config->target_position;
		state.last_acceleration = config->acceleration;
		state.last_deceleration = config->deceleration;
		state.last_maximum_speed = config->maximum_speed;
		target_direction = 0;
		if (fast_abs(config->target_position - measured_position) >
			config->position_error_window)
			target_direction = config->target_position > measured_position ? 1 : -1;
		if (config->friction_feedforward_enabled && target_direction != 0)
		{
			state.friction_state = POSITION_IMPEDANCE_FRICTION_TRACK;
			state.friction_direction = target_direction;
			state.friction_breakaway_active = true;
			state.friction_breakaway_start_position = measured_position;
		}
		trajectory_needs_update = true;
	}
	else
	{
		trajectory_needs_update =
			state.last_target != config->target_position ||
			state.last_acceleration != config->acceleration ||
			state.last_deceleration != config->deceleration ||
			state.last_maximum_speed != config->maximum_speed;
		if (state.last_target != config->target_position)
		{
			state.target_reached = false;
			state.hold_counter = 0U;
			state.integral_transport_active = true;
			target_direction = 0;
			if (fast_abs(config->target_position - measured_position) >
				config->position_error_window)
				target_direction = config->target_position > measured_position ? 1 : -1;
			if (config->friction_feedforward_enabled && target_direction != 0 &&
				(float)target_direction * state.friction_current < 0.0f)
			{
				/* Unload old-direction friction before allowing the new trajectory to run. */
				state.friction_state = POSITION_IMPEDANCE_FRICTION_DIRECTION_CHANGE;
				state.friction_direction = 0;
				state.friction_pending_direction = target_direction;
				state.friction_breakaway_active = false;
			}
			else
			{
				state.friction_state = target_direction == 0 ?
					POSITION_IMPEDANCE_FRICTION_LANDING :
					POSITION_IMPEDANCE_FRICTION_TRACK;
				state.friction_direction = target_direction;
				state.friction_pending_direction = 0;
				state.friction_breakaway_active =
					config->friction_feedforward_enabled && target_direction != 0 &&
					fast_abs(state.velocity_filtered) <= config->friction_stop_speed;
			}
			state.friction_stuck_counter = 0U;
			state.friction_recovery_delay_counter = 0U;
			state.friction_recovery_pulse_counter = 0U;
			state.friction_recovery_cooldown_counter = 0U;
			state.friction_breakaway_start_position = measured_position;
			state.friction_recovery_start_position = measured_position;
			state.friction_recovery_landing_start_distance = 0.0f;
		}
	}

	if (trajectory_needs_update)
	{
		TRAJ_plan(config->target_position, state.position_reference,
			state.friction_state == POSITION_IMPEDANCE_FRICTION_DIRECTION_CHANGE ?
				0.0f : state.speed_reference,
			config->maximum_speed,
			config->acceleration, config->deceleration);
		state.last_target = config->target_position;
		state.last_acceleration = config->acceleration;
		state.last_deceleration = config->deceleration;
		state.last_maximum_speed = config->maximum_speed;
	}

	if (++state.loop_counter < POSITION_LOOP_DIVIDER)
	{
		PositionImpedance_CopyOutput(output);
		return true;
	}
	state.loop_counter = 0U;

	trajectory_held_for_friction = config->friction_feedforward_enabled &&
		state.friction_state == POSITION_IMPEDANCE_FRICTION_DIRECTION_CHANGE;
	if (trajectory_held_for_friction)
	{
		state.speed_reference = 0.0f;
	}
	else
	{
		TRAJ_eval(Position_Ts);
		state.position_reference = TRAJ_Get_Y();
		state.speed_reference = TRAJ_Get_Yd();
	}

	raw_velocity = (measured_position - state.last_position) / Position_Ts;
	state.last_position = measured_position;
	if (!isfinite(raw_velocity))
		raw_velocity = 0.0f;
	raw_velocity = constrain(raw_velocity, -config->speed_limit, config->speed_limit);
	velocity_filter_ratio = _2PI * POSITION_IMPEDANCE_VELOCITY_FILTER_HZ * Position_Ts;
	velocity_filter_alpha = velocity_filter_ratio / (1.0f + velocity_filter_ratio);
	state.velocity_filtered += velocity_filter_alpha *
		(raw_velocity - state.velocity_filtered);
	if (!isfinite(state.velocity_filtered))
		state.velocity_filtered = 0.0f;

	target_error = config->target_position - measured_position;
	hold_exit_position = 2.0f * config->position_error_window;
	if (config->friction_feedforward_enabled)
		hold_exit_position = fast_max(hold_exit_position,
			config->friction_position_exit);
	hold_ticks = (uint32_t)(POSITION_IMPEDANCE_HOLD_TIME_S / Position_Ts + 0.5f);
	if (hold_ticks < 1U)
		hold_ticks = 1U;
	if (state.target_reached)
	{
		if (fast_abs(target_error) > hold_exit_position ||
			fast_abs(state.velocity_filtered) > POSITION_IMPEDANCE_HOLD_EXIT_SPEED_RAD_S)
		{
			state.target_reached = false;
			state.hold_counter = 0U;
		}
	}
	else if (fast_abs(config->target_position - state.position_reference) <=
			 config->position_error_window &&
			 fast_abs(state.speed_reference) <= POSITION_IMPEDANCE_INTEGRAL_SPEED_RAD_S &&
			 fast_abs(target_error) <= config->position_error_window &&
			 fast_abs(state.velocity_filtered) <= POSITION_IMPEDANCE_HOLD_ENTER_SPEED_RAD_S)
	{
		if (state.hold_counter < hold_ticks)
			state.hold_counter++;
		if (state.hold_counter >= hold_ticks)
			state.target_reached = true;
	}
	else
	{
		state.hold_counter = 0U;
	}

	position_error = state.position_reference - measured_position;
	velocity_error = state.speed_reference - state.velocity_filtered;
	proportional_current = config->kp * position_error;
	damping_current = config->kd * velocity_error;
	PositionImpedance_UpdateFriction(config, target_error);
	friction_allows_integral = !config->friction_feedforward_enabled ||
		(state.friction_state == POSITION_IMPEDANCE_FRICTION_LANDING &&
		 fast_abs(config->target_position - state.position_reference) <=
			config->position_error_window &&
		 fast_abs(state.friction_current) <=
			POSITION_IMPEDANCE_INTEGRAL_FRICTION_MAX_A &&
		 fast_abs(state.velocity_filtered) <= config->friction_stop_speed &&
		 fast_abs(target_error) <= config->friction_landing_position);
	integral_limit = fast_min(config->integral_limit, config->output_limit);
	if (!isfinite(state.integral))
		state.integral = 0.0f;
	state.integral = constrain(state.integral, -integral_limit, integral_limit);
	if (state.integral_transport_active &&
		(state.target_reached ||
		 fast_abs(state.integral) <= POSITION_IMPEDANCE_INTEGRAL_ZERO_THRESHOLD_A))
		state.integral_transport_active = false;

	if (integral_limit <= 0.0f)
	{
		state.integral = 0.0f;
		state.integral_transport_active = false;
	}
	else if (config->ki <= 0.0f)
	{
		integral_decay_step = constrain(state.integral *
			POSITION_IMPEDANCE_INTEGRAL_OPPOSING_DECAY_RATIO_PER_TICK,
			-POSITION_IMPEDANCE_INTEGRAL_DECAY_MAX_STEP_A,
			 POSITION_IMPEDANCE_INTEGRAL_DECAY_MAX_STEP_A);
		state.integral -= integral_decay_step;
		if (fast_abs(state.integral) <= POSITION_IMPEDANCE_INTEGRAL_ZERO_THRESHOLD_A)
		{
			state.integral = 0.0f;
			state.integral_transport_active = false;
		}
	}
	else
	{
		trajectory_is_moving =
			fast_abs(state.speed_reference) > POSITION_IMPEDANCE_INTEGRAL_SPEED_RAD_S;
		target_is_unsettled = fast_abs(target_error) > config->position_error_window;
		desired_motion = trajectory_is_moving ? state.speed_reference : target_error;
		integral_opposes_motion =
			target_is_unsettled && state.integral * desired_motion < 0.0f;
		feedback_follows_motion = target_is_unsettled &&
			fast_abs(state.velocity_filtered) > POSITION_IMPEDANCE_INTEGRAL_DECAY_MIN_SPEED_RAD_S &&
			desired_motion * state.velocity_filtered > 0.0f &&
			desired_motion * raw_velocity > 0.0f;

		if (state.integral_transport_active &&
			(trajectory_is_moving || target_is_unsettled) &&
			(integral_opposes_motion || feedback_follows_motion))
		{
			if (integral_opposes_motion)
				integral_decay_ratio = POSITION_IMPEDANCE_INTEGRAL_OPPOSING_DECAY_RATIO_PER_TICK;
			else
			{
				integral_transport_distance = fast_abs(raw_velocity) * Position_Ts;
				integral_decay_ratio = constrain(integral_transport_distance /
					POSITION_IMPEDANCE_INTEGRAL_DECAY_DISTANCE_RAD, 0.0f,
					POSITION_IMPEDANCE_INTEGRAL_DECAY_MAX_RATIO_PER_TICK);
			}
			integral_decay_step = constrain(state.integral * integral_decay_ratio,
				-POSITION_IMPEDANCE_INTEGRAL_DECAY_MAX_STEP_A,
				 POSITION_IMPEDANCE_INTEGRAL_DECAY_MAX_STEP_A);
			state.integral -= integral_decay_step;
			if (fast_abs(state.integral) <= POSITION_IMPEDANCE_INTEGRAL_ZERO_THRESHOLD_A)
			{
				state.integral = 0.0f;
				state.integral_transport_active = false;
			}
		}

		if (!trajectory_held_for_friction && friction_allows_integral &&
			!state.target_reached &&
			fast_abs(state.speed_reference) <= POSITION_IMPEDANCE_INTEGRAL_SPEED_RAD_S &&
			fast_abs(state.velocity_filtered) <= POSITION_IMPEDANCE_INTEGRAL_SPEED_RAD_S &&
			fast_abs(position_error) > config->position_error_window &&
			fast_abs(position_error) <= POSITION_IMPEDANCE_INTEGRAL_ZONE_RAD)
		{
			integral_candidate = constrain(state.integral +
				config->ki * position_error * Position_Ts, -integral_limit, integral_limit);
			current_candidate = proportional_current + damping_current + integral_candidate +
				state.friction_current;
			if ((current_candidate >= -config->output_limit &&
				 current_candidate <= config->output_limit) ||
				(current_candidate > config->output_limit && position_error < 0.0f) ||
				(current_candidate < -config->output_limit && position_error > 0.0f))
				state.integral = integral_candidate;
		}
	}

	current_candidate = proportional_current + damping_current + state.integral +
		state.friction_current;
	if (!isfinite(current_candidate))
		return false;
	state.iq_reference = constrain(current_candidate,
		-config->output_limit, config->output_limit);
	PositionImpedance_CopyOutput(output);
	return true;
}
