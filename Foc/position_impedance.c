#include "position_impedance.h"

#include <math.h>
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
	uint32_t hold_counter;
	uint16_t loop_counter;
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
		isfinite(config->output_limit) && config->output_limit > 0.0f;
}

static void PositionImpedance_CopyOutput(PositionImpedanceOutput_TypeDef *output)
{
	output->position_reference = state.position_reference;
	output->speed_reference = state.speed_reference;
	output->velocity_feedback = state.velocity_filtered;
	output->iq_reference = state.iq_reference;
	output->target_reached = state.target_reached;
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
	uint32_t hold_ticks;
	bool trajectory_is_moving;
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
		}
	}

	if (trajectory_needs_update)
	{
		TRAJ_plan(config->target_position, state.position_reference,
			state.speed_reference, config->maximum_speed,
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

	TRAJ_eval(Position_Ts);
	state.position_reference = TRAJ_Get_Y();
	state.speed_reference = TRAJ_Get_Yd();

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
	hold_ticks = (uint32_t)(POSITION_IMPEDANCE_HOLD_TIME_S / Position_Ts + 0.5f);
	if (hold_ticks < 1U)
		hold_ticks = 1U;
	if (state.target_reached)
	{
		if (fast_abs(target_error) > 2.0f * config->position_error_window ||
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

		if (fast_abs(state.speed_reference) <= POSITION_IMPEDANCE_INTEGRAL_SPEED_RAD_S &&
			fast_abs(state.velocity_filtered) <= POSITION_IMPEDANCE_INTEGRAL_SPEED_RAD_S &&
			fast_abs(position_error) > config->position_error_window &&
			fast_abs(position_error) <= POSITION_IMPEDANCE_INTEGRAL_ZONE_RAD)
		{
			integral_candidate = constrain(state.integral +
				config->ki * position_error * Position_Ts, -integral_limit, integral_limit);
			current_candidate = proportional_current + damping_current + integral_candidate;
			if ((current_candidate >= -config->output_limit &&
				 current_candidate <= config->output_limit) ||
				(current_candidate > config->output_limit && position_error < 0.0f) ||
				(current_candidate < -config->output_limit && position_error > 0.0f))
				state.integral = integral_candidate;
		}
	}

	current_candidate = proportional_current + damping_current + state.integral;
	if (!isfinite(current_candidate))
		return false;
	state.iq_reference = constrain(current_candidate,
		-config->output_limit, config->output_limit);
	PositionImpedance_CopyOutput(output);
	return true;
}
