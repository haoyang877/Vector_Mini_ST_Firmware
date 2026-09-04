#include "position_cascade.h"

#include <math.h>
#include <string.h>

#include "foc_pid.h"
#include "foc_traptraj.h"
#include "hw_conf.h"
#include "position_cascade_config.h"
#include "utils.h"

typedef struct
{
	PI_Controller_TypeDef speed_controller;
	uint16_t position_loop_count;
	uint16_t speed_loop_count;
	float last_target;
	float last_acceleration;
	float last_deceleration;
	float last_maximum_speed;
	float last_position_error;
	float position_reference;
	float speed_reference;
	float iq_reference;
	bool target_reached;
	bool initialized;
} PositionCascadeState_TypeDef;

static PositionCascadeState_TypeDef state;

static bool PositionCascade_ConfigIsValid(const PositionCascadeConfig_TypeDef *config)
{
	return config != NULL &&
		isfinite(config->target_position) &&
		isfinite(config->position_error_window) && config->position_error_window > 0.0f &&
		isfinite(config->acceleration) && config->acceleration > 0.0f &&
		isfinite(config->deceleration) && config->deceleration > 0.0f &&
		isfinite(config->maximum_speed) && config->maximum_speed > 0.0f &&
		isfinite(config->position_kp) && config->position_kp >= 0.0f &&
		config->position_kp <= CASCADE_POSITION_KP_MAX_PER_S &&
		isfinite(config->position_kd) && config->position_kd >= 0.0f &&
		config->position_kd <= CASCADE_POSITION_KD_MAX &&
		isfinite(config->speed_kp) && config->speed_kp >= 0.0f &&
		isfinite(config->speed_ki) && config->speed_ki >= 0.0f &&
		isfinite(config->current_limit) && config->current_limit > 0.0f;
}

static void PositionCascade_CopyOutput(PositionCascadeOutput_TypeDef *output,
	float measured_speed)
{
	output->position_reference = state.position_reference;
	output->speed_reference = state.speed_reference;
	output->speed_feedback = measured_speed;
	output->iq_reference = state.iq_reference;
	output->target_reached = state.target_reached;
}

void PositionCascade_Reset(void)
{
	memset(&state, 0, sizeof(state));
	PI_Controller_Reset(&state.speed_controller);
}

bool PositionCascade_Update(const PositionCascadeConfig_TypeDef *config,
	float measured_position, float measured_speed,
	PositionCascadeOutput_TypeDef *output)
{
	bool trajectory_needs_update;

	if (output == NULL || !isfinite(measured_position) ||
		!isfinite(measured_speed) || !PositionCascade_ConfigIsValid(config))
		return false;

	trajectory_needs_update = !state.initialized ||
		state.last_target != config->target_position ||
		state.last_acceleration != config->acceleration ||
		state.last_deceleration != config->deceleration ||
		state.last_maximum_speed != config->maximum_speed;
	if (trajectory_needs_update)
	{
		if (!state.initialized)
		{
			state.position_reference = measured_position;
			state.speed_reference = 0.0f;
			state.initialized = true;
		}
		if (state.last_target != config->target_position)
			state.target_reached = false;
		TRAJ_plan(config->target_position, state.position_reference,
			state.speed_reference, config->maximum_speed,
			config->acceleration, config->deceleration);
		state.last_target = config->target_position;
		state.last_acceleration = config->acceleration;
		state.last_deceleration = config->deceleration;
		state.last_maximum_speed = config->maximum_speed;
	}

	if (++state.position_loop_count >= CASCADE_POSITION_LOOP_DIVIDER)
	{
		float position_error;
		float derivative_error;
		float speed_correction;

		state.position_loop_count = 0U;
		TRAJ_eval(Cascade_Position_Ts);
		state.position_reference = TRAJ_Get_Y();
		state.speed_reference = TRAJ_Get_Yd();
		position_error = state.position_reference - measured_position;
		derivative_error =
			(position_error - state.last_position_error) / Cascade_Position_Ts;
		speed_correction = config->position_kp * position_error;
		if (state.target_reached)
			speed_correction += config->position_kd * derivative_error;
		state.speed_controller.Ref = constrain(
			state.speed_reference + speed_correction,
			-config->maximum_speed, config->maximum_speed);
		if (fast_abs(measured_position - config->target_position) <=
			config->position_error_window)
			state.target_reached = true;
		state.last_position_error = position_error;
	}

	if (++state.speed_loop_count >= SPEED_LOOP_DIVIDER)
	{
		state.speed_loop_count = 0U;
		PI_Controller_Configure(&state.speed_controller, config->speed_kp,
			config->speed_ki, Speed_Ts, -1.0f, 1.0f);
		state.iq_reference = PI_Controller_Run(&state.speed_controller,
			state.speed_controller.Ref, measured_speed) * config->current_limit;
	}
	if (!isfinite(state.iq_reference))
		return false;
	state.iq_reference = constrain(state.iq_reference,
		-config->current_limit, config->current_limit);
	PositionCascade_CopyOutput(output, measured_speed);
	return true;
}
