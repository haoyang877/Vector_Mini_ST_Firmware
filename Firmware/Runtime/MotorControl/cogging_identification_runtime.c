#include "cogging_identification_runtime.h"

#include <limits.h>
#include <math.h>
#include <string.h>

#include "control_loop_config.h"
#include "fast_math.h"
#include "motor_state_runtime.h"

static void StopOutput(CurrentControlContext *current_control,
	MotorControlContext *motor, PiController *speed_controller)
{
	motor->targets.speed_rad_s = 0.0f;
	motor->runtime.speed_command_ramp_rad_s = 0.0f;
	motor->targets.d_axis_current_a = 0.0f;
	motor->targets.q_axis_current_a = 0.0f;
	PI_Controller_Reset(speed_controller);
	CurrentControlRuntime_ResetControllers(current_control);
	CurrentControlRuntime_ApplyHighSideZeroVector(current_control);
}

void CoggingIdentificationRuntime_Reset(
	CoggingIdentificationRuntimeContext *context)
{
	if (context == 0)
		return;
	memset(context, 0, sizeof(*context));
	context->state = COGGING_IDENT_IDLE;
}

static float SpeedTolerance(const MotorControlContext *motor, float target)
{
	return FastMath_Max(0.05f,
		FastMath_Abs(target) * motor->mechanical_load_profile->
			cogging_identification_speed_tolerance_ratio);
}

static bool SpeedReferenceIsSettled(const MotorControlContext *motor,
	float target)
{
	return FastMath_Abs(motor->runtime.speed_command_ramp_rad_s - target) <=
		SpeedTolerance(motor, target);
}

static bool SampleCurrent(CoggingIdentificationRuntimeContext *context,
	const CurrentControlContext *current_control, const EncoderContext *encoder,
	uint8_t direction)
{
	uint16_t index = encoder->linearized_q15 >> 9U;
	float current_ma = current_control->filtered_q_axis_current_a * 1000.0f;
	if (!isfinite(current_ma) || direction > 1U)
		return false;
	if (context->sample_count[direction][index] < UINT16_MAX)
	{
		context->current_sum_ma[direction][index] += (int32_t)current_ma;
		context->sample_count[direction][index]++;
	}
	return true;
}

static bool BuildMapStep(CoggingIdentificationRuntimeContext *context,
	EncoderContext *encoder, const MotorControlContext *motor)
{
	uint16_t index = context->build_index;
	uint16_t minimum_samples = motor->mechanical_load_profile->
		cogging_identification_min_samples_per_bin;
	int32_t map_value_ma;

	if (context->sample_count[0][index] < minimum_samples ||
		context->sample_count[1][index] < minimum_samples)
		return false;
	map_value_ma = (context->current_sum_ma[0][index] /
		(int32_t)context->sample_count[0][index] +
		context->current_sum_ma[1][index] /
		(int32_t)context->sample_count[1][index]) / 2;
	if (context->build_pass == 0U)
	{
		if (map_value_ma > INT16_MAX || map_value_ma < INT16_MIN)
			return false;
		/* The map remains invalid until pass 2 has removed its DC component. */
		encoder->cogging_compensation_map_ma[index] = (int16_t)map_value_ma;
		context->map_sum_ma += map_value_ma;
	}
	else
	{
		int32_t zero_mean_ma =
			(int32_t)encoder->cogging_compensation_map_ma[index] -
			context->map_sum_ma / (int32_t)COGGING_COMPENSATION_MAP_SIZE;
		int32_t limit_ma = (int32_t)(motor->mechanical_load_profile->
			cogging_identification_max_current_a * 1000.0f);
		if (zero_mean_ma > limit_ma) zero_mean_ma = limit_ma;
		if (zero_mean_ma < -limit_ma) zero_mean_ma = -limit_ma;
		if (zero_mean_ma > INT16_MAX || zero_mean_ma < INT16_MIN)
			return false;
		encoder->cogging_compensation_map_ma[index] = (int16_t)zero_mean_ma;
	}

	context->build_index++;
	if (context->build_index < COGGING_COMPENSATION_MAP_SIZE)
		return true;
	context->build_index = 0U;
	if (context->build_pass == 0U)
	{
		context->build_pass = 1U;
		return true;
	}
	encoder->calib_flag |= ENC_CALIB_COGGING;
	context->state = COGGING_IDENT_COMPLETE;
	return true;
}

void CoggingIdentificationRuntime_ExecuteStep(
	CoggingIdentificationRuntimeContext *context, MotionControlContext *motion,
	CurrentControlContext *current_control, MotorControlContext *motor,
	PiController *speed_controller, EncoderContext *encoder,
	MotorStateContext *motor_state)
{
	float speed;
	float target;
	int64_t required_travel_q15;
	bool waiting;

	if (context == 0 || motion == 0 || current_control == 0 || motor == 0 ||
		speed_controller == 0 || encoder == 0 || motor->mechanical_load_profile == 0 ||
		!Encoder_IsOnline(encoder) ||
		(encoder->calib_flag & ENC_CALIB_ALL) != ENC_CALIB_ALL)
	{
		MotorState_RaiseFault(motor_state, MOTOR_FAULT_COGGING_IDENTIFICATION);
		return;
	}
	speed = motor->mechanical_load_profile->cogging_identification_speed_rad_s;
	required_travel_q15 = (int64_t)motor->mechanical_load_profile->
		cogging_identification_turns * (int64_t)ENCODER_Q15_CPR;
	if (speed <= 0.0f || required_travel_q15 <= 0 ||
		motor->mechanical_load_profile->cogging_identification_min_samples_per_bin == 0U)
	{
		MotorState_RaiseFault(motor_state, MOTOR_FAULT_INVALID_PARAMETER);
		return;
	}

	if (context->state == COGGING_IDENT_IDLE)
	{
		memset(context->current_sum_ma, 0, sizeof(context->current_sum_ma));
		memset(context->sample_count, 0, sizeof(context->sample_count));
		memset(encoder->cogging_compensation_map_ma, 0,
			sizeof(encoder->cogging_compensation_map_ma));
		encoder->calib_flag &= (uint8_t)~ENC_CALIB_COGGING;
		context->stable_ticks = 0U;
		context->stable_speed_sum_rad_s = 0.0f;
		context->stage_ticks = 0U;
		context->state = COGGING_IDENT_WAIT_CW;
		PI_Controller_Reset(speed_controller);
	}

	waiting = context->state == COGGING_IDENT_WAIT_CW ||
		context->state == COGGING_IDENT_WAIT_CCW;
	if (waiting || context->state == COGGING_IDENT_SAMPLE_CW ||
		context->state == COGGING_IDENT_SAMPLE_CCW)
	{
		target = (context->state == COGGING_IDENT_WAIT_CW ||
			context->state == COGGING_IDENT_SAMPLE_CW) ? speed : -speed;
		motor->targets.speed_rad_s = target;
		ControlModeRuntime_RunSpeed(motion, current_control, motor,
			speed_controller, encoder);
		context->stage_ticks++;
		if (context->stage_ticks >= (uint32_t)(motor->mechanical_load_profile->
			cogging_identification_stage_timeout_s / CURRENT_LOOP_PERIOD_S))
		{
			MotorState_RaiseFault(motor_state, MOTOR_FAULT_COGGING_IDENTIFICATION);
			return;
		}
		if (waiting)
		{
			if (SpeedReferenceIsSettled(motor, target))
			{
				context->stable_ticks++;
				context->stable_speed_sum_rad_s += encoder->vel_mech;
			}
			else
			{
				context->stable_ticks = 0U;
				context->stable_speed_sum_rad_s = 0.0f;
			}
			if (context->stable_ticks >= (uint32_t)(motor->mechanical_load_profile->
				cogging_identification_stable_time_s / CURRENT_LOOP_PERIOD_S))
			{
				float mean_speed = context->stable_speed_sum_rad_s /
					(float)context->stable_ticks;
				if (FastMath_Abs(mean_speed - target) <=
					SpeedTolerance(motor, target))
				{
					context->start_shadow_q15 = encoder->shadow_q15;
					context->stage_ticks = 0U;
					context->stable_ticks = 0U;
					context->stable_speed_sum_rad_s = 0.0f;
					context->state = context->state == COGGING_IDENT_WAIT_CW ?
						COGGING_IDENT_SAMPLE_CW : COGGING_IDENT_SAMPLE_CCW;
				}
				else
				{
					context->stable_ticks = 0U;
					context->stable_speed_sum_rad_s = 0.0f;
				}
			}
			return;
		}
		/* Once the mean speed has settled, retain every finite sample across
		 * the complete revolutions. Rejecting samples at instantaneous speed
		 * excursions biases the result by removing the rotor angles with the
		 * strongest cogging disturbance. Per-bin coverage is validated while
		 * building the map. */
		if (!SampleCurrent(context, current_control, encoder,
				context->state == COGGING_IDENT_SAMPLE_CW ? 0U : 1U))
			return;
		if (FastMath_Abs((float)(encoder->shadow_q15 -
			context->start_shadow_q15)) < (float)required_travel_q15)
			return;
		context->stage_ticks = 0U;
		context->stable_ticks = 0U;
		context->stable_speed_sum_rad_s = 0.0f;
		context->state = context->state == COGGING_IDENT_SAMPLE_CW ?
			COGGING_IDENT_WAIT_CCW : COGGING_IDENT_BUILD;
		return;
	}

	if (context->state == COGGING_IDENT_BUILD)
	{
		StopOutput(current_control, motor, speed_controller);
		if (!BuildMapStep(context, encoder, motor))
		{
			MotorState_RaiseFault(motor_state, MOTOR_FAULT_COGGING_IDENTIFICATION);
			return;
		}
	}
	if (context->state == COGGING_IDENT_COMPLETE && !context->completion_reported)
	{
		context->completion_reported = true;
		MotorLifecycle_ReportServiceComplete(motor_state, true);
	}
}

void CoggingIdentificationRuntime_Cancel(
	CoggingIdentificationRuntimeContext *context,
	CurrentControlContext *current_control, MotorControlContext *motor,
	PiController *speed_controller)
{
	if (context == 0)
		return;
	if (current_control != 0 && motor != 0 && speed_controller != 0)
		StopOutput(current_control, motor, speed_controller);
	CoggingIdentificationRuntime_Reset(context);
}
