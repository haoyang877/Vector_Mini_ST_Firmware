#include "encoder_direction_calibration_runtime.h"

#include "control_loop_config.h"
#include "fast_math.h"
#include "motor_state_runtime.h"

void EncoderDirectionCalibrationRuntime_Reset(
	EncoderDirectionCalibrationContext *context)
{
	if (context == 0)
		return;
	context->state = ENCODER_DIRECTION_CALIBRATION_IDLE;
	context->loop_count = 0U;
	context->drive_phase_rad = 0.0f;
	context->previous_raw_q15 = 0U;
	context->raw_travel_q15 = 0;
	context->completion_reported = false;
}

void EncoderDirectionCalibrationRuntime_ExecuteStep(
	EncoderDirectionCalibrationContext *context,
	CurrentControlContext *current_control, MotorControlContext *motor,
	EncoderContext *encoder, MotorStateContext *motor_state)
{
	float align_current_a;
	float align_time_s;
	float drive_speed_electrical_rad_s;
	float required_electrical_travel_rad;
	float time_s;

	if (context == 0 || current_control == 0 || motor == 0 || encoder == 0 ||
		motor->tuning_profile == 0 || motor->mechanical_load_profile == 0 ||
		motor->configuration.pole_pairs <= 0)
	{
		MotorState_RaiseFault(motor_state, MOTOR_FAULT_INVALID_PARAMETER);
		return;
	}
	if (!Encoder_IsOnline(encoder))
	{
		MotorState_RaiseFault(motor_state, MOTOR_FAULT_ENCODER);
		return;
	}

	align_time_s = motor->tuning_profile->encoder_direction_align_time_s;
	drive_speed_electrical_rad_s = motor->tuning_profile->
		encoder_direction_speed_electrical_rad_s;
	align_current_a = motor->configuration.calibration_current_a;
	if (align_current_a < motor->mechanical_load_profile->
		encoder_electrical_zero_min_align_current_a)
		align_current_a = motor->mechanical_load_profile->
			encoder_electrical_zero_min_align_current_a;
	if (align_current_a > motor->configuration.current_limit_a)
		align_current_a = motor->configuration.current_limit_a;
	if (align_time_s <= 0.0f || drive_speed_electrical_rad_s <= 0.0f ||
		align_current_a <= 0.0f)
	{
		MotorState_RaiseFault(motor_state, MOTOR_FAULT_INVALID_PARAMETER);
		return;
	}
	required_electrical_travel_rad = MATH_TWO_PI *
		(float)motor->configuration.pole_pairs;
	time_s = (float)context->loop_count * CURRENT_LOOP_PERIOD_S;

	if (context->state == ENCODER_DIRECTION_CALIBRATION_IDLE)
	{
		CurrentControlRuntime_ResetControllers(current_control);
		context->state = ENCODER_DIRECTION_CALIBRATION_ALIGN;
		context->loop_count = 0U;
		context->drive_phase_rad = 0.0f;
		context->raw_travel_q15 = 0;
	}

	if (context->state == ENCODER_DIRECTION_CALIBRATION_ALIGN)
	{
		motor->targets.d_axis_current_a = align_current_a *
			FastMath_Clamp(((float)context->loop_count + 1.0f) *
				CURRENT_LOOP_PERIOD_S / align_time_s, 0.0f, 1.0f);
		motor->targets.q_axis_current_a = 0.0f;
		CurrentControlRuntime_RunClosedLoop(current_control, motor, 0.0f, 0.0f);
		if (time_s >= align_time_s)
		{
			context->previous_raw_q15 = encoder->raw_q15;
			context->raw_travel_q15 = 0;
			context->loop_count = 0U;
			context->state = ENCODER_DIRECTION_CALIBRATION_ROTATE;
		}
		else
			context->loop_count++;
		return;
	}

	if (context->state == ENCODER_DIRECTION_CALIBRATION_ROTATE)
	{
		int16_t raw_delta_q15 = (int16_t)(uint16_t)(encoder->raw_q15 -
			context->previous_raw_q15);
		context->previous_raw_q15 = encoder->raw_q15;
		context->raw_travel_q15 += raw_delta_q15;
		context->drive_phase_rad += drive_speed_electrical_rad_s *
			CURRENT_LOOP_PERIOD_S;
		motor->targets.d_axis_current_a = align_current_a;
		motor->targets.q_axis_current_a = 0.0f;
		CurrentControlRuntime_RunClosedLoop(current_control, motor,
			context->drive_phase_rad, drive_speed_electrical_rad_s);
		if (context->drive_phase_rad < required_electrical_travel_rad)
			return;

		motor->targets.d_axis_current_a = 0.0f;
		motor->targets.q_axis_current_a = 0.0f;
		CurrentControlRuntime_ResetControllers(current_control);
		CurrentControlRuntime_ApplyHighSideZeroVector(current_control);
		if (context->raw_travel_q15 > ENCODER_Q15_HALF_TURN)
			Encoder_SetReverse(encoder, false);
		else if (context->raw_travel_q15 < -ENCODER_Q15_HALF_TURN)
			Encoder_SetReverse(encoder, true);
		else
		{
			MotorState_RaiseFault(motor_state, MOTOR_FAULT_ENCODER_DIRECTION);
			return;
		}
		/* Positive/negative friction coefficients belong to the old direction
		 * convention. A standalone direction calibration must not save them. */
		motor->configuration.friction_model_valid = false;
		context->state = ENCODER_DIRECTION_CALIBRATION_COMPLETE;
	}

	if (context->state == ENCODER_DIRECTION_CALIBRATION_COMPLETE &&
		!context->completion_reported)
	{
		context->completion_reported = true;
		MotorLifecycle_ReportServiceComplete(motor_state, true);
	}
}
