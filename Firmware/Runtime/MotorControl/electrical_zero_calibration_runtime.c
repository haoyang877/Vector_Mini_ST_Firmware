#include "electrical_zero_calibration_runtime.h"

#include "control_loop_config.h"
#include "fast_math.h"
#include "motor_state_runtime.h"

void ElectricalZeroCalibrationRuntime_Reset(
	ElectricalZeroCalibrationContext *context)
{
	if (context == 0)
		return;
	context->loop_count = 0U;
	context->sample_count = 0U;
	context->sample_anchor = 0U;
	context->unwrapped_sum = 0;
	context->completion_reported = false;
}

void ElectricalZeroCalibrationRuntime_ExecuteStep(
	ElectricalZeroCalibrationContext *context,
	CurrentControlContext *current_control, MotorControlContext *motor,
	EncoderContext *encoder)
{
	float ramp_time_s;
	float align_time_s;
	float hold_time_s;
	float time;
	float align_current;

	if (context == 0 || current_control == 0 || motor == 0 || encoder == 0 ||
		motor->tuning_profile == 0 || motor->mechanical_load_profile == 0)
	{
		MotorState_RaiseFault(MOTOR_FAULT_INVALID_PARAMETER);
		return;
	}
	if (context->completion_reported)
	{
		CurrentControlRuntime_ApplyHighSideZeroVector(current_control);
		return;
	}
	ramp_time_s = motor->tuning_profile->encoder_electrical_zero_current_ramp_time_s;
	hold_time_s = motor->tuning_profile->encoder_electrical_zero_hold_time_s;
	align_time_s = ramp_time_s + hold_time_s;
	time = (float)context->loop_count * CURRENT_LOOP_PERIOD_S;
	align_current = motor->configuration.calibration_current_a;
	if (align_current < motor->mechanical_load_profile->
		encoder_electrical_zero_min_align_current_a)
		align_current = motor->mechanical_load_profile->
			encoder_electrical_zero_min_align_current_a;

	if (!Encoder_IsOnline(encoder))
	{
		MotorState_RaiseFault(MOTOR_FAULT_ENCODER);
		ElectricalZeroCalibrationRuntime_Reset(context);
		CurrentControlRuntime_ApplyHighSideZeroVector(current_control);
		return;
	}
	if ((encoder->calib_flag & ENC_CALIB_LINEARIZED) == 0U)
	{
		MotorState_RaiseFault(MOTOR_FAULT_ENCODER_NOT_CALIBRATED);
		return;
	}
	if (align_current <= 0.0f || ramp_time_s <= 0.0f || hold_time_s <= 0.0f)
	{
		MotorState_RaiseFault(MOTOR_FAULT_INVALID_PARAMETER);
		return;
	}
	if (motor->configuration.current_limit_a > 0.0f &&
		align_current > motor->configuration.current_limit_a)
		align_current = motor->configuration.current_limit_a;

	if (context->loop_count == 0U)
	{
		context->sample_count = 0U;
		context->sample_anchor = encoder->linearized_q15;
		context->unwrapped_sum = 0;
		encoder->calib_flag &= (uint8_t)~ENC_CALIB_ELECTRICAL_ZERO;
		CurrentControlRuntime_ResetControllers(current_control);
	}
	motor->targets.d_axis_current_a = align_current * FastMath_Clamp(
		((float)context->loop_count + 1.0f) * CURRENT_LOOP_PERIOD_S /
		ramp_time_s, 0.0f, 1.0f);
	motor->targets.q_axis_current_a = 0.0f;
	CurrentControlRuntime_RunClosedLoop(current_control, motor, 0.0f, 0.0f);

	if (time >= align_time_s - hold_time_s &&
		encoder->read_status == ENCODER_READ_OK)
	{
		int32_t unwrapped_q15 = (int32_t)context->sample_anchor +
			(int16_t)(uint16_t)(encoder->linearized_q15 - context->sample_anchor);
		context->unwrapped_sum += unwrapped_q15;
		context->sample_count++;
	}
	if (time >= align_time_s)
	{
		bool calibrated = false;
		if (context->sample_count > 0U)
		{
			uint16_t electrical_zero_q15 = (uint16_t)(context->unwrapped_sum /
				(int64_t)context->sample_count);
			calibrated = Encoder_SetElectricalZeroQ15(encoder, electrical_zero_q15);
		}
		motor->targets.d_axis_current_a = 0.0f;
		motor->targets.q_axis_current_a = 0.0f;
		CurrentControlRuntime_ResetControllers(current_control);
		CurrentControlRuntime_ApplyHighSideZeroVector(current_control);
		if (!calibrated)
		{
			ElectricalZeroCalibrationRuntime_Reset(context);
			MotorState_RaiseFault(MOTOR_FAULT_ENCODER);
			return;
		}
		context->completion_reported = true;
		MotorLifecycle_ReportServiceComplete(true);
		return;
	}
	context->loop_count++;
}
