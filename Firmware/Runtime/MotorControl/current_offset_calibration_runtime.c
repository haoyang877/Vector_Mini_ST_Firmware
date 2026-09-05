#include "current_offset_calibration_runtime.h"

#include "motor_state_runtime.h"

void CurrentOffsetCalibrationRuntime_Reset(
	CurrentOffsetCalibrationContext *context)
{
	if (context == 0)
		return;
	context->offset_count = 0U;
	context->phase_a_offset_sum = 0U;
	context->phase_b_offset_sum = 0U;
	context->phase_c_offset_sum = 0U;
	context->phase_a_offset_adc = 0U;
	context->phase_b_offset_adc = 0U;
	context->phase_c_offset_adc = 0U;
	context->result_is_ready = false;
}

CurrentOffsetCalibrationStatus CurrentOffsetCalibrationRuntime_ExecuteStep(
	CurrentOffsetCalibrationContext *context,
	CurrentControlContext *current_control,
	const BoardProfile *board_profile)
{
	if (context == 0 || current_control == 0 || board_profile == 0 ||
		board_profile->current_offset_calibration_sample_count == 0U)
	{
		MotorState_RaiseFault(MOTOR_FAULT_INVALID_PARAMETER);
		return CURRENT_OFFSET_CALIBRATION_RUNNING;
	}
	context->phase_a_offset_sum += current_control->measurement_raw.phase_a_adc;
	context->phase_b_offset_sum += current_control->measurement_raw.phase_b_adc;
	context->phase_c_offset_sum += current_control->measurement_raw.phase_c_adc;
	context->offset_count++;
	if (context->offset_count <
		board_profile->current_offset_calibration_sample_count)
		return CURRENT_OFFSET_CALIBRATION_RUNNING;

	context->phase_a_offset_adc =
		(uint16_t)(context->phase_a_offset_sum / context->offset_count);
	context->phase_b_offset_adc =
		(uint16_t)(context->phase_b_offset_sum / context->offset_count);
	context->phase_c_offset_adc =
		(uint16_t)(context->phase_c_offset_sum / context->offset_count);
	context->result_is_ready = true;
	return CURRENT_OFFSET_CALIBRATION_COMPLETE;
}

bool CurrentOffsetCalibrationRuntime_ReadResult(
	const CurrentOffsetCalibrationContext *context,
	uint16_t *phase_a_offset_adc, uint16_t *phase_b_offset_adc,
	uint16_t *phase_c_offset_adc)
{
	if (context == 0 || phase_a_offset_adc == 0 || phase_b_offset_adc == 0 ||
		phase_c_offset_adc == 0 || !context->result_is_ready)
		return false;
	*phase_a_offset_adc = context->phase_a_offset_adc;
	*phase_b_offset_adc = context->phase_b_offset_adc;
	*phase_c_offset_adc = context->phase_c_offset_adc;
	return true;
}
