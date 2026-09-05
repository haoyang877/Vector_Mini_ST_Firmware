#include "calibration_service.h"

#include <stddef.h>

bool CalibrationService_Initialize(CalibrationServiceContext *context,
	DeviceLifecycleContext *lifecycle, const BoardProfile *board_profile,
	uint32_t timeout_ticks)
{
	if (context == NULL || lifecycle == NULL || board_profile == NULL ||
		timeout_ticks == 0U)
		return false;
	context->lifecycle = lifecycle;
	context->board_profile = board_profile;
	context->elapsed_ticks = 0U;
	context->timeout_ticks = timeout_ticks;
	return true;
}

bool CalibrationService_AcceptCurrentOffsetResult(
	const CalibrationServiceContext *context, uint16_t phase_a_offset_adc,
	uint16_t phase_b_offset_adc, uint16_t phase_c_offset_adc)
{
	if (context == NULL || context->board_profile == NULL)
		return false;
	return phase_a_offset_adc >=
			context->board_profile->minimum_current_offset_adc &&
		phase_a_offset_adc <=
			context->board_profile->maximum_current_offset_adc &&
		phase_b_offset_adc >=
			context->board_profile->minimum_current_offset_adc &&
		phase_b_offset_adc <=
			context->board_profile->maximum_current_offset_adc &&
		phase_c_offset_adc >=
			context->board_profile->minimum_current_offset_adc &&
		phase_c_offset_adc <=
			context->board_profile->maximum_current_offset_adc;
}

bool CalibrationService_OwnsProcedure(ServiceProcedure procedure)
{
	return procedure == SERVICE_PROCEDURE_CURRENT_OFFSET_CALIBRATION ||
		procedure == SERVICE_PROCEDURE_ENCODER_LINEARIZATION ||
		procedure == SERVICE_PROCEDURE_ELECTRICAL_ZERO_CALIBRATION ||
		procedure == SERVICE_PROCEDURE_OBSERVER_CALIBRATION ||
		procedure == SERVICE_PROCEDURE_ENCODER_DIRECTION_CALIBRATION ||
		procedure == SERVICE_PROCEDURE_SET_MECHANICAL_ZERO;
}

bool CalibrationService_Supervise1kHz(CalibrationServiceContext *context)
{
	DeviceLifecycleContext *lifecycle;
	if (context == NULL || context->lifecycle == NULL)
		return false;
	lifecycle = context->lifecycle;
	if (lifecycle->device_state != DEVICE_STATE_SERVICING ||
		!CalibrationService_OwnsProcedure(lifecycle->service_procedure))
	{
		context->elapsed_ticks = 0U;
		return true;
	}
	if (lifecycle->procedure_state == PROCEDURE_STATE_PRECHECK)
	{
		context->elapsed_ticks = 0U;
		return DeviceLifecycle_BeginServiceRun(lifecycle);
	}
	if (lifecycle->procedure_state != PROCEDURE_STATE_RUNNING)
		return true;
	if (context->elapsed_ticks < UINT32_MAX)
		context->elapsed_ticks++;
	if (context->elapsed_ticks < context->timeout_ticks)
		return true;
	DeviceLifecycle_FailService(lifecycle);
	return false;
}
