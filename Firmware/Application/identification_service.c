#include "identification_service.h"

#include <stddef.h>
#include <math.h>

bool IdentificationService_Initialize(IdentificationServiceContext *context,
	DeviceLifecycleContext *lifecycle, const MotorProfile *motor_profile,
	uint32_t timeout_ticks)
{
	if (context == NULL || lifecycle == NULL || motor_profile == NULL ||
		timeout_ticks == 0U)
		return false;
	context->lifecycle = lifecycle;
	context->motor_profile = motor_profile;
	context->elapsed_ticks = 0U;
	context->timeout_ticks = timeout_ticks;
	return true;
}

bool IdentificationService_AcceptPhaseResistanceResult(
	const IdentificationServiceContext *context,
	float phase_a_resistance_ohm, float phase_b_resistance_ohm,
	float phase_c_resistance_ohm, float spread_pct, bool is_balanced,
	float *mean_resistance_ohm)
{
	float mean;
	if (context == NULL || context->motor_profile == NULL ||
		mean_resistance_ohm == NULL || !isfinite(phase_a_resistance_ohm) ||
		!isfinite(phase_b_resistance_ohm) ||
		!isfinite(phase_c_resistance_ohm) || !isfinite(spread_pct) ||
		!is_balanced || phase_a_resistance_ohm <= 0.0f ||
		phase_b_resistance_ohm <= 0.0f || phase_c_resistance_ohm <= 0.0f ||
		spread_pct > context->motor_profile->phase_resistance_balance_fault_pct)
		return false;
	mean = (phase_a_resistance_ohm + phase_b_resistance_ohm +
		phase_c_resistance_ohm) / 3.0f;
	if (!isfinite(mean) || mean < 0.0001f || mean > 1.0f)
		return false;
	*mean_resistance_ohm = mean;
	return true;
}

bool IdentificationService_OwnsProcedure(ServiceProcedure procedure)
{
	return procedure == SERVICE_PROCEDURE_PHASE_RESISTANCE_IDENTIFICATION ||
		procedure == SERVICE_PROCEDURE_FRICTION_IDENTIFICATION;
}

bool IdentificationService_Supervise1kHz(IdentificationServiceContext *context)
{
	DeviceLifecycleContext *lifecycle;
	if (context == NULL || context->lifecycle == NULL)
		return false;
	lifecycle = context->lifecycle;
	if (lifecycle->device_state != DEVICE_STATE_SERVICING ||
		!IdentificationService_OwnsProcedure(lifecycle->service_procedure))
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
	/* The friction core owns per-stage timeouts and fit rejection. */
	if (lifecycle->service_procedure ==
		SERVICE_PROCEDURE_FRICTION_IDENTIFICATION)
		return true;
	if (context->elapsed_ticks < UINT32_MAX)
		context->elapsed_ticks++;
	if (context->elapsed_ticks < context->timeout_ticks)
		return true;
	DeviceLifecycle_FailService(lifecycle);
	return false;
}
