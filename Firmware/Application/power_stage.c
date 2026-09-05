#include "power_stage.h"

#include <stddef.h>

#define POWER_STAGE_INACTIVE_DUTY 1.0f

static bool PowerStage_IsValidDutyCycle(float duty_cycle)
{
	return duty_cycle == duty_cycle && duty_cycle >= 0.0f && duty_cycle <= 1.0f;
}

void PowerStage_Initialize(PowerStageContext *context, const PowerStagePort *port)
{
	if (context == NULL)
		return;

	context->is_initialized = false;
	context->outputs_enabled = false;
	context->has_latched_fault = false;
	if (port == NULL || port->enable_outputs == NULL ||
		port->disable_outputs == NULL || port->write_duty_cycles == NULL)
		return;

	context->port = *port;
	context->is_initialized = true;
	context->port.disable_outputs(context->port.context);
	context->port.write_duty_cycles(context->port.context,
		POWER_STAGE_INACTIVE_DUTY, POWER_STAGE_INACTIVE_DUTY,
		POWER_STAGE_INACTIVE_DUTY);
}

bool PowerStage_RequestEnable(PowerStageContext *context, bool safety_interlock_clear)
{
	if (context == NULL || !context->is_initialized || !safety_interlock_clear)
		return false;

	if (context->outputs_enabled)
		return true;

	if (!context->port.enable_outputs(context->port.context))
	{
		context->port.disable_outputs(context->port.context);
		context->port.write_duty_cycles(context->port.context,
			POWER_STAGE_INACTIVE_DUTY, POWER_STAGE_INACTIVE_DUTY,
			POWER_STAGE_INACTIVE_DUTY);
		context->outputs_enabled = false;
		context->has_latched_fault = true;
		return false;
	}

	context->outputs_enabled = true;
	return true;
}

void PowerStage_ForceDisable(PowerStageContext *context)
{
	if (context == NULL || !context->is_initialized)
		return;

	if (context->outputs_enabled)
	{
		context->port.disable_outputs(context->port.context);
		context->port.write_duty_cycles(context->port.context,
			POWER_STAGE_INACTIVE_DUTY, POWER_STAGE_INACTIVE_DUTY,
			POWER_STAGE_INACTIVE_DUTY);
		context->outputs_enabled = false;
	}
}

bool PowerStage_ApplyDutyCycles(PowerStageContext *context,
	float phase_a, float phase_b, float phase_c)
{
	if (context == NULL || !context->is_initialized ||
		!PowerStage_IsValidDutyCycle(phase_a) ||
		!PowerStage_IsValidDutyCycle(phase_b) ||
		!PowerStage_IsValidDutyCycle(phase_c))
	{
		PowerStage_ForceDisable(context);
		if (context != NULL && context->is_initialized)
			context->has_latched_fault = true;
		return false;
	}

	context->port.write_duty_cycles(context->port.context,
		phase_a, phase_b, phase_c);
	return true;
}

bool PowerStage_AreOutputsEnabled(const PowerStageContext *context)
{
	return context != NULL && context->is_initialized && context->outputs_enabled;
}

bool PowerStage_HasLatchedFault(const PowerStageContext *context)
{
	return context != NULL && context->is_initialized &&
		context->has_latched_fault;
}

void PowerStage_RejectOutputCommand(PowerStageContext *context)
{
	PowerStage_ForceDisable(context);
	if (context != NULL && context->is_initialized)
		context->has_latched_fault = true;
}

void PowerStage_ClearLatchedFault(PowerStageContext *context)
{
	if (context != NULL && context->is_initialized && !context->outputs_enabled)
		context->has_latched_fault = false;
}
