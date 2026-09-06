#include "Core/Application/motor_commissioning_workflow.h"

#include <stddef.h>

static ServiceProcedure ProcedureForStage(MotorCommissioningStage stage)
{
	switch (stage)
	{
		case COMMISSIONING_STAGE_CURRENT_OFFSET:
			return SERVICE_PROCEDURE_CURRENT_OFFSET_CALIBRATION;
		case COMMISSIONING_STAGE_PHASE_RESISTANCE_CHECK:
			return SERVICE_PROCEDURE_PHASE_RESISTANCE_IDENTIFICATION;
		case COMMISSIONING_STAGE_ENCODER_DIRECTION:
			return SERVICE_PROCEDURE_ENCODER_DIRECTION_CALIBRATION;
		case COMMISSIONING_STAGE_ENCODER_LUT:
			return SERVICE_PROCEDURE_OBSERVER_CALIBRATION;
		case COMMISSIONING_STAGE_ELECTRICAL_AND_MECHANICAL_ZERO:
			return SERVICE_PROCEDURE_ELECTRICAL_ZERO_CALIBRATION;
		case COMMISSIONING_STAGE_FRICTION:
			return SERVICE_PROCEDURE_FRICTION_IDENTIFICATION;
		case COMMISSIONING_STAGE_COGGING:
			return SERVICE_PROCEDURE_COGGING_IDENTIFICATION;
		case COMMISSIONING_STAGE_SAVE:
			return SERVICE_PROCEDURE_PARAMETER_SAVE;
		default:
			return SERVICE_PROCEDURE_NONE;
	}
}

static MotorCommissioningStage NextEnabledStage(
	const MotorCommissioningWorkflowContext *context,
	MotorCommissioningStage completed_stage)
{
	uint8_t stage;

	for (stage = (uint8_t)completed_stage + 1U;
		stage <= (uint8_t)COMMISSIONING_STAGE_SAVE; stage++)
	{
		if ((context->enabled_stage_mask &
			MOTOR_COMMISSIONING_STAGE_MASK(stage)) != 0U)
		{
			return (MotorCommissioningStage)stage;
		}
	}
	return COMMISSIONING_STAGE_COMPLETE;
}

static uint8_t CountEnabledStages(MotorCommissioningStageMask mask)
{
	uint8_t count = 0U;

	while (mask != 0U)
	{
		count = (uint8_t)(count + (uint8_t)(mask & 1U));
		mask >>= 1U;
	}
	return count;
}

void MotorCommissioningWorkflow_Initialize(
	MotorCommissioningWorkflowContext *context)
{
	if (context == NULL)
		return;
	context->stage = COMMISSIONING_STAGE_IDLE;
	context->failure_stage = COMMISSIONING_STAGE_IDLE;
	context->enabled_stage_mask = MOTOR_COMMISSIONING_SUPPORTED_STAGE_MASK;
	context->completed_stage_mask = 0U;
	context->failure_reason = 0U;
	context->active = false;
}

bool MotorCommissioningWorkflow_Configure(
	MotorCommissioningWorkflowContext *context,
	MotorCommissioningStageMask enabled_stage_mask)
{
	if (context == NULL || context->active ||
		(enabled_stage_mask & ~MOTOR_COMMISSIONING_SUPPORTED_STAGE_MASK) != 0U)
	{
		return false;
	}
	context->enabled_stage_mask = enabled_stage_mask;
	return true;
}

bool MotorCommissioningWorkflow_IsServiceEnabled(
	const MotorCommissioningWorkflowContext *context,
	ServiceProcedure procedure)
{
	MotorCommissioningStage stage;

	if (context == NULL)
		return false;
	switch (procedure)
	{
		case SERVICE_PROCEDURE_CURRENT_OFFSET_CALIBRATION:
			stage = COMMISSIONING_STAGE_CURRENT_OFFSET; break;
		case SERVICE_PROCEDURE_PHASE_RESISTANCE_IDENTIFICATION:
			stage = COMMISSIONING_STAGE_PHASE_RESISTANCE_CHECK; break;
		case SERVICE_PROCEDURE_ENCODER_DIRECTION_CALIBRATION:
			stage = COMMISSIONING_STAGE_ENCODER_DIRECTION; break;
		case SERVICE_PROCEDURE_ENCODER_LINEARIZATION:
		case SERVICE_PROCEDURE_OBSERVER_CALIBRATION:
			stage = COMMISSIONING_STAGE_ENCODER_LUT; break;
		case SERVICE_PROCEDURE_ELECTRICAL_ZERO_CALIBRATION:
		case SERVICE_PROCEDURE_SET_MECHANICAL_ZERO:
			stage = COMMISSIONING_STAGE_ELECTRICAL_AND_MECHANICAL_ZERO; break;
		case SERVICE_PROCEDURE_FRICTION_IDENTIFICATION:
			stage = COMMISSIONING_STAGE_FRICTION; break;
		case SERVICE_PROCEDURE_COGGING_IDENTIFICATION:
			stage = COMMISSIONING_STAGE_COGGING; break;
		case SERVICE_PROCEDURE_PARAMETER_SAVE:
		case SERVICE_PROCEDURE_RESTORE_DEFAULTS:
		case SERVICE_PROCEDURE_FULL_COMMISSIONING:
			return true;
		default:
			return false;
	}
	return (context->enabled_stage_mask &
		MOTOR_COMMISSIONING_STAGE_MASK(stage)) != 0U;
}

bool MotorCommissioningWorkflow_Start(
	MotorCommissioningWorkflowContext *context,
	ServiceProcedure *first_procedure)
{
	if (context == NULL || first_procedure == NULL || context->active)
		return false;
	context->stage = COMMISSIONING_STAGE_IDLE;
	context->failure_stage = COMMISSIONING_STAGE_IDLE;
	context->completed_stage_mask = 0U;
	context->failure_reason = 0U;
	if (context->enabled_stage_mask == 0U ||
		(context->enabled_stage_mask &
			~MOTOR_COMMISSIONING_SUPPORTED_STAGE_MASK) != 0U)
	{
		return false;
	}
	context->active = true;
	context->stage = NextEnabledStage(context, COMMISSIONING_STAGE_IDLE);
	*first_procedure = ProcedureForStage(context->stage);
	return *first_procedure != SERVICE_PROCEDURE_NONE;
}

bool MotorCommissioningWorkflow_CompleteProcedure(
	MotorCommissioningWorkflowContext *context,
	ServiceProcedure completed_procedure,
	ServiceProcedure *next_procedure, bool *workflow_complete)
{
	if (context == NULL || next_procedure == NULL || workflow_complete == NULL ||
		!context->active || completed_procedure != ProcedureForStage(context->stage))
		return false;

	context->completed_stage_mask |= (uint16_t)(1U << (uint8_t)context->stage);
	context->stage = NextEnabledStage(context, context->stage);
	if (context->stage == COMMISSIONING_STAGE_COMPLETE)
	{
		context->active = false;
		*next_procedure = SERVICE_PROCEDURE_NONE;
		*workflow_complete = true;
		return true;
	}

	*next_procedure = ProcedureForStage(context->stage);
	*workflow_complete = false;
	return *next_procedure != SERVICE_PROCEDURE_NONE;
}

void MotorCommissioningWorkflow_Fail(
	MotorCommissioningWorkflowContext *context, uint8_t reason)
{
	if (context == NULL || !context->active)
		return;
	context->failure_stage = context->stage;
	context->failure_reason = reason;
	context->stage = COMMISSIONING_STAGE_FAILED;
	context->active = false;
}

ServiceProcedure MotorCommissioningWorkflow_GetExpectedProcedure(
	const MotorCommissioningWorkflowContext *context)
{
	return context == NULL || !context->active ? SERVICE_PROCEDURE_NONE :
		ProcedureForStage(context->stage);
}

uint8_t MotorCommissioningWorkflow_GetProgressPercent(
	const MotorCommissioningWorkflowContext *context)
{
	uint8_t enabled_count;
	uint8_t completed_count;

	if (context == NULL)
		return 0U;
	if (context->stage == COMMISSIONING_STAGE_COMPLETE)
		return 100U;
	if (context->stage == COMMISSIONING_STAGE_IDLE)
		return 0U;
	enabled_count = CountEnabledStages(context->enabled_stage_mask);
	if (enabled_count == 0U)
		return 0U;
	completed_count = CountEnabledStages((MotorCommissioningStageMask)
		(context->completed_stage_mask & context->enabled_stage_mask));
	return (uint8_t)((uint16_t)completed_count * 100U / enabled_count);
}
