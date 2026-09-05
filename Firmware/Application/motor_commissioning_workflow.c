#include "motor_commissioning_workflow.h"

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

void MotorCommissioningWorkflow_Initialize(
	MotorCommissioningWorkflowContext *context)
{
	if (context == NULL)
		return;
	context->stage = COMMISSIONING_STAGE_IDLE;
	context->failure_stage = COMMISSIONING_STAGE_IDLE;
	context->completed_stage_mask = 0U;
	context->failure_reason = 0U;
	context->active = false;
}

bool MotorCommissioningWorkflow_Start(
	MotorCommissioningWorkflowContext *context,
	ServiceProcedure *first_procedure)
{
	if (context == NULL || first_procedure == NULL || context->active)
		return false;
	MotorCommissioningWorkflow_Initialize(context);
	context->active = true;
	context->stage = COMMISSIONING_STAGE_CURRENT_OFFSET;
	*first_procedure = ProcedureForStage(context->stage);
	return true;
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
	if (context->stage == COMMISSIONING_STAGE_SAVE)
	{
		context->stage = COMMISSIONING_STAGE_COMPLETE;
		context->active = false;
		*next_procedure = SERVICE_PROCEDURE_NONE;
		*workflow_complete = true;
		return true;
	}

	context->stage = (MotorCommissioningStage)((uint8_t)context->stage + 1U);
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
	if (context == NULL)
		return 0U;
	if (context->stage == COMMISSIONING_STAGE_COMPLETE)
		return 100U;
	if (context->stage == COMMISSIONING_STAGE_FAILED ||
		context->stage == COMMISSIONING_STAGE_IDLE)
	{
		if (context->stage == COMMISSIONING_STAGE_FAILED &&
			context->failure_stage >= COMMISSIONING_STAGE_CURRENT_OFFSET &&
			context->failure_stage <= COMMISSIONING_STAGE_SAVE)
			return (uint8_t)(((uint8_t)context->failure_stage - 1U) * 100U / 8U);
		return 0U;
	}
	return (uint8_t)(((uint8_t)context->stage - 1U) * 100U / 8U);
}
