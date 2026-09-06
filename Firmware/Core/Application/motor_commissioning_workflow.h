#ifndef CORE_APPLICATION_MOTOR_COMMISSIONING_WORKFLOW_H
#define CORE_APPLICATION_MOTOR_COMMISSIONING_WORKFLOW_H

#include <stdbool.h>
#include <stdint.h>

#include "Core/Application/device_lifecycle.h"

typedef enum
{
	COMMISSIONING_STAGE_IDLE = 0,
	COMMISSIONING_STAGE_CURRENT_OFFSET,
	COMMISSIONING_STAGE_PHASE_RESISTANCE_CHECK,
	COMMISSIONING_STAGE_ENCODER_DIRECTION,
	COMMISSIONING_STAGE_ENCODER_LUT,
	COMMISSIONING_STAGE_ELECTRICAL_AND_MECHANICAL_ZERO,
	COMMISSIONING_STAGE_FRICTION,
	COMMISSIONING_STAGE_COGGING,
	COMMISSIONING_STAGE_SAVE,
	COMMISSIONING_STAGE_COMPLETE,
	COMMISSIONING_STAGE_FAILED
} MotorCommissioningStage;

typedef uint16_t MotorCommissioningStageMask;

#define MOTOR_COMMISSIONING_STAGE_MASK(stage) \
	((MotorCommissioningStageMask)1U << (uint8_t)(stage))
#define MOTOR_COMMISSIONING_SUPPORTED_STAGE_MASK \
	((MotorCommissioningStageMask)0x01FEU)

typedef struct
{
	MotorCommissioningStage stage;
	MotorCommissioningStage failure_stage;
	MotorCommissioningStageMask enabled_stage_mask;
	uint16_t completed_stage_mask;
	uint8_t failure_reason;
	bool active;
} MotorCommissioningWorkflowContext;

void MotorCommissioningWorkflow_Initialize(
	MotorCommissioningWorkflowContext *context);
bool MotorCommissioningWorkflow_Configure(
	MotorCommissioningWorkflowContext *context,
	MotorCommissioningStageMask enabled_stage_mask);
/* Pure policy gate for standalone service requests.  Maintenance commands and
 * FULL_COMMISSIONING remain admissible here; FULL is finally admitted by
 * MotorCommissioningWorkflow_Start so an empty mask cannot start Mode 21. */
bool MotorCommissioningWorkflow_IsServiceEnabled(
	const MotorCommissioningWorkflowContext *context,
	ServiceProcedure procedure);
bool MotorCommissioningWorkflow_Start(
	MotorCommissioningWorkflowContext *context,
	ServiceProcedure *first_procedure);
bool MotorCommissioningWorkflow_CompleteProcedure(
	MotorCommissioningWorkflowContext *context,
	ServiceProcedure completed_procedure,
	ServiceProcedure *next_procedure, bool *workflow_complete);
void MotorCommissioningWorkflow_Fail(
	MotorCommissioningWorkflowContext *context, uint8_t reason);
ServiceProcedure MotorCommissioningWorkflow_GetExpectedProcedure(
	const MotorCommissioningWorkflowContext *context);
uint8_t MotorCommissioningWorkflow_GetProgressPercent(
	const MotorCommissioningWorkflowContext *context);

#endif
