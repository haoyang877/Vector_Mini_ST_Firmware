#include "Core/Application/motor_commissioning_workflow.h"

#define TEST_CHECK(condition) do { if (!(condition)) return __LINE__; } while (0)

int MotorCommissioningWorkflow_RunHostTests(void)
{
	static const ServiceProcedure expected[] = {
		SERVICE_PROCEDURE_CURRENT_OFFSET_CALIBRATION,
		SERVICE_PROCEDURE_PHASE_RESISTANCE_IDENTIFICATION,
		SERVICE_PROCEDURE_ENCODER_DIRECTION_CALIBRATION,
		SERVICE_PROCEDURE_OBSERVER_CALIBRATION,
		SERVICE_PROCEDURE_ELECTRICAL_ZERO_CALIBRATION,
		SERVICE_PROCEDURE_FRICTION_IDENTIFICATION,
		SERVICE_PROCEDURE_COGGING_IDENTIFICATION,
		SERVICE_PROCEDURE_PARAMETER_SAVE
	};
	MotorCommissioningWorkflowContext workflow;
	ServiceProcedure procedure = SERVICE_PROCEDURE_NONE;
	ServiceProcedure next = SERVICE_PROCEDURE_NONE;
	bool complete = false;
	uint8_t index;

	MotorCommissioningWorkflow_Initialize(&workflow);
	TEST_CHECK(workflow.stage == COMMISSIONING_STAGE_IDLE);
	TEST_CHECK(!workflow.active);
	TEST_CHECK(MotorCommissioningWorkflow_Start(&workflow, &procedure));
	TEST_CHECK(workflow.active);
	TEST_CHECK(procedure == expected[0]);

	for (index = 0U; index < (uint8_t)(sizeof(expected) / sizeof(expected[0]));
		index++)
	{
		TEST_CHECK(MotorCommissioningWorkflow_GetExpectedProcedure(&workflow) ==
			expected[index]);
		TEST_CHECK(MotorCommissioningWorkflow_CompleteProcedure(&workflow,
			expected[index], &next, &complete));
		if (index + 1U < (uint8_t)(sizeof(expected) / sizeof(expected[0])))
		{
			TEST_CHECK(!complete);
			TEST_CHECK(next == expected[index + 1U]);
		}
	}
	TEST_CHECK(complete);
	TEST_CHECK(!workflow.active);
	TEST_CHECK(workflow.stage == COMMISSIONING_STAGE_COMPLETE);
	TEST_CHECK(workflow.completed_stage_mask == 0x01FEU);
	TEST_CHECK(MotorCommissioningWorkflow_GetProgressPercent(&workflow) == 100U);

	TEST_CHECK(MotorCommissioningWorkflow_Start(&workflow, &procedure));
	TEST_CHECK(MotorCommissioningWorkflow_CompleteProcedure(&workflow,
		procedure, &next, &complete));
	MotorCommissioningWorkflow_Fail(&workflow, 7U);
	TEST_CHECK(!workflow.active);
	TEST_CHECK(workflow.stage == COMMISSIONING_STAGE_FAILED);
	TEST_CHECK(workflow.failure_stage ==
		COMMISSIONING_STAGE_PHASE_RESISTANCE_CHECK);
	TEST_CHECK(workflow.failure_reason == 7U);
	TEST_CHECK(workflow.completed_stage_mask == 0x0002U);
	TEST_CHECK(MotorCommissioningWorkflow_GetProgressPercent(&workflow) == 12U);
	return 0;
}
