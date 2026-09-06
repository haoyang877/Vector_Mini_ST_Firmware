#include "Core/Application/motor_commissioning_workflow.h"

#define TEST_CHECK(condition) do { if (!(condition)) return __LINE__; } while (0)

int MotorCommissioningWorkflow_RunHostTests(void)
{
	static const struct
	{
		ServiceProcedure procedure;
		MotorCommissioningStage stage;
	} standalone_gate_cases[] = {
		{SERVICE_PROCEDURE_CURRENT_OFFSET_CALIBRATION,
			COMMISSIONING_STAGE_CURRENT_OFFSET},
		{SERVICE_PROCEDURE_PHASE_RESISTANCE_IDENTIFICATION,
			COMMISSIONING_STAGE_PHASE_RESISTANCE_CHECK},
		{SERVICE_PROCEDURE_ENCODER_DIRECTION_CALIBRATION,
			COMMISSIONING_STAGE_ENCODER_DIRECTION},
		{SERVICE_PROCEDURE_ENCODER_LINEARIZATION,
			COMMISSIONING_STAGE_ENCODER_LUT},
		{SERVICE_PROCEDURE_OBSERVER_CALIBRATION,
			COMMISSIONING_STAGE_ENCODER_LUT},
		{SERVICE_PROCEDURE_ELECTRICAL_ZERO_CALIBRATION,
			COMMISSIONING_STAGE_ELECTRICAL_AND_MECHANICAL_ZERO},
		{SERVICE_PROCEDURE_SET_MECHANICAL_ZERO,
			COMMISSIONING_STAGE_ELECTRICAL_AND_MECHANICAL_ZERO},
		{SERVICE_PROCEDURE_FRICTION_IDENTIFICATION,
			COMMISSIONING_STAGE_FRICTION},
		{SERVICE_PROCEDURE_COGGING_IDENTIFICATION,
			COMMISSIONING_STAGE_COGGING}
	};
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
	TEST_CHECK(workflow.enabled_stage_mask ==
		MOTOR_COMMISSIONING_SUPPORTED_STAGE_MASK);
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

	MotorCommissioningWorkflow_Initialize(&workflow);
	TEST_CHECK(MotorCommissioningWorkflow_Configure(&workflow,
		MOTOR_COMMISSIONING_STAGE_MASK(COMMISSIONING_STAGE_CURRENT_OFFSET) |
		MOTOR_COMMISSIONING_STAGE_MASK(COMMISSIONING_STAGE_PHASE_RESISTANCE_CHECK) |
		MOTOR_COMMISSIONING_STAGE_MASK(COMMISSIONING_STAGE_SAVE)));
	TEST_CHECK(MotorCommissioningWorkflow_Start(&workflow, &procedure));
	TEST_CHECK(procedure == SERVICE_PROCEDURE_CURRENT_OFFSET_CALIBRATION);
	TEST_CHECK(MotorCommissioningWorkflow_CompleteProcedure(&workflow,
		procedure, &next, &complete));
	TEST_CHECK(next == SERVICE_PROCEDURE_PHASE_RESISTANCE_IDENTIFICATION);
	TEST_CHECK(MotorCommissioningWorkflow_GetProgressPercent(&workflow) == 33U);
	TEST_CHECK(MotorCommissioningWorkflow_CompleteProcedure(&workflow,
		next, &procedure, &complete));
	TEST_CHECK(procedure == SERVICE_PROCEDURE_PARAMETER_SAVE);
	TEST_CHECK(MotorCommissioningWorkflow_GetProgressPercent(&workflow) == 66U);
	TEST_CHECK(MotorCommissioningWorkflow_CompleteProcedure(&workflow,
		procedure, &next, &complete));
	TEST_CHECK(complete);
	TEST_CHECK(workflow.completed_stage_mask ==
		(MOTOR_COMMISSIONING_STAGE_MASK(COMMISSIONING_STAGE_CURRENT_OFFSET) |
		 MOTOR_COMMISSIONING_STAGE_MASK(COMMISSIONING_STAGE_PHASE_RESISTANCE_CHECK) |
		 MOTOR_COMMISSIONING_STAGE_MASK(COMMISSIONING_STAGE_SAVE)));

	MotorCommissioningWorkflow_Initialize(&workflow);
	TEST_CHECK(MotorCommissioningWorkflow_Configure(&workflow, 0U));
	TEST_CHECK(workflow.enabled_stage_mask == 0U);
	for (index = 0U; index < (uint8_t)(sizeof(standalone_gate_cases) /
		sizeof(standalone_gate_cases[0])); index++)
	{
		TEST_CHECK(!MotorCommissioningWorkflow_IsServiceEnabled(&workflow,
			standalone_gate_cases[index].procedure));
		TEST_CHECK(MotorCommissioningWorkflow_Configure(&workflow,
			MOTOR_COMMISSIONING_STAGE_MASK(
				standalone_gate_cases[index].stage)));
		TEST_CHECK(MotorCommissioningWorkflow_IsServiceEnabled(&workflow,
			standalone_gate_cases[index].procedure));
		TEST_CHECK(MotorCommissioningWorkflow_Configure(&workflow, 0U));
	}
	TEST_CHECK(MotorCommissioningWorkflow_IsServiceEnabled(&workflow,
		SERVICE_PROCEDURE_PARAMETER_SAVE));
	TEST_CHECK(MotorCommissioningWorkflow_IsServiceEnabled(&workflow,
		SERVICE_PROCEDURE_RESTORE_DEFAULTS));
	TEST_CHECK(MotorCommissioningWorkflow_IsServiceEnabled(&workflow,
		SERVICE_PROCEDURE_FULL_COMMISSIONING));
	TEST_CHECK(!MotorCommissioningWorkflow_IsServiceEnabled(&workflow,
		SERVICE_PROCEDURE_NONE));
	procedure = SERVICE_PROCEDURE_NONE;
	TEST_CHECK(!MotorCommissioningWorkflow_Start(&workflow, &procedure));
	TEST_CHECK(!workflow.active);
	TEST_CHECK(workflow.stage == COMMISSIONING_STAGE_IDLE);
	TEST_CHECK(procedure == SERVICE_PROCEDURE_NONE);
	TEST_CHECK(!MotorCommissioningWorkflow_Configure(&workflow,
		(MotorCommissioningStageMask)0x8000U));
	return 0;
}
