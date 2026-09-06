#include <stdio.h>
#include <stdlib.h>

typedef int (*HostTestFunction)(void);

typedef struct
{
	const char *name;
	HostTestFunction run;
} HostTestCase;

int ContextIsolation_RunHostTests(void);
int AngleSerialStm32g431Config_RunHostTests(void);
int ControlAuthorityService_RunHostTests(void);
int DeviceLifecycle_RunHostTests(void);
int Encoder_RunHostTests(void);
int FaultManager_RunHostTests(void);
int FrictionIdentification_RunHostTests(void);
int MeasurementModel_RunHostTests(void);
int PhaseCurrentStrategy_RunHostTests(void);
int MechanicalLoadProfile_RunHostTests(void);
int MotorCommissioningWorkflow_RunHostTests(void);
int ParameterManager_RunHostTests(void);
int ParameterService_RunHostTests(void);
int ParameterTransactionService_RunHostTests(void);
int ProductConfig_RunHostTests(void);
int ProductVariant_RunHostTests(void);
int BspBoard_RunHostTests(void);
int ProductConfigBridge_RunHostTests(void);
int ServiceResultValidation_RunHostTests(void);
int TextWriter_RunHostTests(void);
int Tle5012bDriver_RunHostTests(void);
int UpdateService_RunHostTests(void);

static const HostTestCase HostTests[] =
{
	{ "angle_serial_stm32g431_config",
		AngleSerialStm32g431Config_RunHostTests },
	{ "context_isolation", ContextIsolation_RunHostTests },
	{ "control_authority_service", ControlAuthorityService_RunHostTests },
	{ "device_lifecycle", DeviceLifecycle_RunHostTests },
	{ "encoder", Encoder_RunHostTests },
	{ "fault_manager", FaultManager_RunHostTests },
	{ "friction_identification", FrictionIdentification_RunHostTests },
	{ "measurement_model", MeasurementModel_RunHostTests },
	{ "phase_current_strategy", PhaseCurrentStrategy_RunHostTests },
	{ "mechanical_load_profile", MechanicalLoadProfile_RunHostTests },
	{ "motor_commissioning_workflow", MotorCommissioningWorkflow_RunHostTests },
	{ "parameter_manager", ParameterManager_RunHostTests },
	{ "parameter_service", ParameterService_RunHostTests },
	{ "parameter_transaction_service", ParameterTransactionService_RunHostTests },
	{ "product_config_v2", ProductConfig_RunHostTests },
	{ "product_variant", ProductVariant_RunHostTests },
	{ "bsp_board_v2", BspBoard_RunHostTests },
	{ "product_config_bridge", ProductConfigBridge_RunHostTests },
	{ "service_result_validation", ServiceResultValidation_RunHostTests },
	{ "text_writer", TextWriter_RunHostTests },
	{ "tle5012b_driver", Tle5012bDriver_RunHostTests },
	{ "update_service", UpdateService_RunHostTests }
};

int main(void)
{
	size_t index;
	size_t failure_count = 0U;
	const size_t test_count = sizeof(HostTests) / sizeof(HostTests[0]);

	for (index = 0U; index < test_count; ++index)
	{
		const int result = HostTests[index].run();
		if (result == 0)
		{
			(void)printf("PASS %s\n", HostTests[index].name);
		}
		else
		{
			(void)printf("FAIL %s at source line %d\n",
				HostTests[index].name, result);
			++failure_count;
		}
	}

	(void)printf("HOST_TEST_SUMMARY total=%lu passed=%lu failed=%lu\n",
		(unsigned long)test_count,
		(unsigned long)(test_count - failure_count),
		(unsigned long)failure_count);
	return failure_count == 0U ? EXIT_SUCCESS : EXIT_FAILURE;
}
