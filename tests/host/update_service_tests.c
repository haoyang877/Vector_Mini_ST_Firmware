#include "Core/Application/Update/update_service.h"

#define TEST_CHECK(condition) do { if (!(condition)) return __LINE__; } while (0)

typedef struct
{
	bool outputs_enabled;
	bool candidate_valid;
	bool write_succeeds;
	bool reset_succeeds;
	unsigned int disable_count;
	unsigned int clear_count;
} FakeUpdateContext;

static void Fake_DisableOutputs(void *context)
{
	FakeUpdateContext *fake = (FakeUpdateContext *)context;
	fake->outputs_enabled = false;
	fake->disable_count++;
}
static bool Fake_OutputsAreEnabled(void *context)
{
	return ((FakeUpdateContext *)context)->outputs_enabled;
}
static bool Fake_CandidateCompatible(void *context, uint32_t address,
	uint32_t size_bytes)
{
	(void)address; (void)size_bytes;
	return ((FakeUpdateContext *)context)->candidate_valid;
}
static bool Fake_WriteInstallRequest(void *context, uint32_t address,
	uint32_t size_bytes)
{
	(void)address; (void)size_bytes;
	return ((FakeUpdateContext *)context)->write_succeeds;
}
static void Fake_ClearRequest(void *context)
{
	((FakeUpdateContext *)context)->clear_count++;
}
static bool Fake_RequestReset(void *context)
{
	return ((FakeUpdateContext *)context)->reset_succeeds;
}
static ParameterTransactionOperation Fake_GetParameterOperation(void *context)
{
	(void)context;
	return PARAMETER_TRANSACTION_NONE;
}

int UpdateService_RunHostTests(void)
{
	FakeUpdateContext fake = { false, true, false, true, 0U, 0U };
	MotorOutputSafetyPort motor_output_safety;
	UpdateControlPort update_port;
	UpdateServiceContext service;
	ParameterTransactionServiceContext parameter_transactions;
	DeviceLifecycleContext lifecycle;

	motor_output_safety.context = &fake;
	motor_output_safety.disable_immediate = Fake_DisableOutputs;
	motor_output_safety.outputs_are_enabled = Fake_OutputsAreEnabled;
	DeviceLifecycle_Initialize(&lifecycle);
	TEST_CHECK(DeviceLifecycle_CompleteBoot(&lifecycle));
	update_port.context = &fake;
	update_port.candidate_is_compatible = Fake_CandidateCompatible;
	update_port.write_install_request = Fake_WriteInstallRequest;
	update_port.clear_request = Fake_ClearRequest;
	update_port.request_system_reset = Fake_RequestReset;
	parameter_transactions.port.context = &fake;
	parameter_transactions.port.get_operation = Fake_GetParameterOperation;
	parameter_transactions.is_initialized = true;
	parameter_transactions.operation_has_run = false;
	TEST_CHECK(UpdateService_Initialize(&service, &lifecycle,
		&motor_output_safety,
		&parameter_transactions, &update_port));

	TEST_CHECK(UpdateService_PrepareInstall(&service, 0x08020000U, 4096U) ==
		UPDATE_SERVICE_INTERNAL_ERROR);
	TEST_CHECK(lifecycle.device_state == DEVICE_STATE_STANDBY);
	fake.write_succeeds = true;
	TEST_CHECK(UpdateService_PrepareInstall(&service, 0x08020000U, 4096U) ==
		UPDATE_SERVICE_ACCEPTED);
	TEST_CHECK(lifecycle.device_state == DEVICE_STATE_UPDATING);
	TEST_CHECK(!fake.outputs_enabled);
	TEST_CHECK(UpdateService_CommitReset(&service) == UPDATE_SERVICE_ACCEPTED);
	TEST_CHECK(UpdateService_Cancel(&service));
	TEST_CHECK(fake.clear_count == 1U);
	TEST_CHECK(lifecycle.device_state == DEVICE_STATE_STANDBY);
	return 0;
}
