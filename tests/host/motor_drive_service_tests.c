#include "Core/Application/MotorControl/motor_drive_service.h"

#include <string.h>

#define TEST_CHECK(condition) do { if (!(condition)) return __LINE__; } while (0)

typedef struct
{
	BspResult initialize_result;
	BspResult arm_result;
	BspResult commit_result;
	BspResult read_result;
	BspMotorDriveFaultSet faults;
	BspMotorDriveSample sample;
	BspMotorDriveCycleCommand last_command;
	unsigned int disable_count;
	unsigned int disarm_count;
	unsigned int arm_count;
	unsigned int commit_count;
} FakeMotorDrive;

static BspResult Fake_Initialize(void *context,
	const BspMotorDriveConfiguration *configuration)
{
	(void)configuration;
	return ((FakeMotorDrive *)context)->initialize_result;
}

static BspResult Fake_Arm(void *context)
{
	FakeMotorDrive *fake = (FakeMotorDrive *)context;
	fake->arm_count++;
	return fake->arm_result;
}

static BspResult Fake_Disarm(void *context)
{
	((FakeMotorDrive *)context)->disarm_count++;
	return BSP_RESULT_OK;
}

static BspResult Fake_Read(void *context, BspMotorDriveSample *sample)
{
	FakeMotorDrive *fake = (FakeMotorDrive *)context;
	*sample = fake->sample;
	if ((fake->sample.status & BSP_MOTOR_DRIVE_SAMPLE_ADC_OVERRUN) != 0U)
		fake->faults |= BSP_MOTOR_DRIVE_FAULT_SAMPLING;
	return fake->read_result;
}

static BspResult Fake_Commit(void *context,
	const BspMotorDriveCycleCommand *command)
{
	FakeMotorDrive *fake = (FakeMotorDrive *)context;
	fake->last_command = *command;
	fake->commit_count++;
	return fake->commit_result;
}

static void Fake_Disable(void *context)
{
	((FakeMotorDrive *)context)->disable_count++;
}

static BspMotorDriveFaultSet Fake_ReadFaults(void *context)
{
	return ((FakeMotorDrive *)context)->faults;
}

static BspMotorDrivePort Fake_CreatePort(FakeMotorDrive *fake,
	const BspMotorDriveEndpointCapabilities *capabilities)
{
	BspMotorDrivePort port;

	(void)memset(&port, 0, sizeof(port));
	port.context = fake;
	port.capabilities = capabilities;
	port.initialize_safe = Fake_Initialize;
	port.arm = Fake_Arm;
	port.disarm = Fake_Disarm;
	port.read_sample = Fake_Read;
	port.commit_cycle = Fake_Commit;
	port.disable_immediate = Fake_Disable;
	port.read_faults = Fake_ReadFaults;
	return port;
}

int MotorDriveService_RunHostTests(void)
{
	BspMotorDriveEndpointCapabilities capabilities = {0};
	BspMotorDriveConfiguration configuration = {0};
	BspMotorDrivePort port;
	BspMotorDriveCycleCommand command = {0};
	MotorDriveServiceAcquisition acquisition;
	MotorDriveServiceContext service;
	MotorOutputSafetyPort safety;
	FakeMotorDrive fake = {0};

	fake.initialize_result = BSP_RESULT_OK;
	fake.arm_result = BSP_RESULT_OK;
	fake.commit_result = BSP_RESULT_OK;
	fake.read_result = BSP_RESULT_OK;
	fake.sample.status = BSP_MOTOR_DRIVE_SAMPLE_VALID;
	fake.sample.current_sample_count = 3U;
	fake.sample.valid_phase_currents = BSP_MOTOR_PHASE_ALL;
	capabilities.endpoint_id = 1U;
	capabilities.availability = BSP_ENDPOINT_AVAILABLE;
	capabilities.supported_current_sense_topologies =
		BSP_CURRENT_SENSE_TOPOLOGY_BIT(
			BSP_CURRENT_SENSE_LOW_SIDE_THREE_SHUNT);
	capabilities.current_sensor_capacity = 3U;
	capabilities.supports_synchronized_sampling = true;
	capabilities.supported_sampling_modes =
		BSP_CURRENT_SAMPLING_MODE_BIT(BSP_CURRENT_SAMPLING_MODE_FIXED);
	port = Fake_CreatePort(&fake, &capabilities);
	configuration.current_sense_topology =
		BSP_CURRENT_SENSE_LOW_SIDE_THREE_SHUNT;
	configuration.pwm_frequency_hz = 20000U;
	configuration.sampling_mode = BSP_CURRENT_SAMPLING_MODE_FIXED;
	configuration.fixed_sample_count = 3U;
	configuration.fixed_direct_phase_currents = BSP_MOTOR_PHASE_ALL;

	TEST_CHECK(MotorDriveService_Initialize(&service, &port, &configuration));
	TEST_CHECK(fake.disable_count == 1U);
	TEST_CHECK(fake.commit_count == 1U);
	TEST_CHECK(fake.last_command.pwm.phase_duty[0] == 1.0f);
	TEST_CHECK(fake.last_command.sampling.mode ==
		BSP_CURRENT_SAMPLING_MODE_FIXED);
	TEST_CHECK(fake.last_command.sampling.cycle_valid_phase_currents ==
		BSP_MOTOR_PHASE_ALL);
	TEST_CHECK(fake.last_command.sampling.sequence == 0U);
	TEST_CHECK(!MotorDriveService_RequestEnable(&service, true));
	TEST_CHECK(fake.arm_count == 0U);
	TEST_CHECK(MotorDriveService_ReadSample(&service, &acquisition));
	TEST_CHECK(acquisition.sampling.sequence == 0U);
	TEST_CHECK(MotorDriveService_RequestEnable(&service, true));
	TEST_CHECK(fake.arm_count == 1U);
	TEST_CHECK(MotorDriveService_AreOutputsEnabled(&service));

	command.pwm.phase_duty[0] = 0.25f;
	command.pwm.phase_duty[1] = 0.50f;
	command.pwm.phase_duty[2] = 0.75f;
	command.sampling.mode = BSP_CURRENT_SAMPLING_MODE_FIXED;
	command.sampling.cycle_valid_phase_currents = BSP_MOTOR_PHASE_ALL;
	command.sampling.sequence = 999U;
	TEST_CHECK(MotorDriveService_CommitCycle(&service, &command));
	TEST_CHECK(fake.last_command.sampling.sequence == 1U);
	fake.sample.sequence = 1U;
	TEST_CHECK(MotorDriveService_ReadSample(&service, &acquisition));
	TEST_CHECK(acquisition.sampling.sequence == 1U);

	TEST_CHECK(MotorDriveService_CommitCycle(&service, &command));
	fake.sample.sequence = 2U;
	fake.sample.status = BSP_MOTOR_DRIVE_SAMPLE_VALID |
		BSP_MOTOR_DRIVE_SAMPLE_ADC_OVERRUN;
	TEST_CHECK(!MotorDriveService_ReadSample(&service, &acquisition));
	TEST_CHECK(MotorDriveService_HasLatchedFault(&service));
	TEST_CHECK(!MotorDriveService_AreOutputsEnabled(&service));
	TEST_CHECK(fake.disable_count == 2U);
	TEST_CHECK(!MotorDriveService_ClearLatchedFault(&service));

	fake.faults = 0U;
	TEST_CHECK(MotorDriveService_ClearLatchedFault(&service));
	fake.sample.status = BSP_MOTOR_DRIVE_SAMPLE_VALID;
	TEST_CHECK(MotorDriveService_ApplyFixedDuty(&service, 0.5f, 0.5f, 0.5f));
	TEST_CHECK(MotorDriveService_RequestEnable(&service, true));
	safety = MotorDriveService_CreateOutputSafetyPort(&service);
	safety.disable_immediate(safety.context);
	TEST_CHECK(!safety.outputs_are_enabled(safety.context));
	fake.sample.sequence = 3U;
	TEST_CHECK(MotorDriveService_ReadSample(&service, &acquisition));

	fake.commit_result = BSP_RESULT_IO_ERROR;
	TEST_CHECK(!MotorDriveService_ApplyFixedDuty(&service, 0.5f, 0.5f, 0.5f));
	TEST_CHECK(MotorDriveService_HasLatchedFault(&service));
	return 0;
}

#if defined(MOTOR_DRIVE_SERVICE_TEST_MAIN)
int main(void)
{
	return MotorDriveService_RunHostTests();
}
#endif
