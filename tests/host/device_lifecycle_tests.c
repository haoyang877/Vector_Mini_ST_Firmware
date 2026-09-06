#include "Core/Application/device_lifecycle.h"

#define TEST_CHECK(condition) do { if (!(condition)) return __LINE__; } while (0)

int DeviceLifecycle_RunHostTests(void)
{
	DeviceLifecycleContext lifecycle;

	DeviceLifecycle_Initialize(&lifecycle);
	TEST_CHECK(lifecycle.device_state == DEVICE_STATE_BOOTING);
	TEST_CHECK(!DeviceLifecycle_RequestMotorControl(&lifecycle,
		MOTOR_CONTROL_MODE_CURRENT));
	TEST_CHECK(DeviceLifecycle_CompleteBoot(&lifecycle));
	TEST_CHECK(lifecycle.device_state == DEVICE_STATE_STANDBY);

	TEST_CHECK(DeviceLifecycle_RequestMotorControl(&lifecycle,
		MOTOR_CONTROL_MODE_CURRENT));
	TEST_CHECK(lifecycle.device_state == DEVICE_STATE_ACTIVE);
	TEST_CHECK(lifecycle.motor_control_mode == MOTOR_CONTROL_MODE_CURRENT);
	TEST_CHECK(DeviceLifecycle_RequestStandby(&lifecycle));

	TEST_CHECK(DeviceLifecycle_RequestService(&lifecycle,
		SERVICE_PROCEDURE_ENCODER_LINEARIZATION));
	TEST_CHECK(lifecycle.device_state == DEVICE_STATE_SERVICING);
	TEST_CHECK(lifecycle.procedure_state == PROCEDURE_STATE_PRECHECK);
	TEST_CHECK(DeviceLifecycle_BeginServiceRun(&lifecycle));
	TEST_CHECK(DeviceLifecycle_BeginServiceVerification(&lifecycle));
	TEST_CHECK(DeviceLifecycle_CompleteService(&lifecycle));
	TEST_CHECK(lifecycle.procedure_state == PROCEDURE_STATE_COMPLETED);
	TEST_CHECK(DeviceLifecycle_RequestStandby(&lifecycle));

	DeviceLifecycle_NotifyFault(&lifecycle);
	TEST_CHECK(lifecycle.device_state == DEVICE_STATE_FAULTED);
	TEST_CHECK(!DeviceLifecycle_RequestStandby(&lifecycle));
	TEST_CHECK(!DeviceLifecycle_ClearFault(&lifecycle, false));
	TEST_CHECK(DeviceLifecycle_ClearFault(&lifecycle, true));
	TEST_CHECK(lifecycle.device_state == DEVICE_STATE_STANDBY);
	TEST_CHECK(DeviceLifecycle_RequestUpdating(&lifecycle));
	TEST_CHECK(lifecycle.device_state == DEVICE_STATE_UPDATING);
	TEST_CHECK(!DeviceLifecycle_RequestMotorControl(&lifecycle,
		MOTOR_CONTROL_MODE_SPEED));
	return 0;
}
