#include "Core/Infrastructure/Telemetry/telemetry_service.h"

#define TEST_CHECK(condition) do { if (!(condition)) return __LINE__; } while (0)

int ContextIsolation_RunHostTests(void)
{
	TelemetryServiceContext first = {0};
	TelemetryServiceContext second = {0};
	MotorTelemetrySnapshot first_snapshot = {0};
	MotorTelemetrySnapshot second_snapshot = {0};
	float value = 0.0f;

	TEST_CHECK(TelemetryService_Initialize(&first));
	TEST_CHECK(TelemetryService_Initialize(&second));
	first_snapshot.mode = 5U;
	first_snapshot.bus_voltage_v = 24.0f;
	second_snapshot.mode = 13U;
	second_snapshot.bus_voltage_v = 28.0f;
	TelemetryService_Publish(&first, &first_snapshot);
	TelemetryService_Publish(&second, &second_snapshot);

	TEST_CHECK(TelemetryService_ReadValue(&first, MOTOR_TELEMETRY_MODE,
		&value));
	TEST_CHECK(value == 5.0f);
	TEST_CHECK(TelemetryService_ReadValue(&second, MOTOR_TELEMETRY_MODE,
		&value));
	TEST_CHECK(value == 13.0f);
	TEST_CHECK(TelemetryService_ReadValue(&first,
		MOTOR_TELEMETRY_BUS_VOLTAGE_V, &value));
	TEST_CHECK(value == 24.0f);
	TEST_CHECK(TelemetryService_ReadValue(&second,
		MOTOR_TELEMETRY_BUS_VOLTAGE_V, &value));
	TEST_CHECK(value == 28.0f);
	return 0;
}
