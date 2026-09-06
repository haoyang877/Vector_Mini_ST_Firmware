#include "Core/Infrastructure/Telemetry/telemetry_service.h"
#include "motor_control_types.h"

#include <string.h>

#define TEST_CHECK(condition) do { if (!(condition)) return __LINE__; } while (0)

int ContextIsolation_RunHostTests(void)
{
	TelemetryServiceContext first = {0};
	TelemetryServiceContext second = {0};
	MotorTelemetrySnapshot first_snapshot = {0};
	MotorTelemetrySnapshot second_snapshot = {0};
	MotorTelemetrySnapshot readback;
	MotorTelemetrySnapshot unchanged;
	float value = 0.0f;
	MotorControlLoopSchedule schedule;

	TEST_CHECK(MotorControlLoopSchedule_Derive(&schedule,
		20000U, 2000U, 1000U, 5000U));
	TEST_CHECK(schedule.current_divider == 1U);
	TEST_CHECK(schedule.speed_divider == 10U);
	TEST_CHECK(schedule.position_divider == 20U);
	TEST_CHECK(schedule.cascade_position_divider == 4U);
	TEST_CHECK(schedule.current_period_s == 0.00005f);
	TEST_CHECK(schedule.speed_period_s == 0.0005f);
	TEST_CHECK(schedule.position_period_s == 0.001f);
	TEST_CHECK(schedule.cascade_position_period_s == 0.0002f);
	TEST_CHECK(MotorControlLoopSchedule_Derive(&schedule,
		20000U, 1U, 1000U, 5000U));
	TEST_CHECK(schedule.speed_divider == 20000U);
	TEST_CHECK(schedule.speed_divider > UINT8_MAX);
	TEST_CHECK(!MotorControlLoopSchedule_Derive(&schedule,
		0U, 2000U, 1000U, 5000U));
	TEST_CHECK(!MotorControlLoopSchedule_Derive(&schedule,
		20000U, 0U, 1000U, 5000U));
	TEST_CHECK(!MotorControlLoopSchedule_Derive(&schedule,
		20000U, 3000U, 1000U, 5000U));
	TEST_CHECK(!MotorControlLoopSchedule_Derive(&schedule,
		65536U, 1U, 1U, 1U));

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

	/* Publishing and reading must copy every byte, not just the fields used by
	 * the scalar accessor.  Non-zero fill also covers future snapshot fields. */
	memset(&first_snapshot, 0xA5, sizeof(first_snapshot));
	first_snapshot.mode = 21U;
	first_snapshot.primary_error = 7U;
	first_snapshot.fault_occurrence_count[
		MOTOR_TELEMETRY_FAULT_CODE_COUNT - 1U] = 0x12345678UL;
	first_snapshot.bus_voltage_v = 31.25f;
	first_snapshot.phase_resistance_design_error_percent = -4.5f;
	TelemetryService_Publish(&first, &first_snapshot);
	memset(&readback, 0, sizeof(readback));
	TEST_CHECK(TelemetryService_ReadSnapshot(&first, &readback));
	TEST_CHECK(memcmp(&readback, &first_snapshot, sizeof(readback)) == 0);
	TEST_CHECK(first.sequence[first.published_buffer] == 2U);

	/* An in-progress (odd sequence) publication remains unreadable after the
	 * bounded three attempts and must not partially overwrite the caller. */
	memset(&unchanged, 0x3C, sizeof(unchanged));
	readback = unchanged;
	first.sequence[first.published_buffer]++;
	TEST_CHECK(!TelemetryService_ReadSnapshot(&first, &readback));
	TEST_CHECK(memcmp(&readback, &unchanged, sizeof(readback)) == 0);
	first.sequence[first.published_buffer]++;
	TEST_CHECK(TelemetryService_ReadSnapshot(&first, &readback));
	TEST_CHECK(memcmp(&readback, &first_snapshot, sizeof(readback)) == 0);

	TEST_CHECK(!TelemetryService_Initialize(0));
	TEST_CHECK(!TelemetryService_ReadSnapshot(0, &readback));
	TEST_CHECK(!TelemetryService_ReadSnapshot(&first, 0));
	TelemetryService_Publish(0, &first_snapshot);
	TelemetryService_Publish(&first, 0);
	return 0;
}
