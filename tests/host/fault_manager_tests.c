#include "Core/Services/Safety/fault_manager.h"

#define TEST_CHECK(condition) do { if (!(condition)) return __LINE__; } while (0)

int FaultManager_RunHostTests(void)
{
	FaultManagerContext faults;
	FaultObservation first = { 10U, 24.0f, 1.0f, 2.0f, 3.0f,
		42.0f, 0.5f, 6.0f };
	FaultObservation second = { 20U, 23.0f, 4.0f, 5.0f, 6.0f,
		43.0f, 0.6f, 7.0f };
	FaultRecord record;

	FaultManager_Initialize(&faults);
	TEST_CHECK(FaultManager_Raise(&faults, 3U, &first));
	TEST_CHECK(FaultManager_Raise(&faults, 5U, &second));
	TEST_CHECK(FaultManager_GetPrimaryFault(&faults) == 3U);
	TEST_CHECK(FaultManager_GetActiveFaults(&faults) ==
		(((FaultSet)1U << 3U) | ((FaultSet)1U << 5U)));

	TEST_CHECK(FaultManager_ClearActive(&faults, 3U));
	TEST_CHECK(FaultManager_GetPrimaryFault(&faults) == 5U);
	TEST_CHECK((FaultManager_GetActiveFaults(&faults) &
		((FaultSet)1U << 3U)) == 0U);
	TEST_CHECK((FaultManager_GetLatchedFaults(&faults) &
		((FaultSet)1U << 3U)) != 0U);
	TEST_CHECK(FaultManager_GetRecord(&faults, 3U, &record));
	TEST_CHECK(record.occurrence_count == 1U);
	TEST_CHECK(record.first_time_ms == 10U);
	TEST_CHECK(record.latest_observation.bus_voltage_v == 24.0f);

	FaultManager_ClearAll(&faults);
	TEST_CHECK(FaultManager_GetActiveFaults(&faults) == 0U);
	TEST_CHECK(FaultManager_GetLatchedFaults(&faults) == 0U);
	TEST_CHECK(FaultManager_GetRecord(&faults, 3U, &record));
	TEST_CHECK(record.occurrence_count == 1U);
	return 0;
}
