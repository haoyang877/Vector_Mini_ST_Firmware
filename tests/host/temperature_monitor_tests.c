#include "temperature_monitor.h"

#include <stdint.h>

#define TEST_CHECK(condition) do { if (!(condition)) return __LINE__; } while (0)

static float TemperatureMonitorTest_Nan(void)
{
	union
	{
		uint32_t bits;
		float value;
	} value;

	value.bits = UINT32_C(0x7FC00000);
	return value.value;
}

static TemperatureMonitorConfig TemperatureMonitorTest_Config(
	bool protection_enabled)
{
	TemperatureMonitorConfig config = {0};

	config.protection_enabled = protection_enabled;
	config.trip_temperature_c = 100.0f;
	config.trip_on_invalid_sample = true;
	config.trip_on_stale_sample = true;
	config.trip_on_sensor_open = true;
	config.trip_on_sensor_short = true;
	config.trip_on_sensor_fault = true;
	return config;
}

static int TemperatureMonitorTest_MonitorOnlyNeverTrips(void)
{
	const TemperatureMonitorSampleStatusSet diagnostic_states[] =
	{
		0U,
		TEMPERATURE_MONITOR_SAMPLE_VALID |
			TEMPERATURE_MONITOR_SAMPLE_STALE,
		TEMPERATURE_MONITOR_SENSOR_OPEN,
		TEMPERATURE_MONITOR_SENSOR_SHORT,
		TEMPERATURE_MONITOR_SENSOR_FAULT
	};
	TemperatureMonitorContext context;
	TemperatureMonitorConfig config = TemperatureMonitorTest_Config(false);
	TemperatureMonitorInput input = {0};
	TemperatureMonitorOutput output;
	uint8_t index;

	TEST_CHECK(TemperatureMonitor_Configure(&context, &config) ==
		TEMPERATURE_MONITOR_STATUS_OK);
	input.temperature_c = 150.0f;
	input.sequence = 1U;
	input.status = TEMPERATURE_MONITOR_SAMPLE_VALID;
	TEST_CHECK(TemperatureMonitor_Process(&context, &input, &output) ==
		TEMPERATURE_MONITOR_STATUS_OK);
	TEST_CHECK(!output.trip_requested);
	TEST_CHECK(output.trip_reasons == TEMPERATURE_MONITOR_TRIP_NONE);

	for (index = 0U;
		index < (uint8_t)(sizeof(diagnostic_states) /
			sizeof(diagnostic_states[0])); index++)
	{
		input.sequence++;
		input.status = diagnostic_states[index];
		TEST_CHECK(TemperatureMonitor_Process(&context, &input, &output) ==
			TEMPERATURE_MONITOR_STATUS_OK);
		TEST_CHECK(!output.trip_requested);
		TEST_CHECK(output.trip_reasons == TEMPERATURE_MONITOR_TRIP_NONE);
	}

	/* Sequence errors are stale diagnostics but remain non-tripping. */
	TEST_CHECK(TemperatureMonitor_Process(&context, &input, &output) ==
		TEMPERATURE_MONITOR_STATUS_DUPLICATE_SEQUENCE);
	TEST_CHECK((output.observed_status & TEMPERATURE_MONITOR_SAMPLE_STALE) != 0U);
	TEST_CHECK(!output.trip_requested);
	input.sequence--;
	TEST_CHECK(TemperatureMonitor_Process(&context, &input, &output) ==
		TEMPERATURE_MONITOR_STATUS_OUT_OF_ORDER_SEQUENCE);
	TEST_CHECK(!output.trip_requested);
	return 0;
}

static int TemperatureMonitorTest_ProtectionPolicy(void)
{
	TemperatureMonitorContext context;
	TemperatureMonitorConfig config = TemperatureMonitorTest_Config(true);
	TemperatureMonitorInput input = {0};
	TemperatureMonitorOutput output;

	TEST_CHECK(TemperatureMonitor_Configure(&context, &config) ==
		TEMPERATURE_MONITOR_STATUS_OK);
	input.sequence = 1U;
	input.temperature_c = 99.0f;
	input.status = TEMPERATURE_MONITOR_SAMPLE_VALID;
	TEST_CHECK(TemperatureMonitor_Process(&context, &input, &output) ==
		TEMPERATURE_MONITOR_STATUS_OK);
	TEST_CHECK(!output.trip_requested);
	TEST_CHECK(output.has_valid_temperature);

	input.sequence = 2U;
	input.temperature_c = 100.0f;
	TEST_CHECK(TemperatureMonitor_Process(&context, &input, &output) ==
		TEMPERATURE_MONITOR_STATUS_OK);
	TEST_CHECK(output.trip_requested);
	TEST_CHECK(output.trip_reasons ==
		TEMPERATURE_MONITOR_TRIP_OVER_TEMPERATURE);

	input.sequence = 3U;
	input.status = 0U;
	TEST_CHECK(TemperatureMonitor_Process(&context, &input, &output) ==
		TEMPERATURE_MONITOR_STATUS_OK);
	TEST_CHECK(output.trip_reasons ==
		TEMPERATURE_MONITOR_TRIP_INVALID_SAMPLE);

	input.sequence = 4U;
	input.status = TEMPERATURE_MONITOR_SAMPLE_STALE |
		TEMPERATURE_MONITOR_SENSOR_OPEN |
		TEMPERATURE_MONITOR_SENSOR_SHORT |
		TEMPERATURE_MONITOR_SENSOR_FAULT;
	TEST_CHECK(TemperatureMonitor_Process(&context, &input, &output) ==
		TEMPERATURE_MONITOR_STATUS_OK);
	TEST_CHECK((output.trip_reasons &
		TEMPERATURE_MONITOR_TRIP_INVALID_SAMPLE) != 0U);
	TEST_CHECK((output.trip_reasons &
		TEMPERATURE_MONITOR_TRIP_STALE_SAMPLE) != 0U);
	TEST_CHECK((output.trip_reasons &
		TEMPERATURE_MONITOR_TRIP_SENSOR_OPEN) != 0U);
	TEST_CHECK((output.trip_reasons &
		TEMPERATURE_MONITOR_TRIP_SENSOR_SHORT) != 0U);
	TEST_CHECK((output.trip_reasons &
		TEMPERATURE_MONITOR_TRIP_SENSOR_FAULT) != 0U);

	/* Duplicate/out-of-order samples use only stale/invalid policy, never data. */
	input.temperature_c = 200.0f;
	TEST_CHECK(TemperatureMonitor_Process(&context, &input, &output) ==
		TEMPERATURE_MONITOR_STATUS_DUPLICATE_SEQUENCE);
	TEST_CHECK((output.trip_reasons &
		TEMPERATURE_MONITOR_TRIP_STALE_SAMPLE) != 0U);
	TEST_CHECK((output.trip_reasons &
		TEMPERATURE_MONITOR_TRIP_OVER_TEMPERATURE) == 0U);
	return 0;
}

static int TemperatureMonitorTest_IndividualPolicies(void)
{
	TemperatureMonitorContext context;
	TemperatureMonitorConfig config = TemperatureMonitorTest_Config(true);
	TemperatureMonitorInput input = {0};
	TemperatureMonitorOutput output;

	config.trip_on_invalid_sample = false;
	config.trip_on_stale_sample = false;
	config.trip_on_sensor_open = false;
	config.trip_on_sensor_short = false;
	config.trip_on_sensor_fault = false;
	TEST_CHECK(TemperatureMonitor_Configure(&context, &config) ==
		TEMPERATURE_MONITOR_STATUS_OK);
	input.temperature_c = 50.0f;
	input.sequence = 10U;
	input.status = TEMPERATURE_MONITOR_SAMPLE_STALE |
		TEMPERATURE_MONITOR_SENSOR_OPEN |
		TEMPERATURE_MONITOR_SENSOR_SHORT |
		TEMPERATURE_MONITOR_SENSOR_FAULT;
	TEST_CHECK(TemperatureMonitor_Process(&context, &input, &output) ==
		TEMPERATURE_MONITOR_STATUS_OK);
	TEST_CHECK(!output.trip_requested);

	config.trip_on_sensor_open = true;
	TEST_CHECK(TemperatureMonitor_Configure(&context, &config) ==
		TEMPERATURE_MONITOR_STATUS_OK);
	input.status = TEMPERATURE_MONITOR_SENSOR_OPEN;
	TEST_CHECK(TemperatureMonitor_Process(&context, &input, &output) ==
		TEMPERATURE_MONITOR_STATUS_OK);
	TEST_CHECK(output.trip_reasons == TEMPERATURE_MONITOR_TRIP_SENSOR_OPEN);
	return 0;
}

static int TemperatureMonitorTest_SequenceAndMalformedSamples(void)
{
	TemperatureMonitorContext context;
	TemperatureMonitorConfig config = TemperatureMonitorTest_Config(true);
	TemperatureMonitorInput input = {0};
	TemperatureMonitorOutput output;

	TEST_CHECK(TemperatureMonitor_Configure(&context, &config) ==
		TEMPERATURE_MONITOR_STATUS_OK);
	input.temperature_c = 25.0f;
	input.status = TEMPERATURE_MONITOR_SAMPLE_VALID;
	input.sequence = UINT32_MAX;
	TEST_CHECK(TemperatureMonitor_Process(&context, &input, &output) ==
		TEMPERATURE_MONITOR_STATUS_OK);
	input.sequence = 0U;
	TEST_CHECK(TemperatureMonitor_Process(&context, &input, &output) ==
		TEMPERATURE_MONITOR_STATUS_OK);
	TEST_CHECK(output.latest_valid_sequence == 0U);

	input.sequence = 1U;
	input.temperature_c = TemperatureMonitorTest_Nan();
	TEST_CHECK(TemperatureMonitor_Process(&context, &input, &output) ==
		TEMPERATURE_MONITOR_STATUS_NONFINITE_TEMPERATURE);
	TEST_CHECK((output.trip_reasons &
		TEMPERATURE_MONITOR_TRIP_INVALID_SAMPLE) != 0U);
	TEST_CHECK(output.latest_valid_sequence == 0U);

	input.sequence = 2U;
	input.temperature_c = 25.0f;
	input.status = UINT32_C(0x80000000);
	TEST_CHECK(TemperatureMonitor_Process(&context, &input, &output) ==
		TEMPERATURE_MONITOR_STATUS_INVALID_SAMPLE_STATUS);
	TEST_CHECK(output.latest_valid_sequence == 0U);
	return 0;
}

static int TemperatureMonitorTest_Configuration(void)
{
	TemperatureMonitorContext context;
	TemperatureMonitorConfig config = TemperatureMonitorTest_Config(true);
	TemperatureMonitorInput input = {0};
	TemperatureMonitorOutput output;

	TemperatureMonitor_Reset(&context);
	TEST_CHECK(TemperatureMonitor_Process(&context, &input, &output) ==
		TEMPERATURE_MONITOR_STATUS_NOT_CONFIGURED);
	config.trip_temperature_c = TemperatureMonitorTest_Nan();
	TEST_CHECK(TemperatureMonitor_Configure(&context, &config) ==
		TEMPERATURE_MONITOR_STATUS_INVALID_CONFIGURATION);
	TEST_CHECK(!context.is_configured);

	config.protection_enabled = false;
	TEST_CHECK(TemperatureMonitor_Configure(&context, &config) ==
		TEMPERATURE_MONITOR_STATUS_OK);
	return 0;
}

int TemperatureMonitor_RunHostTests(void)
{
	int result;

	result = TemperatureMonitorTest_MonitorOnlyNeverTrips();
	if (result != 0)
		return result;
	result = TemperatureMonitorTest_ProtectionPolicy();
	if (result != 0)
		return result;
	result = TemperatureMonitorTest_IndividualPolicies();
	if (result != 0)
		return result;
	result = TemperatureMonitorTest_SequenceAndMalformedSamples();
	if (result != 0)
		return result;
	return TemperatureMonitorTest_Configuration();
}

#ifdef TEMPERATURE_MONITOR_TEST_STANDALONE
int main(void)
{
	return TemperatureMonitor_RunHostTests();
}
#endif
