#include "Core/Application/Supervision/temperature_supervision.h"

#include <string.h>

#define TEST_CHECK(condition) do { if (!(condition)) return __LINE__; } while (0)

typedef struct
{
	BspResult initialize_result;
	BspResult request_result;
	BspResult read_result;
	BspTemperatureStatusSet reported_status;
	BspTemperatureSample sample;
	uint32_t initialize_count;
	uint32_t request_count;
	uint32_t read_count;
} FakeTemperature;

static BspResult FakeTemperature_Initialize(void *context)
{
	FakeTemperature *fake = (FakeTemperature *)context;

	fake->initialize_count++;
	return fake->initialize_result;
}

static BspResult FakeTemperature_Request(void *context)
{
	FakeTemperature *fake = (FakeTemperature *)context;

	fake->request_count++;
	return fake->request_result;
}

static BspResult FakeTemperature_Read(void *context,
	BspTemperatureSample *sample)
{
	FakeTemperature *fake = (FakeTemperature *)context;

	fake->read_count++;
	if (fake->read_result == BSP_RESULT_OK)
		*sample = fake->sample;
	return fake->read_result;
}

static BspTemperatureStatusSet FakeTemperature_ReadStatus(void *context)
{
	return ((FakeTemperature *)context)->reported_status;
}

static BspTemperaturePort FakeTemperature_CreatePort(
	FakeTemperature *fake,
	const BspTemperatureEndpointCapabilities *capabilities)
{
	BspTemperaturePort port;

	(void)memset(&port, 0, sizeof(port));
	port.context = fake;
	port.capabilities = capabilities;
	port.initialize = FakeTemperature_Initialize;
	port.request_sample = FakeTemperature_Request;
	port.try_read_latest = FakeTemperature_Read;
	port.read_status = FakeTemperature_ReadStatus;
	return port;
}

static BspTemperatureEndpointCapabilities FakeTemperature_Capabilities(void)
{
	BspTemperatureEndpointCapabilities capabilities;

	(void)memset(&capabilities, 0, sizeof(capabilities));
	capabilities.endpoint_id = 1U;
	capabilities.availability = BSP_ENDPOINT_AVAILABLE;
	capabilities.source_kind = BSP_TEMPERATURE_SOURCE_PROCESSOR_DIE;
	capabilities.location = BSP_TEMPERATURE_LOCATION_PROCESSOR;
	return capabilities;
}

static TemperatureSupervisionConfig TemperatureSupervisionTest_Config(
	bool protection_enabled)
{
	TemperatureSupervisionConfig config;

	(void)memset(&config, 0, sizeof(config));
	config.sample_period_ms = 3U;
	config.pending_timeout_ms = 3U;
	config.monitor.protection_enabled = protection_enabled;
	config.monitor.trip_temperature_c = 100.0f;
	config.monitor.trip_on_invalid_sample = true;
	config.monitor.trip_on_stale_sample = true;
	config.monitor.trip_on_sensor_open = true;
	config.monitor.trip_on_sensor_short = true;
	config.monitor.trip_on_sensor_fault = true;
	return config;
}

static void FakeTemperature_Reset(FakeTemperature *fake)
{
	(void)memset(fake, 0, sizeof(*fake));
	fake->initialize_result = BSP_RESULT_OK;
	fake->request_result = BSP_RESULT_OK;
	fake->read_result = BSP_RESULT_NOT_READY;
	fake->sample.status = BSP_TEMPERATURE_SAMPLE_VALID;
}

static int TemperatureSupervisionTest_NormalAndMillisecondSchedule(void)
{
	BspTemperatureEndpointCapabilities capabilities =
		FakeTemperature_Capabilities();
	TemperatureSupervisionConfig config =
		TemperatureSupervisionTest_Config(true);
	TemperatureSupervisionContext context;
	TemperatureSupervisionOutput output;
	FakeTemperature fake;
	BspTemperaturePort port;

	FakeTemperature_Reset(&fake);
	port = FakeTemperature_CreatePort(&fake, &capabilities);
	TEST_CHECK(TemperatureSupervision_Initialize(&context, &port, &config) ==
		TEMPERATURE_SUPERVISION_STATUS_OK);
	TEST_CHECK(fake.initialize_count == 1U);
	TEST_CHECK(TemperatureSupervision_Execute1kHz(&context, &output) ==
		TEMPERATURE_SUPERVISION_STATUS_PENDING);
	TEST_CHECK(fake.request_count == 1U && fake.read_count == 0U);

	fake.read_result = BSP_RESULT_OK;
	fake.sample.temperature_c = 42.5f;
	fake.sample.timestamp_us = 12000U;
	fake.sample.sequence = 7U;
	TEST_CHECK(TemperatureSupervision_Execute1kHz(&context, &output) ==
		TEMPERATURE_SUPERVISION_STATUS_SAMPLE_PROCESSED);
	TEST_CHECK(output.last_event == TEMPERATURE_SUPERVISION_EVENT_SAMPLE);
	TEST_CHECK(output.has_source_sample && !output.source_sequence_stale);
	TEST_CHECK(output.source_timestamp_us == 12000U &&
		output.source_sequence == 7U);
	TEST_CHECK(output.latest_temperature.has_valid_temperature);
	TEST_CHECK(output.latest_temperature.latest_valid_temperature_c == 42.5f);
	TEST_CHECK(!output.trip_requested && !output.sample_pending);
	TEST_CHECK(TemperatureSupervision_GetLatestOutput(&context) ==
		&context.output);

	/* A period of three milliseconds requests on the third later 1 kHz call. */
	TEST_CHECK(TemperatureSupervision_Execute1kHz(&context, &output) ==
		TEMPERATURE_SUPERVISION_STATUS_OK);
	TEST_CHECK(TemperatureSupervision_Execute1kHz(&context, &output) ==
		TEMPERATURE_SUPERVISION_STATUS_OK);
	TEST_CHECK(fake.request_count == 1U);
	TEST_CHECK(TemperatureSupervision_Execute1kHz(&context, &output) ==
		TEMPERATURE_SUPERVISION_STATUS_PENDING);
	TEST_CHECK(fake.request_count == 2U);
	return 0;
}

static int TemperatureSupervisionTest_BusyNotReadyThenSample(void)
{
	BspTemperatureEndpointCapabilities capabilities =
		FakeTemperature_Capabilities();
	TemperatureSupervisionConfig config =
		TemperatureSupervisionTest_Config(true);
	TemperatureSupervisionContext context;
	TemperatureSupervisionOutput output;
	FakeTemperature fake;
	BspTemperaturePort port;

	FakeTemperature_Reset(&fake);
	config.pending_timeout_ms = 4U;
	fake.request_result = BSP_RESULT_BUSY;
	port = FakeTemperature_CreatePort(&fake, &capabilities);
	TEST_CHECK(TemperatureSupervision_Initialize(&context, &port, &config) ==
		TEMPERATURE_SUPERVISION_STATUS_OK);
	TEST_CHECK(TemperatureSupervision_Execute1kHz(&context, &output) ==
		TEMPERATURE_SUPERVISION_STATUS_PENDING);
	fake.read_result = BSP_RESULT_BUSY;
	TEST_CHECK(TemperatureSupervision_Execute1kHz(&context, &output) ==
		TEMPERATURE_SUPERVISION_STATUS_PENDING);
	TEST_CHECK(output.pending_elapsed_ms == 1U);
	fake.read_result = BSP_RESULT_NOT_READY;
	TEST_CHECK(TemperatureSupervision_Execute1kHz(&context, &output) ==
		TEMPERATURE_SUPERVISION_STATUS_PENDING);
	TEST_CHECK(output.pending_elapsed_ms == 2U);
	fake.read_result = BSP_RESULT_OK;
	fake.sample.temperature_c = 35.0f;
	fake.sample.sequence = 1U;
	TEST_CHECK(TemperatureSupervision_Execute1kHz(&context, &output) ==
		TEMPERATURE_SUPERVISION_STATUS_SAMPLE_PROCESSED);
	TEST_CHECK(fake.request_count == 1U && fake.read_count == 3U);
	TEST_CHECK(!output.trip_requested);
	return 0;
}

static int TemperatureSupervisionTest_TimeoutAndPortFailure(void)
{
	BspTemperatureEndpointCapabilities capabilities =
		FakeTemperature_Capabilities();
	TemperatureSupervisionConfig config =
		TemperatureSupervisionTest_Config(true);
	TemperatureSupervisionContext context;
	TemperatureSupervisionOutput output;
	FakeTemperature fake;
	BspTemperaturePort port;

	FakeTemperature_Reset(&fake);
	config.sample_period_ms = 1U;
	config.pending_timeout_ms = 2U;
	port = FakeTemperature_CreatePort(&fake, &capabilities);
	TEST_CHECK(TemperatureSupervision_Initialize(&context, &port, &config) ==
		TEMPERATURE_SUPERVISION_STATUS_OK);
	TEST_CHECK(TemperatureSupervision_Execute1kHz(&context, &output) ==
		TEMPERATURE_SUPERVISION_STATUS_PENDING);
	TEST_CHECK(TemperatureSupervision_Execute1kHz(&context, &output) ==
		TEMPERATURE_SUPERVISION_STATUS_PENDING);
	TEST_CHECK(TemperatureSupervision_Execute1kHz(&context, &output) ==
		TEMPERATURE_SUPERVISION_STATUS_SAMPLE_TIMEOUT);
	TEST_CHECK(output.last_event == TEMPERATURE_SUPERVISION_EVENT_TIMEOUT);
	TEST_CHECK((output.domain_status & TEMPERATURE_MONITOR_SAMPLE_STALE) != 0U);
	TEST_CHECK((output.domain_status & TEMPERATURE_MONITOR_SENSOR_FAULT) != 0U);
	TEST_CHECK((output.domain_status & TEMPERATURE_MONITOR_SAMPLE_VALID) == 0U);
	TEST_CHECK(output.trip_requested && !output.has_source_sample);

	/* The next period starts a fresh request; a hard read error becomes a
	 * domain fault observation without any blocking retry loop. */
	TEST_CHECK(TemperatureSupervision_Execute1kHz(&context, &output) ==
		TEMPERATURE_SUPERVISION_STATUS_PENDING);
	fake.read_result = BSP_RESULT_IO_ERROR;
	TEST_CHECK(TemperatureSupervision_Execute1kHz(&context, &output) ==
		TEMPERATURE_SUPERVISION_STATUS_BSP_READ_FAILED);
	TEST_CHECK(output.last_event ==
		TEMPERATURE_SUPERVISION_EVENT_PORT_FAILURE);
	TEST_CHECK(output.trip_requested);
	return 0;
}

static int TemperatureSupervisionTest_OverrunMapping(void)
{
	BspTemperatureEndpointCapabilities capabilities =
		FakeTemperature_Capabilities();
	TemperatureSupervisionConfig config =
		TemperatureSupervisionTest_Config(true);
	TemperatureSupervisionContext context;
	TemperatureSupervisionOutput output;
	FakeTemperature fake;
	BspTemperaturePort port;
	TemperatureMonitorSampleStatusSet mapped;

	mapped = TemperatureSupervision_MapSampleStatus(
		BSP_TEMPERATURE_SAMPLE_VALID | BSP_TEMPERATURE_ADC_OVERRUN);
	TEST_CHECK((mapped & TEMPERATURE_MONITOR_SAMPLE_VALID) != 0U);
	TEST_CHECK((mapped & TEMPERATURE_MONITOR_SAMPLE_STALE) != 0U);
	TEST_CHECK((mapped & TEMPERATURE_MONITOR_SENSOR_FAULT) != 0U);
	mapped = TemperatureSupervision_MapSampleStatus(UINT32_C(0x80000000));
	TEST_CHECK(mapped == TEMPERATURE_MONITOR_SENSOR_FAULT);

	FakeTemperature_Reset(&fake);
	port = FakeTemperature_CreatePort(&fake, &capabilities);
	TEST_CHECK(TemperatureSupervision_Initialize(&context, &port, &config) ==
		TEMPERATURE_SUPERVISION_STATUS_OK);
	TEST_CHECK(TemperatureSupervision_Execute1kHz(&context, &output) ==
		TEMPERATURE_SUPERVISION_STATUS_PENDING);
	fake.read_result = BSP_RESULT_OK;
	fake.sample.temperature_c = 150.0f;
	fake.sample.sequence = 1U;
	fake.sample.status = BSP_TEMPERATURE_SAMPLE_VALID |
		BSP_TEMPERATURE_ADC_OVERRUN;
	TEST_CHECK(TemperatureSupervision_Execute1kHz(&context, &output) ==
		TEMPERATURE_SUPERVISION_STATUS_SAMPLE_PROCESSED);
	TEST_CHECK(!output.latest_temperature.has_valid_temperature);
	TEST_CHECK((output.latest_temperature.trip_reasons &
		TEMPERATURE_MONITOR_TRIP_STALE_SAMPLE) != 0U);
	TEST_CHECK((output.latest_temperature.trip_reasons &
		TEMPERATURE_MONITOR_TRIP_SENSOR_FAULT) != 0U);
	TEST_CHECK((output.latest_temperature.trip_reasons &
		TEMPERATURE_MONITOR_TRIP_OVER_TEMPERATURE) == 0U);
	return 0;
}

static TemperatureSupervisionStatus TemperatureSupervisionTest_Publish(
	TemperatureSupervisionContext *context, FakeTemperature *fake,
	TemperatureSupervisionOutput *output, uint32_t sequence,
	float temperature_c)
{
	TemperatureSupervisionStatus status;

	status = TemperatureSupervision_Execute1kHz(context, output);
	if (status != TEMPERATURE_SUPERVISION_STATUS_PENDING)
		return status;
	fake->read_result = BSP_RESULT_OK;
	fake->sample.status = BSP_TEMPERATURE_SAMPLE_VALID;
	fake->sample.sequence = sequence;
	fake->sample.temperature_c = temperature_c;
	return TemperatureSupervision_Execute1kHz(context, output);
}

static int TemperatureSupervisionTest_SourceSequence(void)
{
	BspTemperatureEndpointCapabilities capabilities =
		FakeTemperature_Capabilities();
	TemperatureSupervisionConfig config =
		TemperatureSupervisionTest_Config(true);
	TemperatureSupervisionContext context;
	TemperatureSupervisionOutput output;
	FakeTemperature fake;
	BspTemperaturePort port;

	FakeTemperature_Reset(&fake);
	config.sample_period_ms = 1U;
	port = FakeTemperature_CreatePort(&fake, &capabilities);
	TEST_CHECK(TemperatureSupervision_Initialize(&context, &port, &config) ==
		TEMPERATURE_SUPERVISION_STATUS_OK);
	TEST_CHECK(TemperatureSupervisionTest_Publish(&context, &fake, &output,
		UINT32_MAX, 20.0f) == TEMPERATURE_SUPERVISION_STATUS_SAMPLE_PROCESSED);
	TEST_CHECK(!output.source_sequence_stale);
	TEST_CHECK(TemperatureSupervisionTest_Publish(&context, &fake, &output,
		0U, 21.0f) == TEMPERATURE_SUPERVISION_STATUS_SAMPLE_PROCESSED);
	TEST_CHECK(!output.source_sequence_stale);
	TEST_CHECK(output.latest_temperature.latest_valid_temperature_c == 21.0f);

	TEST_CHECK(TemperatureSupervisionTest_Publish(&context, &fake, &output,
		0U, 80.0f) == TEMPERATURE_SUPERVISION_STATUS_SAMPLE_PROCESSED);
	TEST_CHECK(output.source_sequence_stale);
	TEST_CHECK((output.domain_status & TEMPERATURE_MONITOR_SAMPLE_STALE) != 0U);
	TEST_CHECK(output.latest_temperature.latest_valid_temperature_c == 21.0f);
	TEST_CHECK(TemperatureSupervisionTest_Publish(&context, &fake, &output,
		UINT32_MAX, 90.0f) == TEMPERATURE_SUPERVISION_STATUS_SAMPLE_PROCESSED);
	TEST_CHECK(output.source_sequence_stale);
	TEST_CHECK(output.latest_temperature.latest_valid_temperature_c == 21.0f);
	TEST_CHECK(TemperatureSupervisionTest_Publish(&context, &fake, &output,
		1U, 22.0f) == TEMPERATURE_SUPERVISION_STATUS_SAMPLE_PROCESSED);
	TEST_CHECK(!output.source_sequence_stale);
	TEST_CHECK(output.latest_temperature.latest_valid_temperature_c == 22.0f);
	return 0;
}

static int TemperatureSupervisionTest_MonitorOnlyNeverTrips(void)
{
	BspTemperatureEndpointCapabilities capabilities =
		FakeTemperature_Capabilities();
	TemperatureSupervisionConfig config =
		TemperatureSupervisionTest_Config(false);
	TemperatureSupervisionContext context;
	TemperatureSupervisionOutput output;
	FakeTemperature fake;
	BspTemperaturePort port;

	FakeTemperature_Reset(&fake);
	config.sample_period_ms = 1U;
	config.pending_timeout_ms = 1U;
	port = FakeTemperature_CreatePort(&fake, &capabilities);
	TEST_CHECK(TemperatureSupervision_Initialize(&context, &port, &config) ==
		TEMPERATURE_SUPERVISION_STATUS_OK);
	TEST_CHECK(TemperatureSupervisionTest_Publish(&context, &fake, &output,
		1U, 150.0f) == TEMPERATURE_SUPERVISION_STATUS_SAMPLE_PROCESSED);
	TEST_CHECK(!output.trip_requested);

	fake.sample.status = BSP_TEMPERATURE_SAMPLE_VALID |
		BSP_TEMPERATURE_ADC_OVERRUN;
	TEST_CHECK(TemperatureSupervisionTest_Publish(&context, &fake, &output,
		2U, 150.0f) == TEMPERATURE_SUPERVISION_STATUS_SAMPLE_PROCESSED);
	TEST_CHECK(!output.trip_requested);

	fake.read_result = BSP_RESULT_NOT_READY;
	TEST_CHECK(TemperatureSupervision_Execute1kHz(&context, &output) ==
		TEMPERATURE_SUPERVISION_STATUS_PENDING);
	TEST_CHECK(TemperatureSupervision_Execute1kHz(&context, &output) ==
		TEMPERATURE_SUPERVISION_STATUS_SAMPLE_TIMEOUT);
	TEST_CHECK(!output.trip_requested);

	fake.read_result = BSP_RESULT_IO_ERROR;
	TEST_CHECK(TemperatureSupervision_Execute1kHz(&context, &output) ==
		TEMPERATURE_SUPERVISION_STATUS_PENDING);
	TEST_CHECK(TemperatureSupervision_Execute1kHz(&context, &output) ==
		TEMPERATURE_SUPERVISION_STATUS_BSP_READ_FAILED);
	TEST_CHECK(!output.trip_requested);
	return 0;
}

static int TemperatureSupervisionTest_Validation(void)
{
	BspTemperatureEndpointCapabilities capabilities =
		FakeTemperature_Capabilities();
	TemperatureSupervisionConfig config =
		TemperatureSupervisionTest_Config(false);
	TemperatureSupervisionContext context;
	TemperatureSupervisionOutput output;
	FakeTemperature fake;
	BspTemperaturePort port;

	FakeTemperature_Reset(&fake);
	port = FakeTemperature_CreatePort(&fake, &capabilities);
	config.sample_period_ms = 0U;
	TEST_CHECK(TemperatureSupervision_Initialize(&context, &port, &config) ==
		TEMPERATURE_SUPERVISION_STATUS_INVALID_CONFIGURATION);
	config.sample_period_ms = 1U;
	port.read_status = NULL;
	TEST_CHECK(TemperatureSupervision_Initialize(&context, &port, &config) ==
		TEMPERATURE_SUPERVISION_STATUS_INVALID_PORT);
	port = FakeTemperature_CreatePort(&fake, &capabilities);
	fake.initialize_result = BSP_RESULT_IO_ERROR;
	TEST_CHECK(TemperatureSupervision_Initialize(&context, &port, &config) ==
		TEMPERATURE_SUPERVISION_STATUS_BSP_INITIALIZE_FAILED);
	TEST_CHECK(TemperatureSupervision_Execute1kHz(&context, &output) ==
		TEMPERATURE_SUPERVISION_STATUS_NOT_INITIALIZED);
	return 0;
}

int TemperatureSupervision_RunHostTests(void)
{
	int result;

	result = TemperatureSupervisionTest_NormalAndMillisecondSchedule();
	if (result != 0)
		return result;
	result = TemperatureSupervisionTest_BusyNotReadyThenSample();
	if (result != 0)
		return result;
	result = TemperatureSupervisionTest_TimeoutAndPortFailure();
	if (result != 0)
		return result;
	result = TemperatureSupervisionTest_OverrunMapping();
	if (result != 0)
		return result;
	result = TemperatureSupervisionTest_SourceSequence();
	if (result != 0)
		return result;
	result = TemperatureSupervisionTest_MonitorOnlyNeverTrips();
	if (result != 0)
		return result;
	return TemperatureSupervisionTest_Validation();
}

#if defined(TEMPERATURE_SUPERVISION_TEST_STANDALONE)
int main(void)
{
	return TemperatureSupervision_RunHostTests();
}
#endif
