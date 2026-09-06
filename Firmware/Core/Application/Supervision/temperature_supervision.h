#ifndef FIRMWARE_CORE_APPLICATION_SUPERVISION_TEMPERATURE_SUPERVISION_H
#define FIRMWARE_CORE_APPLICATION_SUPERVISION_TEMPERATURE_SUPERVISION_H

#include "Bsp/Api/bsp_temperature.h"
#include "Core/Services/Measurement/temperature_monitor.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
	TEMPERATURE_SUPERVISION_STATUS_OK = 0,
	TEMPERATURE_SUPERVISION_STATUS_PENDING,
	TEMPERATURE_SUPERVISION_STATUS_SAMPLE_PROCESSED,
	TEMPERATURE_SUPERVISION_STATUS_SAMPLE_TIMEOUT,
	TEMPERATURE_SUPERVISION_STATUS_NULL_ARGUMENT,
	TEMPERATURE_SUPERVISION_STATUS_INVALID_CONFIGURATION,
	TEMPERATURE_SUPERVISION_STATUS_INVALID_PORT,
	TEMPERATURE_SUPERVISION_STATUS_BSP_INITIALIZE_FAILED,
	TEMPERATURE_SUPERVISION_STATUS_BSP_REQUEST_FAILED,
	TEMPERATURE_SUPERVISION_STATUS_BSP_READ_FAILED,
	TEMPERATURE_SUPERVISION_STATUS_MONITOR_REJECTED,
	TEMPERATURE_SUPERVISION_STATUS_NOT_INITIALIZED
} TemperatureSupervisionStatus;

typedef enum
{
	TEMPERATURE_SUPERVISION_EVENT_NONE = 0,
	TEMPERATURE_SUPERVISION_EVENT_SAMPLE,
	TEMPERATURE_SUPERVISION_EVENT_TIMEOUT,
	TEMPERATURE_SUPERVISION_EVENT_PORT_FAILURE
} TemperatureSupervisionEvent;

typedef struct
{
	/* Execute1kHz uses millisecond periods directly. These are not motor-loop
	 * dividers and therefore do not depend on the PWM/control frequency. */
	uint32_t sample_period_ms;
	uint32_t pending_timeout_ms;
	TemperatureMonitorConfig monitor;
} TemperatureSupervisionConfig;

typedef struct
{
	TemperatureMonitorOutput latest_temperature;
	TemperatureMonitorStatus monitor_status;
	TemperatureMonitorSampleStatusSet domain_status;
	BspResult last_bsp_result;
	uint32_t source_timestamp_us;
	uint32_t source_sequence;
	uint32_t pending_elapsed_ms;
	TemperatureSupervisionEvent last_event;
	bool has_source_sample;
	bool source_sequence_stale;
	bool sample_pending;
	bool trip_requested;
} TemperatureSupervisionOutput;

typedef struct
{
	BspTemperaturePort port;
	TemperatureMonitorContext monitor;
	TemperatureSupervisionOutput output;
	uint32_t sample_period_ms;
	uint32_t pending_timeout_ms;
	uint32_t request_countdown_ms;
	uint32_t pending_elapsed_ms;
	uint32_t next_observation_sequence;
	uint32_t last_source_sequence;
	bool has_last_source_sequence;
	bool sample_pending;
	bool is_initialized;
} TemperatureSupervisionContext;

/* ADC overrun represents a lost acquisition, so it maps to both stale and
 * sensor-fault domain quality. Unknown BSP bits fail closed as sensor fault. */
TemperatureMonitorSampleStatusSet TemperatureSupervision_MapSampleStatus(
	BspTemperatureStatusSet status);

TemperatureSupervisionStatus TemperatureSupervision_Initialize(
	TemperatureSupervisionContext *context,
	const BspTemperaturePort *port,
	const TemperatureSupervisionConfig *config);

/* Must be called once per millisecond. All BSP calls are non-blocking. A
 * pending request is polled at most once per call and is converted into a
 * domain timeout observation after pending_timeout_ms calls. This module only
 * reports trip_requested; it has no motor-output shutdown authority. */
TemperatureSupervisionStatus TemperatureSupervision_Execute1kHz(
	TemperatureSupervisionContext *context,
	TemperatureSupervisionOutput *output);

const TemperatureSupervisionOutput *TemperatureSupervision_GetLatestOutput(
	const TemperatureSupervisionContext *context);

#ifdef __cplusplus
}
#endif

#endif
