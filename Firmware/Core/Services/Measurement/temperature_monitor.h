#ifndef FIRMWARE_CORE_SERVICES_MEASUREMENT_TEMPERATURE_MONITOR_H
#define FIRMWARE_CORE_SERVICES_MEASUREMENT_TEMPERATURE_MONITOR_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Pure domain status. Hardware adapters translate their native/BSP status into
 * this type at the composition boundary; this service has no BSP dependency.
 */
typedef uint32_t TemperatureMonitorSampleStatusSet;

enum
{
	TEMPERATURE_MONITOR_SAMPLE_VALID = UINT32_C(1) << 0,
	TEMPERATURE_MONITOR_SAMPLE_STALE = UINT32_C(1) << 1,
	TEMPERATURE_MONITOR_SENSOR_OPEN = UINT32_C(1) << 2,
	TEMPERATURE_MONITOR_SENSOR_SHORT = UINT32_C(1) << 3,
	TEMPERATURE_MONITOR_SENSOR_FAULT = UINT32_C(1) << 4
};

typedef uint32_t TemperatureMonitorTripReasonSet;

enum
{
	TEMPERATURE_MONITOR_TRIP_NONE = 0U,
	TEMPERATURE_MONITOR_TRIP_OVER_TEMPERATURE = UINT32_C(1) << 0,
	TEMPERATURE_MONITOR_TRIP_INVALID_SAMPLE = UINT32_C(1) << 1,
	TEMPERATURE_MONITOR_TRIP_STALE_SAMPLE = UINT32_C(1) << 2,
	TEMPERATURE_MONITOR_TRIP_SENSOR_OPEN = UINT32_C(1) << 3,
	TEMPERATURE_MONITOR_TRIP_SENSOR_SHORT = UINT32_C(1) << 4,
	TEMPERATURE_MONITOR_TRIP_SENSOR_FAULT = UINT32_C(1) << 5
};

typedef enum
{
	TEMPERATURE_MONITOR_STATUS_OK = 0,
	TEMPERATURE_MONITOR_STATUS_NULL_ARGUMENT,
	TEMPERATURE_MONITOR_STATUS_NOT_CONFIGURED,
	TEMPERATURE_MONITOR_STATUS_INVALID_CONFIGURATION,
	TEMPERATURE_MONITOR_STATUS_INVALID_SAMPLE_STATUS,
	TEMPERATURE_MONITOR_STATUS_NONFINITE_TEMPERATURE,
	TEMPERATURE_MONITOR_STATUS_DUPLICATE_SEQUENCE,
	TEMPERATURE_MONITOR_STATUS_OUT_OF_ORDER_SEQUENCE
} TemperatureMonitorStatus;

typedef struct
{
	/* false makes this a strictly monitor-only instance. */
	bool protection_enabled;
	float trip_temperature_c;
	bool trip_on_invalid_sample;
	bool trip_on_stale_sample;
	bool trip_on_sensor_open;
	bool trip_on_sensor_short;
	bool trip_on_sensor_fault;
} TemperatureMonitorConfig;

typedef struct
{
	float temperature_c;
	uint32_t sequence;
	TemperatureMonitorSampleStatusSet status;
} TemperatureMonitorInput;

typedef struct
{
	/* Last trustworthy value; faulted or stale observations never replace it. */
	float latest_valid_temperature_c;
	uint32_t latest_valid_sequence;
	bool has_valid_temperature;

	/* Status and sequence of the observation evaluated by the latest call. */
	uint32_t observed_sequence;
	TemperatureMonitorSampleStatusSet observed_status;
	TemperatureMonitorTripReasonSet trip_reasons;
	bool trip_requested;
} TemperatureMonitorOutput;

typedef struct
{
	TemperatureMonitorConfig config;
	TemperatureMonitorOutput output;
	uint32_t last_sequence;
	bool has_last_sequence;
	bool is_configured;
} TemperatureMonitorContext;

void TemperatureMonitor_Reset(TemperatureMonitorContext *context);
TemperatureMonitorStatus TemperatureMonitor_Configure(
	TemperatureMonitorContext *context,
	const TemperatureMonitorConfig *config);

/*
 * Sequence ordering uses modulo-2^32 arithmetic. UINT32_MAX -> 0 is forward;
 * equal is duplicate, and a backward distance is out of order. Duplicate and
 * out-of-order observations are treated as stale for protection policy and do
 * not replace the last trustworthy temperature.
 *
 * trip_requested is never true when protection_enabled is false, regardless
 * of temperature or sample status. Fault latching belongs to the safety/fault
 * manager above this service.
 */
TemperatureMonitorStatus TemperatureMonitor_Process(
	TemperatureMonitorContext *context,
	const TemperatureMonitorInput *input,
	TemperatureMonitorOutput *output);

#ifdef __cplusplus
}
#endif

#endif
