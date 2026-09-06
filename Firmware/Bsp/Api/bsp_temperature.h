#ifndef FIRMWARE_BSP_API_BSP_TEMPERATURE_H
#define FIRMWARE_BSP_API_BSP_TEMPERATURE_H

#include "bsp_system.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
	BSP_TEMPERATURE_SOURCE_UNSPECIFIED = 0,
	BSP_TEMPERATURE_SOURCE_PROCESSOR_DIE,
	BSP_TEMPERATURE_SOURCE_ANALOG_RESISTIVE,
	BSP_TEMPERATURE_SOURCE_ANALOG_VOLTAGE,
	BSP_TEMPERATURE_SOURCE_DIGITAL
} BspTemperatureSourceKind;

typedef enum
{
	BSP_TEMPERATURE_LOCATION_UNSPECIFIED = 0,
	BSP_TEMPERATURE_LOCATION_PROCESSOR,
	BSP_TEMPERATURE_LOCATION_POWER_STAGE,
	BSP_TEMPERATURE_LOCATION_MOTOR,
	BSP_TEMPERATURE_LOCATION_BOARD_AMBIENT
} BspTemperatureLocation;

typedef struct
{
	BspEndpointId endpoint_id;
	BspEndpointAvailability availability;
	BspTemperatureSourceKind source_kind;
	BspTemperatureLocation location;
	bool supports_open_circuit_diagnostic;
	bool supports_short_circuit_diagnostic;
} BspTemperatureEndpointCapabilities;

typedef uint32_t BspTemperatureStatusSet;

enum
{
	BSP_TEMPERATURE_SAMPLE_VALID = UINT32_C(1) << 0,
	BSP_TEMPERATURE_SAMPLE_STALE = UINT32_C(1) << 1,
	BSP_TEMPERATURE_SENSOR_OPEN = UINT32_C(1) << 2,
	BSP_TEMPERATURE_SENSOR_SHORT = UINT32_C(1) << 3,
	BSP_TEMPERATURE_SENSOR_FAULT = UINT32_C(1) << 4
};

typedef struct
{
	float temperature_c;
	uint32_t timestamp_us;
	uint32_t sequence;
	BspTemperatureStatusSet status;
} BspTemperatureSample;

/*
 * Temperature sampling is a supervisory operation, not a motor-loop service.
 * try_read_latest() must still be non-blocking; conversion may run in the
 * background after request_sample(). request_sample() may be NULL.
 */
typedef struct
{
	void *context;
	/* Identifies the bound endpoint and its physical capabilities. */
	const BspTemperatureEndpointCapabilities *capabilities;
	BspResult (*initialize)(void *context);
	BspResult (*request_sample)(void *context);
	BspResult (*try_read_latest)(void *context, BspTemperatureSample *sample);
	BspTemperatureStatusSet (*read_status)(void *context);
} BspTemperaturePort;

#ifdef __cplusplus
}
#endif

#endif
