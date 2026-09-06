#ifndef FIRMWARE_BSP_API_BSP_ANGLE_SENSOR_H
#define FIRMWARE_BSP_API_BSP_ANGLE_SENSOR_H

#include "bsp_system.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
	BSP_ANGLE_ENDPOINT_UNSPECIFIED = 0,
	BSP_ANGLE_ENDPOINT_SYNCHRONOUS_SERIAL,
	BSP_ANGLE_ENDPOINT_INCREMENTAL,
	BSP_ANGLE_ENDPOINT_ANALOG,
	BSP_ANGLE_ENDPOINT_PULSE
} BspAngleEndpointKind;

typedef struct
{
	BspEndpointId endpoint_id;
	BspEndpointAvailability availability;
	BspAngleEndpointKind kind;
	uint8_t maximum_transfer_word_bits;
	bool has_dedicated_select;
	bool supports_triggered_acquisition;
	bool supports_dma;
} BspAngleSensorEndpointCapabilities;

typedef struct
{
	uint8_t resolution_bits;
	bool is_absolute;
	bool supports_multiturn;
	bool provides_velocity;
} BspAngleSensorCapabilities;

typedef uint32_t BspAngleSampleStatusSet;

enum
{
	BSP_ANGLE_SAMPLE_POSITION_VALID = UINT32_C(1) << 0,
	BSP_ANGLE_SAMPLE_TURN_COUNT_VALID = UINT32_C(1) << 1,
	BSP_ANGLE_SAMPLE_VELOCITY_VALID = UINT32_C(1) << 2,
	BSP_ANGLE_SAMPLE_STALE = UINT32_C(1) << 3,
	BSP_ANGLE_SAMPLE_SENSOR_FAULT = UINT32_C(1) << 4
};

/*
 * single_turn_position_u32 maps one mechanical revolution to [0, 2^32).
 * It is the uncorrected sensor reading; calibration remains above the BSP.
 */
typedef struct
{
	uint32_t single_turn_position_u32;
	int32_t turn_count;
	float velocity_rad_s;
	uint32_t timestamp_us;
	uint32_t sequence;
	BspAngleSampleStatusSet status;
} BspAngleSensorSample;

/*
 * request_sample() may be NULL for continuous sources. request_sample() and
 * try_read_latest() must be bounded and non-blocking when used by a motor ISR.
 * A pending transfer is reported as BSP_RESULT_NOT_READY, never by waiting.
 */
typedef struct
{
	void *context;
	const BspAngleSensorCapabilities *capabilities;
	BspResult (*initialize)(void *context);
	BspResult (*start_acquisition)(void *context);
	BspResult (*stop_acquisition)(void *context);
	BspResult (*request_sample)(void *context);
	BspResult (*try_read_latest)(void *context, BspAngleSensorSample *sample);
	BspAngleSampleStatusSet (*read_status)(void *context);
} BspAngleSensorPort;

#ifdef __cplusplus
}
#endif

#endif
