#ifndef FIRMWARE_CORE_SERVICES_ROTOR_FEEDBACK_FEEDBACK_ROUTER_H
#define FIRMWARE_CORE_SERVICES_ROTOR_FEEDBACK_FEEDBACK_ROUTER_H

#include <stdbool.h>
#include <stdint.h>

#define FEEDBACK_ROUTER_MAX_ANGLE_SENSORS 2U
#define FEEDBACK_ROUTER_SOURCE_INDEX_NONE UINT8_MAX

typedef uint32_t FeedbackRouterSignalMask;

typedef enum
{
	FEEDBACK_ROUTER_SIGNAL_ELECTRICAL_ANGLE = 0,
	FEEDBACK_ROUTER_SIGNAL_MOTOR_VELOCITY,
	FEEDBACK_ROUTER_SIGNAL_MOTOR_POSITION,
	FEEDBACK_ROUTER_SIGNAL_OUTPUT_POSITION,
	FEEDBACK_ROUTER_SIGNAL_CALIBRATION_REFERENCE,
	FEEDBACK_ROUTER_SIGNAL_COUNT
} FeedbackRouterSignal;

#define FEEDBACK_ROUTER_SIGNAL_MASK(signal) \
	((FeedbackRouterSignalMask)(1UL << (uint32_t)(signal)))
#define FEEDBACK_ROUTER_SIGNAL_MASK_ALL \
	((FeedbackRouterSignalMask)((1UL << FEEDBACK_ROUTER_SIGNAL_COUNT) - 1UL))
#define FEEDBACK_ROUTER_OBSERVER_SIGNAL_MASK \
	(FEEDBACK_ROUTER_SIGNAL_MASK(FEEDBACK_ROUTER_SIGNAL_ELECTRICAL_ANGLE) | \
	 FEEDBACK_ROUTER_SIGNAL_MASK(FEEDBACK_ROUTER_SIGNAL_MOTOR_VELOCITY) | \
	 FEEDBACK_ROUTER_SIGNAL_MASK(FEEDBACK_ROUTER_SIGNAL_CALIBRATION_REFERENCE))

typedef enum
{
	FEEDBACK_ROUTER_SOURCE_NONE = 0,
	FEEDBACK_ROUTER_SOURCE_ANGLE_SENSOR,
	FEEDBACK_ROUTER_SOURCE_SENSORLESS_OBSERVER
} FeedbackRouterSourceKind;

typedef struct
{
	FeedbackRouterSourceKind kind;
	uint8_t index;
} FeedbackRouterSourceRef;

/*
 * This configuration is deliberately independent of ProductConfig and BSP
 * endpoint types.  The composition root translates product instances into
 * the fixed sensor slots and advertises only the normalized signals which
 * each slot can actually provide.
 */
typedef struct
{
	uint8_t angle_sensor_count;
	FeedbackRouterSignalMask angle_sensor_capabilities[
		FEEDBACK_ROUTER_MAX_ANGLE_SENSORS];
	bool sensorless_observer_available;
	FeedbackRouterSignalMask sensorless_observer_capabilities;

	FeedbackRouterSourceRef electrical_angle;
	FeedbackRouterSourceRef motor_velocity;
	FeedbackRouterSourceRef motor_position;
	FeedbackRouterSourceRef output_position;
	FeedbackRouterSourceRef calibration_reference;
	/* The only automatic switchover route.  It is used when the configured
	 * electrical-angle source is unavailable, invalid, or non-finite. */
	FeedbackRouterSourceRef fallback_electrical_angle;
} FeedbackRouterConfig;

/*
 * Adapters publish normalized radians/radians-per-second values.  `available`
 * means the source is online and fresh according to adapter-owned timing;
 * `valid_signals` allows one value to be rejected without discarding the
 * source's other values.
 */
typedef struct
{
	bool available;
	FeedbackRouterSignalMask valid_signals;
	float electrical_angle_rad;
	float motor_velocity_rad_s;
	float motor_position_rad;
	float output_position_rad;
	float calibration_reference_rad;
} FeedbackRouterNormalizedSample;

typedef struct
{
	uint8_t angle_sensor_count;
	FeedbackRouterNormalizedSample angle_sensors[
		FEEDBACK_ROUTER_MAX_ANGLE_SENSORS];
	bool sensorless_observer_present;
	FeedbackRouterNormalizedSample sensorless_observer;
} FeedbackRouterInput;

typedef enum
{
	FEEDBACK_ROUTER_SIGNAL_STATE_DISABLED = 0,
	FEEDBACK_ROUTER_SIGNAL_STATE_PRIMARY,
	FEEDBACK_ROUTER_SIGNAL_STATE_FALLBACK,
	FEEDBACK_ROUTER_SIGNAL_STATE_UNAVAILABLE
} FeedbackRouterSignalState;

typedef struct
{
	float value;
	FeedbackRouterSourceRef selected_source;
	FeedbackRouterSignalState state;
} FeedbackRouterRoutedSignal;

typedef enum
{
	FEEDBACK_ROUTER_HEALTH_DISABLED = 0,
	FEEDBACK_ROUTER_HEALTH_HEALTHY,
	FEEDBACK_ROUTER_HEALTH_DEGRADED,
	FEEDBACK_ROUTER_HEALTH_UNAVAILABLE
} FeedbackRouterHealth;

typedef struct
{
	FeedbackRouterRoutedSignal electrical_angle;
	FeedbackRouterRoutedSignal motor_velocity;
	FeedbackRouterRoutedSignal motor_position;
	FeedbackRouterRoutedSignal output_position;
	FeedbackRouterRoutedSignal calibration_reference;
	FeedbackRouterSignalMask configured_signals;
	FeedbackRouterSignalMask valid_signals;
	FeedbackRouterSignalMask fallback_signals;
	FeedbackRouterSignalMask unavailable_signals;
	FeedbackRouterHealth health;
} FeedbackRouterOutput;

typedef enum
{
	FEEDBACK_ROUTER_STATUS_OK = 0,
	FEEDBACK_ROUTER_STATUS_NULL_ARGUMENT,
	FEEDBACK_ROUTER_STATUS_ANGLE_SENSOR_COUNT_EXCEEDED,
	FEEDBACK_ROUTER_STATUS_INVALID_CAPABILITY_MASK,
	FEEDBACK_ROUTER_STATUS_INVALID_SOURCE_KIND,
	FEEDBACK_ROUTER_STATUS_INVALID_SOURCE_INDEX,
	FEEDBACK_ROUTER_STATUS_SOURCE_NOT_AVAILABLE,
	FEEDBACK_ROUTER_STATUS_SOURCE_CAPABILITY_MISMATCH,
	FEEDBACK_ROUTER_STATUS_FALLBACK_WITHOUT_PRIMARY,
	FEEDBACK_ROUTER_STATUS_FALLBACK_DUPLICATES_PRIMARY,
	FEEDBACK_ROUTER_STATUS_NOT_CONFIGURED,
	FEEDBACK_ROUTER_STATUS_INVALID_INPUT_COUNT,
	FEEDBACK_ROUTER_STATUS_INVALID_INPUT_MASK
} FeedbackRouterStatus;

typedef struct
{
	FeedbackRouterStatus status;
	FeedbackRouterSignal signal;
	FeedbackRouterSourceRef source;
	bool fallback_route;
} FeedbackRouterValidationIssue;

typedef struct
{
	FeedbackRouterConfig config;
	bool is_configured;
} FeedbackRouter;

void FeedbackRouter_Reset(FeedbackRouter *router);

FeedbackRouterStatus FeedbackRouter_ValidateConfig(
	const FeedbackRouterConfig *config, FeedbackRouterValidationIssue *issue);

/* A failed configuration clears the previous configuration (fail closed). */
FeedbackRouterStatus FeedbackRouter_Configure(FeedbackRouter *router,
	const FeedbackRouterConfig *config, FeedbackRouterValidationIssue *issue);

/*
 * STATUS_OK means the DTO was processed.  Runtime source loss is represented
 * by output health/state rather than an API error so callers can distinguish
 * a controlled fallback from a malformed invocation.
 */
FeedbackRouterStatus FeedbackRouter_Process(const FeedbackRouter *router,
	const FeedbackRouterInput *input, FeedbackRouterOutput *output);

#endif
