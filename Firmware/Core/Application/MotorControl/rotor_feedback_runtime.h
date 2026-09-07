#ifndef CORE_APPLICATION_MOTOR_CONTROL_ROTOR_FEEDBACK_RUNTIME_H
#define CORE_APPLICATION_MOTOR_CONTROL_ROTOR_FEEDBACK_RUNTIME_H

#include <stdbool.h>
#include <stdint.h>

#include "Bsp/Api/bsp_angle_sensor.h"
#include "Core/Services/RotorFeedback/encoder.h"
#include "Core/Services/RotorFeedback/feedback_frame.h"
#include "Core/Services/RotorFeedback/feedback_router.h"
#include "Core/Services/RotorFeedback/secondary_angle_tracker.h"

#define ROTOR_FEEDBACK_PRIMARY_ENCODER_NONE UINT8_MAX

typedef enum
{
	ROTOR_FEEDBACK_RUNTIME_OK = 0,
	ROTOR_FEEDBACK_RUNTIME_NULL_ARGUMENT,
	ROTOR_FEEDBACK_RUNTIME_INVALID_CONFIGURATION,
	ROTOR_FEEDBACK_RUNTIME_FALLBACK_NOT_QUALIFIED,
	ROTOR_FEEDBACK_RUNTIME_UNSUPPORTED_TOPOLOGY,
	ROTOR_FEEDBACK_RUNTIME_INVALID_PORT,
	ROTOR_FEEDBACK_RUNTIME_PORT_START_FAILED,
	ROTOR_FEEDBACK_RUNTIME_ROUTING_FAILED
} RotorFeedbackRuntimeStatus;

typedef struct
{
	FeedbackRouterConfig routing;
	uint8_t primary_encoder_index;
	bool use_output_position_for_position_control;
	float sample_period_s;
} RotorFeedbackRuntimeConfig;

typedef struct
{
	RotorFeedbackRuntimeConfig config;
	BspAngleSensorPort angle_ports[FEEDBACK_ROUTER_MAX_ANGLE_SENSORS];
	uint32_t last_sequence[FEEDBACK_ROUTER_MAX_ANGLE_SENSORS];
	bool has_sequence[FEEDBACK_ROUTER_MAX_ANGLE_SENSORS];
	uint8_t angle_sensor_count;
	uint8_t secondary_sensor_index;
	SecondaryAngleTrackerContext secondary_tracker;
	FeedbackRouter router;
	FeedbackRouterOutput routed;
	RotorFeedbackFrame frame;
	/* Prevalidated single-primary routing avoids rebuilding and revalidating a
	 * generic graph in every current-control cycle. */
	bool direct_primary_path;
	bool initialized;
} RotorFeedbackRuntimeContext;

RotorFeedbackRuntimeStatus RotorFeedbackRuntime_Initialize(
	RotorFeedbackRuntimeContext *context,
	const RotorFeedbackRuntimeConfig *config,
	const BspAngleSensorPort *angle_ports, uint8_t angle_sensor_count);

void RotorFeedbackRuntime_CapturePhysical(
	RotorFeedbackRuntimeContext *context, EncoderContext *primary_encoder,
	uint32_t pole_pairs);

RotorFeedbackRuntimeStatus RotorFeedbackRuntime_UpdateRoutes(
	RotorFeedbackRuntimeContext *context,
	const EncoderContext *primary_encoder,
	const FeedbackRouterNormalizedSample *observer_sample,
	uint32_t pole_pairs);

bool RotorFeedbackRuntime_SignalUsesObserver(
	const RotorFeedbackRuntimeContext *context, FeedbackRouterSignal signal);
bool RotorFeedbackRuntime_HasPrimaryEncoder(
	const RotorFeedbackRuntimeContext *context);
const RotorFeedbackFrame *RotorFeedbackRuntime_GetFrame(
	const RotorFeedbackRuntimeContext *context);
const FeedbackRouterOutput *RotorFeedbackRuntime_GetRoutedOutput(
	const RotorFeedbackRuntimeContext *context);

#endif
