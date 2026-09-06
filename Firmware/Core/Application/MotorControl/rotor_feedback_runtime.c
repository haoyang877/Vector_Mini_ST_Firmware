#include "rotor_feedback_runtime.h"

#include <float.h>
#include <string.h>

static bool RotorFeedbackRuntime_IsFinite(float value)
{
	return value == value && value <= FLT_MAX && value >= -FLT_MAX;
}

static FeedbackRouterSourceRef RotorFeedbackRuntime_GetRoute(
	const FeedbackRouterConfig *config, FeedbackRouterSignal signal)
{
	FeedbackRouterSourceRef none = {FEEDBACK_ROUTER_SOURCE_NONE,
		FEEDBACK_ROUTER_SOURCE_INDEX_NONE};

	switch (signal)
	{
		case FEEDBACK_ROUTER_SIGNAL_ELECTRICAL_ANGLE:
			return config->electrical_angle;
		case FEEDBACK_ROUTER_SIGNAL_MOTOR_VELOCITY:
			return config->motor_velocity;
		case FEEDBACK_ROUTER_SIGNAL_MOTOR_POSITION:
			return config->motor_position;
		case FEEDBACK_ROUTER_SIGNAL_OUTPUT_POSITION:
			return config->output_position;
		case FEEDBACK_ROUTER_SIGNAL_CALIBRATION_REFERENCE:
			return config->calibration_reference;
		default:
			return none;
	}
}

static RotorFeedbackRuntimeStatus RotorFeedbackRuntime_Validate(
	const RotorFeedbackRuntimeConfig *config, uint8_t angle_sensor_count)
{
	FeedbackRouterValidationIssue issue;
	FeedbackRouterSignal signal;

	if (config == 0 ||
		angle_sensor_count > FEEDBACK_ROUTER_MAX_ANGLE_SENSORS ||
		config->routing.angle_sensor_count != angle_sensor_count ||
		!RotorFeedbackRuntime_IsFinite(config->sample_period_s) ||
		config->sample_period_s <= 0.0f ||
		(config->primary_encoder_index != ROTOR_FEEDBACK_PRIMARY_ENCODER_NONE &&
		 config->primary_encoder_index >= angle_sensor_count))
	{
		return ROTOR_FEEDBACK_RUNTIME_INVALID_CONFIGURATION;
	}
	if (config->routing.fallback_electrical_angle.kind !=
		FEEDBACK_ROUTER_SOURCE_NONE)
	{
		/* Qualification/hysteresis is intentionally not part of this minimum
		 * delivery.  An unqualified automatic commutation-source switch is less
		 * safe than rejecting the product at composition time. */
		return ROTOR_FEEDBACK_RUNTIME_FALLBACK_NOT_QUALIFIED;
	}
	if (angle_sensor_count > 1U &&
		config->primary_encoder_index == ROTOR_FEEDBACK_PRIMARY_ENCODER_NONE)
		return ROTOR_FEEDBACK_RUNTIME_UNSUPPORTED_TOPOLOGY;

	/* Exactly one full EncoderContext exists.  Physical motor-rotor routes must
	 * bind to it; the lightweight secondary path is output-position only. */
	for (signal = FEEDBACK_ROUTER_SIGNAL_ELECTRICAL_ANGLE;
		signal < FEEDBACK_ROUTER_SIGNAL_COUNT; signal++)
	{
		FeedbackRouterSourceRef source = RotorFeedbackRuntime_GetRoute(
			&config->routing, signal);

		if (source.kind != FEEDBACK_ROUTER_SOURCE_ANGLE_SENSOR)
			continue;
		if (signal == FEEDBACK_ROUTER_SIGNAL_OUTPUT_POSITION)
		{
			if (source.index == config->primary_encoder_index)
				return ROTOR_FEEDBACK_RUNTIME_UNSUPPORTED_TOPOLOGY;
		}
		else if (source.index != config->primary_encoder_index)
		{
			return ROTOR_FEEDBACK_RUNTIME_UNSUPPORTED_TOPOLOGY;
		}
	}
	if (FeedbackRouter_ValidateConfig(&config->routing, &issue) !=
		FEEDBACK_ROUTER_STATUS_OK)
		return ROTOR_FEEDBACK_RUNTIME_INVALID_CONFIGURATION;
	return ROTOR_FEEDBACK_RUNTIME_OK;
}

static void RotorFeedbackRuntime_StopStartedPorts(
	RotorFeedbackRuntimeContext *context, uint8_t started_count)
{
	while (started_count > 0U)
	{
		BspAngleSensorPort *port;

		started_count--;
		port = &context->angle_ports[started_count];
		if (port->stop_acquisition != 0)
			(void)port->stop_acquisition(port->context);
	}
}

RotorFeedbackRuntimeStatus RotorFeedbackRuntime_Initialize(
	RotorFeedbackRuntimeContext *context,
	const RotorFeedbackRuntimeConfig *config,
	const BspAngleSensorPort *angle_ports, uint8_t angle_sensor_count)
{
	RotorFeedbackRuntimeStatus status;
	FeedbackRouterValidationIssue issue;
	uint8_t index;

	if (context == 0 || config == 0 ||
		(angle_sensor_count > 0U && angle_ports == 0))
		return ROTOR_FEEDBACK_RUNTIME_NULL_ARGUMENT;
	memset(context, 0, sizeof(*context));
	context->secondary_sensor_index = FEEDBACK_ROUTER_SOURCE_INDEX_NONE;
	status = RotorFeedbackRuntime_Validate(config, angle_sensor_count);
	if (status != ROTOR_FEEDBACK_RUNTIME_OK)
		return status;

	context->config = *config;
	context->angle_sensor_count = angle_sensor_count;
	for (index = 0U; index < angle_sensor_count; index++)
	{
		const BspAngleSensorPort *source = &angle_ports[index];

		if (source->initialize == 0 || source->start_acquisition == 0 ||
			source->stop_acquisition == 0 || source->try_read_latest == 0)
		{
			RotorFeedbackRuntime_StopStartedPorts(context, index);
			return ROTOR_FEEDBACK_RUNTIME_INVALID_PORT;
		}
		context->angle_ports[index] = *source;
		if (index != config->primary_encoder_index)
			context->secondary_sensor_index = index;
		if (context->angle_ports[index].initialize(
				context->angle_ports[index].context) != BSP_RESULT_OK ||
			context->angle_ports[index].start_acquisition(
				context->angle_ports[index].context) != BSP_RESULT_OK)
		{
			RotorFeedbackRuntime_StopStartedPorts(context, index);
			return ROTOR_FEEDBACK_RUNTIME_PORT_START_FAILED;
		}
	}
	SecondaryAngleTracker_Reset(&context->secondary_tracker);
	if (FeedbackRouter_Configure(&context->router, &config->routing, &issue) !=
		FEEDBACK_ROUTER_STATUS_OK)
	{
		RotorFeedbackRuntime_StopStartedPorts(context, angle_sensor_count);
		return ROTOR_FEEDBACK_RUNTIME_INVALID_CONFIGURATION;
	}
	context->initialized = true;
	return ROTOR_FEEDBACK_RUNTIME_OK;
}

static bool RotorFeedbackRuntime_ReadPort(RotorFeedbackRuntimeContext *context,
	uint8_t index, BspAngleSensorSample *sample)
{
	BspAngleSensorPort *port = &context->angle_ports[index];
	BspResult request_status = BSP_RESULT_OK;

	if (port->request_sample != 0)
		request_status = port->request_sample(port->context);
	if (request_status != BSP_RESULT_OK ||
		port->try_read_latest(port->context, sample) != BSP_RESULT_OK ||
		(sample->status & BSP_ANGLE_SAMPLE_POSITION_VALID) == 0U ||
		(sample->status & (BSP_ANGLE_SAMPLE_STALE |
		 BSP_ANGLE_SAMPLE_SENSOR_FAULT)) != 0U ||
		(context->has_sequence[index] &&
		 sample->sequence == context->last_sequence[index]))
	{
		return false;
	}
	context->last_sequence[index] = sample->sequence;
	context->has_sequence[index] = true;
	return true;
}

void RotorFeedbackRuntime_CapturePhysical(
	RotorFeedbackRuntimeContext *context, EncoderContext *primary_encoder,
	uint32_t pole_pairs)
{
	EncoderSample encoder_sample = {ENCODER_READ_TRANSPORT_ERROR, 0U, 0U};
	uint8_t index;

	if (context == 0 || !context->initialized)
		return;
	SecondaryAngleTracker_Invalidate(&context->secondary_tracker);
	for (index = 0U; index < context->angle_sensor_count; index++)
	{
		BspAngleSensorSample sample = {0};
		bool valid = RotorFeedbackRuntime_ReadPort(context, index, &sample);

		if (index == context->config.primary_encoder_index)
		{
			if (primary_encoder == 0)
				continue;
			if (valid)
			{
				encoder_sample.status = ENCODER_READ_OK;
				encoder_sample.raw_angle_q15 = (uint16_t)(
					sample.single_turn_position_u32 >> 16U);
				encoder_sample.raw_data_word = encoder_sample.raw_angle_q15;
			}
			else if ((sample.status & BSP_ANGLE_SAMPLE_POSITION_VALID) == 0U)
			{
				encoder_sample.status = ENCODER_READ_INVALID_ANGLE;
			}
			Encoder_Update(primary_encoder, pole_pairs, &encoder_sample);
		}
		else if (index == context->secondary_sensor_index)
		{
			SecondaryAngleSample tracker_sample;

			tracker_sample.single_turn_position_u32 =
				sample.single_turn_position_u32;
			tracker_sample.turn_count = sample.turn_count;
			tracker_sample.velocity_rad_s = sample.velocity_rad_s;
			tracker_sample.turn_count_valid = (sample.status &
				BSP_ANGLE_SAMPLE_TURN_COUNT_VALID) != 0U;
			tracker_sample.velocity_valid = (sample.status &
				BSP_ANGLE_SAMPLE_VELOCITY_VALID) != 0U;
			if (!valid || !SecondaryAngleTracker_Update(
					&context->secondary_tracker, &tracker_sample,
					context->config.sample_period_s))
			{
				SecondaryAngleTracker_Invalidate(
					&context->secondary_tracker);
			}
		}
	}
}

static void RotorFeedbackRuntime_PublishPrimary(
	const RotorFeedbackRuntimeContext *context,
	const EncoderContext *encoder, FeedbackRouterNormalizedSample *sample)
{
	uint8_t index = context->config.primary_encoder_index;

	if (index == ROTOR_FEEDBACK_PRIMARY_ENCODER_NONE || encoder == 0 ||
		!Encoder_IsOnline(encoder))
		return;
	sample->available = true;
	sample->valid_signals =
		context->config.routing.angle_sensor_capabilities[index];
	sample->electrical_angle_rad = Encoder_GetElePhase(encoder);
	sample->motor_velocity_rad_s = Encoder_GetMecVel(encoder);
	sample->motor_position_rad = Encoder_GetMecPos(encoder);
	sample->output_position_rad = Encoder_GetMecPos(encoder);
	sample->calibration_reference_rad = (float)encoder->directed_q15 *
		(6.28318530717958647692f / 65536.0f);
}

static void RotorFeedbackRuntime_PublishSecondary(
	const RotorFeedbackRuntimeContext *context,
	FeedbackRouterNormalizedSample *sample)
{
	uint8_t index = context->secondary_sensor_index;

	if (index == FEEDBACK_ROUTER_SOURCE_INDEX_NONE ||
		!context->secondary_tracker.fresh)
		return;
	sample->available = true;
	sample->valid_signals =
		context->config.routing.angle_sensor_capabilities[index];
	sample->motor_velocity_rad_s = context->secondary_tracker.velocity_rad_s;
	sample->motor_position_rad = context->secondary_tracker.position_rad;
	sample->output_position_rad = context->secondary_tracker.position_rad;
	sample->calibration_reference_rad = context->secondary_tracker.position_rad;
}

static float RotorFeedbackRuntime_OutputVelocity(
	const RotorFeedbackRuntimeContext *context,
	const EncoderContext *primary_encoder)
{
	FeedbackRouterSourceRef source = context->routed.output_position.selected_source;

	if (source.kind != FEEDBACK_ROUTER_SOURCE_ANGLE_SENSOR)
		return 0.0f;
	if (source.index == context->config.primary_encoder_index &&
		primary_encoder != 0)
		return Encoder_GetMecVel(primary_encoder);
	if (source.index == context->secondary_sensor_index &&
		context->secondary_tracker.fresh)
		return context->secondary_tracker.velocity_rad_s;
	return 0.0f;
}

static float RotorFeedbackRuntime_ElectricalVelocity(
	const RotorFeedbackRuntimeContext *context,
	const EncoderContext *primary_encoder,
	const FeedbackRouterNormalizedSample *observer_sample,
	uint32_t pole_pairs)
{
	FeedbackRouterSourceRef source =
		context->routed.electrical_angle.selected_source;

	if (source.kind == FEEDBACK_ROUTER_SOURCE_ANGLE_SENSOR)
	{
		if (source.index == context->config.primary_encoder_index &&
			primary_encoder != 0)
			return Encoder_GetEleVel(primary_encoder);
		if (source.index == context->secondary_sensor_index &&
			context->secondary_tracker.fresh)
		{
			return context->secondary_tracker.velocity_rad_s *
				(float)pole_pairs;
		}
	}
	else if (source.kind == FEEDBACK_ROUTER_SOURCE_SENSORLESS_OBSERVER &&
		observer_sample != 0 && observer_sample->available &&
		(observer_sample->valid_signals & FEEDBACK_ROUTER_SIGNAL_MASK(
			FEEDBACK_ROUTER_SIGNAL_MOTOR_VELOCITY)) != 0U)
	{
		return observer_sample->motor_velocity_rad_s * (float)pole_pairs;
	}
	return 0.0f;
}

RotorFeedbackRuntimeStatus RotorFeedbackRuntime_UpdateRoutes(
	RotorFeedbackRuntimeContext *context,
	const EncoderContext *primary_encoder,
	const FeedbackRouterNormalizedSample *observer_sample,
	uint32_t pole_pairs)
{
	FeedbackRouterInput input;
	FeedbackRouterStatus router_status;

	if (context == 0 || !context->initialized || pole_pairs == 0U)
		return ROTOR_FEEDBACK_RUNTIME_INVALID_CONFIGURATION;
	memset(&input, 0, sizeof(input));
	input.angle_sensor_count = context->angle_sensor_count;
	if (context->config.primary_encoder_index !=
		ROTOR_FEEDBACK_PRIMARY_ENCODER_NONE)
	{
		RotorFeedbackRuntime_PublishPrimary(context, primary_encoder,
			&input.angle_sensors[context->config.primary_encoder_index]);
	}
	if (context->secondary_sensor_index !=
		FEEDBACK_ROUTER_SOURCE_INDEX_NONE)
	{
		RotorFeedbackRuntime_PublishSecondary(context,
			&input.angle_sensors[context->secondary_sensor_index]);
	}
	if (observer_sample != 0)
	{
		input.sensorless_observer_present = true;
		input.sensorless_observer = *observer_sample;
	}
	router_status = FeedbackRouter_Process(&context->router, &input,
		&context->routed);
	if (router_status != FEEDBACK_ROUTER_STATUS_OK)
	{
		memset(&context->frame, 0, sizeof(context->frame));
		context->frame.health = FEEDBACK_ROUTER_HEALTH_UNAVAILABLE;
		return ROTOR_FEEDBACK_RUNTIME_ROUTING_FAILED;
	}

	context->frame.electrical_angle_rad =
		context->routed.electrical_angle.value;
	context->frame.motor_velocity_rad_s =
		context->routed.motor_velocity.value;
	context->frame.electrical_velocity_rad_s =
		RotorFeedbackRuntime_ElectricalVelocity(context, primary_encoder,
			observer_sample, pole_pairs);
	context->frame.motor_position_rad =
		context->routed.motor_position.value;
	context->frame.output_position_rad =
		context->routed.output_position.value;
	context->frame.output_velocity_rad_s =
		RotorFeedbackRuntime_OutputVelocity(context, primary_encoder);
	context->frame.calibration_reference_rad =
		context->routed.calibration_reference.value;
	context->frame.valid_signals = context->routed.valid_signals;
	context->frame.health = context->routed.health;
	context->frame.uses_output_position =
		context->config.use_output_position_for_position_control;
	if (context->frame.uses_output_position)
	{
		context->frame.control_position_rad =
			context->frame.output_position_rad;
		context->frame.control_velocity_rad_s =
			context->frame.output_velocity_rad_s;
	}
	else
	{
		context->frame.control_position_rad =
			context->frame.motor_position_rad;
		context->frame.control_velocity_rad_s =
			context->frame.motor_velocity_rad_s;
	}
	return ROTOR_FEEDBACK_RUNTIME_OK;
}

bool RotorFeedbackRuntime_SignalUsesObserver(
	const RotorFeedbackRuntimeContext *context, FeedbackRouterSignal signal)
{
	if (context == 0 || !context->initialized)
		return false;
	return RotorFeedbackRuntime_GetRoute(&context->config.routing, signal).kind ==
		FEEDBACK_ROUTER_SOURCE_SENSORLESS_OBSERVER;
}

bool RotorFeedbackRuntime_HasPrimaryEncoder(
	const RotorFeedbackRuntimeContext *context)
{
	return context != 0 && context->initialized &&
		context->config.primary_encoder_index !=
			ROTOR_FEEDBACK_PRIMARY_ENCODER_NONE;
}

const RotorFeedbackFrame *RotorFeedbackRuntime_GetFrame(
	const RotorFeedbackRuntimeContext *context)
{
	return context != 0 && context->initialized ? &context->frame : 0;
}

const FeedbackRouterOutput *RotorFeedbackRuntime_GetRoutedOutput(
	const RotorFeedbackRuntimeContext *context)
{
	return context != 0 && context->initialized ? &context->routed : 0;
}
