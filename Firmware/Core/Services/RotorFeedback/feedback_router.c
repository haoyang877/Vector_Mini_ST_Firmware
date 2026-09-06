#include "feedback_router.h"

#include <float.h>
#include <stddef.h>
#include <string.h>

static FeedbackRouterSourceRef FeedbackRouter_NoSource(void)
{
	FeedbackRouterSourceRef source;

	source.kind = FEEDBACK_ROUTER_SOURCE_NONE;
	source.index = FEEDBACK_ROUTER_SOURCE_INDEX_NONE;
	return source;
}

static bool FeedbackRouter_SourceEquals(FeedbackRouterSourceRef left,
	FeedbackRouterSourceRef right)
{
	return left.kind == right.kind && left.index == right.index;
}

static bool FeedbackRouter_IsFinite(float value)
{
	return value == value && value <= FLT_MAX && value >= -FLT_MAX;
}

static FeedbackRouterSourceRef FeedbackRouter_GetPrimarySource(
	const FeedbackRouterConfig *config, FeedbackRouterSignal signal)
{
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
		return FeedbackRouter_NoSource();
	}
}

static void FeedbackRouter_SetIssue(FeedbackRouterValidationIssue *issue,
	FeedbackRouterStatus status, FeedbackRouterSignal signal,
	FeedbackRouterSourceRef source, bool fallback_route)
{
	if (issue == NULL)
		return;
	issue->status = status;
	issue->signal = signal;
	issue->source = source;
	issue->fallback_route = fallback_route;
}

static FeedbackRouterStatus FeedbackRouter_ValidateSource(
	const FeedbackRouterConfig *config, FeedbackRouterSourceRef source,
	FeedbackRouterSignal signal, bool fallback_route,
	FeedbackRouterValidationIssue *issue)
{
	FeedbackRouterSignalMask required_capability =
		FEEDBACK_ROUTER_SIGNAL_MASK(signal);
	FeedbackRouterStatus status = FEEDBACK_ROUTER_STATUS_OK;

	switch (source.kind)
	{
	case FEEDBACK_ROUTER_SOURCE_NONE:
		if (source.index != FEEDBACK_ROUTER_SOURCE_INDEX_NONE)
			status = FEEDBACK_ROUTER_STATUS_INVALID_SOURCE_INDEX;
		break;
	case FEEDBACK_ROUTER_SOURCE_ANGLE_SENSOR:
		if (source.index >= config->angle_sensor_count)
			status = FEEDBACK_ROUTER_STATUS_INVALID_SOURCE_INDEX;
		else if ((config->angle_sensor_capabilities[source.index] &
			required_capability) == 0U)
			status = FEEDBACK_ROUTER_STATUS_SOURCE_CAPABILITY_MISMATCH;
		break;
	case FEEDBACK_ROUTER_SOURCE_SENSORLESS_OBSERVER:
		if (source.index != 0U)
			status = FEEDBACK_ROUTER_STATUS_INVALID_SOURCE_INDEX;
		else if (!config->sensorless_observer_available)
			status = FEEDBACK_ROUTER_STATUS_SOURCE_NOT_AVAILABLE;
		else if ((FEEDBACK_ROUTER_OBSERVER_SIGNAL_MASK &
			required_capability) == 0U ||
			(config->sensorless_observer_capabilities &
			required_capability) == 0U)
			status = FEEDBACK_ROUTER_STATUS_SOURCE_CAPABILITY_MISMATCH;
		break;
	default:
		status = FEEDBACK_ROUTER_STATUS_INVALID_SOURCE_KIND;
		break;
	}

	if (status != FEEDBACK_ROUTER_STATUS_OK)
	{
		FeedbackRouter_SetIssue(issue, status, signal, source,
			fallback_route);
	}
	return status;
}

void FeedbackRouter_Reset(FeedbackRouter *router)
{
	if (router != NULL)
		memset(router, 0, sizeof(*router));
}

FeedbackRouterStatus FeedbackRouter_ValidateConfig(
	const FeedbackRouterConfig *config, FeedbackRouterValidationIssue *issue)
{
	FeedbackRouterSourceRef no_source = FeedbackRouter_NoSource();
	FeedbackRouterStatus status;
	uint8_t index;
	FeedbackRouterSignal signal;

	FeedbackRouter_SetIssue(issue, FEEDBACK_ROUTER_STATUS_OK,
		FEEDBACK_ROUTER_SIGNAL_COUNT, no_source, false);
	if (config == NULL)
	{
		FeedbackRouter_SetIssue(issue, FEEDBACK_ROUTER_STATUS_NULL_ARGUMENT,
			FEEDBACK_ROUTER_SIGNAL_COUNT, no_source, false);
		return FEEDBACK_ROUTER_STATUS_NULL_ARGUMENT;
	}
	if (config->angle_sensor_count > FEEDBACK_ROUTER_MAX_ANGLE_SENSORS)
	{
		FeedbackRouter_SetIssue(issue,
			FEEDBACK_ROUTER_STATUS_ANGLE_SENSOR_COUNT_EXCEEDED,
			FEEDBACK_ROUTER_SIGNAL_COUNT, no_source, false);
		return FEEDBACK_ROUTER_STATUS_ANGLE_SENSOR_COUNT_EXCEEDED;
	}

	for (index = 0U; index < FEEDBACK_ROUTER_MAX_ANGLE_SENSORS; index++)
	{
		FeedbackRouterSignalMask capabilities =
			config->angle_sensor_capabilities[index];

		if ((capabilities & ~FEEDBACK_ROUTER_SIGNAL_MASK_ALL) != 0U ||
			(index >= config->angle_sensor_count && capabilities != 0U))
		{
			FeedbackRouterSourceRef source;

			source.kind = FEEDBACK_ROUTER_SOURCE_ANGLE_SENSOR;
			source.index = index;
			FeedbackRouter_SetIssue(issue,
				FEEDBACK_ROUTER_STATUS_INVALID_CAPABILITY_MASK,
				FEEDBACK_ROUTER_SIGNAL_COUNT, source, false);
			return FEEDBACK_ROUTER_STATUS_INVALID_CAPABILITY_MASK;
		}
	}
	if ((config->sensorless_observer_capabilities &
		~FEEDBACK_ROUTER_OBSERVER_SIGNAL_MASK) != 0U ||
		(!config->sensorless_observer_available &&
		 config->sensorless_observer_capabilities != 0U))
	{
		FeedbackRouterSourceRef source;

		source.kind = FEEDBACK_ROUTER_SOURCE_SENSORLESS_OBSERVER;
		source.index = 0U;
		FeedbackRouter_SetIssue(issue,
			FEEDBACK_ROUTER_STATUS_INVALID_CAPABILITY_MASK,
			FEEDBACK_ROUTER_SIGNAL_COUNT, source, false);
		return FEEDBACK_ROUTER_STATUS_INVALID_CAPABILITY_MASK;
	}

	for (signal = FEEDBACK_ROUTER_SIGNAL_ELECTRICAL_ANGLE;
		signal < FEEDBACK_ROUTER_SIGNAL_COUNT; signal++)
	{
		status = FeedbackRouter_ValidateSource(config,
			FeedbackRouter_GetPrimarySource(config, signal), signal, false,
			issue);
		if (status != FEEDBACK_ROUTER_STATUS_OK)
			return status;
	}

	if (config->fallback_electrical_angle.kind !=
		FEEDBACK_ROUTER_SOURCE_NONE)
	{
		if (config->electrical_angle.kind == FEEDBACK_ROUTER_SOURCE_NONE)
		{
			FeedbackRouter_SetIssue(issue,
				FEEDBACK_ROUTER_STATUS_FALLBACK_WITHOUT_PRIMARY,
				FEEDBACK_ROUTER_SIGNAL_ELECTRICAL_ANGLE,
				config->fallback_electrical_angle, true);
			return FEEDBACK_ROUTER_STATUS_FALLBACK_WITHOUT_PRIMARY;
		}
		if (FeedbackRouter_SourceEquals(config->electrical_angle,
			config->fallback_electrical_angle))
		{
			FeedbackRouter_SetIssue(issue,
				FEEDBACK_ROUTER_STATUS_FALLBACK_DUPLICATES_PRIMARY,
				FEEDBACK_ROUTER_SIGNAL_ELECTRICAL_ANGLE,
				config->fallback_electrical_angle, true);
			return FEEDBACK_ROUTER_STATUS_FALLBACK_DUPLICATES_PRIMARY;
		}
	}

	return FeedbackRouter_ValidateSource(config,
		config->fallback_electrical_angle,
		FEEDBACK_ROUTER_SIGNAL_ELECTRICAL_ANGLE, true, issue);
}

FeedbackRouterStatus FeedbackRouter_Configure(FeedbackRouter *router,
	const FeedbackRouterConfig *config, FeedbackRouterValidationIssue *issue)
{
	FeedbackRouterStatus status;

	if (router == NULL)
	{
		FeedbackRouter_SetIssue(issue, FEEDBACK_ROUTER_STATUS_NULL_ARGUMENT,
			FEEDBACK_ROUTER_SIGNAL_COUNT, FeedbackRouter_NoSource(), false);
		return FEEDBACK_ROUTER_STATUS_NULL_ARGUMENT;
	}
	FeedbackRouter_Reset(router);
	status = FeedbackRouter_ValidateConfig(config, issue);
	if (status != FEEDBACK_ROUTER_STATUS_OK)
		return status;

	router->config = *config;
	router->is_configured = true;
	return FEEDBACK_ROUTER_STATUS_OK;
}

static float FeedbackRouter_GetValue(
	const FeedbackRouterNormalizedSample *sample, FeedbackRouterSignal signal)
{
	switch (signal)
	{
	case FEEDBACK_ROUTER_SIGNAL_ELECTRICAL_ANGLE:
		return sample->electrical_angle_rad;
	case FEEDBACK_ROUTER_SIGNAL_MOTOR_VELOCITY:
		return sample->motor_velocity_rad_s;
	case FEEDBACK_ROUTER_SIGNAL_MOTOR_POSITION:
		return sample->motor_position_rad;
	case FEEDBACK_ROUTER_SIGNAL_OUTPUT_POSITION:
		return sample->output_position_rad;
	case FEEDBACK_ROUTER_SIGNAL_CALIBRATION_REFERENCE:
		return sample->calibration_reference_rad;
	default:
		return 0.0f;
	}
}

static const FeedbackRouterNormalizedSample *FeedbackRouter_GetSample(
	const FeedbackRouterInput *input, FeedbackRouterSourceRef source)
{
	if (source.kind == FEEDBACK_ROUTER_SOURCE_ANGLE_SENSOR)
	{
		if (source.index >= input->angle_sensor_count)
			return NULL;
		return &input->angle_sensors[source.index];
	}
	if (source.kind == FEEDBACK_ROUTER_SOURCE_SENSORLESS_OBSERVER)
	{
		if (source.index != 0U || !input->sensorless_observer_present)
			return NULL;
		return &input->sensorless_observer;
	}
	return NULL;
}

static bool FeedbackRouter_TryRead(const FeedbackRouterInput *input,
	FeedbackRouterSourceRef source, FeedbackRouterSignal signal, float *value)
{
	const FeedbackRouterNormalizedSample *sample =
		FeedbackRouter_GetSample(input, source);
	FeedbackRouterSignalMask signal_mask = FEEDBACK_ROUTER_SIGNAL_MASK(signal);
	float candidate;

	if (sample == NULL || !sample->available ||
		(sample->valid_signals & signal_mask) == 0U)
	{
		return false;
	}
	candidate = FeedbackRouter_GetValue(sample, signal);
	if (!FeedbackRouter_IsFinite(candidate))
		return false;
	*value = candidate;
	return true;
}

static FeedbackRouterRoutedSignal *FeedbackRouter_GetOutputSignal(
	FeedbackRouterOutput *output, FeedbackRouterSignal signal)
{
	switch (signal)
	{
	case FEEDBACK_ROUTER_SIGNAL_ELECTRICAL_ANGLE:
		return &output->electrical_angle;
	case FEEDBACK_ROUTER_SIGNAL_MOTOR_VELOCITY:
		return &output->motor_velocity;
	case FEEDBACK_ROUTER_SIGNAL_MOTOR_POSITION:
		return &output->motor_position;
	case FEEDBACK_ROUTER_SIGNAL_OUTPUT_POSITION:
		return &output->output_position;
	case FEEDBACK_ROUTER_SIGNAL_CALIBRATION_REFERENCE:
		return &output->calibration_reference;
	default:
		return NULL;
	}
}

static void FeedbackRouter_RouteSignal(const FeedbackRouter *router,
	const FeedbackRouterInput *input, FeedbackRouterSignal signal,
	FeedbackRouterOutput *output)
{
	FeedbackRouterSourceRef primary = FeedbackRouter_GetPrimarySource(
		&router->config, signal);
	FeedbackRouterSourceRef fallback = FeedbackRouter_NoSource();
	FeedbackRouterRoutedSignal *routed =
		FeedbackRouter_GetOutputSignal(output, signal);
	FeedbackRouterSignalMask signal_mask = FEEDBACK_ROUTER_SIGNAL_MASK(signal);

	if (signal == FEEDBACK_ROUTER_SIGNAL_ELECTRICAL_ANGLE)
		fallback = router->config.fallback_electrical_angle;
	routed->selected_source = FeedbackRouter_NoSource();
	if (primary.kind == FEEDBACK_ROUTER_SOURCE_NONE)
	{
		routed->state = FEEDBACK_ROUTER_SIGNAL_STATE_DISABLED;
		return;
	}

	output->configured_signals |= signal_mask;
	if (FeedbackRouter_TryRead(input, primary, signal, &routed->value))
	{
		routed->selected_source = primary;
		routed->state = FEEDBACK_ROUTER_SIGNAL_STATE_PRIMARY;
		output->valid_signals |= signal_mask;
		return;
	}
	if (fallback.kind != FEEDBACK_ROUTER_SOURCE_NONE &&
		FeedbackRouter_TryRead(input, fallback, signal, &routed->value))
	{
		routed->selected_source = fallback;
		routed->state = FEEDBACK_ROUTER_SIGNAL_STATE_FALLBACK;
		output->valid_signals |= signal_mask;
		output->fallback_signals |= signal_mask;
		return;
	}

	routed->state = FEEDBACK_ROUTER_SIGNAL_STATE_UNAVAILABLE;
	output->unavailable_signals |= signal_mask;
}

FeedbackRouterStatus FeedbackRouter_Process(const FeedbackRouter *router,
	const FeedbackRouterInput *input, FeedbackRouterOutput *output)
{
	FeedbackRouterSignal signal;
	uint8_t index;

	if (router == NULL || input == NULL || output == NULL)
		return FEEDBACK_ROUTER_STATUS_NULL_ARGUMENT;
	memset(output, 0, sizeof(*output));
	output->health = FEEDBACK_ROUTER_HEALTH_UNAVAILABLE;
	if (!router->is_configured)
		return FEEDBACK_ROUTER_STATUS_NOT_CONFIGURED;
	if (input->angle_sensor_count > FEEDBACK_ROUTER_MAX_ANGLE_SENSORS ||
		input->angle_sensor_count > router->config.angle_sensor_count)
	{
		return FEEDBACK_ROUTER_STATUS_INVALID_INPUT_COUNT;
	}
	for (index = 0U; index < input->angle_sensor_count; index++)
	{
		if ((input->angle_sensors[index].valid_signals &
			~FEEDBACK_ROUTER_SIGNAL_MASK_ALL) != 0U)
		{
			return FEEDBACK_ROUTER_STATUS_INVALID_INPUT_MASK;
		}
	}
	if (input->sensorless_observer_present &&
		(input->sensorless_observer.valid_signals &
		 ~FEEDBACK_ROUTER_OBSERVER_SIGNAL_MASK) != 0U)
	{
		return FEEDBACK_ROUTER_STATUS_INVALID_INPUT_MASK;
	}

	for (signal = FEEDBACK_ROUTER_SIGNAL_ELECTRICAL_ANGLE;
		signal < FEEDBACK_ROUTER_SIGNAL_COUNT; signal++)
	{
		FeedbackRouter_RouteSignal(router, input, signal, output);
	}

	if (output->configured_signals == 0U)
		output->health = FEEDBACK_ROUTER_HEALTH_DISABLED;
	else if (output->unavailable_signals != 0U)
		output->health = FEEDBACK_ROUTER_HEALTH_UNAVAILABLE;
	else if (output->fallback_signals != 0U)
		output->health = FEEDBACK_ROUTER_HEALTH_DEGRADED;
	else
		output->health = FEEDBACK_ROUTER_HEALTH_HEALTHY;
	return FEEDBACK_ROUTER_STATUS_OK;
}
