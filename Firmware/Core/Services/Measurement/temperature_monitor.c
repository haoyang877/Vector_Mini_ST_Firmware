#include "temperature_monitor.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define TEMPERATURE_MONITOR_KNOWN_SAMPLE_STATUS \
	(TEMPERATURE_MONITOR_SAMPLE_VALID | TEMPERATURE_MONITOR_SAMPLE_STALE | \
	 TEMPERATURE_MONITOR_SENSOR_OPEN | TEMPERATURE_MONITOR_SENSOR_SHORT | \
	 TEMPERATURE_MONITOR_SENSOR_FAULT)

static bool TemperatureMonitor_ConfigIsValid(
	const TemperatureMonitorConfig *config)
{
	if (config == NULL)
		return false;
	return !config->protection_enabled ||
		isfinite(config->trip_temperature_c);
}

static bool TemperatureMonitor_StatusIsKnown(
	TemperatureMonitorSampleStatusSet status)
{
	return (status & ~TEMPERATURE_MONITOR_KNOWN_SAMPLE_STATUS) == 0U;
}

static bool TemperatureMonitor_HasSensorFailure(
	TemperatureMonitorSampleStatusSet status)
{
	return (status & (TEMPERATURE_MONITOR_SENSOR_OPEN |
		TEMPERATURE_MONITOR_SENSOR_SHORT |
		TEMPERATURE_MONITOR_SENSOR_FAULT)) != 0U;
}

static TemperatureMonitorTripReasonSet TemperatureMonitor_EvaluateTrip(
	const TemperatureMonitorConfig *config,
	TemperatureMonitorSampleStatusSet status,
	bool temperature_is_finite,
	float temperature_c)
{
	TemperatureMonitorTripReasonSet reasons = TEMPERATURE_MONITOR_TRIP_NONE;
	bool sample_is_valid;

	if (!config->protection_enabled)
		return TEMPERATURE_MONITOR_TRIP_NONE;

	sample_is_valid = (status & TEMPERATURE_MONITOR_SAMPLE_VALID) != 0U &&
		temperature_is_finite;
	if (!sample_is_valid && config->trip_on_invalid_sample)
		reasons |= TEMPERATURE_MONITOR_TRIP_INVALID_SAMPLE;
	if ((status & TEMPERATURE_MONITOR_SAMPLE_STALE) != 0U &&
		config->trip_on_stale_sample)
		reasons |= TEMPERATURE_MONITOR_TRIP_STALE_SAMPLE;
	if ((status & TEMPERATURE_MONITOR_SENSOR_OPEN) != 0U &&
		config->trip_on_sensor_open)
		reasons |= TEMPERATURE_MONITOR_TRIP_SENSOR_OPEN;
	if ((status & TEMPERATURE_MONITOR_SENSOR_SHORT) != 0U &&
		config->trip_on_sensor_short)
		reasons |= TEMPERATURE_MONITOR_TRIP_SENSOR_SHORT;
	if ((status & TEMPERATURE_MONITOR_SENSOR_FAULT) != 0U &&
		config->trip_on_sensor_fault)
		reasons |= TEMPERATURE_MONITOR_TRIP_SENSOR_FAULT;

	/* A diagnostic or stale value is not trustworthy evidence of temperature. */
	if (sample_is_valid &&
		(status & TEMPERATURE_MONITOR_SAMPLE_STALE) == 0U &&
		!TemperatureMonitor_HasSensorFailure(status) &&
		temperature_c >= config->trip_temperature_c)
	{
		reasons |= TEMPERATURE_MONITOR_TRIP_OVER_TEMPERATURE;
	}
	return reasons;
}

static TemperatureMonitorStatus TemperatureMonitor_CheckSequence(
	const TemperatureMonitorContext *context, uint32_t sequence)
{
	uint32_t forward_distance;

	if (!context->has_last_sequence)
		return TEMPERATURE_MONITOR_STATUS_OK;
	forward_distance = sequence - context->last_sequence;
	if (forward_distance == 0U)
		return TEMPERATURE_MONITOR_STATUS_DUPLICATE_SEQUENCE;
	if (forward_distance >= UINT32_C(0x80000000))
		return TEMPERATURE_MONITOR_STATUS_OUT_OF_ORDER_SEQUENCE;
	return TEMPERATURE_MONITOR_STATUS_OK;
}

void TemperatureMonitor_Reset(TemperatureMonitorContext *context)
{
	if (context != NULL)
		(void)memset(context, 0, sizeof(*context));
}

TemperatureMonitorStatus TemperatureMonitor_Configure(
	TemperatureMonitorContext *context,
	const TemperatureMonitorConfig *config)
{
	if (context == NULL || config == NULL)
		return TEMPERATURE_MONITOR_STATUS_NULL_ARGUMENT;
	TemperatureMonitor_Reset(context);
	if (!TemperatureMonitor_ConfigIsValid(config))
		return TEMPERATURE_MONITOR_STATUS_INVALID_CONFIGURATION;
	context->config = *config;
	context->is_configured = true;
	return TEMPERATURE_MONITOR_STATUS_OK;
}

TemperatureMonitorStatus TemperatureMonitor_Process(
	TemperatureMonitorContext *context,
	const TemperatureMonitorInput *input,
	TemperatureMonitorOutput *output)
{
	TemperatureMonitorStatus status;
	TemperatureMonitorSampleStatusSet effective_status;
	TemperatureMonitorTripReasonSet trip_reasons;
	bool finite_temperature;
	bool trustworthy_sample;

	if (context == NULL || input == NULL || output == NULL)
		return TEMPERATURE_MONITOR_STATUS_NULL_ARGUMENT;
	if (!context->is_configured)
		return TEMPERATURE_MONITOR_STATUS_NOT_CONFIGURED;

	status = TemperatureMonitor_CheckSequence(context, input->sequence);
	effective_status = input->status;
	if (status == TEMPERATURE_MONITOR_STATUS_DUPLICATE_SEQUENCE ||
		status == TEMPERATURE_MONITOR_STATUS_OUT_OF_ORDER_SEQUENCE)
	{
		effective_status |= TEMPERATURE_MONITOR_SAMPLE_STALE;
		trip_reasons = TemperatureMonitor_EvaluateTrip(&context->config,
			effective_status, false, input->temperature_c);
		context->output.observed_sequence = input->sequence;
		context->output.observed_status = effective_status;
		context->output.trip_reasons = trip_reasons;
		context->output.trip_requested = trip_reasons !=
			TEMPERATURE_MONITOR_TRIP_NONE;
		*output = context->output;
		return status;
	}

	/* Consume a new sequence even when its payload is malformed. */
	context->last_sequence = input->sequence;
	context->has_last_sequence = true;
	finite_temperature = isfinite(input->temperature_c);
	if (!TemperatureMonitor_StatusIsKnown(input->status))
	{
		status = TEMPERATURE_MONITOR_STATUS_INVALID_SAMPLE_STATUS;
		/* Unknown quality information cannot establish a trustworthy value. */
		effective_status &= ~TEMPERATURE_MONITOR_SAMPLE_VALID;
	}
	else if ((input->status & TEMPERATURE_MONITOR_SAMPLE_VALID) != 0U &&
		!finite_temperature)
		status = TEMPERATURE_MONITOR_STATUS_NONFINITE_TEMPERATURE;

	trip_reasons = TemperatureMonitor_EvaluateTrip(&context->config,
		effective_status, finite_temperature, input->temperature_c);
	trustworthy_sample = status == TEMPERATURE_MONITOR_STATUS_OK &&
		(input->status & TEMPERATURE_MONITOR_SAMPLE_VALID) != 0U &&
		(input->status & TEMPERATURE_MONITOR_SAMPLE_STALE) == 0U &&
		!TemperatureMonitor_HasSensorFailure(input->status);
	if (trustworthy_sample)
	{
		context->output.latest_valid_temperature_c = input->temperature_c;
		context->output.latest_valid_sequence = input->sequence;
		context->output.has_valid_temperature = true;
	}
	context->output.observed_sequence = input->sequence;
	context->output.observed_status = input->status;
	context->output.trip_reasons = trip_reasons;
	context->output.trip_requested = trip_reasons !=
		TEMPERATURE_MONITOR_TRIP_NONE;
	*output = context->output;
	return status;
}
