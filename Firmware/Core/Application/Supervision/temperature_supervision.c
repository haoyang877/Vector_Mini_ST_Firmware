#include "Core/Application/Supervision/temperature_supervision.h"

#include <stddef.h>
#include <string.h>

#define TEMPERATURE_SUPERVISION_KNOWN_BSP_STATUS \
	(BSP_TEMPERATURE_SAMPLE_VALID | BSP_TEMPERATURE_SAMPLE_STALE | \
	 BSP_TEMPERATURE_SENSOR_OPEN | BSP_TEMPERATURE_SENSOR_SHORT | \
	 BSP_TEMPERATURE_SENSOR_FAULT | BSP_TEMPERATURE_ADC_OVERRUN)

static bool TemperatureSupervision_PortIsValid(
	const BspTemperaturePort *port)
{
	return port != NULL && port->capabilities != NULL &&
		port->capabilities->endpoint_id != BSP_ENDPOINT_ID_NONE &&
		port->capabilities->availability == BSP_ENDPOINT_AVAILABLE &&
		port->initialize != NULL && port->try_read_latest != NULL &&
		port->read_status != NULL;
}

TemperatureMonitorSampleStatusSet TemperatureSupervision_MapSampleStatus(
	BspTemperatureStatusSet status)
{
	TemperatureMonitorSampleStatusSet mapped = 0U;

	if ((status & BSP_TEMPERATURE_SAMPLE_VALID) != 0U)
		mapped |= TEMPERATURE_MONITOR_SAMPLE_VALID;
	if ((status & BSP_TEMPERATURE_SAMPLE_STALE) != 0U)
		mapped |= TEMPERATURE_MONITOR_SAMPLE_STALE;
	if ((status & BSP_TEMPERATURE_SENSOR_OPEN) != 0U)
		mapped |= TEMPERATURE_MONITOR_SENSOR_OPEN;
	if ((status & BSP_TEMPERATURE_SENSOR_SHORT) != 0U)
		mapped |= TEMPERATURE_MONITOR_SENSOR_SHORT;
	if ((status & BSP_TEMPERATURE_SENSOR_FAULT) != 0U)
		mapped |= TEMPERATURE_MONITOR_SENSOR_FAULT;
	if ((status & BSP_TEMPERATURE_ADC_OVERRUN) != 0U)
	{
		mapped |= TEMPERATURE_MONITOR_SAMPLE_STALE |
			TEMPERATURE_MONITOR_SENSOR_FAULT;
	}
	if ((status & ~TEMPERATURE_SUPERVISION_KNOWN_BSP_STATUS) != 0U)
		mapped |= TEMPERATURE_MONITOR_SENSOR_FAULT;
	return mapped;
}

static void TemperatureSupervision_FinishRequest(
	TemperatureSupervisionContext *context)
{
	context->sample_pending = false;
	context->pending_elapsed_ms = 0U;
	context->request_countdown_ms = context->sample_period_ms;
	context->output.sample_pending = false;
	context->output.pending_elapsed_ms = 0U;
}

static TemperatureSupervisionStatus TemperatureSupervision_ProcessObservation(
	TemperatureSupervisionContext *context, float temperature_c,
	TemperatureMonitorSampleStatusSet domain_status,
	TemperatureSupervisionEvent event,
	TemperatureSupervisionStatus completion_status)
{
	TemperatureMonitorInput input;
	TemperatureMonitorOutput monitor_output;
	TemperatureMonitorStatus monitor_status;

	input.temperature_c = temperature_c;
	input.sequence = context->next_observation_sequence++;
	input.status = domain_status;
	monitor_status = TemperatureMonitor_Process(&context->monitor, &input,
		&monitor_output);
	context->output.latest_temperature = monitor_output;
	context->output.monitor_status = monitor_status;
	context->output.domain_status = domain_status;
	context->output.last_event = event;
	context->output.trip_requested = monitor_output.trip_requested;
	TemperatureSupervision_FinishRequest(context);
	if (completion_status == TEMPERATURE_SUPERVISION_STATUS_SAMPLE_PROCESSED &&
		monitor_status != TEMPERATURE_MONITOR_STATUS_OK)
	{
		return TEMPERATURE_SUPERVISION_STATUS_MONITOR_REJECTED;
	}
	return completion_status;
}

static TemperatureSupervisionStatus TemperatureSupervision_ProcessFailure(
	TemperatureSupervisionContext *context,
	TemperatureSupervisionEvent event,
	TemperatureSupervisionStatus completion_status)
{
	TemperatureMonitorSampleStatusSet status;

	status = TemperatureSupervision_MapSampleStatus(
		context->port.read_status(context->port.context));
	status &= ~TEMPERATURE_MONITOR_SAMPLE_VALID;
	status |= TEMPERATURE_MONITOR_SAMPLE_STALE |
		TEMPERATURE_MONITOR_SENSOR_FAULT;
	context->output.has_source_sample = false;
	context->output.source_sequence_stale = false;
	return TemperatureSupervision_ProcessObservation(context, 0.0f, status,
		event, completion_status);
}

static bool TemperatureSupervision_SourceSequenceIsStale(
	TemperatureSupervisionContext *context, uint32_t sequence)
{
	uint32_t forward_distance;

	if (!context->has_last_source_sequence)
	{
		context->last_source_sequence = sequence;
		context->has_last_source_sequence = true;
		return false;
	}
	forward_distance = sequence - context->last_source_sequence;
	if (forward_distance == 0U ||
		forward_distance >= UINT32_C(0x80000000))
	{
		return true;
	}
	context->last_source_sequence = sequence;
	return false;
}

static TemperatureSupervisionStatus TemperatureSupervision_ProcessSample(
	TemperatureSupervisionContext *context,
	const BspTemperatureSample *sample)
{
	BspTemperatureStatusSet bsp_status;
	TemperatureMonitorSampleStatusSet domain_status;
	bool sequence_is_stale;

	bsp_status = sample->status |
		context->port.read_status(context->port.context);
	domain_status = TemperatureSupervision_MapSampleStatus(bsp_status);
	sequence_is_stale = TemperatureSupervision_SourceSequenceIsStale(context,
		sample->sequence);
	if (sequence_is_stale)
		domain_status |= TEMPERATURE_MONITOR_SAMPLE_STALE;
	context->output.source_timestamp_us = sample->timestamp_us;
	context->output.source_sequence = sample->sequence;
	context->output.has_source_sample = true;
	context->output.source_sequence_stale = sequence_is_stale;
	return TemperatureSupervision_ProcessObservation(context,
		sample->temperature_c, domain_status,
		TEMPERATURE_SUPERVISION_EVENT_SAMPLE,
		TEMPERATURE_SUPERVISION_STATUS_SAMPLE_PROCESSED);
}

TemperatureSupervisionStatus TemperatureSupervision_Initialize(
	TemperatureSupervisionContext *context,
	const BspTemperaturePort *port,
	const TemperatureSupervisionConfig *config)
{
	TemperatureMonitorStatus monitor_status;
	BspResult bsp_result;

	if (context == NULL)
		return TEMPERATURE_SUPERVISION_STATUS_NULL_ARGUMENT;
	(void)memset(context, 0, sizeof(*context));
	if (port == NULL || config == NULL)
		return TEMPERATURE_SUPERVISION_STATUS_NULL_ARGUMENT;
	if (!TemperatureSupervision_PortIsValid(port))
		return TEMPERATURE_SUPERVISION_STATUS_INVALID_PORT;
	if (config->sample_period_ms == 0U || config->pending_timeout_ms == 0U)
		return TEMPERATURE_SUPERVISION_STATUS_INVALID_CONFIGURATION;
	monitor_status = TemperatureMonitor_Configure(&context->monitor,
		&config->monitor);
	if (monitor_status != TEMPERATURE_MONITOR_STATUS_OK)
		return TEMPERATURE_SUPERVISION_STATUS_INVALID_CONFIGURATION;
	context->port = *port;
	context->sample_period_ms = config->sample_period_ms;
	context->pending_timeout_ms = config->pending_timeout_ms;
	context->output.monitor_status = TEMPERATURE_MONITOR_STATUS_OK;
	bsp_result = context->port.initialize(context->port.context);
	context->output.last_bsp_result = bsp_result;
	if (bsp_result != BSP_RESULT_OK)
		return TEMPERATURE_SUPERVISION_STATUS_BSP_INITIALIZE_FAILED;
	context->is_initialized = true;
	return TEMPERATURE_SUPERVISION_STATUS_OK;
}

TemperatureSupervisionStatus TemperatureSupervision_Execute1kHz(
	TemperatureSupervisionContext *context,
	TemperatureSupervisionOutput *output)
{
	BspTemperatureSample sample;
	BspResult bsp_result;
	TemperatureSupervisionStatus status;

	if (context == NULL || output == NULL)
		return TEMPERATURE_SUPERVISION_STATUS_NULL_ARGUMENT;
	if (!context->is_initialized)
	{
		*output = context->output;
		return TEMPERATURE_SUPERVISION_STATUS_NOT_INITIALIZED;
	}

	if (context->sample_pending)
	{
		bsp_result = context->port.try_read_latest(context->port.context,
			&sample);
		context->output.last_bsp_result = bsp_result;
		if (bsp_result == BSP_RESULT_OK)
			status = TemperatureSupervision_ProcessSample(context, &sample);
		else if (bsp_result == BSP_RESULT_BUSY ||
			bsp_result == BSP_RESULT_NOT_READY)
		{
			context->pending_elapsed_ms++;
			context->output.pending_elapsed_ms =
				context->pending_elapsed_ms;
			if (context->pending_elapsed_ms >= context->pending_timeout_ms)
			{
				status = TemperatureSupervision_ProcessFailure(context,
					TEMPERATURE_SUPERVISION_EVENT_TIMEOUT,
					TEMPERATURE_SUPERVISION_STATUS_SAMPLE_TIMEOUT);
			}
			else
			{
				context->output.sample_pending = true;
				status = TEMPERATURE_SUPERVISION_STATUS_PENDING;
			}
		}
		else
		{
			status = TemperatureSupervision_ProcessFailure(context,
				TEMPERATURE_SUPERVISION_EVENT_PORT_FAILURE,
				TEMPERATURE_SUPERVISION_STATUS_BSP_READ_FAILED);
		}
		*output = context->output;
		return status;
	}

	if (context->request_countdown_ms > 0U)
	{
		context->request_countdown_ms--;
		if (context->request_countdown_ms > 0U)
		{
			*output = context->output;
			return TEMPERATURE_SUPERVISION_STATUS_OK;
		}
	}

	bsp_result = BSP_RESULT_OK;
	if (context->port.request_sample != NULL)
		bsp_result = context->port.request_sample(context->port.context);
	context->output.last_bsp_result = bsp_result;
	if (bsp_result == BSP_RESULT_OK || bsp_result == BSP_RESULT_BUSY ||
		bsp_result == BSP_RESULT_NOT_READY)
	{
		context->sample_pending = true;
		context->pending_elapsed_ms = 0U;
		context->output.sample_pending = true;
		context->output.pending_elapsed_ms = 0U;
		status = TEMPERATURE_SUPERVISION_STATUS_PENDING;
	}
	else
	{
		status = TemperatureSupervision_ProcessFailure(context,
			TEMPERATURE_SUPERVISION_EVENT_PORT_FAILURE,
			TEMPERATURE_SUPERVISION_STATUS_BSP_REQUEST_FAILED);
	}
	*output = context->output;
	return status;
}

const TemperatureSupervisionOutput *TemperatureSupervision_GetLatestOutput(
	const TemperatureSupervisionContext *context)
{
	if (context == NULL || !context->is_initialized)
		return NULL;
	return &context->output;
}
