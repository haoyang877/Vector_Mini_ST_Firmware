#include "tle5012b_angle_sensor_adapter.h"

#include <string.h>

#if defined(__CC_ARM)
#pragma O3
#pragma Ospace
#endif

static const BspAngleSensorCapabilities Tle5012bAngleCapabilities =
{
	15U,
	true,
	false,
	false
};

static BspResult Tle5012bAngle_Initialize(void *context)
{
	Tle5012bAngleSensorAdapterContext *adapter =
		(Tle5012bAngleSensorAdapterContext *)context;

	if (adapter == NULL)
		return BSP_RESULT_INVALID_ARGUMENT;
	adapter->initialized = false;
	adapter->acquisition_started = false;
	adapter->has_latest_sample = false;
	adapter->observed_status = 0U;
	adapter->next_sequence = 1U;
	if (!Tle5012b_Initialize(&adapter->device, &adapter->transport))
	{
		adapter->observed_status = BSP_ANGLE_SAMPLE_SENSOR_FAULT;
		return BSP_RESULT_IO_ERROR;
	}
	adapter->initialized = true;
	return BSP_RESULT_OK;
}

static BspResult Tle5012bAngle_StartAcquisition(void *context)
{
	Tle5012bAngleSensorAdapterContext *adapter =
		(Tle5012bAngleSensorAdapterContext *)context;

	if (adapter == NULL)
		return BSP_RESULT_INVALID_ARGUMENT;
	if (!adapter->initialized)
		return BSP_RESULT_NOT_READY;
	adapter->acquisition_started = true;
	adapter->has_latest_sample = false;
	adapter->observed_status = 0U;
	return BSP_RESULT_OK;
}

static BspResult Tle5012bAngle_StopAcquisition(void *context)
{
	Tle5012bAngleSensorAdapterContext *adapter =
		(Tle5012bAngleSensorAdapterContext *)context;

	if (adapter == NULL)
		return BSP_RESULT_INVALID_ARGUMENT;
	if (!adapter->initialized)
		return BSP_RESULT_NOT_READY;
	adapter->acquisition_started = false;
	adapter->has_latest_sample = false;
	adapter->observed_status = 0U;
	return BSP_RESULT_OK;
}

static BspResult Tle5012bAngle_RequestSample(void *context)
{
	Tle5012bAngleSensorAdapterContext *adapter =
		(Tle5012bAngleSensorAdapterContext *)context;
	Tle5012bSample device_sample;
	Tle5012bReadStatus read_status;

	if (adapter == NULL)
		return BSP_RESULT_INVALID_ARGUMENT;
	if (!adapter->initialized || !adapter->acquisition_started)
		return BSP_RESULT_NOT_READY;

	read_status = Tle5012b_ReadAngle(&adapter->device, &device_sample);
	if (read_status != TLE5012B_READ_OK)
	{
		adapter->has_latest_sample = false;
		adapter->observed_status = BSP_ANGLE_SAMPLE_SENSOR_FAULT;
		if (read_status == TLE5012B_READ_INVALID_ARGUMENT)
			return BSP_RESULT_INVALID_ARGUMENT;
		if (read_status == TLE5012B_READ_NOT_INITIALIZED)
			return BSP_RESULT_NOT_READY;
		return BSP_RESULT_IO_ERROR;
	}

	/* raw_angle_q15 retains the deployed 0..65535 turns representation. */
	adapter->latest_sample.single_turn_position_u32 =
		(uint32_t)device_sample.raw_angle_q15 << 16U;
	adapter->latest_sample.turn_count = 0;
	adapter->latest_sample.velocity_rad_s = 0.0f;
	/* The device adapter owns no clock; POSITION_VALID does not imply time. */
	adapter->latest_sample.timestamp_us = 0U;
	adapter->latest_sample.sequence = adapter->next_sequence++;
	adapter->latest_sample.status = BSP_ANGLE_SAMPLE_POSITION_VALID;
	adapter->observed_status = adapter->latest_sample.status;
	adapter->has_latest_sample = true;
	return BSP_RESULT_OK;
}

static BspResult Tle5012bAngle_TryReadLatest(void *context,
	BspAngleSensorSample *sample)
{
	Tle5012bAngleSensorAdapterContext *adapter =
		(Tle5012bAngleSensorAdapterContext *)context;

	if (adapter == NULL || sample == NULL)
		return BSP_RESULT_INVALID_ARGUMENT;
	if (!adapter->initialized || !adapter->acquisition_started ||
		!adapter->has_latest_sample)
	{
		return BSP_RESULT_NOT_READY;
	}
	*sample = adapter->latest_sample;
	return BSP_RESULT_OK;
}

static BspAngleSampleStatusSet Tle5012bAngle_ReadStatus(void *context)
{
	Tle5012bAngleSensorAdapterContext *adapter =
		(Tle5012bAngleSensorAdapterContext *)context;

	return adapter != NULL ? adapter->observed_status :
		BSP_ANGLE_SAMPLE_SENSOR_FAULT;
}

bool Tle5012bAngleSensorAdapter_CreatePort(
	Tle5012bAngleSensorAdapterContext *context,
	const BspSynchronousSerialPort *transport,
	BspAngleSensorPort *port)
{
	if (port == NULL)
		return false;
	(void)memset(port, 0, sizeof(*port));
	if (context == NULL)
		return false;
	(void)memset(context, 0, sizeof(*context));
	if (transport == NULL || transport->initialize == NULL ||
		transport->execute == NULL)
	{
		return false;
	}

	context->transport = *transport;
	port->context = context;
	port->capabilities = &Tle5012bAngleCapabilities;
	port->initialize = Tle5012bAngle_Initialize;
	port->start_acquisition = Tle5012bAngle_StartAcquisition;
	port->stop_acquisition = Tle5012bAngle_StopAcquisition;
	port->request_sample = Tle5012bAngle_RequestSample;
	port->try_read_latest = Tle5012bAngle_TryReadLatest;
	port->read_status = Tle5012bAngle_ReadStatus;
	return true;
}
