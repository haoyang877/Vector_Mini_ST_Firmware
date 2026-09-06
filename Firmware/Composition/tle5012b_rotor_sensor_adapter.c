#include "tle5012b_rotor_sensor_adapter.h"

#if defined(__CC_ARM)
#pragma O3
#pragma Ospace
#endif

static bool Tle5012bRotorSensorAdapter_Initialize(void *context)
{
	Tle5012bRotorSensorAdapterContext *adapter =
		(Tle5012bRotorSensorAdapterContext *)context;

	return adapter != 0 &&
		Tle5012b_Initialize(&adapter->device, &adapter->transport);
}

static RotorSensorReadStatus Tle5012bRotorSensorAdapter_Read(void *context,
	RotorSensorSample *sample)
{
	Tle5012bRotorSensorAdapterContext *adapter =
		(Tle5012bRotorSensorAdapterContext *)context;
	Tle5012bSample device_sample;
	Tle5012bReadStatus status;

	if (adapter == 0 || sample == 0)
		return ROTOR_SENSOR_READ_TRANSPORT_ERROR;

	status = Tle5012b_ReadAngle(&adapter->device, &device_sample);
	if (status != TLE5012B_READ_OK)
		return ROTOR_SENSOR_READ_TRANSPORT_ERROR;

	sample->raw_data_word = device_sample.angle_word;
	sample->raw_angle_q15 = device_sample.raw_angle_q15;
	return ROTOR_SENSOR_READ_OK;
}

RotorSensorPort Tle5012bRotorSensorAdapter_CreatePort(
	Tle5012bRotorSensorAdapterContext *context,
	const BspSynchronousSerialPort *transport)
{
	RotorSensorPort port = { 0 };

	if (context == 0 || transport == 0)
		return port;
	context->transport = *transport;
	port.context = context;
	port.initialize = Tle5012bRotorSensorAdapter_Initialize;
	port.read_sample = Tle5012bRotorSensorAdapter_Read;
	return port;
}
