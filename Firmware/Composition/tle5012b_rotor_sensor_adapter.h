#ifndef FIRMWARE_COMPOSITION_TLE5012B_ROTOR_SENSOR_ADAPTER_H
#define FIRMWARE_COMPOSITION_TLE5012B_ROTOR_SENSOR_ADAPTER_H

#include "bsp_synchronous_serial.h"
#include "rotor_sensor_port.h"
#include "tle5012b.h"

typedef struct
{
	Tle5012b device;
	BspSynchronousSerialPort transport;
} Tle5012bRotorSensorAdapterContext;

RotorSensorPort Tle5012bRotorSensorAdapter_CreatePort(
	Tle5012bRotorSensorAdapterContext *context,
	const BspSynchronousSerialPort *transport);

#endif
