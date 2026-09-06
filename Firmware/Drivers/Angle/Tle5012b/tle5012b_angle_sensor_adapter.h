#ifndef DRIVERS_ANGLE_TLE5012B_ANGLE_SENSOR_ADAPTER_H
#define DRIVERS_ANGLE_TLE5012B_ANGLE_SENSOR_ADAPTER_H

#include "bsp_angle_sensor.h"
#include "bsp_synchronous_serial.h"
#include "tle5012b.h"

/*
 * One context represents one physical TLE5012B. Multiple contexts may share
 * the immutable capabilities object while retaining independent transports,
 * acquisition state, samples, and sequence counters.
 */
typedef struct
{
	Tle5012b device;
	BspSynchronousSerialPort transport;
	BspAngleSensorSample latest_sample;
	BspAngleSampleStatusSet observed_status;
	uint32_t next_sequence;
	bool initialized;
	bool acquisition_started;
	bool has_latest_sample;
} Tle5012bAngleSensorAdapterContext;

/* On failure, port is cleared and context is left in a stopped safe state. */
bool Tle5012bAngleSensorAdapter_CreatePort(
	Tle5012bAngleSensorAdapterContext *context,
	const BspSynchronousSerialPort *transport,
	BspAngleSensorPort *port);

#endif
