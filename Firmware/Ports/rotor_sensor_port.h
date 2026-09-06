#ifndef PORTS_ROTOR_SENSOR_PORT_H
#define PORTS_ROTOR_SENSOR_PORT_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
	uint16_t raw_data_word;
	uint16_t raw_angle_q15;
} RotorSensorSample;

typedef enum
{
	ROTOR_SENSOR_READ_OK = 0,
	ROTOR_SENSOR_READ_TRANSPORT_ERROR = 1,
	ROTOR_SENSOR_READ_CRC_MISMATCH = 2,
	ROTOR_SENSOR_READ_FIELD_TOO_STRONG = 3,
	ROTOR_SENSOR_READ_FIELD_TOO_WEAK = 4,
	ROTOR_SENSOR_READ_FIELD_INVALID = 5,
	ROTOR_SENSOR_READ_OVERSPEED = 6,
	ROTOR_SENSOR_READ_DEVICE_RESET = 7,
	ROTOR_SENSOR_READ_DEVICE_SYSTEM_ERROR = 8,
	ROTOR_SENSOR_READ_DEVICE_INTERFACE_ERROR = 9,
	ROTOR_SENSOR_READ_INVALID_ANGLE = 10
} RotorSensorReadStatus;

typedef struct
{
	void *context;
	bool (*initialize)(void *context);
	RotorSensorReadStatus (*read_sample)(void *context,
		RotorSensorSample *sample);
} RotorSensorPort;

#endif
