#ifndef PORTS_ROTOR_SENSOR_PORT_H
#define PORTS_ROTOR_SENSOR_PORT_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
	uint16_t angle_word;
	uint16_t raw_angle_q15;
} RotorSensorRawSample;

typedef struct
{
	void *context;
	bool (*initialize)(void *context);
	bool (*read_sample)(void *context, RotorSensorRawSample *sample);
} RotorSensorPort;

#endif
