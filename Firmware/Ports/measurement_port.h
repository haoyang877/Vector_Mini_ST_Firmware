#ifndef PORTS_MEASUREMENT_PORT_H
#define PORTS_MEASUREMENT_PORT_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
	uint16_t phase_a_adc;
	uint16_t phase_b_adc;
	uint16_t phase_c_adc;
	uint16_t bus_voltage_adc;
	uint16_t temperature_adc;
} MeasurementRawSample;

typedef struct
{
	void *context;
	bool (*read_raw_sample)(void *context, MeasurementRawSample *sample);
} MeasurementPort;

#endif
