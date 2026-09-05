#ifndef PORTS_ROTOR_CALIBRATION_PORT_H
#define PORTS_ROTOR_CALIBRATION_PORT_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
	uint16_t raw_angle_q15;
	int32_t error_q15;
} RotorCalibrationEntry;

typedef struct
{
	void *context;
	uint16_t entry_count;
	uint32_t counts_per_revolution;
	bool (*set_reverse)(void *context, bool reverse);
	bool (*read_entry)(void *context, uint16_t index,
		RotorCalibrationEntry *entry);
} RotorCalibrationPort;

#endif
