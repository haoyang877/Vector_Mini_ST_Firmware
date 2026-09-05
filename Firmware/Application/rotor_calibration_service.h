#ifndef APPLICATION_ROTOR_CALIBRATION_SERVICE_H
#define APPLICATION_ROTOR_CALIBRATION_SERVICE_H

#include <stdbool.h>
#include <stdint.h>
#include "rotor_calibration_port.h"

typedef struct
{
	RotorCalibrationPort port;
	bool is_initialized;
} RotorCalibrationServiceContext;

bool RotorCalibrationService_Initialize(RotorCalibrationServiceContext *context,
	const RotorCalibrationPort *port);
bool RotorCalibrationService_SetReverse(bool reverse);
uint16_t RotorCalibrationService_GetEntryCount(void);
uint32_t RotorCalibrationService_GetCountsPerRevolution(void);
bool RotorCalibrationService_ReadEntry(uint16_t index,
	RotorCalibrationEntry *entry);

#endif
