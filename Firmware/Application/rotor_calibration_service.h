#ifndef APPLICATION_ROTOR_CALIBRATION_SERVICE_H
#define APPLICATION_ROTOR_CALIBRATION_SERVICE_H

#include <stdbool.h>
#include <stdint.h>
#include "Core/Application/Contracts/rotor_calibration_port.h"

typedef struct
{
	RotorCalibrationPort port;
	bool is_initialized;
} RotorCalibrationServiceContext;

bool RotorCalibrationService_Initialize(RotorCalibrationServiceContext *context,
	const RotorCalibrationPort *port);
bool RotorCalibrationService_SetReverse(RotorCalibrationServiceContext *context,
	bool reverse);
uint16_t RotorCalibrationService_GetEntryCount(
	const RotorCalibrationServiceContext *context);
uint32_t RotorCalibrationService_GetCountsPerRevolution(
	const RotorCalibrationServiceContext *context);
bool RotorCalibrationService_ReadEntry(
	const RotorCalibrationServiceContext *context, uint16_t index,
	RotorCalibrationEntry *entry);

#endif
