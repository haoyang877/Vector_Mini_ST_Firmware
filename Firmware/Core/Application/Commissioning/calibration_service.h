#ifndef CORE_APPLICATION_COMMISSIONING_CALIBRATION_SERVICE_H
#define CORE_APPLICATION_COMMISSIONING_CALIBRATION_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

#include "Core/Application/device_lifecycle.h"

typedef struct
{
	uint16_t minimum_current_offset_adc[3];
	uint16_t maximum_current_offset_adc[3];
} CalibrationCurrentOffsetLimits;

typedef struct
{
	DeviceLifecycleContext *lifecycle;
	CalibrationCurrentOffsetLimits current_offset_limits;
	uint32_t elapsed_ticks;
	uint32_t timeout_ticks;
} CalibrationServiceContext;

bool CalibrationService_Initialize(CalibrationServiceContext *context,
	DeviceLifecycleContext *lifecycle,
	const CalibrationCurrentOffsetLimits *current_offset_limits,
	uint32_t timeout_ticks);
bool CalibrationService_OwnsProcedure(ServiceProcedure procedure);
bool CalibrationService_Supervise1kHz(CalibrationServiceContext *context);
bool CalibrationService_AcceptCurrentOffsetResult(
	const CalibrationServiceContext *context, uint16_t phase_a_offset_adc,
	uint16_t phase_b_offset_adc, uint16_t phase_c_offset_adc);

#endif
