#ifndef APPLICATION_CALIBRATION_SERVICE_H
#define APPLICATION_CALIBRATION_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

#include "Core/Application/device_lifecycle.h"
#include "board_profile.h"

typedef struct
{
	DeviceLifecycleContext *lifecycle;
	const BoardProfile *board_profile;
	uint32_t elapsed_ticks;
	uint32_t timeout_ticks;
} CalibrationServiceContext;

bool CalibrationService_Initialize(CalibrationServiceContext *context,
	DeviceLifecycleContext *lifecycle, const BoardProfile *board_profile,
	uint32_t timeout_ticks);
bool CalibrationService_OwnsProcedure(ServiceProcedure procedure);
bool CalibrationService_Supervise1kHz(CalibrationServiceContext *context);
bool CalibrationService_AcceptCurrentOffsetResult(
	const CalibrationServiceContext *context, uint16_t phase_a_offset_adc,
	uint16_t phase_b_offset_adc, uint16_t phase_c_offset_adc);

#endif
