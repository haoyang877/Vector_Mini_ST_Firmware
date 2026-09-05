#include "rotor_calibration_service.h"

static RotorCalibrationServiceContext *ActiveContext;
#define CalibrationPort (ActiveContext->port)
#define CalibrationPortInitialized (ActiveContext != 0 && ActiveContext->is_initialized)

bool RotorCalibrationService_Initialize(RotorCalibrationServiceContext *context,
	const RotorCalibrationPort *port)
{
	if (context == 0 || port == 0 || port->entry_count == 0U ||
		port->counts_per_revolution == 0U || port->set_reverse == 0 ||
		port->read_entry == 0)
		return false;

	context->port = *port;
	context->is_initialized = true;
	ActiveContext = context;
	return true;
}

bool RotorCalibrationService_SetReverse(bool reverse)
{
	return CalibrationPortInitialized &&
		CalibrationPort.set_reverse(CalibrationPort.context, reverse);
}

uint16_t RotorCalibrationService_GetEntryCount(void)
{
	return CalibrationPortInitialized ? CalibrationPort.entry_count : 0U;
}

uint32_t RotorCalibrationService_GetCountsPerRevolution(void)
{
	return CalibrationPortInitialized ?
		CalibrationPort.counts_per_revolution : 0U;
}

bool RotorCalibrationService_ReadEntry(uint16_t index,
	RotorCalibrationEntry *entry)
{
	return CalibrationPortInitialized && entry != 0 &&
		index < CalibrationPort.entry_count &&
		CalibrationPort.read_entry(CalibrationPort.context, index, entry);
}
