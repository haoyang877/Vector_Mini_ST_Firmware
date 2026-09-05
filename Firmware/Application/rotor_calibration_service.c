#include "rotor_calibration_service.h"

bool RotorCalibrationService_Initialize(RotorCalibrationServiceContext *context,
	const RotorCalibrationPort *port)
{
	if (context == 0 || port == 0 || port->entry_count == 0U ||
		port->counts_per_revolution == 0U || port->set_reverse == 0 ||
		port->read_entry == 0)
		return false;

	context->port = *port;
	context->is_initialized = true;
	return true;
}

bool RotorCalibrationService_SetReverse(RotorCalibrationServiceContext *context,
	bool reverse)
{
	return context != 0 && context->is_initialized &&
		context->port.set_reverse(context->port.context, reverse);
}

uint16_t RotorCalibrationService_GetEntryCount(
	const RotorCalibrationServiceContext *context)
{
	return context != 0 && context->is_initialized ?
		context->port.entry_count : 0U;
}

uint32_t RotorCalibrationService_GetCountsPerRevolution(
	const RotorCalibrationServiceContext *context)
{
	return context != 0 && context->is_initialized ?
		context->port.counts_per_revolution : 0U;
}

bool RotorCalibrationService_ReadEntry(
	const RotorCalibrationServiceContext *context, uint16_t index,
	RotorCalibrationEntry *entry)
{
	return context != 0 && context->is_initialized && entry != 0 &&
		index < context->port.entry_count &&
		context->port.read_entry(context->port.context, index, entry);
}
