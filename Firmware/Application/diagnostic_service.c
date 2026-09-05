#include "diagnostic_service.h"

#include <stddef.h>

bool DiagnosticService_Initialize(DiagnosticServiceContext *context,
	const ResetReasonPort *reset_reason_port,
	const DeviceIdentityPort *device_identity_port,
	const TelemetryServiceContext *telemetry)
{
	if (context == NULL || reset_reason_port == NULL ||
		reset_reason_port->read_and_clear == NULL ||
		device_identity_port == NULL || telemetry == NULL ||
		device_identity_port->read_words == NULL)
		return false;
	context->reset_reason_flags = reset_reason_port->read_and_clear(
		reset_reason_port->context);
	if (!device_identity_port->read_words(device_identity_port->context,
		context->device_identity))
		return false;
	context->is_initialized = true;
	context->telemetry = telemetry;
	return true;
}

bool DiagnosticService_ReadSnapshot(const DiagnosticServiceContext *context,
	DeviceDiagnosticSnapshot *snapshot)
{
	const ProductManifest *manifest;

	if (snapshot == NULL || context == NULL || !context->is_initialized)
		return false;
	manifest = ProductManifest_Get();
	if (manifest == NULL ||
		!TelemetryService_ReadSnapshot(context->telemetry, &snapshot->motor))
		return false;
	snapshot->product = *manifest;
	snapshot->reset_reason_flags = context->reset_reason_flags;
	snapshot->device_identity[0] = context->device_identity[0];
	snapshot->device_identity[1] = context->device_identity[1];
	snapshot->device_identity[2] = context->device_identity[2];
	return true;
}
