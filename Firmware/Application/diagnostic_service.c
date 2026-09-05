#include "diagnostic_service.h"

#include <stddef.h>

static DiagnosticServiceContext *ActiveContext;

bool DiagnosticService_Initialize(DiagnosticServiceContext *context,
	const ResetReasonPort *reset_reason_port,
	const DeviceIdentityPort *device_identity_port)
{
	if (context == NULL || reset_reason_port == NULL ||
		reset_reason_port->read_and_clear == NULL ||
		device_identity_port == NULL ||
		device_identity_port->read_words == NULL)
		return false;
	context->reset_reason_flags = reset_reason_port->read_and_clear(
		reset_reason_port->context);
	if (!device_identity_port->read_words(device_identity_port->context,
		context->device_identity))
		return false;
	context->is_initialized = true;
	ActiveContext = context;
	return true;
}

bool DiagnosticService_ReadSnapshot(DeviceDiagnosticSnapshot *snapshot)
{
	const ProductManifest *manifest;

	if (snapshot == NULL || ActiveContext == NULL ||
		!ActiveContext->is_initialized)
		return false;
	manifest = ProductManifest_Get();
	if (manifest == NULL || !TelemetryService_ReadSnapshot(&snapshot->motor))
		return false;
	snapshot->product = *manifest;
	snapshot->reset_reason_flags = ActiveContext->reset_reason_flags;
	snapshot->device_identity[0] = ActiveContext->device_identity[0];
	snapshot->device_identity[1] = ActiveContext->device_identity[1];
	snapshot->device_identity[2] = ActiveContext->device_identity[2];
	return true;
}
