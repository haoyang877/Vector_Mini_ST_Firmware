#include "Core/Application/Diagnostics/diagnostic_service.h"

#include <stddef.h>
#include <string.h>

bool DiagnosticService_Initialize(DiagnosticServiceContext *context,
	const BspResetReasonPort *reset_reason_port,
	const BspUniqueIdPort *device_identity_port,
	const ProductManifest *product_manifest,
	const TelemetryServiceContext *telemetry)
{
	size_t identity_length = 0U;

	if (context == NULL || reset_reason_port == NULL ||
		reset_reason_port->read_and_clear == NULL ||
		device_identity_port == NULL || product_manifest == NULL ||
		product_manifest->product_id == 0U || telemetry == NULL ||
		device_identity_port->read == NULL)
		return false;
	context->reset_reason_flags = reset_reason_port->read_and_clear(
		reset_reason_port->context);
	if (device_identity_port->read(device_identity_port->context,
		context->device_identity, sizeof(context->device_identity),
		&identity_length) != BSP_RESULT_OK || identity_length == 0U ||
		identity_length > sizeof(context->device_identity))
		return false;
	context->device_identity_length = (uint8_t)identity_length;
	context->product_manifest = *product_manifest;
	context->is_initialized = true;
	context->telemetry = telemetry;
	return true;
}

bool DiagnosticService_ReadSnapshot(const DiagnosticServiceContext *context,
	DeviceDiagnosticSnapshot *snapshot)
{
	if (snapshot == NULL || context == NULL || !context->is_initialized)
		return false;
	if (!TelemetryService_ReadSnapshot(context->telemetry, &snapshot->motor))
		return false;
	snapshot->product = context->product_manifest;
	snapshot->reset_reason_flags = context->reset_reason_flags;
	snapshot->device_identity_length = context->device_identity_length;
	memset(snapshot->device_identity, 0, sizeof(snapshot->device_identity));
	memcpy(snapshot->device_identity, context->device_identity,
		context->device_identity_length);
	return true;
}
