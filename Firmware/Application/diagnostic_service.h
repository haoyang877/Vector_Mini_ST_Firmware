#ifndef APPLICATION_DIAGNOSTIC_SERVICE_H
#define APPLICATION_DIAGNOSTIC_SERVICE_H

#include <stdbool.h>

#include "product_manifest.h"
#include "telemetry_service.h"
#include "reset_reason_port.h"
#include "device_identity_port.h"

typedef struct
{
	ProductManifest product;
	MotorTelemetrySnapshot motor;
	uint32_t reset_reason_flags;
	uint32_t device_identity[DEVICE_IDENTITY_WORD_COUNT];
} DeviceDiagnosticSnapshot;

typedef struct
{
	uint32_t reset_reason_flags;
	uint32_t device_identity[DEVICE_IDENTITY_WORD_COUNT];
	bool is_initialized;
} DiagnosticServiceContext;

bool DiagnosticService_Initialize(DiagnosticServiceContext *context,
	const ResetReasonPort *reset_reason_port,
	const DeviceIdentityPort *device_identity_port);
bool DiagnosticService_ReadSnapshot(DeviceDiagnosticSnapshot *snapshot);

#endif
