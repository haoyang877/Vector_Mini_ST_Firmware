#ifndef CORE_APPLICATION_DIAGNOSTICS_DIAGNOSTIC_SERVICE_H
#define CORE_APPLICATION_DIAGNOSTICS_DIAGNOSTIC_SERVICE_H

#include <stdbool.h>

#include "bsp_system.h"
#include "Core/Config/product_manifest.h"
#include "Core/Infrastructure/Telemetry/telemetry_service.h"

#define DIAGNOSTIC_DEVICE_IDENTITY_CAPACITY 12U

typedef struct
{
	ProductManifest product;
	MotorTelemetrySnapshot motor;
	uint32_t reset_reason_flags;
	uint8_t device_identity[DIAGNOSTIC_DEVICE_IDENTITY_CAPACITY];
	uint8_t device_identity_length;
} DeviceDiagnosticSnapshot;

typedef struct
{
	uint32_t reset_reason_flags;
	uint8_t device_identity[DIAGNOSTIC_DEVICE_IDENTITY_CAPACITY];
	uint8_t device_identity_length;
	ProductManifest product_manifest;
	const TelemetryServiceContext *telemetry;
	bool is_initialized;
} DiagnosticServiceContext;

bool DiagnosticService_Initialize(DiagnosticServiceContext *context,
	const BspResetReasonPort *reset_reason_port,
	const BspUniqueIdPort *device_identity_port,
	const ProductManifest *product_manifest,
	const TelemetryServiceContext *telemetry);
bool DiagnosticService_ReadSnapshot(const DiagnosticServiceContext *context,
	DeviceDiagnosticSnapshot *snapshot);

#endif
