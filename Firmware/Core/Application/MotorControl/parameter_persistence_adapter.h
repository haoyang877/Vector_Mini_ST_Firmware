#ifndef CORE_APPLICATION_MOTOR_CONTROL_PARAMETER_PERSISTENCE_ADAPTER_H
#define CORE_APPLICATION_MOTOR_CONTROL_PARAMETER_PERSISTENCE_ADAPTER_H

#include <stdbool.h>
#include "parameter_snapshot.h"
#include "Core/Infrastructure/Parameters/parameter_manager.h"
#include "bsp_system.h"

typedef struct
{
	uint32_t product_id;
	uint16_t hardware_compatibility_id;
	uint16_t motor_compatibility_id;
	uint16_t parameter_schema_version;
	uint32_t configuration_fingerprint;
	bool allow_erased_fingerprint_migration;
} ParameterPersistenceRuntimeConfig;

typedef struct
{
	ParameterSnapshot transfer_buffer;
	ParameterManagerContext manager;
	BspNonvolatileStoragePort store;
	ParameterSnapshotContext *snapshot;
	ParameterPersistenceRuntimeConfig runtime_config;
	bool manager_is_initialized;
} ParameterPersistenceAdapterContext;

bool ParameterPersistenceAdapter_Initialize(
	ParameterPersistenceAdapterContext *context,
	const BspNonvolatileStoragePort *store,
	ParameterSnapshotContext *snapshot,
	const ParameterPersistenceRuntimeConfig *runtime_config);
bool ParameterPersistenceAdapter_Save(ParameterPersistenceAdapterContext *context);
void ParameterPersistenceAdapter_Load(ParameterPersistenceAdapterContext *context);

#endif
