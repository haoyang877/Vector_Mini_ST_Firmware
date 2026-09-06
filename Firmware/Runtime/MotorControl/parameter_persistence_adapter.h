#ifndef RUNTIME_PARAMETER_PERSISTENCE_ADAPTER_H
#define RUNTIME_PARAMETER_PERSISTENCE_ADAPTER_H

#include <stdbool.h>
#include "parameter_snapshot.h"
#include "Core/Infrastructure/Parameters/parameter_manager.h"
#include "bsp_system.h"

typedef struct
{
	ParameterSnapshot transfer_buffer;
	ParameterManagerContext manager;
	BspNonvolatileStoragePort store;
	ParameterSnapshotContext *snapshot;
	bool manager_is_initialized;
} ParameterPersistenceAdapterContext;

bool ParameterPersistenceAdapter_Initialize(
	ParameterPersistenceAdapterContext *context,
	const BspNonvolatileStoragePort *store,
	ParameterSnapshotContext *snapshot);
bool ParameterPersistenceAdapter_Save(ParameterPersistenceAdapterContext *context);
void ParameterPersistenceAdapter_Load(ParameterPersistenceAdapterContext *context);

#endif
