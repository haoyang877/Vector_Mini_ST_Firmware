#ifndef RUNTIME_PARAMETER_PERSISTENCE_ADAPTER_H
#define RUNTIME_PARAMETER_PERSISTENCE_ADAPTER_H

#include <stdbool.h>
#include "parameter_snapshot.h"
#include "parameter_manager.h"
#include "parameter_store_port.h"

typedef struct
{
	ParameterSnapshot transfer_buffer;
	ParameterManagerContext manager;
	ParameterStorePort store;
	bool manager_is_initialized;
} ParameterPersistenceAdapterContext;

bool ParameterPersistenceAdapter_Initialize(
	ParameterPersistenceAdapterContext *context,
	const ParameterStorePort *store);
bool ParameterPersistenceAdapter_Save(void);
void ParameterPersistenceAdapter_Load(void);

#endif
