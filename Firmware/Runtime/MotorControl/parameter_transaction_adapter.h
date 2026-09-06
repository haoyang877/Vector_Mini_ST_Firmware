#ifndef RUNTIME_PARAMETER_TRANSACTION_ADAPTER_H
#define RUNTIME_PARAMETER_TRANSACTION_ADAPTER_H

#include <stdbool.h>
#include <stdint.h>

#include "bsp_system.h"
#include "Core/Application/Contracts/parameter_transaction_port.h"
#include "parameter_persistence_adapter.h"
#include "parameter_snapshot.h"

typedef struct MotorStateContext MotorStateContext;

typedef struct
{
	BspCriticalSectionPort critical_section;
	ParameterPersistenceAdapterContext *persistence;
	ParameterSnapshotContext *snapshot;
	MotorStateContext *motor_state;
	BspCriticalSectionToken interrupt_state;
	bool transaction_is_open;
} ParameterTransactionAdapterContext;

ParameterTransactionPort ParameterTransactionAdapter_CreatePort(
	ParameterTransactionAdapterContext *context,
	const BspCriticalSectionPort *critical_section,
	ParameterPersistenceAdapterContext *persistence,
	ParameterSnapshotContext *snapshot, MotorStateContext *motor_state);

#endif
