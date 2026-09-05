#ifndef RUNTIME_PARAMETER_TRANSACTION_ADAPTER_H
#define RUNTIME_PARAMETER_TRANSACTION_ADAPTER_H

#include <stdbool.h>
#include <stdint.h>

#include "critical_section_port.h"
#include "parameter_transaction_port.h"
#include "parameter_persistence_adapter.h"
#include "parameter_snapshot.h"

typedef struct MotorStateContext MotorStateContext;

typedef struct
{
	CriticalSectionPort critical_section;
	ParameterPersistenceAdapterContext *persistence;
	ParameterSnapshotContext *snapshot;
	MotorStateContext *motor_state;
	uint32_t interrupt_state;
	bool transaction_is_open;
} ParameterTransactionAdapterContext;

ParameterTransactionPort ParameterTransactionAdapter_CreatePort(
	ParameterTransactionAdapterContext *context,
	const CriticalSectionPort *critical_section,
	ParameterPersistenceAdapterContext *persistence,
	ParameterSnapshotContext *snapshot, MotorStateContext *motor_state);

#endif
