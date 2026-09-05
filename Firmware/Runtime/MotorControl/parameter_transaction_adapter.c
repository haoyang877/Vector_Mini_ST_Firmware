#include "parameter_transaction_adapter.h"

#include "motor_control_types.h"
#include "motor_state_runtime.h"
#include "parameter_persistence_adapter.h"
#include "parameter_snapshot.h"

static ParameterTransactionOperation ParameterTransactionAdapter_GetOperation(
	void *context)
{
	ParameterTransactionAdapterContext *adapter =
		(ParameterTransactionAdapterContext *)context;
	ServiceProcedure procedure;
	if (adapter == 0 || adapter->motor_state == 0)
		return PARAMETER_TRANSACTION_NONE;
	if (MotorLifecycle_GetDeviceState(adapter->motor_state) != DEVICE_STATE_SERVICING ||
		MotorLifecycle_GetProcedureState(adapter->motor_state) != PROCEDURE_STATE_RUNNING)
		return PARAMETER_TRANSACTION_NONE;
	procedure = MotorLifecycle_GetServiceProcedure(adapter->motor_state);
	if (procedure == SERVICE_PROCEDURE_PARAMETER_SAVE)
		return PARAMETER_TRANSACTION_SAVE;
	if (procedure == SERVICE_PROCEDURE_RESTORE_DEFAULTS)
		return PARAMETER_TRANSACTION_RESTORE_DEFAULTS;
	return PARAMETER_TRANSACTION_NONE;
}

static bool ParameterTransactionAdapter_Begin(void *context)
{
	ParameterTransactionAdapterContext *adapter =
		(ParameterTransactionAdapterContext *)context;
	if (adapter == 0 || adapter->transaction_is_open)
		return false;
	MotorState_DisablePowerStage(adapter->motor_state);
	adapter->interrupt_state = adapter->critical_section.enter(
		adapter->critical_section.context);
	adapter->transaction_is_open = true;
	return true;
}

static bool ParameterTransactionAdapter_RestoreDefaults(void *context)
{
	ParameterTransactionAdapterContext *adapter =
		(ParameterTransactionAdapterContext *)context;
	if (adapter == 0 || adapter->snapshot == 0)
		return false;
	ParameterSnapshot_LoadDefaults(adapter->snapshot);
	return true;
}

static bool ParameterTransactionAdapter_Save(void *context)
{
	ParameterTransactionAdapterContext *adapter =
		(ParameterTransactionAdapterContext *)context;
	return adapter != 0 && adapter->persistence != 0 &&
		ParameterPersistenceAdapter_Save(adapter->persistence);
}

static void ParameterTransactionAdapter_End(void *context)
{
	ParameterTransactionAdapterContext *adapter =
		(ParameterTransactionAdapterContext *)context;
	if (adapter == 0 || !adapter->transaction_is_open)
		return;
	adapter->critical_section.exit(adapter->critical_section.context,
		adapter->interrupt_state);
	adapter->transaction_is_open = false;
}

static void ParameterTransactionAdapter_Complete(void *context)
{
	ParameterTransactionAdapterContext *adapter =
		(ParameterTransactionAdapterContext *)context;
	MotorLifecycle_ReportServiceComplete(adapter->motor_state, false);
}

static void ParameterTransactionAdapter_Fail(void *context)
{
	ParameterTransactionAdapterContext *adapter =
		(ParameterTransactionAdapterContext *)context;
	MotorLifecycle_ReportServiceFailed(adapter->motor_state);
	MotorState_RaiseFault(adapter->motor_state, MOTOR_FAULT_PARAMETER_STORE);
}

ParameterTransactionPort ParameterTransactionAdapter_CreatePort(
	ParameterTransactionAdapterContext *context,
	const CriticalSectionPort *critical_section,
	ParameterPersistenceAdapterContext *persistence,
	ParameterSnapshotContext *snapshot, MotorStateContext *motor_state)
{
	ParameterTransactionPort port = {0};
	if (context == 0 || critical_section == 0 ||
		critical_section->enter == 0 || critical_section->exit == 0 ||
		persistence == 0 || snapshot == 0 || motor_state == 0)
		return port;
	context->critical_section = *critical_section;
	context->persistence = persistence;
	context->snapshot = snapshot;
	context->motor_state = motor_state;
	context->interrupt_state = 0U;
	context->transaction_is_open = false;
	port.context = context;
	port.get_operation = ParameterTransactionAdapter_GetOperation;
	port.begin = ParameterTransactionAdapter_Begin;
	port.restore_defaults = ParameterTransactionAdapter_RestoreDefaults;
	port.save = ParameterTransactionAdapter_Save;
	port.end = ParameterTransactionAdapter_End;
	port.complete = ParameterTransactionAdapter_Complete;
	port.fail = ParameterTransactionAdapter_Fail;
	return port;
}
