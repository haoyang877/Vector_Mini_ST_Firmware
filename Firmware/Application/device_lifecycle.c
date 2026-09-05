#include "device_lifecycle.h"

#include <stddef.h>

void DeviceLifecycle_Initialize(DeviceLifecycleContext *context)
{
	if (context == NULL)
		return;
	context->device_state = DEVICE_STATE_BOOTING;
	context->motor_control_mode = MOTOR_CONTROL_MODE_NONE;
	context->service_procedure = SERVICE_PROCEDURE_NONE;
	context->procedure_state = PROCEDURE_STATE_IDLE;
}

bool DeviceLifecycle_CompleteBoot(DeviceLifecycleContext *context)
{
	if (context == NULL || context->device_state != DEVICE_STATE_BOOTING)
		return false;
	context->device_state = DEVICE_STATE_STANDBY;
	return true;
}

bool DeviceLifecycle_RequestMotorControl(DeviceLifecycleContext *context,
	MotorControlMode mode)
{
	if (context == NULL || mode == MOTOR_CONTROL_MODE_NONE ||
		(context->device_state != DEVICE_STATE_STANDBY &&
		 context->device_state != DEVICE_STATE_ACTIVE))
		return false;
	context->device_state = DEVICE_STATE_ACTIVE;
	context->motor_control_mode = mode;
	context->service_procedure = SERVICE_PROCEDURE_NONE;
	context->procedure_state = PROCEDURE_STATE_IDLE;
	return true;
}

bool DeviceLifecycle_RequestService(DeviceLifecycleContext *context,
	ServiceProcedure procedure)
{
	if (context == NULL || procedure == SERVICE_PROCEDURE_NONE ||
		procedure >= SERVICE_PROCEDURE_COUNT ||
		(context->device_state != DEVICE_STATE_STANDBY &&
		 context->device_state != DEVICE_STATE_SERVICING))
		return false;
	context->device_state = DEVICE_STATE_SERVICING;
	context->motor_control_mode = MOTOR_CONTROL_MODE_NONE;
	context->service_procedure = procedure;
	context->procedure_state = PROCEDURE_STATE_PRECHECK;
	return true;
}

bool DeviceLifecycle_RequestStandby(DeviceLifecycleContext *context)
{
	if (context == NULL || context->device_state == DEVICE_STATE_BOOTING ||
		context->device_state == DEVICE_STATE_FAULTED ||
		context->device_state == DEVICE_STATE_UPDATING)
		return false;
	context->device_state = DEVICE_STATE_STANDBY;
	context->motor_control_mode = MOTOR_CONTROL_MODE_NONE;
	context->service_procedure = SERVICE_PROCEDURE_NONE;
	context->procedure_state = PROCEDURE_STATE_IDLE;
	return true;
}

bool DeviceLifecycle_BeginServiceRun(DeviceLifecycleContext *context)
{
	if (context == NULL || context->device_state != DEVICE_STATE_SERVICING ||
		context->procedure_state != PROCEDURE_STATE_PRECHECK)
		return false;
	context->procedure_state = PROCEDURE_STATE_RUNNING;
	return true;
}

bool DeviceLifecycle_BeginServiceVerification(DeviceLifecycleContext *context)
{
	if (context == NULL || context->device_state != DEVICE_STATE_SERVICING ||
		context->procedure_state != PROCEDURE_STATE_RUNNING)
		return false;
	context->procedure_state = PROCEDURE_STATE_VERIFYING;
	return true;
}

bool DeviceLifecycle_CompleteService(DeviceLifecycleContext *context)
{
	if (context == NULL || context->device_state != DEVICE_STATE_SERVICING ||
		context->procedure_state != PROCEDURE_STATE_VERIFYING)
		return false;
	context->procedure_state = PROCEDURE_STATE_COMPLETED;
	return true;
}

void DeviceLifecycle_FailService(DeviceLifecycleContext *context)
{
	if (context != NULL && context->device_state == DEVICE_STATE_SERVICING)
		context->procedure_state = PROCEDURE_STATE_FAILED;
}

void DeviceLifecycle_CancelService(DeviceLifecycleContext *context)
{
	if (context != NULL && context->device_state == DEVICE_STATE_SERVICING)
		context->procedure_state = PROCEDURE_STATE_CANCELLED;
}

bool DeviceLifecycle_RequestUpdating(DeviceLifecycleContext *context)
{
	if (context == NULL || context->device_state != DEVICE_STATE_STANDBY)
		return false;
	context->device_state = DEVICE_STATE_UPDATING;
	context->motor_control_mode = MOTOR_CONTROL_MODE_NONE;
	context->service_procedure = SERVICE_PROCEDURE_NONE;
	context->procedure_state = PROCEDURE_STATE_IDLE;
	return true;
}

bool DeviceLifecycle_AbortUpdating(DeviceLifecycleContext *context)
{
	if (context == NULL || context->device_state != DEVICE_STATE_UPDATING)
		return false;
	context->device_state = DEVICE_STATE_STANDBY;
	context->motor_control_mode = MOTOR_CONTROL_MODE_NONE;
	context->service_procedure = SERVICE_PROCEDURE_NONE;
	context->procedure_state = PROCEDURE_STATE_IDLE;
	return true;
}

void DeviceLifecycle_NotifyFault(DeviceLifecycleContext *context)
{
	if (context == NULL)
		return;
	context->device_state = DEVICE_STATE_FAULTED;
	context->motor_control_mode = MOTOR_CONTROL_MODE_NONE;
	context->service_procedure = SERVICE_PROCEDURE_NONE;
	context->procedure_state = PROCEDURE_STATE_FAILED;
}

bool DeviceLifecycle_ClearFault(DeviceLifecycleContext *context,
	bool all_faults_cleared)
{
	if (context == NULL || !all_faults_cleared ||
		context->device_state != DEVICE_STATE_FAULTED)
		return false;
	context->device_state = DEVICE_STATE_STANDBY;
	context->procedure_state = PROCEDURE_STATE_IDLE;
	return true;
}
