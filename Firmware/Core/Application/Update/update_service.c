#include "Core/Application/Update/update_service.h"

#include <stddef.h>

bool UpdateService_Initialize(UpdateServiceContext *context,
	DeviceLifecycleContext *lifecycle,
	const MotorOutputSafetyPort *motor_output_safety,
	ParameterTransactionServiceContext *parameter_transactions,
	const UpdateControlPort *port)
{
	if (context == NULL || lifecycle == NULL || motor_output_safety == NULL ||
		motor_output_safety->disable_immediate == NULL ||
		motor_output_safety->outputs_are_enabled == NULL ||
		parameter_transactions == NULL ||
		port == NULL || port->candidate_is_compatible == NULL ||
		port->write_install_request == NULL || port->clear_request == NULL ||
		port->request_system_reset == NULL)
		return false;
	context->lifecycle = lifecycle;
	context->motor_output_safety = *motor_output_safety;
	context->parameter_transactions = parameter_transactions;
	context->port = *port;
	context->request_is_prepared = false;
	context->is_initialized = true;
	return true;
}

UpdateServiceResult UpdateService_PrepareInstall(UpdateServiceContext *context,
	uint32_t candidate_address, uint32_t candidate_size_bytes)
{
	if (context == NULL || !context->is_initialized)
		return UPDATE_SERVICE_NOT_AVAILABLE;
	if (context->lifecycle->device_state != DEVICE_STATE_STANDBY)
		return UPDATE_SERVICE_INVALID_STATE;
	if (!ParameterTransactionService_IsIdle(context->parameter_transactions))
		return UPDATE_SERVICE_INVALID_STATE;
	if (candidate_size_bytes == 0U ||
		!context->port.candidate_is_compatible(context->port.context,
			candidate_address, candidate_size_bytes))
		return UPDATE_SERVICE_INVALID_CANDIDATE;

	context->motor_output_safety.disable_immediate(
		context->motor_output_safety.context);
	if (context->motor_output_safety.outputs_are_enabled(
		context->motor_output_safety.context))
		return UPDATE_SERVICE_INTERNAL_ERROR;
	if (!DeviceLifecycle_RequestUpdating(context->lifecycle))
		return UPDATE_SERVICE_INVALID_STATE;
	if (!context->port.write_install_request(context->port.context,
		candidate_address, candidate_size_bytes))
	{
		(void)DeviceLifecycle_AbortUpdating(context->lifecycle);
		return UPDATE_SERVICE_INTERNAL_ERROR;
	}
	context->request_is_prepared = true;
	return UPDATE_SERVICE_ACCEPTED;
}

UpdateServiceResult UpdateService_CommitReset(UpdateServiceContext *context)
{
	if (context == NULL || !context->is_initialized)
		return UPDATE_SERVICE_NOT_AVAILABLE;
	if (!context->request_is_prepared ||
		context->lifecycle->device_state != DEVICE_STATE_UPDATING ||
		context->motor_output_safety.outputs_are_enabled(
			context->motor_output_safety.context))
		return UPDATE_SERVICE_INVALID_STATE;
	return context->port.request_system_reset(context->port.context) ?
		UPDATE_SERVICE_ACCEPTED : UPDATE_SERVICE_INTERNAL_ERROR;
}

bool UpdateService_Cancel(UpdateServiceContext *context)
{
	if (context == NULL || !context->is_initialized ||
		context->lifecycle->device_state != DEVICE_STATE_UPDATING)
		return false;
	context->port.clear_request(context->port.context);
	context->request_is_prepared = false;
	return DeviceLifecycle_AbortUpdating(context->lifecycle);
}
