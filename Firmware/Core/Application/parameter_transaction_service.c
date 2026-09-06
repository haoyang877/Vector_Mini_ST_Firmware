#include "Core/Application/parameter_transaction_service.h"

#include <stddef.h>

bool ParameterTransactionService_Initialize(
	ParameterTransactionServiceContext *context,
	const ParameterTransactionPort *port)
{
	if (context == NULL || port == NULL || port->get_operation == NULL ||
		port->begin == NULL || port->restore_defaults == NULL ||
		port->save == NULL || port->end == NULL || port->complete == NULL ||
		port->fail == NULL)
		return false;
	context->port = *port;
	context->is_initialized = true;
	context->operation_has_run = false;
	return true;
}

void ParameterTransactionService_RunBackground(
	ParameterTransactionServiceContext *context)
{
	ParameterTransactionOperation operation;
	bool succeeded;

	if (context == NULL || !context->is_initialized)
		return;
	operation = context->port.get_operation(context->port.context);
	if (operation == PARAMETER_TRANSACTION_NONE)
	{
		context->operation_has_run = false;
		return;
	}
	if (context->operation_has_run)
		return;
	context->operation_has_run = true;
	if (!context->port.begin(context->port.context))
	{
		context->port.fail(context->port.context);
		return;
	}
	succeeded = operation != PARAMETER_TRANSACTION_RESTORE_DEFAULTS ||
		context->port.restore_defaults(context->port.context);
	if (succeeded)
		succeeded = context->port.save(context->port.context);
	context->port.end(context->port.context);
	if (succeeded)
		context->port.complete(context->port.context);
	else
		context->port.fail(context->port.context);
}

bool ParameterTransactionService_IsIdle(
	const ParameterTransactionServiceContext *context)
{
	return context != 0 && context->is_initialized &&
		!context->operation_has_run && context->port.get_operation != 0 &&
		context->port.get_operation(context->port.context) ==
			PARAMETER_TRANSACTION_NONE;
}
