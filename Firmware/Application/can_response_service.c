#include "can_response_service.h"

bool CanResponseService_Initialize(CanResponseServiceContext *context,
	const CanResponsePort *port)
{
	if (context == 0 || port == 0 || port->queue_response == 0)
		return false;
	context->port = *port;
	context->is_initialized = true;
	return true;
}

bool CanResponseService_Queue(CanResponseServiceContext *context,
	uint8_t parameter_id, float value)
{
	return context != 0 && context->is_initialized &&
		context->port.queue_response(context->port.context, parameter_id, value);
}
