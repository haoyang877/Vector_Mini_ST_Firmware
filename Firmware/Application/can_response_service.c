#include "can_response_service.h"

static CanResponseServiceContext *ActiveContext;
#define ResponsePort (ActiveContext->port)
#define ResponsePortInitialized (ActiveContext != 0 && ActiveContext->is_initialized)

bool CanResponseService_Initialize(CanResponseServiceContext *context,
	const CanResponsePort *port)
{
	if (context == 0 || port == 0 || port->queue_response == 0)
		return false;
	context->port = *port;
	context->is_initialized = true;
	ActiveContext = context;
	return true;
}

bool CanResponseService_Queue(uint8_t parameter_id, float value)
{
	return ResponsePortInitialized &&
		ResponsePort.queue_response(ResponsePort.context, parameter_id, value);
}
