#include "friction_identification_service.h"

static FrictionIdentificationServiceContext *ActiveContext;

bool FrictionIdentificationService_Initialize(
	FrictionIdentificationServiceContext *context,
	const FrictionIdentificationPort *port)
{
	if (context == 0 || port == 0 || port->read_status == 0 ||
		port->read_sample == 0 || port->apply_candidate == 0)
		return false;
	context->port = *port;
	context->is_initialized = true;
	ActiveContext = context;
	return true;
}

bool FrictionIdentificationService_ReadStatus(
	FrictionIdentificationPortStatus *status)
{
	return ActiveContext != 0 && ActiveContext->is_initialized && status != 0 &&
		ActiveContext->port.read_status(ActiveContext->port.context, status);
}

bool FrictionIdentificationService_ReadSample(uint8_t index,
	FrictionIdentificationPortSample *sample)
{
	return ActiveContext != 0 && ActiveContext->is_initialized && sample != 0 &&
		ActiveContext->port.read_sample(ActiveContext->port.context, index, sample);
}

bool FrictionIdentificationService_ApplyCandidate(void)
{
	return ActiveContext != 0 && ActiveContext->is_initialized &&
		ActiveContext->port.apply_candidate(ActiveContext->port.context);
}
