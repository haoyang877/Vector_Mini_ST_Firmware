#include "friction_identification_service.h"

bool FrictionIdentificationService_Initialize(
	FrictionIdentificationServiceContext *context,
	const FrictionIdentificationPort *port)
{
	if (context == 0 || port == 0 || port->read_status == 0 ||
		port->read_sample == 0 || port->apply_candidate == 0)
		return false;
	context->port = *port;
	context->is_initialized = true;
	return true;
}

bool FrictionIdentificationService_ReadStatus(
	const FrictionIdentificationServiceContext *context,
	FrictionIdentificationPortStatus *status)
{
	return context != 0 && context->is_initialized && status != 0 &&
		context->port.read_status(context->port.context, status);
}

bool FrictionIdentificationService_ReadSample(
	const FrictionIdentificationServiceContext *context, uint8_t index,
	FrictionIdentificationPortSample *sample)
{
	return context != 0 && context->is_initialized && sample != 0 &&
		context->port.read_sample(context->port.context, index, sample);
}

bool FrictionIdentificationService_ApplyCandidate(
	FrictionIdentificationServiceContext *context)
{
	return context != 0 && context->is_initialized &&
		context->port.apply_candidate(context->port.context);
}
