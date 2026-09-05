#include "communication_watchdog_service.h"

bool CommunicationWatchdogService_Initialize(
	CommunicationWatchdogServiceContext *context, const FaultCommandPort *port,
	uint32_t disconnect_fault_code)
{
	if (context == 0 || port == 0 || port->raise == 0 || port->clear == 0)
		return false;
	context->port = *port;
	context->disconnect_fault_code = disconnect_fault_code;
	context->is_initialized = true;
	return true;
}

bool CommunicationWatchdogService_ReportDisconnected(
	CommunicationWatchdogServiceContext *context)
{
	return context != 0 && context->is_initialized && context->port.raise(
		context->port.context, context->disconnect_fault_code);
}

bool CommunicationWatchdogService_ReportFrameReceived(
	CommunicationWatchdogServiceContext *context)
{
	return context != 0 && context->is_initialized && context->port.clear(
		context->port.context, context->disconnect_fault_code);
}
