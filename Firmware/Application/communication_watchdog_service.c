#include "communication_watchdog_service.h"

static CommunicationWatchdogServiceContext *ActiveContext;
#define WatchdogFaultPort (ActiveContext->port)
#define DisconnectFaultCode (ActiveContext->disconnect_fault_code)
#define WatchdogInitialized (ActiveContext != 0 && ActiveContext->is_initialized)

bool CommunicationWatchdogService_Initialize(
	CommunicationWatchdogServiceContext *context, const FaultCommandPort *port,
	uint32_t disconnect_fault_code)
{
	if (context == 0 || port == 0 || port->raise == 0 || port->clear == 0)
		return false;
	context->port = *port;
	context->disconnect_fault_code = disconnect_fault_code;
	context->is_initialized = true;
	ActiveContext = context;
	return true;
}

bool CommunicationWatchdogService_ReportDisconnected(void)
{
	return WatchdogInitialized && WatchdogFaultPort.raise(
		WatchdogFaultPort.context, DisconnectFaultCode);
}

bool CommunicationWatchdogService_ReportFrameReceived(void)
{
	return WatchdogInitialized && WatchdogFaultPort.clear(
		WatchdogFaultPort.context, DisconnectFaultCode);
}
