#ifndef APPLICATION_COMMUNICATION_WATCHDOG_SERVICE_H
#define APPLICATION_COMMUNICATION_WATCHDOG_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

#include "fault_command_port.h"

typedef struct
{
	FaultCommandPort port;
	uint32_t disconnect_fault_code;
	bool is_initialized;
} CommunicationWatchdogServiceContext;

bool CommunicationWatchdogService_Initialize(
	CommunicationWatchdogServiceContext *context, const FaultCommandPort *port,
	uint32_t disconnect_fault_code);
bool CommunicationWatchdogService_ReportDisconnected(
	CommunicationWatchdogServiceContext *context);
bool CommunicationWatchdogService_ReportFrameReceived(
	CommunicationWatchdogServiceContext *context);

#endif
