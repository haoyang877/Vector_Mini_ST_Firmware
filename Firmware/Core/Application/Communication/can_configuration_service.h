#ifndef CORE_APPLICATION_COMMUNICATION_CAN_CONFIGURATION_SERVICE_H
#define CORE_APPLICATION_COMMUNICATION_CAN_CONFIGURATION_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

#include "Core/Application/Contracts/can_configuration_port.h"

typedef struct
{
	CanConfigurationPort port;
	bool is_initialized;
} CanConfigurationServiceContext;

bool CanConfigurationService_Initialize(CanConfigurationServiceContext *context,
	const CanConfigurationPort *port);
bool CanConfigurationService_SetNodeId(CanConfigurationServiceContext *context,
	uint8_t node_id);
uint8_t CanConfigurationService_GetNodeId(
	const CanConfigurationServiceContext *context);
bool CanConfigurationService_SetBitrateKbps(
	CanConfigurationServiceContext *context, uint32_t bitrate_kbps);
uint32_t CanConfigurationService_GetBitrateKbps(
	const CanConfigurationServiceContext *context);
bool CanConfigurationService_SetHeartbeatMs(
	CanConfigurationServiceContext *context, uint32_t heartbeat_ms);
uint32_t CanConfigurationService_GetHeartbeatMs(
	const CanConfigurationServiceContext *context);

#endif
