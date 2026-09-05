#ifndef APPLICATION_CAN_CONFIGURATION_SERVICE_H
#define APPLICATION_CAN_CONFIGURATION_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

#include "can_configuration_port.h"

typedef struct
{
	CanConfigurationPort port;
	bool is_initialized;
} CanConfigurationServiceContext;

bool CanConfigurationService_Initialize(CanConfigurationServiceContext *context,
	const CanConfigurationPort *port);
bool CanConfigurationService_SetNodeId(uint8_t node_id);
uint8_t CanConfigurationService_GetNodeId(void);
bool CanConfigurationService_SetBitrateKbps(uint32_t bitrate_kbps);
uint32_t CanConfigurationService_GetBitrateKbps(void);
bool CanConfigurationService_SetHeartbeatMs(uint32_t heartbeat_ms);
uint32_t CanConfigurationService_GetHeartbeatMs(void);

#endif
