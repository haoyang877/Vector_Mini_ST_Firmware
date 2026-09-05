#ifndef PORTS_CAN_CONFIGURATION_PORT_H
#define PORTS_CAN_CONFIGURATION_PORT_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
	void *context;
	bool (*set_node_id)(void *context, uint8_t node_id);
	uint8_t (*get_node_id)(void *context);
	bool (*set_bitrate_kbps)(void *context, uint32_t bitrate_kbps);
	uint32_t (*get_bitrate_kbps)(void *context);
	bool (*set_heartbeat_ms)(void *context, uint32_t heartbeat_ms);
	uint32_t (*get_heartbeat_ms)(void *context);
} CanConfigurationPort;

#endif
