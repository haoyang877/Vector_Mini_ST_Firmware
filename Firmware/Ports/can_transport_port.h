#ifndef PORTS_CAN_TRANSPORT_PORT_H
#define PORTS_CAN_TRANSPORT_PORT_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
	uint16_t identifier;
	uint8_t length;
	uint8_t data[8];
} CanTransportFrame;

typedef struct
{
	void *context;
	uint32_t maximum_bitrate_kbps;
	bool (*initialize)(void *context, uint8_t node_id);
	bool (*configure_node_id)(void *context, uint8_t node_id);
	bool (*configure_bitrate_kbps)(void *context, uint32_t bitrate_kbps);
	bool (*receive)(void *context, CanTransportFrame *frame);
	bool (*transmit)(void *context, const CanTransportFrame *frame);
} CanTransportPort;

#endif
