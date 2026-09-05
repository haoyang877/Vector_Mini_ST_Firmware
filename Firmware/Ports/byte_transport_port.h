#ifndef PORTS_BYTE_TRANSPORT_PORT_H
#define PORTS_BYTE_TRANSPORT_PORT_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
	void *context;
	bool (*transmit)(void *context, const uint8_t *data, uint16_t length);
	bool (*cancel_transmit)(void *context);
} ByteTransportPort;

#endif
