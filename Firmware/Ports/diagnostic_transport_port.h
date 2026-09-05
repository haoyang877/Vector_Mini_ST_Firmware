#ifndef PORTS_DIAGNOSTIC_TRANSPORT_PORT_H
#define PORTS_DIAGNOSTIC_TRANSPORT_PORT_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
	void *context;
	bool (*write)(void *context, const void *data, uint16_t size_bytes);
} DiagnosticTransportPort;

#endif
