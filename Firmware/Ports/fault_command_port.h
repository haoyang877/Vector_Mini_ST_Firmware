#ifndef PORTS_FAULT_COMMAND_PORT_H
#define PORTS_FAULT_COMMAND_PORT_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
	void *context;
	bool (*raise)(void *context, uint32_t fault_code);
	bool (*clear)(void *context, uint32_t fault_code);
} FaultCommandPort;

#endif
