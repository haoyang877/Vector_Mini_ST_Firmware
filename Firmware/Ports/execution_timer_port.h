#ifndef PORTS_EXECUTION_TIMER_PORT_H
#define PORTS_EXECUTION_TIMER_PORT_H

#include <stdint.h>

typedef struct
{
	void *context;
	uint32_t (*read_cycles)(void *context);
	uint32_t (*cycles_per_second)(void *context);
} ExecutionTimerPort;

#endif
