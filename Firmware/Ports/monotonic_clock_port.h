#ifndef PORTS_MONOTONIC_CLOCK_PORT_H
#define PORTS_MONOTONIC_CLOCK_PORT_H

#include <stdint.h>

typedef struct
{
	void *context;
	uint32_t (*read_ms)(void *context);
} MonotonicClockPort;

#endif
