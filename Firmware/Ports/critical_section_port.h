#ifndef PORTS_CRITICAL_SECTION_PORT_H
#define PORTS_CRITICAL_SECTION_PORT_H

#include <stdint.h>

typedef struct
{
	void *context;
	uint32_t (*enter)(void *context);
	void (*exit)(void *context, uint32_t state);
} CriticalSectionPort;

#endif
