#ifndef PORTS_UPDATE_CONTROL_PORT_H
#define PORTS_UPDATE_CONTROL_PORT_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
	void *context;
	bool (*candidate_is_compatible)(void *context, uint32_t address,
		uint32_t size_bytes);
	bool (*write_install_request)(void *context, uint32_t address,
		uint32_t size_bytes);
	void (*clear_request)(void *context);
	bool (*request_system_reset)(void *context);
} UpdateControlPort;

#endif
