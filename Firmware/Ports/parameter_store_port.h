#ifndef PORTS_PARAMETER_STORE_PORT_H
#define PORTS_PARAMETER_STORE_PORT_H

#include <stdbool.h>
#include <stdint.h>

#define PARAMETER_STORE_SLOT_COUNT 2U

typedef struct
{
	void *context;
	bool (*read)(void *context, uint8_t slot, uint32_t offset,
		void *destination, uint32_t size_bytes);
	bool (*erase)(void *context, uint8_t slot);
	bool (*program)(void *context, uint8_t slot, uint32_t offset,
		const void *source, uint32_t size_bytes);
	bool (*read_previous_format)(void *context, void *destination,
		uint32_t size_bytes);
} ParameterStorePort;

#endif
