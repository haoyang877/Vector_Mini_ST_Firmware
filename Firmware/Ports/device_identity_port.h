#ifndef PORTS_DEVICE_IDENTITY_PORT_H
#define PORTS_DEVICE_IDENTITY_PORT_H

#include <stdbool.h>
#include <stdint.h>

#define DEVICE_IDENTITY_WORD_COUNT 3U

typedef struct
{
	void *context;
	bool (*read_words)(void *context,
		uint32_t words[DEVICE_IDENTITY_WORD_COUNT]);
} DeviceIdentityPort;

#endif
