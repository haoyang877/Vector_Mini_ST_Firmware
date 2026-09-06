#ifndef CORE_APPLICATION_CONTRACTS_CAN_RESPONSE_PORT_H
#define CORE_APPLICATION_CONTRACTS_CAN_RESPONSE_PORT_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
	void *context;
	bool (*queue_response)(void *context, uint8_t parameter_id, float value);
} CanResponsePort;

#endif
