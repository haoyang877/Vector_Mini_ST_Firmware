#ifndef APPLICATION_CAN_RESPONSE_SERVICE_H
#define APPLICATION_CAN_RESPONSE_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

#include "can_response_port.h"

typedef struct
{
	CanResponsePort port;
	bool is_initialized;
} CanResponseServiceContext;

bool CanResponseService_Initialize(CanResponseServiceContext *context,
	const CanResponsePort *port);
bool CanResponseService_Queue(CanResponseServiceContext *context,
	uint8_t parameter_id, float value);

#endif
