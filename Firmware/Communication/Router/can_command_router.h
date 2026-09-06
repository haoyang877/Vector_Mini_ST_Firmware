#ifndef COMMUNICATION_CAN_COMMAND_ROUTER_H
#define COMMUNICATION_CAN_COMMAND_ROUTER_H

#include "can_protocol_v1.h"
#include "application_endpoints.h"
#include "Core/Communication/Can/can_response_service.h"

typedef struct
{
	ApplicationEndpoints *application;
	CanResponseServiceContext *response;
} CanCommandRouterContext;

bool CanCommandRouter_Initialize(CanCommandRouterContext *context,
	ApplicationEndpoints *application, CanResponseServiceContext *response);
void CanCommandRouter_Handle(CanCommandRouterContext *context,
	CanParameterId parameter_id, float value);

#endif
