#ifndef COMMUNICATION_CAN_COMMAND_ROUTER_H
#define COMMUNICATION_CAN_COMMAND_ROUTER_H

#include "can_protocol_v1.h"

void CanCommandRouter_Handle(CanParameterId parameter_id, float value);

#endif
