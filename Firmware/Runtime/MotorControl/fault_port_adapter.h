#ifndef RUNTIME_FAULT_PORT_ADAPTER_H
#define RUNTIME_FAULT_PORT_ADAPTER_H

#include "fault_command_port.h"

typedef struct MotorStateContext MotorStateContext;

FaultCommandPort FaultApplicationAdapter_CreatePort(
	MotorStateContext *motor_state);

#endif
