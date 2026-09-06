#ifndef CORE_APPLICATION_MOTOR_CONTROL_FAULT_PORT_ADAPTER_H
#define CORE_APPLICATION_MOTOR_CONTROL_FAULT_PORT_ADAPTER_H

#include "Core/Application/Contracts/fault_command_port.h"

typedef struct MotorStateContext MotorStateContext;

FaultCommandPort FaultApplicationAdapter_CreatePort(
	MotorStateContext *motor_state);

#endif
