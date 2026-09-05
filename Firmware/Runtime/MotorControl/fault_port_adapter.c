#include "fault_port_adapter.h"

#include "motor_control_types.h"
#include "motor_state_runtime.h"

static bool FaultApplicationAdapter_Raise(void *context, uint32_t fault_code)
{
	MotorStateContext *motor_state = (MotorStateContext *)context;
	if (motor_state == 0)
		return false;
	if (fault_code > (uint32_t)MOTOR_FAULT_POWER_STAGE)
		return false;
	MotorState_RaiseFault(motor_state, (MotorFaultCode)fault_code);
	return true;
}

static bool FaultApplicationAdapter_Clear(void *context, uint32_t fault_code)
{
	MotorStateContext *motor_state = (MotorStateContext *)context;
	if (motor_state == 0)
		return false;
	if (fault_code > (uint32_t)MOTOR_FAULT_POWER_STAGE)
		return false;
	MotorState_ClearFault(motor_state, (MotorFaultCode)fault_code);
	return true;
}

FaultCommandPort FaultApplicationAdapter_CreatePort(
	MotorStateContext *motor_state)
{
	FaultCommandPort port;
	port.context = motor_state;
	port.raise = FaultApplicationAdapter_Raise;
	port.clear = FaultApplicationAdapter_Clear;
	return port;
}
