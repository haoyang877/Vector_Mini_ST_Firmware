#include "fault_port_adapter.h"

#include "motor_control_types.h"
#include "motor_state_runtime.h"

static bool FaultApplicationAdapter_Raise(void *context, uint32_t fault_code)
{
	(void)context;
	if (fault_code > (uint32_t)MOTOR_FAULT_POWER_STAGE)
		return false;
	MotorState_RaiseFault((MotorFaultCode)fault_code);
	return true;
}

static bool FaultApplicationAdapter_Clear(void *context, uint32_t fault_code)
{
	(void)context;
	if (fault_code > (uint32_t)MOTOR_FAULT_POWER_STAGE)
		return false;
	MotorState_ClearFault((MotorFaultCode)fault_code);
	return true;
}

FaultCommandPort FaultApplicationAdapter_CreatePort(void)
{
	FaultCommandPort port;
	port.context = 0;
	port.raise = FaultApplicationAdapter_Raise;
	port.clear = FaultApplicationAdapter_Clear;
	return port;
}
