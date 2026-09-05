#include "supervisor_task.h"

#include <stdint.h>
#include "motor_control_types.h"
#include "motor_control_runtime.h"
#include "interface_can.h"
#include "interface_usb.h"
#include "led.h"
#include "rgb.h"
#include "telemetry_service.h"
#include "motor_state_runtime.h"

static SupervisorTaskContext *ActiveContext;
#define Led_Cnt (ActiveContext->led_ticks)
#define RGB_Cnt (ActiveContext->rgb_ticks)
#define CANBRSwitching_Cnt (ActiveContext->can_bitrate_ticks)

bool SupervisorTask_Initialize(SupervisorTaskContext *context,
	const DiagnosticTransportPort *diagnostic_transport)
{
	if (context == 0 || diagnostic_transport == 0 ||
		diagnostic_transport->write == 0)
		return false;
	context->led_ticks = 0U;
	context->rgb_ticks = 0U;
	context->can_bitrate_ticks = 0U;
	context->diagnostic_ticks = 0U;
	context->published_diagnostic_buffer = 0U;
	context->diagnostic_pending = false;
	context->diagnostic_transport = *diagnostic_transport;
	context->is_initialized = true;
	ActiveContext = context;
	return true;
}

/**
	* @brief  System supervision task executed at 1 kHz
			  update USB print, LEDs, RGB, encoder state and CAN status
 **/
void SupervisorTask_Execute1kHz(void)
{
	DeviceState device_state;
	MotorControlMode control_mode;
	ServiceProcedure service_procedure;
	if (ActiveContext == 0 || !ActiveContext->is_initialized)
		return;

	MotorLifecycle_Supervise1kHz();
	MotorControlRuntime_PublishTelemetry();
	if (++ActiveContext->diagnostic_ticks >= 10U)
	{
		uint8_t next_buffer = ActiveContext->published_diagnostic_buffer == 0U ?
			1U : 0U;
		ActiveContext->diagnostic_ticks = 0U;
		if (MotorControlRuntime_ReadDiagnosticFrame(
			&ActiveContext->diagnostic_buffers[next_buffer]))
		{
			ActiveContext->published_diagnostic_buffer = next_buffer;
			ActiveContext->diagnostic_pending = true;
		}
	}
	device_state = MotorLifecycle_GetDeviceState();
	control_mode = MotorLifecycle_GetControlMode();
	service_procedure = MotorLifecycle_GetServiceProcedure();
	UsbInterface_UpdateTelemetryStream();

	if (MotorState_HasChanged())
	{
		if (MotorState_GetPrimaryFault() == MOTOR_FAULT_NONE)
			LED_SetState(0, MotorLifecycle_GetProtocolActionCode());
		else
			LED_SetState(1, (uint8_t)MotorState_GetPrimaryFault());
		MotorState_ClearChangeFlag();
	}
	
	if(++Led_Cnt >= 200)
	{
		LED_Task();
		Led_Cnt=0;
	}
		
	if(++RGB_Cnt >= 50)
	{
		if (device_state == DEVICE_STATE_SERVICING &&
			service_procedure != SERVICE_PROCEDURE_NONE)
			Set_RGB_BreathingColor(PURPLE);
		else switch(control_mode)
		{
			case MOTOR_CONTROL_MODE_CURRENT:
				Set_RGB_BreathingColor(GREEN);
			break;
			
			case MOTOR_CONTROL_MODE_SPEED:
				Set_RGB_BreathingColor(CYAN);
			break;

			case MOTOR_CONTROL_MODE_SENSORLESS_SPEED:
				Set_RGB_BreathingColor(YELLOW);
			break;
			
			case MOTOR_CONTROL_MODE_POSITION_CASCADE:
				Set_RGB_BreathingColor(BLUE);
			break;

			case MOTOR_CONTROL_MODE_POSITION_IMPEDANCE:
				Set_RGB_BreathingColor(WHITE);
			break;
			
			default:
				Set_RGB_BreathingColor(COLOR_NULL);
			break;
		}
		
		RGB_Cnt = 0;
	}
	
	
	if(++CANBRSwitching_Cnt >= 100)
	{
		CanInterface_ApplyPendingBitrate();
		CANBRSwitching_Cnt = 0;
	}
	
	CanInterface_UpdateWatchdog();
}

void SupervisorTask_RunBackground(void)
{
	MotorDiagnosticFrame frame;
	uint8_t buffer;
	if (ActiveContext == 0 || !ActiveContext->is_initialized ||
		!ActiveContext->diagnostic_pending)
		return;
	buffer = ActiveContext->published_diagnostic_buffer;
	frame = ActiveContext->diagnostic_buffers[buffer];
	ActiveContext->diagnostic_pending = false;
	(void)ActiveContext->diagnostic_transport.write(
		ActiveContext->diagnostic_transport.context, &frame, sizeof(frame));
}
