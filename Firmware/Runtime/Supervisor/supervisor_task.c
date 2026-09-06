#include "supervisor_task.h"

#include <stdint.h>
#include "motor_control_types.h"
#include "motor_control_runtime.h"
#include "motor_state_runtime.h"

#define Led_Cnt (context->led_ticks)
#define RGB_Cnt (context->rgb_ticks)
#define CANBRSwitching_Cnt (context->can_bitrate_ticks)

bool SupervisorTask_Initialize(SupervisorTaskContext *context,
	const BspDiagnosticSinkPort *diagnostic_transport,
	MotorControlRuntimeContext *motor_control,
	TelemetryServiceContext *telemetry, CanInterfaceContext *can_interface,
	UsbInterfaceContext *usb_interface,
	CommunicationWatchdogServiceContext *communication_watchdog,
	CanConfigurationServiceContext *can_configuration,
	LedServiceContext *led, RgbServiceContext *rgb)
{
	if (context == 0 || diagnostic_transport == 0 ||
		diagnostic_transport->write == 0 || motor_control == 0 ||
		telemetry == 0 || can_interface == 0 || usb_interface == 0 ||
		communication_watchdog == 0 || can_configuration == 0 || led == 0 ||
		rgb == 0)
		return false;
	context->led_ticks = 0U;
	context->rgb_ticks = 0U;
	context->can_bitrate_ticks = 0U;
	context->diagnostic_ticks = 0U;
	context->published_diagnostic_buffer = 0U;
	context->diagnostic_pending = false;
	context->diagnostic_transport = *diagnostic_transport;
	context->motor_control = motor_control;
	context->telemetry = telemetry;
	context->can_interface = can_interface;
	context->usb_interface = usb_interface;
	context->communication_watchdog = communication_watchdog;
	context->can_configuration = can_configuration;
	context->led = led;
	context->rgb = rgb;
	context->is_initialized = true;
	return true;
}

/**
	* @brief  System supervision task executed at 1 kHz
			  update USB print, LEDs, RGB, encoder state and CAN status
 **/
void SupervisorTask_Execute1kHz(SupervisorTaskContext *context)
{
	DeviceState device_state;
	MotorControlMode control_mode;
	ServiceProcedure service_procedure;
	if (context == 0 || !context->is_initialized)
		return;

	MotorLifecycle_Supervise1kHz(&context->motor_control->motor_state);
	MotorControlRuntime_PublishTelemetry(context->motor_control,
		context->telemetry);
	if (++context->diagnostic_ticks >= 10U)
	{
		uint8_t next_buffer = context->published_diagnostic_buffer == 0U ?
			1U : 0U;
		context->diagnostic_ticks = 0U;
		if (MotorControlRuntime_ReadDiagnosticFrame(context->motor_control,
			&context->diagnostic_buffers[next_buffer]))
		{
			context->published_diagnostic_buffer = next_buffer;
			context->diagnostic_pending = true;
		}
	}
	device_state = MotorLifecycle_GetDeviceState(
		&context->motor_control->motor_state);
	control_mode = MotorLifecycle_GetControlMode(
		&context->motor_control->motor_state);
	service_procedure = MotorLifecycle_GetServiceProcedure(
		&context->motor_control->motor_state);
	UsbInterface_UpdateTelemetryStream(context->usb_interface,
		context->telemetry, context->can_configuration);

	if (MotorState_HasChanged(&context->motor_control->motor_state))
	{
		if (MotorState_GetPrimaryFault(&context->motor_control->motor_state) ==
			MOTOR_FAULT_NONE)
			LED_SetState(context->led, 0,
				MotorLifecycle_GetProtocolActionCode(
					&context->motor_control->motor_state));
		else
			LED_SetState(context->led, 1,
				(uint8_t)MotorState_GetPrimaryFault(
					&context->motor_control->motor_state));
		MotorState_ClearChangeFlag(&context->motor_control->motor_state);
	}
	
	if(++Led_Cnt >= 200)
	{
		LED_Task(context->led);
		Led_Cnt=0;
	}
		
	if(++RGB_Cnt >= 50)
	{
		if (device_state == DEVICE_STATE_SERVICING &&
			service_procedure != SERVICE_PROCEDURE_NONE)
			Set_RGB_BreathingColor(context->rgb, PURPLE);
		else switch(control_mode)
		{
			case MOTOR_CONTROL_MODE_CURRENT:
				Set_RGB_BreathingColor(context->rgb, GREEN);
			break;
			
			case MOTOR_CONTROL_MODE_SPEED:
				Set_RGB_BreathingColor(context->rgb, CYAN);
			break;

			case MOTOR_CONTROL_MODE_SENSORLESS_SPEED:
				Set_RGB_BreathingColor(context->rgb, YELLOW);
			break;
			
			case MOTOR_CONTROL_MODE_POSITION_CASCADE:
				Set_RGB_BreathingColor(context->rgb, BLUE);
			break;

			case MOTOR_CONTROL_MODE_POSITION_IMPEDANCE:
				Set_RGB_BreathingColor(context->rgb, WHITE);
			break;
			
			default:
				Set_RGB_BreathingColor(context->rgb, COLOR_NULL);
			break;
		}
		
		RGB_Cnt = 0;
	}
	
	
	if(++CANBRSwitching_Cnt >= 100)
	{
		CanInterface_ApplyPendingBitrate(context->can_interface,
			context->communication_watchdog);
		CANBRSwitching_Cnt = 0;
	}
	
	CanInterface_UpdateWatchdog(context->can_interface, context->telemetry,
		context->communication_watchdog);
}

void SupervisorTask_RunBackground(SupervisorTaskContext *context)
{
	MotorDiagnosticFrame frame;
	uint8_t buffer;
	if (context == 0 || !context->is_initialized ||
		!context->diagnostic_pending)
		return;
	buffer = context->published_diagnostic_buffer;
	frame = context->diagnostic_buffers[buffer];
	context->diagnostic_pending = false;
	(void)context->diagnostic_transport.write(
		context->diagnostic_transport.context, &frame, sizeof(frame));
}
