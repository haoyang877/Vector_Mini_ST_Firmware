#ifndef RUNTIME_SUPERVISOR_TASK_H
#define RUNTIME_SUPERVISOR_TASK_H

#include <stdbool.h>
#include <stdint.h>
#include "bsp_system.h"
#include "motor_control_runtime.h"
#include "interface_can.h"
#include "interface_usb.h"
#include "Core/Application/communication_watchdog_service.h"
#include "can_configuration_service.h"
#include "Core/Infrastructure/Telemetry/telemetry_service.h"
#include "led.h"
#include "rgb.h"

typedef struct
{
	uint16_t led_ticks;
	uint16_t rgb_ticks;
	uint16_t can_bitrate_ticks;
	uint16_t diagnostic_ticks;
	MotorDiagnosticFrame diagnostic_buffers[2];
	volatile uint8_t published_diagnostic_buffer;
	volatile bool diagnostic_pending;
	BspDiagnosticSinkPort diagnostic_transport;
	MotorControlRuntimeContext *motor_control;
	TelemetryServiceContext *telemetry;
	CanInterfaceContext *can_interface;
	UsbInterfaceContext *usb_interface;
	CommunicationWatchdogServiceContext *communication_watchdog;
	CanConfigurationServiceContext *can_configuration;
	LedServiceContext *led;
	RgbServiceContext *rgb;
	bool is_initialized;
} SupervisorTaskContext;

bool SupervisorTask_Initialize(SupervisorTaskContext *context,
	const BspDiagnosticSinkPort *diagnostic_transport,
	MotorControlRuntimeContext *motor_control,
	TelemetryServiceContext *telemetry, CanInterfaceContext *can_interface,
	UsbInterfaceContext *usb_interface,
	CommunicationWatchdogServiceContext *communication_watchdog,
	CanConfigurationServiceContext *can_configuration,
	LedServiceContext *led, RgbServiceContext *rgb);
void SupervisorTask_Execute1kHz(SupervisorTaskContext *context);
void SupervisorTask_RunBackground(SupervisorTaskContext *context);

#endif
