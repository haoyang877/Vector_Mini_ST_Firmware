#ifndef CORE_APPLICATION_SUPERVISION_SUPERVISOR_TASK_H
#define CORE_APPLICATION_SUPERVISION_SUPERVISOR_TASK_H

#include <stdbool.h>
#include <stdint.h>
#include "bsp_system.h"
#include "Core/Application/Contracts/communication_supervision_port.h"
#include "Core/Application/Supervision/temperature_supervision.h"
#include "motor_control_runtime.h"
#include "Core/Infrastructure/Telemetry/telemetry_service.h"
#include "Core/Application/Indicators/led.h"
#include "Core/Application/Indicators/rgb.h"

typedef struct
{
	uint16_t led_ticks;
	uint16_t rgb_ticks;
	uint16_t diagnostic_ticks;
	MotorDiagnosticFrame diagnostic_buffers[2];
	volatile uint8_t published_diagnostic_buffer;
	volatile bool diagnostic_pending;
	BspDiagnosticSinkPort diagnostic_transport;
	CommunicationSupervisionPort communication_supervision;
	TemperatureSupervisionContext *temperature_supervision;
	MotorControlRuntimeContext *motor_control;
	TelemetryServiceContext *telemetry;
	LedServiceContext *led;
	RgbServiceContext *rgb;
	bool is_initialized;
} SupervisorTaskContext;

bool SupervisorTask_Initialize(SupervisorTaskContext *context,
	const BspDiagnosticSinkPort *diagnostic_transport,
	const CommunicationSupervisionPort *communication_supervision,
	TemperatureSupervisionContext *temperature_supervision,
	MotorControlRuntimeContext *motor_control,
	TelemetryServiceContext *telemetry,
	LedServiceContext *led, RgbServiceContext *rgb);
void SupervisorTask_Execute1kHz(SupervisorTaskContext *context);
void SupervisorTask_RunBackground(SupervisorTaskContext *context);

#endif
