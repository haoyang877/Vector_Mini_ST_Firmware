#ifndef RUNTIME_SUPERVISOR_TASK_H
#define RUNTIME_SUPERVISOR_TASK_H

#include <stdbool.h>
#include <stdint.h>
#include "diagnostic_transport_port.h"
#include "motor_control_runtime.h"

typedef struct
{
	uint16_t led_ticks;
	uint16_t rgb_ticks;
	uint16_t can_bitrate_ticks;
	uint16_t diagnostic_ticks;
	MotorDiagnosticFrame diagnostic_buffers[2];
	volatile uint8_t published_diagnostic_buffer;
	volatile bool diagnostic_pending;
	DiagnosticTransportPort diagnostic_transport;
	bool is_initialized;
} SupervisorTaskContext;

bool SupervisorTask_Initialize(SupervisorTaskContext *context,
	const DiagnosticTransportPort *diagnostic_transport);
void SupervisorTask_Execute1kHz(void);
void SupervisorTask_RunBackground(void);

#endif
