#ifndef CORE_APPLICATION_CONTRACTS_MOTOR_OUTPUT_SAFETY_PORT_H
#define CORE_APPLICATION_CONTRACTS_MOTOR_OUTPUT_SAFETY_PORT_H

#include <stdbool.h>

/*
 * Narrow application boundary for transitions which must prove that motor
 * outputs are de-energized. The implementation may be a motor-drive runtime,
 * a boot-safe board adapter, or a host fake; application services never own
 * PWM, timer, or gate-driver details.
 */
typedef struct
{
	void *context;
	void (*disable_immediate)(void *context);
	bool (*outputs_are_enabled)(void *context);
} MotorOutputSafetyPort;

#endif
