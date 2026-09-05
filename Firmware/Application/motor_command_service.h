#ifndef APPLICATION_MOTOR_COMMAND_SERVICE_H
#define APPLICATION_MOTOR_COMMAND_SERVICE_H

#include <stdint.h>
#include <stdbool.h>
#include "motor_command_port.h"

typedef enum
{
	MOTOR_COMMAND_ACCEPTED = 0,
	MOTOR_COMMAND_INVALID_VALUE,
	MOTOR_COMMAND_OUT_OF_RANGE,
	MOTOR_COMMAND_INVALID_STATE
} MotorCommandResult;

typedef struct
{
	MotorCommandPort port;
	bool is_initialized;
} MotorCommandServiceContext;

bool MotorCommandService_Initialize(MotorCommandServiceContext *context,
	const MotorCommandPort *port);
MotorCommandResult MotorCommandService_RequestActionCode(uint8_t action_code);
MotorCommandResult MotorCommandService_SetCurrentReferenceA(float current_a);
MotorCommandResult MotorCommandService_SetSpeedReferenceRps(float speed_rps);
MotorCommandResult MotorCommandService_SetPositionReferenceRevolutions(
	float position_revolutions);

#endif
