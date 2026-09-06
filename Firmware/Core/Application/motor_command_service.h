#ifndef CORE_APPLICATION_MOTOR_COMMAND_SERVICE_H
#define CORE_APPLICATION_MOTOR_COMMAND_SERVICE_H

#include <stdint.h>
#include <stdbool.h>
#include "Core/Application/Contracts/motor_command_port.h"

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
MotorCommandResult MotorCommandService_RequestActionCode(
	MotorCommandServiceContext *context, uint8_t action_code);
MotorCommandResult MotorCommandService_SetCurrentReferenceA(
	MotorCommandServiceContext *context, float current_a);
MotorCommandResult MotorCommandService_SetSpeedReferenceRps(
	MotorCommandServiceContext *context, float speed_rps);
MotorCommandResult MotorCommandService_SetPositionReferenceRevolutions(
	MotorCommandServiceContext *context, float position_revolutions);

#endif
