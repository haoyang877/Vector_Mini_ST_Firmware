#ifndef APPLICATION_PARAMETER_SERVICE_H
#define APPLICATION_PARAMETER_SERVICE_H

#include <stdbool.h>
#include "Core/Application/Contracts/motor_configuration_port.h"
#include "motor_profiles.h"

typedef struct
{
	float command_current_limit_a;
	float calibration_current_limit_a;
} ParameterServiceLimits;

typedef enum
{
	PARAMETER_SERVICE_ACCEPTED = 0,
	PARAMETER_SERVICE_INVALID_VALUE,
	PARAMETER_SERVICE_OUT_OF_RANGE,
	PARAMETER_SERVICE_INVALID_STATE,
	PARAMETER_SERVICE_NOT_SUPPORTED
} ParameterServiceResult;

typedef struct
{
	MotorConfigurationPort port;
	ParameterServiceLimits limits;
	const MotorProfile *motor_profile;
	bool is_initialized;
} ParameterServiceContext;

bool ParameterService_Initialize(ParameterServiceContext *context,
	const MotorConfigurationPort *port,
	const ParameterServiceLimits *limits, const MotorProfile *motor_profile);
ParameterServiceResult ParameterService_WriteMotorParameter(
	ParameterServiceContext *context, MotorParameterId parameter, float value);
ParameterServiceResult ParameterService_ReadMotorParameter(
	const ParameterServiceContext *context, MotorParameterId parameter,
	float *value);

#endif
