#ifndef CORE_APPLICATION_PARAMETERS_PARAMETER_SERVICE_H
#define CORE_APPLICATION_PARAMETERS_PARAMETER_SERVICE_H

#include <stdbool.h>
#include "Core/Application/Contracts/motor_configuration_port.h"

typedef struct
{
	float command_current_limit_a;
	float calibration_current_limit_a;
	float speed_limit_max_rad_s;
	float speed_ramp_max_rad_s2;
	float position_ramp_max_rad_s2;
	float position_speed_limit_rad_s;
	float position_kp_limit_a_per_rad;
	float position_kd_limit_a_per_rad_s;
	float position_ki_limit_a_per_rad_s;
	float cascade_position_kp_limit_per_s;
	float cascade_position_kd_limit;
	float phase_resistance_min_ohm;
	float phase_resistance_max_ohm;
	float inductance_min_h;
	float inductance_max_h;
	float flux_min_weber;
	float flux_max_weber;
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
	bool is_initialized;
} ParameterServiceContext;

bool ParameterService_Initialize(ParameterServiceContext *context,
	const MotorConfigurationPort *port,
	const ParameterServiceLimits *limits);
ParameterServiceResult ParameterService_WriteMotorParameter(
	ParameterServiceContext *context, MotorParameterId parameter, float value);
ParameterServiceResult ParameterService_ReadMotorParameter(
	const ParameterServiceContext *context, MotorParameterId parameter,
	float *value);

#endif
