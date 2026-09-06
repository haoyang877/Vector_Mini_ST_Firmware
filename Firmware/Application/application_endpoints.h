#ifndef APPLICATION_ENDPOINTS_H
#define APPLICATION_ENDPOINTS_H

#include <stdbool.h>

#include "can_configuration_service.h"
#include "Core/Application/friction_identification_service.h"
#include "Core/Application/motor_command_service.h"
#include "parameter_service.h"
#include "Core/Application/rotor_calibration_service.h"
#include "Core/Infrastructure/Telemetry/telemetry_service.h"
#include "Core/Application/control_authority_service.h"

typedef struct
{
	CanConfigurationServiceContext *can_configuration;
	FrictionIdentificationServiceContext *friction_identification;
	MotorCommandServiceContext *motor_command;
	ParameterServiceContext *parameters;
	RotorCalibrationServiceContext *rotor_calibration;
	TelemetryServiceContext *telemetry;
	ControlAuthorityServiceContext *control_authority;
} ApplicationEndpoints;

bool ApplicationEndpoints_Initialize(ApplicationEndpoints *context,
	CanConfigurationServiceContext *can_configuration,
	FrictionIdentificationServiceContext *friction_identification,
	MotorCommandServiceContext *motor_command,
	ParameterServiceContext *parameters,
	RotorCalibrationServiceContext *rotor_calibration,
	TelemetryServiceContext *telemetry,
	ControlAuthorityServiceContext *control_authority);

#endif
