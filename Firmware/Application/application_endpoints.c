#include "application_endpoints.h"

bool ApplicationEndpoints_Initialize(ApplicationEndpoints *context,
	CanConfigurationServiceContext *can_configuration,
	FrictionIdentificationServiceContext *friction_identification,
	MotorCommandServiceContext *motor_command,
	ParameterServiceContext *parameters,
	RotorCalibrationServiceContext *rotor_calibration,
	TelemetryServiceContext *telemetry,
	ControlAuthorityServiceContext *control_authority)
{
	if (context == 0 || can_configuration == 0 ||
		friction_identification == 0 || motor_command == 0 || parameters == 0 ||
		rotor_calibration == 0 || telemetry == 0 || control_authority == 0)
		return false;
	context->can_configuration = can_configuration;
	context->friction_identification = friction_identification;
	context->motor_command = motor_command;
	context->parameters = parameters;
	context->rotor_calibration = rotor_calibration;
	context->telemetry = telemetry;
	context->control_authority = control_authority;
	return true;
}
