#include "parameter_service.h"

#define TEST_CHECK(condition) do { if (!(condition)) return __LINE__; } while (0)

typedef struct
{
	float speed_limit_rad_s;
	MotorParameterId staged_parameter;
	float staged_value;
} ParameterServiceFakeConfiguration;

static bool ParameterServiceTest_CanStage(void *context)
{
	return context != 0;
}

static bool ParameterServiceTest_Read(void *context,
	MotorParameterId parameter, float *value)
{
	ParameterServiceFakeConfiguration *configuration =
		(ParameterServiceFakeConfiguration *)context;
	if (configuration == 0 || value == 0 ||
		parameter != MOTOR_PARAMETER_SPEED_LIMIT_RAD_S)
		return false;
	*value = configuration->speed_limit_rad_s;
	return true;
}

static bool ParameterServiceTest_Stage(void *context,
	MotorParameterId parameter, float value)
{
	ParameterServiceFakeConfiguration *configuration =
		(ParameterServiceFakeConfiguration *)context;
	if (configuration == 0)
		return false;
	configuration->staged_parameter = parameter;
	configuration->staged_value = value;
	return true;
}

int ParameterService_RunHostTests(void)
{
	ParameterServiceFakeConfiguration configuration = {0};
	MotorConfigurationPort port = {0};
	ParameterServiceContext service;
	ParameterServiceLimits limits = {0};
	MotorProfile motor = {0};

	configuration.speed_limit_rad_s = 100.0f;
	port.context = &configuration;
	port.can_stage = ParameterServiceTest_CanStage;
	port.read = ParameterServiceTest_Read;
	port.stage = ParameterServiceTest_Stage;
	limits.command_current_limit_a = 10.0f;
	limits.calibration_current_limit_a = 10.0f;
	motor.phase_resistance_min_ohm = 0.0001f;
	motor.phase_resistance_max_ohm = 5.0f;
	motor.inductance_min_h = 1.0e-6f;
	motor.inductance_max_h = 5.0e-3f;
	motor.flux_min_weber = 1.0e-5f;
	motor.flux_max_weber = 1.0f;
	motor.speed_limit_max_rad_s = 200.0f;
	motor.speed_ramp_max_rad_s2 = 1000.0f;
	motor.position_ramp_max_rad_s2 = 200.0f;
	motor.position_speed_limit_rps = 1.0f;
	motor.position_kp_limit_a_per_rad = 50.0f;
	motor.position_kd_limit_a_per_rad_s = 10.0f;
	motor.position_ki_limit_a_per_rad_s = 10.0f;
	motor.cascade_position_kp_limit_per_s = 50.0f;
	motor.cascade_position_kd_limit = 10.0f;

	TEST_CHECK(ParameterService_Initialize(&service, &port, &limits, &motor));
	TEST_CHECK(ParameterService_WriteMotorParameter(
		&service,
		MOTOR_PARAMETER_PHASE_RESISTANCE_OHM, 1.905f) ==
		PARAMETER_SERVICE_ACCEPTED);
	TEST_CHECK(configuration.staged_parameter ==
		MOTOR_PARAMETER_PHASE_RESISTANCE_OHM);
	TEST_CHECK(configuration.staged_value == 1.905f);
	TEST_CHECK(ParameterService_WriteMotorParameter(
		&service,
		MOTOR_PARAMETER_PHASE_RESISTANCE_OHM, 5.001f) ==
		PARAMETER_SERVICE_OUT_OF_RANGE);
	TEST_CHECK(ParameterService_WriteMotorParameter(
		&service,
		MOTOR_PARAMETER_D_AXIS_INDUCTANCE_H, 0.001635f) ==
		PARAMETER_SERVICE_ACCEPTED);
	TEST_CHECK(ParameterService_WriteMotorParameter(
		&service,
		MOTOR_PARAMETER_FLUX_WEBER, 0.0175025f) ==
		PARAMETER_SERVICE_ACCEPTED);
	return 0;
}
