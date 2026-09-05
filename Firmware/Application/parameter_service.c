#include "parameter_service.h"

#include <math.h>
#define TWO_PI                       6.28318530717958647692f

static ParameterServiceContext *ActiveContext;

#define ConfigurationPort (ActiveContext->port)
#define ConfigurationPortInitialized (ActiveContext != 0 && ActiveContext->is_initialized)
#define ActiveBoardProfile (ActiveContext->board_profile)
#define ActiveMotorProfile (ActiveContext->motor_profile)

bool ParameterService_Initialize(ParameterServiceContext *context,
	const MotorConfigurationPort *port,
	const BoardProfile *board_profile, const MotorProfile *motor_profile)
{
	if (context == 0 || port == 0 || port->can_stage == 0 || port->read == 0 ||
		port->stage == 0 ||
		board_profile == 0 || motor_profile == 0)
		return false;
	context->port = *port;
	context->board_profile = board_profile;
	context->motor_profile = motor_profile;
	context->is_initialized = true;
	ActiveContext = context;
	return true;
}

static bool ParameterService_IsInRange(float value, float minimum, float maximum)
{
	return value >= minimum && value <= maximum;
}

ParameterServiceResult ParameterService_WriteMotorParameter(
	MotorParameterId parameter, float value)
{
	float speed_limit;

	if (!ConfigurationPortInitialized)
		return PARAMETER_SERVICE_NOT_SUPPORTED;
	if (!ConfigurationPort.can_stage(ConfigurationPort.context))
		return PARAMETER_SERVICE_INVALID_STATE;
	if (!isfinite(value))
		return PARAMETER_SERVICE_INVALID_VALUE;

	switch (parameter)
	{
		case MOTOR_PARAMETER_POLE_PAIRS:
			if (!ParameterService_IsInRange(value, 2.0f, 30.0f) ||
				value != (float)(int)value)
				return PARAMETER_SERVICE_OUT_OF_RANGE;
			break;
		case MOTOR_PARAMETER_CALIBRATION_CURRENT_A:
			if (!ParameterService_IsInRange(value, 0.0f,
				ActiveBoardProfile->calibration_current_limit_a))
				return PARAMETER_SERVICE_OUT_OF_RANGE;
			break;
		case MOTOR_PARAMETER_CURRENT_LIMIT_A:
			if (!ParameterService_IsInRange(value, 0.0f,
				ActiveBoardProfile->current_command_limit_a))
				return PARAMETER_SERVICE_OUT_OF_RANGE;
			break;
		case MOTOR_PARAMETER_SPEED_LIMIT_RAD_S:
			if (value <= 0.0f ||
				value > ActiveMotorProfile->speed_limit_max_rad_s)
				return PARAMETER_SERVICE_OUT_OF_RANGE;
			break;
		case MOTOR_PARAMETER_SPEED_ACCELERATION_RAD_S2:
		case MOTOR_PARAMETER_SPEED_DECELERATION_RAD_S2:
			if (!ParameterService_IsInRange(value, 0.0f,
				ActiveMotorProfile->speed_ramp_max_rad_s2))
				return PARAMETER_SERVICE_OUT_OF_RANGE;
			break;
		case MOTOR_PARAMETER_SPEED_KP:
			if (!ParameterService_IsInRange(value, 0.01f, 2.0f))
				return PARAMETER_SERVICE_OUT_OF_RANGE;
			break;
		case MOTOR_PARAMETER_SPEED_KI:
			if (!ParameterService_IsInRange(value, 0.0f, 2.0f))
				return PARAMETER_SERVICE_OUT_OF_RANGE;
			break;
		case MOTOR_PARAMETER_POSITION_ACCELERATION_RAD_S2:
		case MOTOR_PARAMETER_POSITION_DECELERATION_RAD_S2:
			if (value <= 0.0f ||
				value > ActiveMotorProfile->position_ramp_max_rad_s2)
				return PARAMETER_SERVICE_OUT_OF_RANGE;
			break;
		case MOTOR_PARAMETER_POSITION_MAX_SPEED_RAD_S:
			if (!ConfigurationPort.read(ConfigurationPort.context,
				MOTOR_PARAMETER_SPEED_LIMIT_RAD_S, &speed_limit) || value <= 0.0f ||
				value > ActiveMotorProfile->position_speed_limit_rps * TWO_PI ||
				value > speed_limit)
				return PARAMETER_SERVICE_OUT_OF_RANGE;
			break;
		case MOTOR_PARAMETER_POSITION_KP_A_PER_RAD:
			if (!ParameterService_IsInRange(value, 0.0f,
				ActiveMotorProfile->position_kp_limit_a_per_rad))
				return PARAMETER_SERVICE_OUT_OF_RANGE;
			break;
		case MOTOR_PARAMETER_POSITION_KD_A_PER_RAD_S:
			if (!ParameterService_IsInRange(value, 0.0f,
				ActiveMotorProfile->position_kd_limit_a_per_rad_s))
				return PARAMETER_SERVICE_OUT_OF_RANGE;
			break;
		case MOTOR_PARAMETER_POSITION_KI_A_PER_RAD_S:
			if (!ParameterService_IsInRange(value, 0.0f,
				ActiveMotorProfile->position_ki_limit_a_per_rad_s))
				return PARAMETER_SERVICE_OUT_OF_RANGE;
			break;
		case MOTOR_PARAMETER_POSITION_INTEGRAL_LIMIT_A:
			if (!ParameterService_IsInRange(value, 0.0f,
				ActiveBoardProfile->current_command_limit_a))
				return PARAMETER_SERVICE_OUT_OF_RANGE;
			break;
		case MOTOR_PARAMETER_CASCADE_POSITION_KP_PER_S:
			if (!ParameterService_IsInRange(value, 0.0f,
				ActiveMotorProfile->cascade_position_kp_limit_per_s))
				return PARAMETER_SERVICE_OUT_OF_RANGE;
			break;
		case MOTOR_PARAMETER_CASCADE_POSITION_KD:
			if (!ParameterService_IsInRange(value, 0.0f,
				ActiveMotorProfile->cascade_position_kd_limit))
				return PARAMETER_SERVICE_OUT_OF_RANGE;
			break;
		case MOTOR_PARAMETER_PHASE_RESISTANCE_OHM:
			if (!ParameterService_IsInRange(value,
				ActiveMotorProfile->phase_resistance_min_ohm,
				ActiveMotorProfile->phase_resistance_max_ohm))
				return PARAMETER_SERVICE_OUT_OF_RANGE;
			break;
		case MOTOR_PARAMETER_D_AXIS_INDUCTANCE_H:
		case MOTOR_PARAMETER_Q_AXIS_INDUCTANCE_H:
			if (!ParameterService_IsInRange(value,
				ActiveMotorProfile->inductance_min_h,
				ActiveMotorProfile->inductance_max_h))
				return PARAMETER_SERVICE_OUT_OF_RANGE;
			break;
		case MOTOR_PARAMETER_FLUX_WEBER:
			if (!ParameterService_IsInRange(value,
				ActiveMotorProfile->flux_min_weber,
				ActiveMotorProfile->flux_max_weber))
				return PARAMETER_SERVICE_OUT_OF_RANGE;
			break;
		default:
			return PARAMETER_SERVICE_NOT_SUPPORTED;
	}

	return ConfigurationPort.stage(ConfigurationPort.context, parameter, value) ?
		PARAMETER_SERVICE_ACCEPTED : PARAMETER_SERVICE_NOT_SUPPORTED;
}

ParameterServiceResult ParameterService_ReadMotorParameter(
	MotorParameterId parameter, float *value)
{
	if (value == 0)
		return PARAMETER_SERVICE_INVALID_VALUE;
	if (!ConfigurationPortInitialized ||
		!ConfigurationPort.read(ConfigurationPort.context, parameter, value))
		return PARAMETER_SERVICE_NOT_SUPPORTED;
	return PARAMETER_SERVICE_ACCEPTED;
}
