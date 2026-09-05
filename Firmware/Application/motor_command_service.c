#include "motor_command_service.h"

#include <math.h>

#define MOTOR_COMMAND_TWO_PI      6.2831853072f
#define MOTOR_COMMAND_ONE_BY_2PI  0.1591549431f

/* Stable v1 wire values. Protocol compatibility ends at this service boundary. */
enum
{
	ACTION_CODE_DISABLED = 0U,
	ACTION_CODE_CURRENT_CONTROL = 1U,
	ACTION_CODE_SPEED_CONTROL = 2U,
	ACTION_CODE_POSITION_CONTROL = 3U,
	ACTION_CODE_IDENTIFY_MOTOR_PARAMETERS = 4U,
	ACTION_CODE_CALIBRATE_ENCODER = 5U,
	ACTION_CODE_CALIBRATE_COGGING = 6U,
	ACTION_CODE_SET_MECHANICAL_ZERO = 7U,
	ACTION_CODE_RESTORE_DEFAULTS = 8U,
	ACTION_CODE_SAVE_PARAMETERS = 9U,
	ACTION_CODE_CLEAR_FAULTS = 10U,
	ACTION_CODE_CALIBRATE_CURRENT_OFFSET = 11U,
	ACTION_CODE_VOLTAGE_OPEN_LOOP = 12U,
	ACTION_CODE_CALIBRATE_ENCODER_OBSERVER = 13U,
	ACTION_CODE_VQ_CONTROL = 14U,
	ACTION_CODE_CALIBRATE_ELECTRICAL_ZERO = 15U,
	ACTION_CODE_SENSORLESS_SPEED_CONTROL = 16U,
	ACTION_CODE_IDENTIFY_PHASE_RESISTANCE = 17U,
	ACTION_CODE_POSITION_IMPEDANCE_CONTROL = 18U,
	ACTION_CODE_IDENTIFY_FRICTION = 19U,
	ACTION_CODE_COUNT = 20U
};

bool MotorCommandService_Initialize(MotorCommandServiceContext *context,
	const MotorCommandPort *port)
{
	if (context == 0 || port == 0 || port->get_mode == 0 || port->request_mode == 0 ||
		port->request_service == 0 || port->request_standby == 0 ||
		port->request_clear_faults == 0 || port->get_current_limit_a == 0 ||
		port->get_speed_limit_rad_s == 0 || port->set_current_reference_a == 0 ||
		port->set_speed_reference_rad_s == 0 ||
		port->set_position_reference_rad == 0)
		return false;
	context->port = *port;
	context->is_initialized = true;
	return true;
}

MotorCommandResult MotorCommandService_RequestActionCode(
	MotorCommandServiceContext *context, uint8_t action_code)
{
	MotorCommandPort *port;
	if (context == 0 || !context->is_initialized)
		return MOTOR_COMMAND_INVALID_STATE;
	port = &context->port;
	if (action_code >= ACTION_CODE_COUNT)
		return MOTOR_COMMAND_OUT_OF_RANGE;

	switch (action_code)
	{
		case ACTION_CODE_DISABLED:
			return port->request_standby(port->context) ?
				MOTOR_COMMAND_ACCEPTED : MOTOR_COMMAND_INVALID_STATE;
		case ACTION_CODE_CURRENT_CONTROL:
			return port->request_mode(port->context, MOTOR_PORT_MODE_CURRENT) ?
				MOTOR_COMMAND_ACCEPTED : MOTOR_COMMAND_INVALID_STATE;
		case ACTION_CODE_SPEED_CONTROL:
			return port->request_mode(port->context, MOTOR_PORT_MODE_SPEED) ?
				MOTOR_COMMAND_ACCEPTED : MOTOR_COMMAND_INVALID_STATE;
		case ACTION_CODE_POSITION_CONTROL:
			return port->request_mode(port->context, MOTOR_PORT_MODE_POSITION) ?
				MOTOR_COMMAND_ACCEPTED : MOTOR_COMMAND_INVALID_STATE;
		case ACTION_CODE_CALIBRATE_ENCODER:
			return port->request_service(port->context,
				MOTOR_PORT_SERVICE_ENCODER_LINEARIZATION) ? MOTOR_COMMAND_ACCEPTED :
				MOTOR_COMMAND_INVALID_STATE;
		case ACTION_CODE_SET_MECHANICAL_ZERO:
			return port->request_service(port->context,
				MOTOR_PORT_SERVICE_SET_MECHANICAL_ZERO) ? MOTOR_COMMAND_ACCEPTED :
				MOTOR_COMMAND_INVALID_STATE;
		case ACTION_CODE_RESTORE_DEFAULTS:
			return port->request_service(port->context,
				MOTOR_PORT_SERVICE_RESTORE_DEFAULTS) ? MOTOR_COMMAND_ACCEPTED :
				MOTOR_COMMAND_INVALID_STATE;
		case ACTION_CODE_SAVE_PARAMETERS:
			return port->request_service(port->context,
				MOTOR_PORT_SERVICE_PARAMETER_SAVE) ? MOTOR_COMMAND_ACCEPTED :
				MOTOR_COMMAND_INVALID_STATE;
		case ACTION_CODE_CLEAR_FAULTS:
			return port->request_clear_faults(port->context) ?
				MOTOR_COMMAND_ACCEPTED : MOTOR_COMMAND_INVALID_STATE;
		case ACTION_CODE_CALIBRATE_CURRENT_OFFSET:
			return port->request_service(port->context,
				MOTOR_PORT_SERVICE_CURRENT_OFFSET_CALIBRATION) ? MOTOR_COMMAND_ACCEPTED :
				MOTOR_COMMAND_INVALID_STATE;
		case ACTION_CODE_VOLTAGE_OPEN_LOOP:
			return port->request_mode(port->context,
				MOTOR_PORT_MODE_VOLTAGE_OPEN_LOOP) ? MOTOR_COMMAND_ACCEPTED :
				MOTOR_COMMAND_INVALID_STATE;
		case ACTION_CODE_CALIBRATE_ENCODER_OBSERVER:
			return port->request_service(port->context,
				MOTOR_PORT_SERVICE_OBSERVER_CALIBRATION) ? MOTOR_COMMAND_ACCEPTED :
				MOTOR_COMMAND_INVALID_STATE;
		case ACTION_CODE_VQ_CONTROL:
			return port->request_mode(port->context, MOTOR_PORT_MODE_VQ) ?
				MOTOR_COMMAND_ACCEPTED : MOTOR_COMMAND_INVALID_STATE;
		case ACTION_CODE_CALIBRATE_ELECTRICAL_ZERO:
			return port->request_service(port->context,
				MOTOR_PORT_SERVICE_ELECTRICAL_ZERO_CALIBRATION) ? MOTOR_COMMAND_ACCEPTED :
				MOTOR_COMMAND_INVALID_STATE;
		case ACTION_CODE_SENSORLESS_SPEED_CONTROL:
			return port->request_mode(port->context,
				MOTOR_PORT_MODE_SENSORLESS_SPEED) ? MOTOR_COMMAND_ACCEPTED :
				MOTOR_COMMAND_INVALID_STATE;
		case ACTION_CODE_IDENTIFY_PHASE_RESISTANCE:
			return port->request_service(port->context,
				MOTOR_PORT_SERVICE_PHASE_RESISTANCE_IDENTIFICATION) ?
				MOTOR_COMMAND_ACCEPTED : MOTOR_COMMAND_INVALID_STATE;
		case ACTION_CODE_POSITION_IMPEDANCE_CONTROL:
			return port->request_mode(port->context,
				MOTOR_PORT_MODE_POSITION_IMPEDANCE) ? MOTOR_COMMAND_ACCEPTED :
				MOTOR_COMMAND_INVALID_STATE;
		case ACTION_CODE_IDENTIFY_FRICTION:
			return port->request_service(port->context,
				MOTOR_PORT_SERVICE_FRICTION_IDENTIFICATION) ?
				MOTOR_COMMAND_ACCEPTED : MOTOR_COMMAND_INVALID_STATE;
		case ACTION_CODE_IDENTIFY_MOTOR_PARAMETERS:
		case ACTION_CODE_CALIBRATE_COGGING:
		default:
			return MOTOR_COMMAND_INVALID_STATE;
	}
}

MotorCommandResult MotorCommandService_SetCurrentReferenceA(
	MotorCommandServiceContext *context, float current_a)
{
	MotorCommandPort *port;
	if (context == 0 || !context->is_initialized)
		return MOTOR_COMMAND_INVALID_STATE;
	port = &context->port;
	if (!isfinite(current_a))
		return MOTOR_COMMAND_INVALID_VALUE;
	if (fabs(current_a) > port->get_current_limit_a(port->context))
		return MOTOR_COMMAND_OUT_OF_RANGE;
	if (port->get_mode(port->context) != MOTOR_PORT_MODE_CURRENT &&
		!port->request_mode(port->context, MOTOR_PORT_MODE_CURRENT))
		return MOTOR_COMMAND_INVALID_STATE;
	port->set_current_reference_a(port->context, current_a);
	return MOTOR_COMMAND_ACCEPTED;
}

MotorCommandResult MotorCommandService_SetSpeedReferenceRps(
	MotorCommandServiceContext *context, float speed_rps)
{
	MotorPortMode mode;
	MotorCommandPort *port;

	if (context == 0 || !context->is_initialized)
		return MOTOR_COMMAND_INVALID_STATE;
	port = &context->port;
	if (!isfinite(speed_rps))
		return MOTOR_COMMAND_INVALID_VALUE;
	if (fabs(speed_rps) > port->get_speed_limit_rad_s(port->context) *
		MOTOR_COMMAND_ONE_BY_2PI)
		return MOTOR_COMMAND_OUT_OF_RANGE;
	mode = port->get_mode(port->context);
	if (mode != MOTOR_PORT_MODE_SPEED && mode != MOTOR_PORT_MODE_SENSORLESS_SPEED &&
		!port->request_mode(port->context, MOTOR_PORT_MODE_SPEED))
		return MOTOR_COMMAND_INVALID_STATE;
	port->set_speed_reference_rad_s(port->context,
		speed_rps * MOTOR_COMMAND_TWO_PI);
	return MOTOR_COMMAND_ACCEPTED;
}

MotorCommandResult MotorCommandService_SetPositionReferenceRevolutions(
	MotorCommandServiceContext *context, float position_revolutions)
{
	float position_rad;
	MotorPortMode mode;
	MotorCommandPort *port;

	if (context == 0 || !context->is_initialized)
		return MOTOR_COMMAND_INVALID_STATE;
	port = &context->port;
	if (!isfinite(position_revolutions))
		return MOTOR_COMMAND_INVALID_VALUE;
	position_rad = position_revolutions * MOTOR_COMMAND_TWO_PI;
	if (!isfinite(position_rad))
		return MOTOR_COMMAND_OUT_OF_RANGE;
	mode = port->get_mode(port->context);
	if (mode != MOTOR_PORT_MODE_POSITION &&
		mode != MOTOR_PORT_MODE_POSITION_IMPEDANCE &&
		!port->request_mode(port->context, MOTOR_PORT_MODE_POSITION))
		return MOTOR_COMMAND_INVALID_STATE;
	port->set_position_reference_rad(port->context, position_rad);
	return MOTOR_COMMAND_ACCEPTED;
}
