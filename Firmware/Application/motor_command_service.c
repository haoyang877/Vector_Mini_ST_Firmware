#include "motor_command_service.h"

#include <math.h>

#define MOTOR_COMMAND_TWO_PI      6.2831853072f
#define MOTOR_COMMAND_ONE_BY_2PI  0.1591549431f

static MotorCommandServiceContext *ActiveContext;

#define CommandPort (ActiveContext->port)
#define CommandPortInitialized (ActiveContext != 0 && ActiveContext->is_initialized)

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
	ActiveContext = context;
	return true;
}

MotorCommandResult MotorCommandService_RequestActionCode(uint8_t action_code)
{
	if (!CommandPortInitialized)
		return MOTOR_COMMAND_INVALID_STATE;
	if (action_code >= ACTION_CODE_COUNT)
		return MOTOR_COMMAND_OUT_OF_RANGE;

	switch (action_code)
	{
		case ACTION_CODE_DISABLED:
			return CommandPort.request_standby(CommandPort.context) ?
				MOTOR_COMMAND_ACCEPTED : MOTOR_COMMAND_INVALID_STATE;
		case ACTION_CODE_CURRENT_CONTROL:
			return CommandPort.request_mode(CommandPort.context, MOTOR_PORT_MODE_CURRENT) ?
				MOTOR_COMMAND_ACCEPTED : MOTOR_COMMAND_INVALID_STATE;
		case ACTION_CODE_SPEED_CONTROL:
			return CommandPort.request_mode(CommandPort.context, MOTOR_PORT_MODE_SPEED) ?
				MOTOR_COMMAND_ACCEPTED : MOTOR_COMMAND_INVALID_STATE;
		case ACTION_CODE_POSITION_CONTROL:
			return CommandPort.request_mode(CommandPort.context, MOTOR_PORT_MODE_POSITION) ?
				MOTOR_COMMAND_ACCEPTED : MOTOR_COMMAND_INVALID_STATE;
		case ACTION_CODE_CALIBRATE_ENCODER:
			return CommandPort.request_service(CommandPort.context,
				MOTOR_PORT_SERVICE_ENCODER_LINEARIZATION) ? MOTOR_COMMAND_ACCEPTED :
				MOTOR_COMMAND_INVALID_STATE;
		case ACTION_CODE_SET_MECHANICAL_ZERO:
			return CommandPort.request_service(CommandPort.context,
				MOTOR_PORT_SERVICE_SET_MECHANICAL_ZERO) ? MOTOR_COMMAND_ACCEPTED :
				MOTOR_COMMAND_INVALID_STATE;
		case ACTION_CODE_RESTORE_DEFAULTS:
			return CommandPort.request_service(CommandPort.context,
				MOTOR_PORT_SERVICE_RESTORE_DEFAULTS) ? MOTOR_COMMAND_ACCEPTED :
				MOTOR_COMMAND_INVALID_STATE;
		case ACTION_CODE_SAVE_PARAMETERS:
			return CommandPort.request_service(CommandPort.context,
				MOTOR_PORT_SERVICE_PARAMETER_SAVE) ? MOTOR_COMMAND_ACCEPTED :
				MOTOR_COMMAND_INVALID_STATE;
		case ACTION_CODE_CLEAR_FAULTS:
			return CommandPort.request_clear_faults(CommandPort.context) ?
				MOTOR_COMMAND_ACCEPTED : MOTOR_COMMAND_INVALID_STATE;
		case ACTION_CODE_CALIBRATE_CURRENT_OFFSET:
			return CommandPort.request_service(CommandPort.context,
				MOTOR_PORT_SERVICE_CURRENT_OFFSET_CALIBRATION) ? MOTOR_COMMAND_ACCEPTED :
				MOTOR_COMMAND_INVALID_STATE;
		case ACTION_CODE_VOLTAGE_OPEN_LOOP:
			return CommandPort.request_mode(CommandPort.context,
				MOTOR_PORT_MODE_VOLTAGE_OPEN_LOOP) ? MOTOR_COMMAND_ACCEPTED :
				MOTOR_COMMAND_INVALID_STATE;
		case ACTION_CODE_CALIBRATE_ENCODER_OBSERVER:
			return CommandPort.request_service(CommandPort.context,
				MOTOR_PORT_SERVICE_OBSERVER_CALIBRATION) ? MOTOR_COMMAND_ACCEPTED :
				MOTOR_COMMAND_INVALID_STATE;
		case ACTION_CODE_VQ_CONTROL:
			return CommandPort.request_mode(CommandPort.context, MOTOR_PORT_MODE_VQ) ?
				MOTOR_COMMAND_ACCEPTED : MOTOR_COMMAND_INVALID_STATE;
		case ACTION_CODE_CALIBRATE_ELECTRICAL_ZERO:
			return CommandPort.request_service(CommandPort.context,
				MOTOR_PORT_SERVICE_ELECTRICAL_ZERO_CALIBRATION) ? MOTOR_COMMAND_ACCEPTED :
				MOTOR_COMMAND_INVALID_STATE;
		case ACTION_CODE_SENSORLESS_SPEED_CONTROL:
			return CommandPort.request_mode(CommandPort.context,
				MOTOR_PORT_MODE_SENSORLESS_SPEED) ? MOTOR_COMMAND_ACCEPTED :
				MOTOR_COMMAND_INVALID_STATE;
		case ACTION_CODE_IDENTIFY_PHASE_RESISTANCE:
			return CommandPort.request_service(CommandPort.context,
				MOTOR_PORT_SERVICE_PHASE_RESISTANCE_IDENTIFICATION) ?
				MOTOR_COMMAND_ACCEPTED : MOTOR_COMMAND_INVALID_STATE;
		case ACTION_CODE_POSITION_IMPEDANCE_CONTROL:
			return CommandPort.request_mode(CommandPort.context,
				MOTOR_PORT_MODE_POSITION_IMPEDANCE) ? MOTOR_COMMAND_ACCEPTED :
				MOTOR_COMMAND_INVALID_STATE;
		case ACTION_CODE_IDENTIFY_FRICTION:
			return CommandPort.request_service(CommandPort.context,
				MOTOR_PORT_SERVICE_FRICTION_IDENTIFICATION) ?
				MOTOR_COMMAND_ACCEPTED : MOTOR_COMMAND_INVALID_STATE;
		case ACTION_CODE_IDENTIFY_MOTOR_PARAMETERS:
		case ACTION_CODE_CALIBRATE_COGGING:
		default:
			return MOTOR_COMMAND_INVALID_STATE;
	}
}

MotorCommandResult MotorCommandService_SetCurrentReferenceA(float current_a)
{
	if (!CommandPortInitialized)
		return MOTOR_COMMAND_INVALID_STATE;
	if (!isfinite(current_a))
		return MOTOR_COMMAND_INVALID_VALUE;
	if (fabs(current_a) > CommandPort.get_current_limit_a(CommandPort.context))
		return MOTOR_COMMAND_OUT_OF_RANGE;
	if (CommandPort.get_mode(CommandPort.context) != MOTOR_PORT_MODE_CURRENT &&
		!CommandPort.request_mode(CommandPort.context, MOTOR_PORT_MODE_CURRENT))
		return MOTOR_COMMAND_INVALID_STATE;
	CommandPort.set_current_reference_a(CommandPort.context, current_a);
	return MOTOR_COMMAND_ACCEPTED;
}

MotorCommandResult MotorCommandService_SetSpeedReferenceRps(float speed_rps)
{
	MotorPortMode mode;

	if (!CommandPortInitialized)
		return MOTOR_COMMAND_INVALID_STATE;
	if (!isfinite(speed_rps))
		return MOTOR_COMMAND_INVALID_VALUE;
	if (fabs(speed_rps) > CommandPort.get_speed_limit_rad_s(CommandPort.context) *
		MOTOR_COMMAND_ONE_BY_2PI)
		return MOTOR_COMMAND_OUT_OF_RANGE;
	mode = CommandPort.get_mode(CommandPort.context);
	if (mode != MOTOR_PORT_MODE_SPEED && mode != MOTOR_PORT_MODE_SENSORLESS_SPEED &&
		!CommandPort.request_mode(CommandPort.context, MOTOR_PORT_MODE_SPEED))
		return MOTOR_COMMAND_INVALID_STATE;
	CommandPort.set_speed_reference_rad_s(CommandPort.context,
		speed_rps * MOTOR_COMMAND_TWO_PI);
	return MOTOR_COMMAND_ACCEPTED;
}

MotorCommandResult MotorCommandService_SetPositionReferenceRevolutions(
	float position_revolutions)
{
	float position_rad;
	MotorPortMode mode;

	if (!CommandPortInitialized)
		return MOTOR_COMMAND_INVALID_STATE;
	if (!isfinite(position_revolutions))
		return MOTOR_COMMAND_INVALID_VALUE;
	position_rad = position_revolutions * MOTOR_COMMAND_TWO_PI;
	if (!isfinite(position_rad))
		return MOTOR_COMMAND_OUT_OF_RANGE;
	mode = CommandPort.get_mode(CommandPort.context);
	if (mode != MOTOR_PORT_MODE_POSITION &&
		mode != MOTOR_PORT_MODE_POSITION_IMPEDANCE &&
		!CommandPort.request_mode(CommandPort.context, MOTOR_PORT_MODE_POSITION))
		return MOTOR_COMMAND_INVALID_STATE;
	CommandPort.set_position_reference_rad(CommandPort.context, position_rad);
	return MOTOR_COMMAND_ACCEPTED;
}
