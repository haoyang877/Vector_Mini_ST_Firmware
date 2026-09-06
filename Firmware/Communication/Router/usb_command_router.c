#include "usb_command_router.h"

#include "can_configuration_service.h"
#include "motor_command_service.h"
#include "parameter_service.h"
#include "rotor_calibration_service.h"
#include "telemetry_service.h"
#include "friction_identification_service.h"
#include "Core/Communication/Formatting/text_writer.h"

#include <limits.h>
#include <math.h>

#define USB_COMMAND_ROUTER_TWO_PI                6.2831853072f
#define USB_COMMAND_ROUTER_ONE_BY_2PI            0.15915494309f
#define USB_COMMAND_ROUTER_DISABLED_MODE         0U
#define USB_COMMAND_ROUTER_MAX_MODE_VALUE        255

bool UsbCommandRouter_Initialize(UsbCommandRouterContext *context,
	ApplicationEndpoints *application)
{
	if (context == NULL || application == NULL)
		return false;
	context->application = application;
	return true;
}

static UsbCommandError UsbCommandRouter_MapParameterResult(
	ParameterServiceResult result)
{
	if (result == PARAMETER_SERVICE_ACCEPTED)
		return USB_NO_ERROR;
	if (result == PARAMETER_SERVICE_INVALID_VALUE)
		return USB_DATA_INVALID;
	if (result == PARAMETER_SERVICE_OUT_OF_RANGE)
		return USB_DATA_OUT_OF_RANGE;
	return USB_WRITE_INVALID;
}

static UsbCommandError UsbCommandRouter_WriteParameter(
	UsbCommandRouterContext *context, MotorParameterId parameter, float value)
{
	return UsbCommandRouter_MapParameterResult(
		ParameterService_WriteMotorParameter(context->application->parameters,
			parameter, value));
}

static float UsbCommandRouter_ReadParameter(UsbCommandRouterContext *context,
	MotorParameterId parameter, float scale)
{
	float value = 0.0f;
	(void)ParameterService_ReadMotorParameter(context->application->parameters,
		parameter, &value);
	return value * scale;
}

static float UsbCommandRouter_ReadTelemetry(UsbCommandRouterContext *context,
	MotorTelemetryId telemetry, float scale)
{
	float value = 0.0f;
	(void)TelemetryService_ReadValue(context->application->telemetry, telemetry, &value);
	return value * scale;
}

static UsbCommandError UsbCommandRouter_Write(UsbCommandRouterContext *context,
	UsbParameterId parameter, float value, uint8_t value_is_float)
{
	int integer_value = (int)value;
	MotorCommandResult command_result;

	switch (parameter)
	{
		case USB_MODE:
			if (value_is_float != 0U)
				return USB_DATA_INVALID;
			if (integer_value < 0 || integer_value >= USB_COMMAND_ROUTER_MAX_MODE_VALUE)
				return USB_DATA_OUT_OF_RANGE;
			command_result = MotorCommandService_RequestActionCode(
				context->application->motor_command, (uint8_t)integer_value);
			if (command_result != MOTOR_COMMAND_ACCEPTED)
				return USB_WRITE_INVALID;
			if (integer_value == USB_COMMAND_ROUTER_DISABLED_MODE)
				ControlAuthorityService_Release(
					context->application->control_authority, CONTROL_AUTHORITY_USB);
			else
				ControlAuthorityService_Claim(
					context->application->control_authority, CONTROL_AUTHORITY_USB);
			return USB_NO_ERROR;
		case USB_CURRENT_SET:
			command_result = MotorCommandService_SetCurrentReferenceA(
				context->application->motor_command, value);
			break;
		case USB_SPEED_SET:
			command_result = MotorCommandService_SetSpeedReferenceRps(
				context->application->motor_command, value);
			break;
		case USB_POS_SET:
			command_result = MotorCommandService_SetPositionReferenceRevolutions(
				context->application->motor_command, value);
			break;
		case USB_NODE_ID:
			if (value_is_float != 0U)
				return USB_DATA_INVALID;
			return integer_value >= 0 && integer_value <= 7 &&
				CanConfigurationService_SetNodeId(context->application->can_configuration,
					(uint8_t)integer_value) ?
				USB_NO_ERROR : USB_DATA_OUT_OF_RANGE;
		case USB_POLEPARIS:
			if (value_is_float != 0U)
				return USB_DATA_INVALID;
			return UsbCommandRouter_WriteParameter(context,
				MOTOR_PARAMETER_POLE_PAIRS, value);
		case USB_ENCODER_STATE:
			return value_is_float != 0U ? USB_DATA_INVALID : USB_DATA_OUT_OF_RANGE;
		case USB_ENCODER_REVERSE:
			if (value_is_float != 0U)
				return USB_DATA_INVALID;
			if ((int)UsbCommandRouter_ReadTelemetry(context,
				MOTOR_TELEMETRY_MODE, 1.0f) !=
				(int)USB_COMMAND_ROUTER_DISABLED_MODE)
				return USB_WRITE_INVALID;
			if ((integer_value != 0 && integer_value != 1) ||
				!RotorCalibrationService_SetReverse(context->application->rotor_calibration,
					integer_value != 0))
				return USB_DATA_OUT_OF_RANGE;
			return USB_NO_ERROR;
		case USB_CURRENT_CAL:
			return UsbCommandRouter_WriteParameter(context,
				MOTOR_PARAMETER_CALIBRATION_CURRENT_A, value);
		case USB_CURRENT_LIMIT:
			return UsbCommandRouter_WriteParameter(context,
				MOTOR_PARAMETER_CURRENT_LIMIT_A, value);
		case USB_SPEED_LIMIT:
			return UsbCommandRouter_WriteParameter(context,
				MOTOR_PARAMETER_SPEED_LIMIT_RAD_S,
				value * USB_COMMAND_ROUTER_TWO_PI);
		case USB_SPEED_ACC:
			return UsbCommandRouter_WriteParameter(context,
				MOTOR_PARAMETER_SPEED_ACCELERATION_RAD_S2, value * USB_COMMAND_ROUTER_TWO_PI);
		case USB_SPEED_DEC:
			return UsbCommandRouter_WriteParameter(context,
				MOTOR_PARAMETER_SPEED_DECELERATION_RAD_S2, value * USB_COMMAND_ROUTER_TWO_PI);
		case USB_SPEED_KP:
			return UsbCommandRouter_WriteParameter(context,
				MOTOR_PARAMETER_SPEED_KP, value);
		case USB_SPEED_KI:
			return UsbCommandRouter_WriteParameter(context,
				MOTOR_PARAMETER_SPEED_KI, value);
		case USB_POS_ACC:
			return UsbCommandRouter_WriteParameter(context,
				MOTOR_PARAMETER_POSITION_ACCELERATION_RAD_S2, value * USB_COMMAND_ROUTER_TWO_PI);
		case USB_POS_DEC:
			return UsbCommandRouter_WriteParameter(context,
				MOTOR_PARAMETER_POSITION_DECELERATION_RAD_S2, value * USB_COMMAND_ROUTER_TWO_PI);
		case USB_POS_MAXSPEED:
			return UsbCommandRouter_WriteParameter(context,
				MOTOR_PARAMETER_POSITION_MAX_SPEED_RAD_S, value * USB_COMMAND_ROUTER_TWO_PI);
		case USB_POS_KP:
			return UsbCommandRouter_WriteParameter(context,
				MOTOR_PARAMETER_POSITION_KP_A_PER_RAD, value);
		case USB_POS_KD:
			return UsbCommandRouter_WriteParameter(context,
				MOTOR_PARAMETER_POSITION_KD_A_PER_RAD_S, value);
		case USB_POS_KI:
			return UsbCommandRouter_WriteParameter(context,
				MOTOR_PARAMETER_POSITION_KI_A_PER_RAD_S, value);
		case USB_POS_INTEGRAL_LIMIT:
			return UsbCommandRouter_WriteParameter(context,
				MOTOR_PARAMETER_POSITION_INTEGRAL_LIMIT_A, value);
		case USB_CASCADE_POS_KP:
			return UsbCommandRouter_WriteParameter(context,
				MOTOR_PARAMETER_CASCADE_POSITION_KP_PER_S, value);
		case USB_CASCADE_POS_KD:
			return UsbCommandRouter_WriteParameter(context,
				MOTOR_PARAMETER_CASCADE_POSITION_KD, value);
		case USB_CAN_BR:
			if (value_is_float != 0U)
				return USB_DATA_INVALID;
			return integer_value >= 0 &&
				CanConfigurationService_SetBitrateKbps(context->application->can_configuration,
					(uint32_t)integer_value) ?
				USB_NO_ERROR : USB_DATA_OUT_OF_RANGE;
		case USB_CAN_HB:
			if (value_is_float != 0U)
				return USB_DATA_INVALID;
			return integer_value >= 0 &&
				CanConfigurationService_SetHeartbeatMs(context->application->can_configuration,
					(uint32_t)integer_value) ?
				USB_NO_ERROR : USB_DATA_OUT_OF_RANGE;
		case USB_RS:
			return UsbCommandRouter_WriteParameter(context,
				MOTOR_PARAMETER_PHASE_RESISTANCE_OHM, value);
		case USB_LD:
			return UsbCommandRouter_WriteParameter(context,
				MOTOR_PARAMETER_D_AXIS_INDUCTANCE_H, value);
		case USB_LQ:
			return UsbCommandRouter_WriteParameter(context,
				MOTOR_PARAMETER_Q_AXIS_INDUCTANCE_H, value);
		case USB_FLUX:
			return UsbCommandRouter_WriteParameter(context,
				MOTOR_PARAMETER_FLUX_WEBER, value);
		case USB_COGGING:
			return USB_NO_ERROR;
		case USB_FRICTION_APPLY:
			if (value_is_float != 0U || integer_value != 1)
				return USB_DATA_INVALID;
			return FrictionIdentificationService_ApplyCandidate(
				context->application->friction_identification) ?
				USB_NO_ERROR : USB_WRITE_INVALID;
		case USB_VBUS:
		case USB_IBUS:
		case USB_IA:
		case USB_IB:
		case USB_IC:
		case USB_ID:
		case USB_IQ:
		case USB_SPEED2_FILT:
		case USB_POS2_FILT:
		case USB_TEMP:
		case USB_ERROR:
		case USB_LUT_EXPORT:
		case USB_FRICTION_STATUS:
		case USB_FRICTION_COULOMB_POS:
		case USB_FRICTION_COULOMB_NEG:
		case USB_FRICTION_VISCOUS_POS:
		case USB_FRICTION_VISCOUS_NEG:
		case USB_FRICTION_RMSE_POS:
		case USB_FRICTION_RMSE_NEG:
		case USB_FRICTION_VALID:
		case USB_FRICTION_DATA_EXPORT:
		case USB_ACTIVE_COULOMB_POS:
		case USB_ACTIVE_COULOMB_NEG:
		case USB_ACTIVE_VISCOUS_POS:
		case USB_ACTIVE_VISCOUS_NEG:
		case USB_COMMISSIONING_STAGE:
		case USB_COMMISSIONING_PROGRESS:
		case USB_RESISTANCE_SPREAD:
		case USB_RESISTANCE_DESIGN_ERROR:
			return USB_WRITE_INVALID;
		default:
			return USB_UNKNOWNED_PARAM;
	}

	if (command_result == MOTOR_COMMAND_INVALID_VALUE)
		return USB_DATA_INVALID;
	if (command_result == MOTOR_COMMAND_OUT_OF_RANGE)
		return USB_DATA_OUT_OF_RANGE;
	if (command_result != MOTOR_COMMAND_ACCEPTED)
		return USB_WRITE_INVALID;
	ControlAuthorityService_Claim(context->application->control_authority,
		CONTROL_AUTHORITY_USB);
	return USB_NO_ERROR;
}

static void UsbCommandRouter_FormatLiteral(UsbCommandRouterResponse *response,
	const char *literal)
{
	TextWriter writer;

	TextWriter_Initialize(&writer, response->text, sizeof(response->text));
	(void)TextWriter_AppendLiteral(&writer, literal);
}

static void UsbCommandRouter_FormatU32(UsbCommandRouterResponse *response,
	const char *prefix, uint32_t value, const char *suffix)
{
	TextWriter writer;

	TextWriter_Initialize(&writer, response->text, sizeof(response->text));
	(void)TextWriter_AppendLiteral(&writer, prefix);
	(void)TextWriter_AppendU32(&writer, value);
	(void)TextWriter_AppendLiteral(&writer, suffix);
}

static void UsbCommandRouter_FormatI32(UsbCommandRouterResponse *response,
	const char *prefix, int32_t value, const char *suffix)
{
	TextWriter writer;

	TextWriter_Initialize(&writer, response->text, sizeof(response->text));
	(void)TextWriter_AppendLiteral(&writer, prefix);
	(void)TextWriter_AppendI32(&writer, value);
	(void)TextWriter_AppendLiteral(&writer, suffix);
}

static void UsbCommandRouter_FormatFixed(UsbCommandRouterResponse *response,
	const char *prefix, float value, uint8_t precision, const char *suffix)
{
	TextWriter writer;

	TextWriter_Initialize(&writer, response->text, sizeof(response->text));
	(void)TextWriter_AppendLiteral(&writer, prefix);
	(void)TextWriter_AppendFixedF32(&writer, value, precision);
	(void)TextWriter_AppendLiteral(&writer, suffix);
}

static UsbCommandError UsbCommandRouter_Read(UsbCommandRouterContext *context,
	const UsbCommandRouterState *state, UsbCommandRouterResponse *response,
	UsbParameterId parameter)
{
	float value;
	TextWriter writer;
	FrictionIdentificationPortStatus friction;
	bool friction_available = FrictionIdentificationService_ReadStatus(
		context->application->friction_identification, &friction);

	switch (parameter)
	{
		case USB_MODE:
			UsbCommandRouter_FormatI32(response, "mode=", (int32_t)
				UsbCommandRouter_ReadTelemetry(context, MOTOR_TELEMETRY_MODE, 1.0f),
				"\r\n"); break;
		case USB_CURRENT_SET:
			UsbCommandRouter_FormatFixed(response, "i_set=",
				UsbCommandRouter_ReadTelemetry(context,
					MOTOR_TELEMETRY_CURRENT_REFERENCE_A, 1.0f), 2U, "A\r\n"); break;
		case USB_SPEED_SET:
			UsbCommandRouter_FormatFixed(response, "spd_set=",
				UsbCommandRouter_ReadTelemetry(context,
					MOTOR_TELEMETRY_SPEED_REFERENCE_RAD_S,
					USB_COMMAND_ROUTER_ONE_BY_2PI), 2U, "r/s\r\n"); break;
		case USB_POS_SET:
			UsbCommandRouter_FormatFixed(response, "pos_set=",
				UsbCommandRouter_ReadTelemetry(context,
					MOTOR_TELEMETRY_POSITION_REFERENCE_RAD,
					USB_COMMAND_ROUTER_ONE_BY_2PI), 2U, "r\r\n"); break;
		case USB_NODE_ID:
			UsbCommandRouter_FormatU32(response, "can_id=",
				CanConfigurationService_GetNodeId(
					context->application->can_configuration), "\r\n"); break;
		case USB_POLEPARIS:
			UsbCommandRouter_FormatI32(response, "pol=", (int32_t)
				UsbCommandRouter_ReadParameter(context, MOTOR_PARAMETER_POLE_PAIRS,
					1.0f), "\r\n"); break;
		case USB_ENCODER_STATE:
			TextWriter_Initialize(&writer, response->text, sizeof(response->text));
			(void)TextWriter_AppendLiteral(&writer, "encoder=");
			(void)TextWriter_AppendLiteral(&writer,
				UsbCommandRouter_ReadTelemetry(context,
					MOTOR_TELEMETRY_ENCODER_ONLINE, 1.0f) != 0.0f ?
					"Online" : "Offline");
			(void)TextWriter_AppendLiteral(&writer, ".\r\n"); break;
		case USB_ENCODER_REVERSE:
			UsbCommandRouter_FormatU32(response, "erv=", (uint32_t)
				UsbCommandRouter_ReadTelemetry(context,
					MOTOR_TELEMETRY_ENCODER_REVERSED, 1.0f), "\r\n"); break;
		case USB_CURRENT_CAL:
			UsbCommandRouter_FormatFixed(response, "i_cal=",
				UsbCommandRouter_ReadParameter(context,
					MOTOR_PARAMETER_CALIBRATION_CURRENT_A, 1.0f), 2U, "A\r\n"); break;
		case USB_CURRENT_LIMIT:
			UsbCommandRouter_FormatFixed(response, "i_lim=",
				UsbCommandRouter_ReadParameter(context,
					MOTOR_PARAMETER_CURRENT_LIMIT_A, 1.0f), 2U, "A\r\n"); break;
		case USB_SPEED_LIMIT:
			UsbCommandRouter_FormatFixed(response, "spd_lim=",
				UsbCommandRouter_ReadParameter(context, MOTOR_PARAMETER_SPEED_LIMIT_RAD_S,
					USB_COMMAND_ROUTER_ONE_BY_2PI), 2U, "r/s\r\n"); break;
		case USB_SPEED_ACC:
			UsbCommandRouter_FormatFixed(response, "spd_acc=",
				UsbCommandRouter_ReadParameter(context,
					MOTOR_PARAMETER_SPEED_ACCELERATION_RAD_S2,
					USB_COMMAND_ROUTER_ONE_BY_2PI), 2U, "r/s2\r\n"); break;
		case USB_SPEED_DEC:
			UsbCommandRouter_FormatFixed(response, "spd_dec=",
				UsbCommandRouter_ReadParameter(context,
					MOTOR_PARAMETER_SPEED_DECELERATION_RAD_S2,
					USB_COMMAND_ROUTER_ONE_BY_2PI), 2U, "r/s2\r\n"); break;
		case USB_SPEED_KP:
			UsbCommandRouter_FormatFixed(response, "spd_kp=",
				UsbCommandRouter_ReadParameter(context, MOTOR_PARAMETER_SPEED_KP,
					1.0f), 2U, "\r\n"); break;
		case USB_SPEED_KI:
			UsbCommandRouter_FormatFixed(response, "spd_ki=",
				UsbCommandRouter_ReadParameter(context, MOTOR_PARAMETER_SPEED_KI,
					1.0f), 2U, "\r\n"); break;
		case USB_POS_ACC:
			UsbCommandRouter_FormatFixed(response, "pos_acc=",
				UsbCommandRouter_ReadParameter(context,
					MOTOR_PARAMETER_POSITION_ACCELERATION_RAD_S2,
					USB_COMMAND_ROUTER_ONE_BY_2PI), 3U, "r/s2\r\n"); break;
		case USB_POS_DEC:
			UsbCommandRouter_FormatFixed(response, "pos_dec=",
				UsbCommandRouter_ReadParameter(context,
					MOTOR_PARAMETER_POSITION_DECELERATION_RAD_S2,
					USB_COMMAND_ROUTER_ONE_BY_2PI), 3U, "r/s2\r\n"); break;
		case USB_POS_MAXSPEED:
			UsbCommandRouter_FormatFixed(response, "pos_maxspd=",
				UsbCommandRouter_ReadParameter(context,
					MOTOR_PARAMETER_POSITION_MAX_SPEED_RAD_S,
					USB_COMMAND_ROUTER_ONE_BY_2PI), 3U, "r/s\r\n"); break;
		case USB_POS_KP:
			UsbCommandRouter_FormatFixed(response, "pos_kp=",
				UsbCommandRouter_ReadParameter(context,
					MOTOR_PARAMETER_POSITION_KP_A_PER_RAD, 1.0f), 2U, "\r\n"); break;
		case USB_POS_KD:
			UsbCommandRouter_FormatFixed(response, "pos_kd=",
				UsbCommandRouter_ReadParameter(context,
					MOTOR_PARAMETER_POSITION_KD_A_PER_RAD_S, 1.0f), 2U, "\r\n"); break;
		case USB_POS_KI:
			UsbCommandRouter_FormatFixed(response, "pos_ki=",
				UsbCommandRouter_ReadParameter(context,
					MOTOR_PARAMETER_POSITION_KI_A_PER_RAD_S, 1.0f), 3U, "\r\n"); break;
		case USB_POS_INTEGRAL_LIMIT:
			UsbCommandRouter_FormatFixed(response, "pos_i_limit=",
				UsbCommandRouter_ReadParameter(context,
					MOTOR_PARAMETER_POSITION_INTEGRAL_LIMIT_A, 1.0f), 3U, "A\r\n"); break;
		case USB_CASCADE_POS_KP:
			UsbCommandRouter_FormatFixed(response, "cascade_pos_kp=",
				UsbCommandRouter_ReadParameter(context,
					MOTOR_PARAMETER_CASCADE_POSITION_KP_PER_S, 1.0f), 3U, "\r\n"); break;
		case USB_CASCADE_POS_KD:
			UsbCommandRouter_FormatFixed(response, "cascade_pos_kd=",
				UsbCommandRouter_ReadParameter(context,
					MOTOR_PARAMETER_CASCADE_POSITION_KD, 1.0f), 3U, "\r\n"); break;
		case USB_CAN_BR:
			UsbCommandRouter_FormatU32(response, "can_br=",
				CanConfigurationService_GetBitrateKbps(
					context->application->can_configuration), "kbps\r\n"); break;
		case USB_CAN_HB:
			UsbCommandRouter_FormatU32(response, "can_hb=",
				CanConfigurationService_GetHeartbeatMs(
					context->application->can_configuration), "ms\r\n"); break;
		case USB_VBUS: value = UsbCommandRouter_ReadTelemetry(context, MOTOR_TELEMETRY_BUS_VOLTAGE_V, 1.0f); UsbCommandRouter_FormatFixed(response, "vbus=", value, 2U, "V\r\n"); break;
		case USB_IBUS: value = UsbCommandRouter_ReadTelemetry(context, MOTOR_TELEMETRY_BUS_CURRENT_A, 1.0f); UsbCommandRouter_FormatFixed(response, "ibus=", value, 2U, "A\r\n"); break;
		case USB_IA: value = UsbCommandRouter_ReadTelemetry(context, MOTOR_TELEMETRY_PHASE_A_CURRENT_A, 1.0f); UsbCommandRouter_FormatFixed(response, "ia=", value, 2U, "A\r\n"); break;
		case USB_IB: value = UsbCommandRouter_ReadTelemetry(context, MOTOR_TELEMETRY_PHASE_B_CURRENT_A, 1.0f); UsbCommandRouter_FormatFixed(response, "ib=", value, 2U, "A\r\n"); break;
		case USB_IC: value = UsbCommandRouter_ReadTelemetry(context, MOTOR_TELEMETRY_PHASE_C_CURRENT_A, 1.0f); UsbCommandRouter_FormatFixed(response, "ic=", value, 2U, "A\r\n"); break;
		case USB_ID: value = UsbCommandRouter_ReadTelemetry(context, MOTOR_TELEMETRY_D_AXIS_CURRENT_A, 1.0f); UsbCommandRouter_FormatFixed(response, "id=", value, 2U, "A\r\n"); break;
		case USB_IQ: value = UsbCommandRouter_ReadTelemetry(context, MOTOR_TELEMETRY_Q_AXIS_CURRENT_A, 1.0f); UsbCommandRouter_FormatFixed(response, "iq=", value, 2U, "A\r\n"); break;
		case USB_SPEED2_FILT: value = UsbCommandRouter_ReadTelemetry(context, MOTOR_TELEMETRY_MECHANICAL_SPEED_RAD_S, 1.0f); UsbCommandRouter_FormatFixed(response, "spd2_filt=", value, 2U, "r/s\r\n"); break;
		case USB_POS2_FILT: value = UsbCommandRouter_ReadTelemetry(context, MOTOR_TELEMETRY_MECHANICAL_POSITION_RAD, 1.0f); UsbCommandRouter_FormatFixed(response, "pos2_filt=", value, 3U, "r\r\n"); break;
		case USB_TEMP: value = UsbCommandRouter_ReadTelemetry(context, MOTOR_TELEMETRY_TEMPERATURE_C, 1.0f); UsbCommandRouter_FormatFixed(response, "temp=", value, 2U, "C\r\n"); break;
		case USB_RS: value = UsbCommandRouter_ReadParameter(context, MOTOR_PARAMETER_PHASE_RESISTANCE_OHM, 1000.0f); UsbCommandRouter_FormatFixed(response, "Rs=", value, 2U, "mohm\r\n"); break;
		case USB_LD: value = UsbCommandRouter_ReadParameter(context, MOTOR_PARAMETER_D_AXIS_INDUCTANCE_H, 1000000.0f); UsbCommandRouter_FormatFixed(response, "Ld=", value, 2U, "uH\r\n"); break;
		case USB_LQ: value = UsbCommandRouter_ReadParameter(context, MOTOR_PARAMETER_Q_AXIS_INDUCTANCE_H, 1000000.0f); UsbCommandRouter_FormatFixed(response, "Lq=", value, 2U, "uH\r\n"); break;
		case USB_FLUX: value = UsbCommandRouter_ReadParameter(context, MOTOR_PARAMETER_FLUX_WEBER, 1000.0f); UsbCommandRouter_FormatFixed(response, "Flux=", value, 2U, "mWb\r\n"); break;
		case USB_ERROR: UsbCommandRouter_FormatI32(response, "error=", (int32_t)UsbCommandRouter_ReadTelemetry(context, MOTOR_TELEMETRY_PRIMARY_ERROR, 1.0f), "\r\n"); break;
		case USB_LUT_EXPORT:
			if (state->print_active || state->lut_export_active ||
				state->friction_export_active)
				return USB_WRITE_INVALID;
			TextWriter_Initialize(&writer, response->text, sizeof(response->text));
			(void)TextWriter_AppendLiteral(&writer, "lut_begin,count=");
			(void)TextWriter_AppendU32(&writer,
				RotorCalibrationService_GetEntryCount(
					context->application->rotor_calibration));
			(void)TextWriter_AppendLiteral(&writer, ",reverse=");
			(void)TextWriter_AppendU32(&writer, (uint32_t)
				UsbCommandRouter_ReadTelemetry(context,
					MOTOR_TELEMETRY_ENCODER_REVERSED, 1.0f));
			(void)TextWriter_AppendLiteral(&writer, "\r\n");
			response->action = USB_COMMAND_ROUTER_ACTION_BEGIN_LUT_EXPORT;
			break;
		case USB_FRICTION_STATUS:
			if (!friction_available) return USB_WRITE_INVALID;
			TextWriter_Initialize(&writer, response->text, sizeof(response->text));
			(void)TextWriter_AppendLiteral(&writer, "friction_state=");
			(void)TextWriter_AppendU32(&writer, friction.state);
			(void)TextWriter_AppendLiteral(&writer, ",reason=");
			(void)TextWriter_AppendU32(&writer, friction.reason);
			(void)TextWriter_AppendLiteral(&writer, ",point=");
			(void)TextWriter_AppendU32(&writer, friction.point_index);
			(void)TextWriter_AppendLiteral(&writer, ",progress=");
			(void)TextWriter_AppendFixedF32(&writer, friction.progress_percent, 1U);
			(void)TextWriter_AppendLiteral(&writer, ",candidate=");
			(void)TextWriter_AppendU32(&writer, friction.candidate_valid);
			(void)TextWriter_AppendLiteral(&writer, "\r\n"); break;
		case USB_FRICTION_COULOMB_POS:
			if (!friction_available) return USB_WRITE_INVALID;
			UsbCommandRouter_FormatFixed(response, "friction_coulomb_pos=", friction.candidate_coulomb_pos_a, 6U, "A\r\n"); break;
		case USB_FRICTION_COULOMB_NEG:
			if (!friction_available) return USB_WRITE_INVALID;
			UsbCommandRouter_FormatFixed(response, "friction_coulomb_neg=", friction.candidate_coulomb_neg_a, 6U, "A\r\n"); break;
		case USB_FRICTION_VISCOUS_POS:
			if (!friction_available) return USB_WRITE_INVALID;
			UsbCommandRouter_FormatFixed(response, "friction_viscous_pos=", friction.candidate_viscous_pos_a_per_rad_s, 6U, "A_per_rad_s\r\n"); break;
		case USB_FRICTION_VISCOUS_NEG:
			if (!friction_available) return USB_WRITE_INVALID;
			UsbCommandRouter_FormatFixed(response, "friction_viscous_neg=", friction.candidate_viscous_neg_a_per_rad_s, 6U, "A_per_rad_s\r\n"); break;
		case USB_FRICTION_RMSE_POS:
			if (!friction_available) return USB_WRITE_INVALID;
			UsbCommandRouter_FormatFixed(response, "friction_rmse_pos=", friction.candidate_rmse_pos_a, 6U, "A\r\n"); break;
		case USB_FRICTION_RMSE_NEG:
			if (!friction_available) return USB_WRITE_INVALID;
			UsbCommandRouter_FormatFixed(response, "friction_rmse_neg=", friction.candidate_rmse_neg_a, 6U, "A\r\n"); break;
		case USB_FRICTION_VALID:
			if (!friction_available) return USB_WRITE_INVALID;
			UsbCommandRouter_FormatU32(response, "friction_model_valid=", friction.active_model_valid, "\r\n"); break;
		case USB_ACTIVE_COULOMB_POS:
			if (!friction_available) return USB_WRITE_INVALID;
			UsbCommandRouter_FormatFixed(response, "active_coulomb_pos=", friction.active_coulomb_pos_a, 6U, "A\r\n"); break;
		case USB_ACTIVE_COULOMB_NEG:
			if (!friction_available) return USB_WRITE_INVALID;
			UsbCommandRouter_FormatFixed(response, "active_coulomb_neg=", friction.active_coulomb_neg_a, 6U, "A\r\n"); break;
		case USB_ACTIVE_VISCOUS_POS:
			if (!friction_available) return USB_WRITE_INVALID;
			UsbCommandRouter_FormatFixed(response, "active_viscous_pos=", friction.active_viscous_pos_a_per_rad_s, 6U, "A_per_rad_s\r\n"); break;
		case USB_ACTIVE_VISCOUS_NEG:
			if (!friction_available) return USB_WRITE_INVALID;
			UsbCommandRouter_FormatFixed(response, "active_viscous_neg=", friction.active_viscous_neg_a_per_rad_s, 6U, "A_per_rad_s\r\n"); break;
		case USB_COMMISSIONING_STAGE:
			UsbCommandRouter_FormatU32(response, "cst=", (uint32_t)
				UsbCommandRouter_ReadTelemetry(context,
					MOTOR_TELEMETRY_COMMISSIONING_STAGE, 1.0f), "\r\n"); break;
		case USB_COMMISSIONING_PROGRESS:
			UsbCommandRouter_FormatU32(response, "cpr=", (uint32_t)
				UsbCommandRouter_ReadTelemetry(context,
					MOTOR_TELEMETRY_COMMISSIONING_PROGRESS_PERCENT, 1.0f), "%\r\n"); break;
		case USB_RESISTANCE_SPREAD:
			UsbCommandRouter_FormatFixed(response, "rsp=",
				UsbCommandRouter_ReadTelemetry(context,
					MOTOR_TELEMETRY_PHASE_RESISTANCE_SPREAD_PERCENT, 1.0f),
				2U, "%\r\n"); break;
		case USB_RESISTANCE_DESIGN_ERROR:
			UsbCommandRouter_FormatFixed(response, "rde=",
				UsbCommandRouter_ReadTelemetry(context,
					MOTOR_TELEMETRY_PHASE_RESISTANCE_DESIGN_ERROR_PERCENT, 1.0f),
				2U, "%\r\n"); break;
		case USB_FRICTION_DATA_EXPORT:
			if (!friction_available || !friction.candidate_valid || state->print_active ||
				state->lut_export_active || state->friction_export_active)
				return USB_WRITE_INVALID;
			UsbCommandRouter_FormatU32(response, "friction_begin,count=",
				friction.sample_count, "\r\n");
			response->action = USB_COMMAND_ROUTER_ACTION_BEGIN_FRICTION_EXPORT;
			break;
		default:
			return USB_UNKNOWNED_PARAM;
	}
	return USB_NO_ERROR;
}

static bool UsbCommandRouter_PrintScale(UsbParameterId parameter, float *scale)
{
	if (scale == NULL)
		return false;
	*scale = 1.0f;
	switch (parameter)
	{
		case USB_SPEED_SET:
		case USB_POS_SET:
		case USB_SPEED_LIMIT:
		case USB_SPEED_ACC:
		case USB_SPEED_DEC:
		case USB_POS_ACC:
		case USB_POS_DEC:
		case USB_POS_MAXSPEED:
			*scale = USB_COMMAND_ROUTER_ONE_BY_2PI; break;
		case USB_RS: *scale = 1000.0f; break;
		case USB_LD:
		case USB_LQ: *scale = 1000000.0f; break;
		case USB_FLUX: *scale = 1000.0f; break;
		case USB_COGGING:
		case USB_LUT_EXPORT:
		case USB_FRICTION_STATUS:
		case USB_FRICTION_APPLY:
		case USB_FRICTION_COULOMB_POS:
		case USB_FRICTION_COULOMB_NEG:
		case USB_FRICTION_VISCOUS_POS:
		case USB_FRICTION_VISCOUS_NEG:
		case USB_FRICTION_RMSE_POS:
		case USB_FRICTION_RMSE_NEG:
		case USB_FRICTION_VALID:
		case USB_FRICTION_DATA_EXPORT:
		case USB_ACTIVE_COULOMB_POS:
		case USB_ACTIVE_COULOMB_NEG:
		case USB_ACTIVE_VISCOUS_POS:
		case USB_ACTIVE_VISCOUS_NEG:
			return false;
		default:
			break;
	}
	return true;
}

UsbCommandError UsbCommandRouter_Handle(UsbCommandRouterContext *context,
	const UsbProtocolV1Command *command,
	const UsbCommandRouterState *state, UsbCommandRouterResponse *response)
{
	UsbCommandError result;
	int channel;
	UsbParameterId parameter_id;

	if (context == NULL || command == NULL || state == NULL || response == NULL ||
		!isfinite(command->value) || command->value >= (float)INT_MAX ||
		command->value <= (float)INT_MIN)
		return USB_DATA_INVALID;
	response->action = USB_COMMAND_ROUTER_ACTION_NONE;
	response->text[0] = '\0';
	parameter_id = (UsbParameterId)command->parameter_id;
	if (command->operation == USB_PROTOCOL_V1_OPERATION_WRITE)
	{
		result = UsbCommandRouter_Write(context, parameter_id, command->value,
			command->value_is_float ? 1U : 0U);
		if (result == USB_NO_ERROR)
		{
			UsbCommandRouter_FormatLiteral(response, "Write Success!\r\n");
			response->action = USB_COMMAND_ROUTER_ACTION_SEND_TEXT;
		}
		return result;
	}
	if (command->operation == USB_PROTOCOL_V1_OPERATION_READ)
	{
		result = UsbCommandRouter_Read(context, state, response, parameter_id);
		if (result == USB_NO_ERROR && response->action == USB_COMMAND_ROUTER_ACTION_NONE)
			response->action = USB_COMMAND_ROUTER_ACTION_SEND_TEXT;
		return result;
	}
	if (command->operation != USB_PROTOCOL_V1_OPERATION_PRINT)
		return USB_SYNTAX_ERROR;

	channel = (int)command->value;
	if (command->value_is_float || channel < 0 || channel >= 5)
		return USB_DATA_INVALID;
	if (!UsbCommandRouter_PrintScale(parameter_id,
		&response->print_scale))
		return USB_UNKNOWNED_PARAM;
	response->print_channel = (uint8_t)channel;
	response->print_parameter = parameter_id;
	response->action = USB_COMMAND_ROUTER_ACTION_CONFIGURE_PRINT;
	return USB_NO_ERROR;
}
