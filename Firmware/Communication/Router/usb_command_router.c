#include "usb_command_router.h"

#include "can_configuration_service.h"
#include "motor_command_service.h"
#include "parameter_service.h"
#include "rotor_calibration_service.h"
#include "telemetry_service.h"
#include "friction_identification_service.h"

#include <limits.h>
#include <math.h>
#include <stdio.h>

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

static UsbCommandError UsbCommandRouter_Read(UsbCommandRouterContext *context,
	const UsbCommandRouterState *state, UsbCommandRouterResponse *response,
	UsbParameterId parameter)
{
	float value;
	FrictionIdentificationPortStatus friction;
	bool friction_available = FrictionIdentificationService_ReadStatus(
		context->application->friction_identification, &friction);

	switch (parameter)
	{
		case USB_MODE:
			snprintf(response->text, sizeof(response->text), "mode=%d\r\n", (int)UsbCommandRouter_ReadTelemetry(context, MOTOR_TELEMETRY_MODE, 1.0f)); break;
		case USB_CURRENT_SET:
			snprintf(response->text, sizeof(response->text), "i_set=%.2fA\r\n", UsbCommandRouter_ReadTelemetry(context, MOTOR_TELEMETRY_CURRENT_REFERENCE_A, 1.0f)); break;
		case USB_SPEED_SET:
			snprintf(response->text, sizeof(response->text), "spd_set=%.2fr/s\r\n", UsbCommandRouter_ReadTelemetry(context, MOTOR_TELEMETRY_SPEED_REFERENCE_RAD_S, USB_COMMAND_ROUTER_ONE_BY_2PI)); break;
		case USB_POS_SET:
			snprintf(response->text, sizeof(response->text), "pos_set=%.2fr\r\n", UsbCommandRouter_ReadTelemetry(context, MOTOR_TELEMETRY_POSITION_REFERENCE_RAD, USB_COMMAND_ROUTER_ONE_BY_2PI)); break;
		case USB_NODE_ID:
			snprintf(response->text, sizeof(response->text), "can_id=%u\r\n", (unsigned int)CanConfigurationService_GetNodeId(context->application->can_configuration)); break;
		case USB_POLEPARIS:
			snprintf(response->text, sizeof(response->text), "pol=%d\r\n", (int)UsbCommandRouter_ReadParameter(context, MOTOR_PARAMETER_POLE_PAIRS, 1.0f)); break;
		case USB_ENCODER_STATE:
			snprintf(response->text, sizeof(response->text), "encoder=TLE5012B-%s.\r\n", UsbCommandRouter_ReadTelemetry(context, MOTOR_TELEMETRY_ENCODER_ONLINE, 1.0f) != 0.0f ? "Online" : "Offline"); break;
		case USB_ENCODER_REVERSE:
			snprintf(response->text, sizeof(response->text), "erv=%u\r\n", (unsigned int)UsbCommandRouter_ReadTelemetry(context, MOTOR_TELEMETRY_ENCODER_REVERSED, 1.0f)); break;
		case USB_CURRENT_CAL:
			snprintf(response->text, sizeof(response->text), "i_cal=%.2fA\r\n", UsbCommandRouter_ReadParameter(context, MOTOR_PARAMETER_CALIBRATION_CURRENT_A, 1.0f)); break;
		case USB_CURRENT_LIMIT:
			snprintf(response->text, sizeof(response->text), "i_lim=%.2fA\r\n", UsbCommandRouter_ReadParameter(context, MOTOR_PARAMETER_CURRENT_LIMIT_A, 1.0f)); break;
		case USB_SPEED_LIMIT:
			snprintf(response->text, sizeof(response->text), "spd_lim=%.2fr/s\r\n", UsbCommandRouter_ReadParameter(context, MOTOR_PARAMETER_SPEED_LIMIT_RAD_S, USB_COMMAND_ROUTER_ONE_BY_2PI)); break;
		case USB_SPEED_ACC:
			snprintf(response->text, sizeof(response->text), "spd_acc=%.2fr/s2\r\n", UsbCommandRouter_ReadParameter(context, MOTOR_PARAMETER_SPEED_ACCELERATION_RAD_S2, USB_COMMAND_ROUTER_ONE_BY_2PI)); break;
		case USB_SPEED_DEC:
			snprintf(response->text, sizeof(response->text), "spd_dec=%.2fr/s2\r\n", UsbCommandRouter_ReadParameter(context, MOTOR_PARAMETER_SPEED_DECELERATION_RAD_S2, USB_COMMAND_ROUTER_ONE_BY_2PI)); break;
		case USB_SPEED_KP:
			snprintf(response->text, sizeof(response->text), "spd_kp=%.2f\r\n", UsbCommandRouter_ReadParameter(context, MOTOR_PARAMETER_SPEED_KP, 1.0f)); break;
		case USB_SPEED_KI:
			snprintf(response->text, sizeof(response->text), "spd_ki=%.2f\r\n", UsbCommandRouter_ReadParameter(context, MOTOR_PARAMETER_SPEED_KI, 1.0f)); break;
		case USB_POS_ACC:
			snprintf(response->text, sizeof(response->text), "pos_acc=%.3fr/s2\r\n", UsbCommandRouter_ReadParameter(context, MOTOR_PARAMETER_POSITION_ACCELERATION_RAD_S2, USB_COMMAND_ROUTER_ONE_BY_2PI)); break;
		case USB_POS_DEC:
			snprintf(response->text, sizeof(response->text), "pos_dec=%.3fr/s2\r\n", UsbCommandRouter_ReadParameter(context, MOTOR_PARAMETER_POSITION_DECELERATION_RAD_S2, USB_COMMAND_ROUTER_ONE_BY_2PI)); break;
		case USB_POS_MAXSPEED:
			snprintf(response->text, sizeof(response->text), "pos_maxspd=%.3fr/s\r\n", UsbCommandRouter_ReadParameter(context, MOTOR_PARAMETER_POSITION_MAX_SPEED_RAD_S, USB_COMMAND_ROUTER_ONE_BY_2PI)); break;
		case USB_POS_KP:
			snprintf(response->text, sizeof(response->text), "pos_kp=%.2f\r\n", UsbCommandRouter_ReadParameter(context, MOTOR_PARAMETER_POSITION_KP_A_PER_RAD, 1.0f)); break;
		case USB_POS_KD:
			snprintf(response->text, sizeof(response->text), "pos_kd=%.2f\r\n", UsbCommandRouter_ReadParameter(context, MOTOR_PARAMETER_POSITION_KD_A_PER_RAD_S, 1.0f)); break;
		case USB_POS_KI:
			snprintf(response->text, sizeof(response->text), "pos_ki=%.3f\r\n", UsbCommandRouter_ReadParameter(context, MOTOR_PARAMETER_POSITION_KI_A_PER_RAD_S, 1.0f)); break;
		case USB_POS_INTEGRAL_LIMIT:
			snprintf(response->text, sizeof(response->text), "pos_i_limit=%.3fA\r\n", UsbCommandRouter_ReadParameter(context, MOTOR_PARAMETER_POSITION_INTEGRAL_LIMIT_A, 1.0f)); break;
		case USB_CASCADE_POS_KP:
			snprintf(response->text, sizeof(response->text), "cascade_pos_kp=%.3f\r\n", UsbCommandRouter_ReadParameter(context, MOTOR_PARAMETER_CASCADE_POSITION_KP_PER_S, 1.0f)); break;
		case USB_CASCADE_POS_KD:
			snprintf(response->text, sizeof(response->text), "cascade_pos_kd=%.3f\r\n", UsbCommandRouter_ReadParameter(context, MOTOR_PARAMETER_CASCADE_POSITION_KD, 1.0f)); break;
		case USB_CAN_BR:
			snprintf(response->text, sizeof(response->text), "can_br=%ukbps\r\n", (unsigned int)CanConfigurationService_GetBitrateKbps(context->application->can_configuration)); break;
		case USB_CAN_HB:
			snprintf(response->text, sizeof(response->text), "can_hb=%ums\r\n", (unsigned int)CanConfigurationService_GetHeartbeatMs(context->application->can_configuration)); break;
		case USB_VBUS: value = UsbCommandRouter_ReadTelemetry(context, MOTOR_TELEMETRY_BUS_VOLTAGE_V, 1.0f); snprintf(response->text, sizeof(response->text), "vbus=%.2fV\r\n", value); break;
		case USB_IBUS: value = UsbCommandRouter_ReadTelemetry(context, MOTOR_TELEMETRY_BUS_CURRENT_A, 1.0f); snprintf(response->text, sizeof(response->text), "ibus=%.2fA\r\n", value); break;
		case USB_IA: value = UsbCommandRouter_ReadTelemetry(context, MOTOR_TELEMETRY_PHASE_A_CURRENT_A, 1.0f); snprintf(response->text, sizeof(response->text), "ia=%.2fA\r\n", value); break;
		case USB_IB: value = UsbCommandRouter_ReadTelemetry(context, MOTOR_TELEMETRY_PHASE_B_CURRENT_A, 1.0f); snprintf(response->text, sizeof(response->text), "ib=%.2fA\r\n", value); break;
		case USB_IC: value = UsbCommandRouter_ReadTelemetry(context, MOTOR_TELEMETRY_PHASE_C_CURRENT_A, 1.0f); snprintf(response->text, sizeof(response->text), "ic=%.2fA\r\n", value); break;
		case USB_ID: value = UsbCommandRouter_ReadTelemetry(context, MOTOR_TELEMETRY_D_AXIS_CURRENT_A, 1.0f); snprintf(response->text, sizeof(response->text), "id=%.2fA\r\n", value); break;
		case USB_IQ: value = UsbCommandRouter_ReadTelemetry(context, MOTOR_TELEMETRY_Q_AXIS_CURRENT_A, 1.0f); snprintf(response->text, sizeof(response->text), "iq=%.2fA\r\n", value); break;
		case USB_SPEED2_FILT: value = UsbCommandRouter_ReadTelemetry(context, MOTOR_TELEMETRY_MECHANICAL_SPEED_RAD_S, 1.0f); snprintf(response->text, sizeof(response->text), "spd2_filt=%.2fr/s\r\n", value); break;
		case USB_POS2_FILT: value = UsbCommandRouter_ReadTelemetry(context, MOTOR_TELEMETRY_MECHANICAL_POSITION_RAD, 1.0f); snprintf(response->text, sizeof(response->text), "pos2_filt=%.3fr\r\n", value); break;
		case USB_TEMP: value = UsbCommandRouter_ReadTelemetry(context, MOTOR_TELEMETRY_TEMPERATURE_C, 1.0f); snprintf(response->text, sizeof(response->text), "temp=%.2fC\r\n", value); break;
		case USB_RS: value = UsbCommandRouter_ReadParameter(context, MOTOR_PARAMETER_PHASE_RESISTANCE_OHM, 1000.0f); snprintf(response->text, sizeof(response->text), "Rs=%.2fmohm\r\n", value); break;
		case USB_LD: value = UsbCommandRouter_ReadParameter(context, MOTOR_PARAMETER_D_AXIS_INDUCTANCE_H, 1000000.0f); snprintf(response->text, sizeof(response->text), "Ld=%.2fuH\r\n", value); break;
		case USB_LQ: value = UsbCommandRouter_ReadParameter(context, MOTOR_PARAMETER_Q_AXIS_INDUCTANCE_H, 1000000.0f); snprintf(response->text, sizeof(response->text), "Lq=%.2fuH\r\n", value); break;
		case USB_FLUX: value = UsbCommandRouter_ReadParameter(context, MOTOR_PARAMETER_FLUX_WEBER, 1000.0f); snprintf(response->text, sizeof(response->text), "Flux=%.2fmWb\r\n", value); break;
		case USB_ERROR: snprintf(response->text, sizeof(response->text), "error=%d\r\n", (int)UsbCommandRouter_ReadTelemetry(context, MOTOR_TELEMETRY_PRIMARY_ERROR, 1.0f)); break;
		case USB_LUT_EXPORT:
			if (state->print_active || state->lut_export_active ||
				state->friction_export_active)
				return USB_WRITE_INVALID;
			snprintf(response->text, sizeof(response->text), "lut_begin,count=%u,reverse=%u\r\n",
				(unsigned int)RotorCalibrationService_GetEntryCount(
					context->application->rotor_calibration),
				(unsigned int)UsbCommandRouter_ReadTelemetry(context,
					MOTOR_TELEMETRY_ENCODER_REVERSED, 1.0f));
			response->action = USB_COMMAND_ROUTER_ACTION_BEGIN_LUT_EXPORT;
			break;
		case USB_FRICTION_STATUS:
			if (!friction_available) return USB_WRITE_INVALID;
			snprintf(response->text, sizeof(response->text),
				"friction_state=%u,reason=%u,point=%u,progress=%.1f,candidate=%u\r\n",
				(unsigned int)friction.state, (unsigned int)friction.reason,
				(unsigned int)friction.point_index, friction.progress_percent,
				(unsigned int)friction.candidate_valid); break;
		case USB_FRICTION_COULOMB_POS:
			if (!friction_available) return USB_WRITE_INVALID;
			snprintf(response->text, sizeof(response->text), "friction_coulomb_pos=%.6fA\r\n", friction.candidate_coulomb_pos_a); break;
		case USB_FRICTION_COULOMB_NEG:
			if (!friction_available) return USB_WRITE_INVALID;
			snprintf(response->text, sizeof(response->text), "friction_coulomb_neg=%.6fA\r\n", friction.candidate_coulomb_neg_a); break;
		case USB_FRICTION_VISCOUS_POS:
			if (!friction_available) return USB_WRITE_INVALID;
			snprintf(response->text, sizeof(response->text), "friction_viscous_pos=%.6fA_per_rad_s\r\n", friction.candidate_viscous_pos_a_per_rad_s); break;
		case USB_FRICTION_VISCOUS_NEG:
			if (!friction_available) return USB_WRITE_INVALID;
			snprintf(response->text, sizeof(response->text), "friction_viscous_neg=%.6fA_per_rad_s\r\n", friction.candidate_viscous_neg_a_per_rad_s); break;
		case USB_FRICTION_RMSE_POS:
			if (!friction_available) return USB_WRITE_INVALID;
			snprintf(response->text, sizeof(response->text), "friction_rmse_pos=%.6fA\r\n", friction.candidate_rmse_pos_a); break;
		case USB_FRICTION_RMSE_NEG:
			if (!friction_available) return USB_WRITE_INVALID;
			snprintf(response->text, sizeof(response->text), "friction_rmse_neg=%.6fA\r\n", friction.candidate_rmse_neg_a); break;
		case USB_FRICTION_VALID:
			if (!friction_available) return USB_WRITE_INVALID;
			snprintf(response->text, sizeof(response->text), "friction_model_valid=%u\r\n", (unsigned int)friction.active_model_valid); break;
		case USB_ACTIVE_COULOMB_POS:
			if (!friction_available) return USB_WRITE_INVALID;
			snprintf(response->text, sizeof(response->text), "active_coulomb_pos=%.6fA\r\n", friction.active_coulomb_pos_a); break;
		case USB_ACTIVE_COULOMB_NEG:
			if (!friction_available) return USB_WRITE_INVALID;
			snprintf(response->text, sizeof(response->text), "active_coulomb_neg=%.6fA\r\n", friction.active_coulomb_neg_a); break;
		case USB_ACTIVE_VISCOUS_POS:
			if (!friction_available) return USB_WRITE_INVALID;
			snprintf(response->text, sizeof(response->text), "active_viscous_pos=%.6fA_per_rad_s\r\n", friction.active_viscous_pos_a_per_rad_s); break;
		case USB_ACTIVE_VISCOUS_NEG:
			if (!friction_available) return USB_WRITE_INVALID;
			snprintf(response->text, sizeof(response->text), "active_viscous_neg=%.6fA_per_rad_s\r\n", friction.active_viscous_neg_a_per_rad_s); break;
		case USB_FRICTION_DATA_EXPORT:
			if (!friction_available || !friction.candidate_valid || state->print_active ||
				state->lut_export_active || state->friction_export_active)
				return USB_WRITE_INVALID;
			snprintf(response->text, sizeof(response->text),
				"friction_begin,count=%u\r\n", (unsigned int)friction.sample_count);
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
			snprintf(response->text, sizeof(response->text), "Write Success!\r\n");
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
