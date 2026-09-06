#include "Core/Communication/Router/can_command_router.h"

#include "Core/Application/Communication/can_configuration_service.h"
#include "Core/Communication/Can/can_response_service.h"
#include "Core/Application/motor_command_service.h"
#include "Core/Application/Parameters/parameter_service.h"
#include "Core/Application/rotor_calibration_service.h"
#include "Core/Infrastructure/Telemetry/telemetry_service.h"
#include "Core/Application/friction_identification_service.h"

#include <limits.h>
#include <math.h>
#include <stdint.h>

#define CAN_COMMAND_ROUTER_TWO_PI       6.2831853072f
#define CAN_COMMAND_ROUTER_ONE_BY_2PI   0.15915494309f
#define PROTOCOL_MODE_DISABLED    0
#define PROTOCOL_MODE_CURRENT     1
#define PROTOCOL_MODE_SPEED       2

typedef struct
{
	uint8_t set_id;
	uint8_t get_id;
	uint8_t parameter;
	uint8_t revolutions_unit;
} CanMotorParameterRoute;

typedef struct
{
	uint8_t response_id;
	uint8_t telemetry;
	uint8_t revolutions_unit;
} CanTelemetryRoute;

#define CAN_MOTOR_PARAMETER_ROUTE(set_id_, get_id_, parameter_, revolutions_) \
	{ (uint8_t)(set_id_), (uint8_t)(get_id_), (uint8_t)(parameter_), (uint8_t)(revolutions_) }

#define CAN_TELEMETRY_ROUTE(response_id_, telemetry_, revolutions_) \
	{ (uint8_t)(response_id_), (uint8_t)(telemetry_), (uint8_t)(revolutions_) }

/* Single CAN-v1 mapping for ordinary motor parameters and telemetry. */
static const CanMotorParameterRoute g_motor_parameter_routes[] =
{
	CAN_MOTOR_PARAMETER_ROUTE(CAN_SET_POLEPARIS, CAN_GET_POLEPARIS, MOTOR_PARAMETER_POLE_PAIRS, 0),
	CAN_MOTOR_PARAMETER_ROUTE(CAN_SET_CURRENT_CAL, CAN_GET_CURRENT_CAL, MOTOR_PARAMETER_CALIBRATION_CURRENT_A, 0),
	CAN_MOTOR_PARAMETER_ROUTE(CAN_SET_CURRENT_LIMIT, CAN_GET_CURRENT_LIMIT, MOTOR_PARAMETER_CURRENT_LIMIT_A, 0),
	CAN_MOTOR_PARAMETER_ROUTE(CAN_SET_SPEED_LIMIT, CAN_GET_SPEED_LIMIT, MOTOR_PARAMETER_SPEED_LIMIT_RAD_S, 1),
	CAN_MOTOR_PARAMETER_ROUTE(CAN_SET_SPEED_ACC, CAN_GET_SPEED_ACC, MOTOR_PARAMETER_SPEED_ACCELERATION_RAD_S2, 1),
	CAN_MOTOR_PARAMETER_ROUTE(CAN_SET_SPEED_DEC, CAN_GET_SPEED_DEC, MOTOR_PARAMETER_SPEED_DECELERATION_RAD_S2, 1),
	CAN_MOTOR_PARAMETER_ROUTE(CAN_SET_SPEED_KP, CAN_GET_SPEED_KP, MOTOR_PARAMETER_SPEED_KP, 0),
	CAN_MOTOR_PARAMETER_ROUTE(CAN_SET_SPEED_KI, CAN_GET_SPEED_KI, MOTOR_PARAMETER_SPEED_KI, 0),
	CAN_MOTOR_PARAMETER_ROUTE(CAN_SET_POS_ACC, CAN_GET_POS_ACC, MOTOR_PARAMETER_POSITION_ACCELERATION_RAD_S2, 1),
	CAN_MOTOR_PARAMETER_ROUTE(CAN_SET_POS_DEC, CAN_GET_POS_DEC, MOTOR_PARAMETER_POSITION_DECELERATION_RAD_S2, 1),
	CAN_MOTOR_PARAMETER_ROUTE(CAN_SET_POS_MAXSPEED, CAN_GET_POS_MAXSPEED, MOTOR_PARAMETER_POSITION_MAX_SPEED_RAD_S, 1),
	CAN_MOTOR_PARAMETER_ROUTE(CAN_SET_POS_KP, CAN_GET_POS_KP, MOTOR_PARAMETER_POSITION_KP_A_PER_RAD, 0),
	CAN_MOTOR_PARAMETER_ROUTE(CAN_SET_POS_KD, CAN_GET_POS_KD, MOTOR_PARAMETER_POSITION_KD_A_PER_RAD_S, 0),
	CAN_MOTOR_PARAMETER_ROUTE(CAN_SET_POS_KI, CAN_GET_POS_KI, MOTOR_PARAMETER_POSITION_KI_A_PER_RAD_S, 0),
	CAN_MOTOR_PARAMETER_ROUTE(CAN_SET_POS_INTEGRAL_LIMIT, CAN_GET_POS_INTEGRAL_LIMIT, MOTOR_PARAMETER_POSITION_INTEGRAL_LIMIT_A, 0),
	CAN_MOTOR_PARAMETER_ROUTE(CAN_SET_CASCADE_POS_KP, CAN_GET_CASCADE_POS_KP, MOTOR_PARAMETER_CASCADE_POSITION_KP_PER_S, 0),
	CAN_MOTOR_PARAMETER_ROUTE(CAN_SET_CASCADE_POS_KD, CAN_GET_CASCADE_POS_KD, MOTOR_PARAMETER_CASCADE_POSITION_KD, 0),
	CAN_MOTOR_PARAMETER_ROUTE(CAN_SET_RS, CAN_GET_RS, MOTOR_PARAMETER_PHASE_RESISTANCE_OHM, 0),
	CAN_MOTOR_PARAMETER_ROUTE(CAN_SET_LD, CAN_GET_LD, MOTOR_PARAMETER_D_AXIS_INDUCTANCE_H, 0),
	CAN_MOTOR_PARAMETER_ROUTE(CAN_SET_LQ, CAN_GET_LQ, MOTOR_PARAMETER_Q_AXIS_INDUCTANCE_H, 0),
	CAN_MOTOR_PARAMETER_ROUTE(CAN_SET_FLUX, CAN_GET_FLUX, MOTOR_PARAMETER_FLUX_WEBER, 0)
};

static const CanTelemetryRoute g_telemetry_routes[] =
{
	CAN_TELEMETRY_ROUTE(CAN_GET_MODE, MOTOR_TELEMETRY_MODE, 0),
	CAN_TELEMETRY_ROUTE(CAN_GET_CURRENT_SET, MOTOR_TELEMETRY_CURRENT_REFERENCE_A, 0),
	CAN_TELEMETRY_ROUTE(CAN_GET_SPEED_SET, MOTOR_TELEMETRY_SPEED_REFERENCE_RAD_S, 1),
	CAN_TELEMETRY_ROUTE(CAN_GET_POS_SET, MOTOR_TELEMETRY_POSITION_REFERENCE_RAD, 1),
	CAN_TELEMETRY_ROUTE(CAN_GET_ENCODER_REVERSE, MOTOR_TELEMETRY_ENCODER_REVERSED, 0),
	CAN_TELEMETRY_ROUTE(CAN_GET_COMMISSIONING_STAGE, MOTOR_TELEMETRY_COMMISSIONING_STAGE, 0),
	CAN_TELEMETRY_ROUTE(CAN_GET_COMMISSIONING_PROGRESS, MOTOR_TELEMETRY_COMMISSIONING_PROGRESS_PERCENT, 0),
	CAN_TELEMETRY_ROUTE(CAN_GET_RESISTANCE_SPREAD, MOTOR_TELEMETRY_PHASE_RESISTANCE_SPREAD_PERCENT, 0),
	CAN_TELEMETRY_ROUTE(CAN_GET_RESISTANCE_DESIGN_ERROR, MOTOR_TELEMETRY_PHASE_RESISTANCE_DESIGN_ERROR_PERCENT, 0),
	CAN_TELEMETRY_ROUTE(CAN_GET_VBUS, MOTOR_TELEMETRY_BUS_VOLTAGE_V, 0),
	CAN_TELEMETRY_ROUTE(CAN_GET_IBUS, MOTOR_TELEMETRY_BUS_CURRENT_A, 0),
	CAN_TELEMETRY_ROUTE(CAN_GET_IA, MOTOR_TELEMETRY_PHASE_A_CURRENT_A, 0),
	CAN_TELEMETRY_ROUTE(CAN_GET_IB, MOTOR_TELEMETRY_PHASE_B_CURRENT_A, 0),
	CAN_TELEMETRY_ROUTE(CAN_GET_IC, MOTOR_TELEMETRY_PHASE_C_CURRENT_A, 0),
	CAN_TELEMETRY_ROUTE(CAN_GET_ID, MOTOR_TELEMETRY_D_AXIS_CURRENT_A, 0),
	CAN_TELEMETRY_ROUTE(CAN_GET_IQ, MOTOR_TELEMETRY_Q_AXIS_CURRENT_A, 0),
	CAN_TELEMETRY_ROUTE(CAN_GET_SPEED2_FILT, MOTOR_TELEMETRY_MECHANICAL_SPEED_RAD_S, 0),
	CAN_TELEMETRY_ROUTE(CAN_GET_POS2_FILT, MOTOR_TELEMETRY_MECHANICAL_POSITION_RAD, 0),
	CAN_TELEMETRY_ROUTE(CAN_GET_TEMP, MOTOR_TELEMETRY_TEMPERATURE_C, 0),
	CAN_TELEMETRY_ROUTE(CAN_GET_ERROR, MOTOR_TELEMETRY_PRIMARY_ERROR, 0)
};

static const CanMotorParameterRoute *CanCommandRouter_FindMotorParameter(CanParameterId param_id)
{
	unsigned int index;
	for (index = 0U; index < sizeof(g_motor_parameter_routes) / sizeof(g_motor_parameter_routes[0]); ++index)
	{
		if ((int)g_motor_parameter_routes[index].set_id == (int)param_id ||
			(int)g_motor_parameter_routes[index].get_id == (int)param_id)
			return &g_motor_parameter_routes[index];
	}
	return 0;
}

static const CanTelemetryRoute *CanCommandRouter_FindTelemetry(CanParameterId param_id)
{
	unsigned int index;
	for (index = 0U; index < sizeof(g_telemetry_routes) / sizeof(g_telemetry_routes[0]); ++index)
	{
		if ((int)g_telemetry_routes[index].response_id == (int)param_id)
			return &g_telemetry_routes[index];
	}
	return 0;
}

bool CanCommandRouter_Initialize(CanCommandRouterContext *context,
	ApplicationEndpoints *application, CanResponseServiceContext *response)
{
	if (context == 0 || application == 0 || response == 0)
		return false;
	context->application = application;
	context->response = response;
	return true;
}

static void CanCommandRouter_SendMotorParameter(
	CanCommandRouterContext *context, CanParameterId response_id,
	MotorParameterId parameter, float scale)
{
    float value;
    if (ParameterService_ReadMotorParameter(context->application->parameters, parameter, &value) ==
        PARAMETER_SERVICE_ACCEPTED)
        CanResponseService_Queue(context->response, response_id, value * scale);
}

static void CanCommandRouter_SendTelemetry(CanCommandRouterContext *context,
	CanParameterId response_id,
    MotorTelemetryId telemetry, float scale)
{
    float value;
    if (TelemetryService_ReadValue(context->application->telemetry, telemetry, &value))
        CanResponseService_Queue(context->response, response_id, value * scale);
}

static void CanCommandRouter_SetEncoderState(int value)
{
    (void)value;
}

static int CanCommandRouter_GetEncoderState(CanCommandRouterContext *context)
{
    float online = 0.0f;
	(void)TelemetryService_ReadValue(context->application->telemetry,
		MOTOR_TELEMETRY_ENCODER_ONLINE, &online);
    return online != 0.0f ? 1 : 0;
}
void CanCommandRouter_Handle(CanCommandRouterContext *context,
	CanParameterId param_id, float data)
{
	const CanMotorParameterRoute *parameter_route;
	const CanTelemetryRoute *telemetry_route;
	float scale;

	if (context == 0 || !isfinite(data) || data >= (float)INT_MAX ||
		data <= (float)INT_MIN)
		return;

	int data_int = (int)data;
	parameter_route = CanCommandRouter_FindMotorParameter(param_id);
	if (parameter_route != 0)
	{
		scale = parameter_route->revolutions_unit != 0U ?
			CAN_COMMAND_ROUTER_TWO_PI : 1.0f;
		if ((int)parameter_route->set_id == (int)param_id)
		{
			(void)ParameterService_WriteMotorParameter(context->application->parameters,
				(MotorParameterId)parameter_route->parameter, data * scale);
		}
		else
		{
			scale = parameter_route->revolutions_unit != 0U ?
				CAN_COMMAND_ROUTER_ONE_BY_2PI : 1.0f;
			CanCommandRouter_SendMotorParameter(context,
				(CanParameterId)parameter_route->get_id,
				(MotorParameterId)parameter_route->parameter, scale);
		}
		return;
	}

	telemetry_route = CanCommandRouter_FindTelemetry(param_id);
	if (telemetry_route != 0)
	{
		scale = telemetry_route->revolutions_unit != 0U ?
			CAN_COMMAND_ROUTER_ONE_BY_2PI : 1.0f;
		CanCommandRouter_SendTelemetry(context,
			(CanParameterId)telemetry_route->response_id,
			(MotorTelemetryId)telemetry_route->telemetry, scale);
		return;
	}

	switch(param_id)
	{
		/*setting parameters*/
		case CAN_SET_MODE:
			if(data_int >= 0)
			{
				MotorCommandResult result = MotorCommandService_RequestActionCode(
					context->application->motor_command, (uint8_t)data_int);
				if (result == MOTOR_COMMAND_ACCEPTED)
				{
					if (data_int == PROTOCOL_MODE_DISABLED)
						ControlAuthorityService_Release(
							context->application->control_authority,
							CONTROL_AUTHORITY_CAN);
					else
						ControlAuthorityService_Claim(
							context->application->control_authority,
							CONTROL_AUTHORITY_CAN);
				}
			}
		break;
		case CAN_SET_CURRENT:
			if (MotorCommandService_SetCurrentReferenceA(
				context->application->motor_command, data) == MOTOR_COMMAND_ACCEPTED)
				ControlAuthorityService_Claim(
					context->application->control_authority, CONTROL_AUTHORITY_CAN);
		break;
		case CAN_SET_SPEED:
			if (MotorCommandService_SetSpeedReferenceRps(
				context->application->motor_command, data) == MOTOR_COMMAND_ACCEPTED)
				ControlAuthorityService_Claim(
					context->application->control_authority, CONTROL_AUTHORITY_CAN);
		break;
		case CAN_SET_POS:
			if (MotorCommandService_SetPositionReferenceRevolutions(
				context->application->motor_command, data) == MOTOR_COMMAND_ACCEPTED)
				ControlAuthorityService_Claim(
					context->application->control_authority, CONTROL_AUTHORITY_CAN);
		break;
		/*user parameters*/
		case CAN_SET_NODE_ID:
			if(data_int >= 0 && data_int <= 7)
				(void)CanConfigurationService_SetNodeId(
					context->application->can_configuration, (uint8_t)data_int);
		break;
		case CAN_GET_NODE_ID:
			CanResponseService_Queue(context->response, CAN_GET_NODE_ID,
				(float)CanConfigurationService_GetNodeId(
					context->application->can_configuration));
		break;

		case CAN_SET_ENCODER_STATE:
		{
			float mode_value = 0.0f;
			int mode;

			(void)TelemetryService_ReadValue(context->application->telemetry,
				MOTOR_TELEMETRY_MODE, &mode_value);
			mode = (int)mode_value;
			if(mode != PROTOCOL_MODE_CURRENT && mode != PROTOCOL_MODE_SPEED)
			{
				CanCommandRouter_SetEncoderState(data_int);
			}
		}
		break;
		case CAN_GET_ENCODER_STATE:
			CanResponseService_Queue(context->response, CAN_GET_ENCODER_STATE,
				(float)CanCommandRouter_GetEncoderState(context));
		break;

		case CAN_SET_ENCODER_REVERSE:
		{
			float mode_value = 0.0f;
			(void)TelemetryService_ReadValue(context->application->telemetry,
				MOTOR_TELEMETRY_MODE, &mode_value);
			if((int)mode_value == PROTOCOL_MODE_DISABLED &&
				(data_int == 0 || data_int == 1))
				(void)RotorCalibrationService_SetReverse(
					context->application->rotor_calibration, data_int != 0);
		}
		break;
		case CAN_APPLY_FRICTION_MODEL:
			if (data_int == 1)
				(void)FrictionIdentificationService_ApplyCandidate(
					context->application->friction_identification);
		break;
		case CAN_GET_FRICTION_STATE:
		case CAN_GET_FRICTION_REASON:
		case CAN_GET_FRICTION_COULOMB_POS:
		case CAN_GET_FRICTION_COULOMB_NEG:
		case CAN_GET_FRICTION_VISCOUS_POS:
		case CAN_GET_FRICTION_VISCOUS_NEG:
		case CAN_GET_FRICTION_RMSE_POS:
		case CAN_GET_FRICTION_RMSE_NEG:
		case CAN_GET_FRICTION_CANDIDATE_VALID:
		case CAN_GET_FRICTION_MODEL_VALID:
		{
			FrictionIdentificationPortStatus status;
			float response = 0.0f;
			if (!FrictionIdentificationService_ReadStatus(
				context->application->friction_identification, &status)) break;
			switch (param_id)
			{
				case CAN_GET_FRICTION_STATE: response = (float)status.state; break;
				case CAN_GET_FRICTION_REASON: response = (float)status.reason; break;
				case CAN_GET_FRICTION_COULOMB_POS: response = status.candidate_coulomb_pos_a; break;
				case CAN_GET_FRICTION_COULOMB_NEG: response = status.candidate_coulomb_neg_a; break;
				case CAN_GET_FRICTION_VISCOUS_POS: response = status.candidate_viscous_pos_a_per_rad_s; break;
				case CAN_GET_FRICTION_VISCOUS_NEG: response = status.candidate_viscous_neg_a_per_rad_s; break;
				case CAN_GET_FRICTION_RMSE_POS: response = status.candidate_rmse_pos_a; break;
				case CAN_GET_FRICTION_RMSE_NEG: response = status.candidate_rmse_neg_a; break;
				case CAN_GET_FRICTION_CANDIDATE_VALID: response = status.candidate_valid ? 1.0f : 0.0f; break;
				case CAN_GET_FRICTION_MODEL_VALID: response = status.active_model_valid ? 1.0f : 0.0f; break;
				default: break;
			}
			CanResponseService_Queue(context->response, param_id, response);
			break;
		}

		case CAN_SET_COGGING:
		break;
		case CAN_GET_COGGING:
		break;
		case CAN_SET_CAN_BR:
			if (data_int >= 0)
				(void)CanConfigurationService_SetBitrateKbps(
					context->application->can_configuration, (uint32_t)data_int);
		break;
		case CAN_GET_CAN_BR:
			CanResponseService_Queue(context->response, CAN_GET_CAN_BR,
				(float)CanConfigurationService_GetBitrateKbps(
					context->application->can_configuration));
		break;

		case CAN_SET_CAN_HB:
			if (data_int >= 0)
				(void)CanConfigurationService_SetHeartbeatMs(
					context->application->can_configuration, (uint32_t)data_int);
		break;
		case CAN_GET_CAN_HB:
			CanResponseService_Queue(context->response, CAN_GET_CAN_HB,
				(float)CanConfigurationService_GetHeartbeatMs(
					context->application->can_configuration));
		break;

		default:break;
	}
}
