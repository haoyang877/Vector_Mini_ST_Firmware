#include "can_command_router.h"

#include "can_configuration_service.h"
#include "can_response_service.h"
#include "motor_command_service.h"
#include "parameter_service.h"
#include "rotor_calibration_service.h"
#include "telemetry_service.h"

#include <limits.h>
#include <math.h>

#define CAN_COMMAND_ROUTER_TWO_PI       6.2831853072f
#define CAN_COMMAND_ROUTER_ONE_BY_2PI   0.15915494309f
#define PROTOCOL_MODE_DISABLED    0
#define PROTOCOL_MODE_CURRENT     1
#define PROTOCOL_MODE_SPEED       2

static void CanCommandRouter_SendMotorParameter(CanParameterId response_id,
    MotorParameterId parameter, float scale)
{
    float value;
    if (ParameterService_ReadMotorParameter(parameter, &value) ==
        PARAMETER_SERVICE_ACCEPTED)
        CanResponseService_Queue(response_id, value * scale);
}

static void CanCommandRouter_SendTelemetry(CanParameterId response_id,
    MotorTelemetryId telemetry, float scale)
{
    float value;
    if (TelemetryService_ReadValue(telemetry, &value))
        CanResponseService_Queue(response_id, value * scale);
}

static void CanCommandRouter_SetEncoderState(int value)
{
    (void)value;
}

static int CanCommandRouter_GetEncoderState(void)
{
    float online = 0.0f;
    (void)TelemetryService_ReadValue(MOTOR_TELEMETRY_ENCODER_ONLINE, &online);
    return online != 0.0f ? 1 : 0;
}
void CanCommandRouter_Handle(CanParameterId param_id, float data)
{
	if (!isfinite(data) || data >= (float)INT_MAX || data <= (float)INT_MIN)
		return;

	int data_int = (int)data;
	
	switch(param_id)
	{
		/*setting parameters*/
		case CAN_SET_MODE:
			if(data_int >= 0)
				(void)MotorCommandService_RequestActionCode((uint8_t)data_int);
		break;
		case CAN_GET_MODE:
			CanCommandRouter_SendTelemetry(CAN_GET_MODE, MOTOR_TELEMETRY_MODE, 1.0f);
		break;
		
		case CAN_SET_CURRENT:
			(void)MotorCommandService_SetCurrentReferenceA(data);
		break;
		case CAN_GET_CURRENT_SET:
			CanCommandRouter_SendTelemetry(CAN_GET_CURRENT_SET,
				MOTOR_TELEMETRY_CURRENT_REFERENCE_A, 1.0f);
		break;
				
		case CAN_SET_SPEED:
			(void)MotorCommandService_SetSpeedReferenceRps(data);
		break;
		case CAN_GET_SPEED_SET:
			CanCommandRouter_SendTelemetry(CAN_GET_SPEED_SET,
				MOTOR_TELEMETRY_SPEED_REFERENCE_RAD_S, CAN_COMMAND_ROUTER_ONE_BY_2PI);
		break;
		
		case CAN_SET_POS:
			(void)MotorCommandService_SetPositionReferenceRevolutions(data);
		break;
		case CAN_GET_POS_SET:
			CanCommandRouter_SendTelemetry(CAN_GET_POS_SET,
				MOTOR_TELEMETRY_POSITION_REFERENCE_RAD, CAN_COMMAND_ROUTER_ONE_BY_2PI);
		break;
		
		
		/*user parameters*/
		case CAN_SET_NODE_ID:
			if(data_int >= 0 && data_int <= 7)
				(void)CanConfigurationService_SetNodeId((uint8_t)data_int);
		break;
		case CAN_GET_NODE_ID:
			CanResponseService_Queue(CAN_GET_NODE_ID,
				(float)CanConfigurationService_GetNodeId());
		break;
			
		case CAN_SET_POLEPARIS:
			(void)ParameterService_WriteMotorParameter(
				MOTOR_PARAMETER_POLE_PAIRS, data);
		break;
		case CAN_GET_POLEPARIS:
			CanCommandRouter_SendMotorParameter(CAN_GET_POLEPARIS,
				MOTOR_PARAMETER_POLE_PAIRS, 1.0f);
		break;
			
		case CAN_SET_ENCODER_STATE:
		{
			float mode_value = 0.0f;
			int mode;

			(void)TelemetryService_ReadValue(MOTOR_TELEMETRY_MODE, &mode_value);
			mode = (int)mode_value;
			if(mode != PROTOCOL_MODE_CURRENT && mode != PROTOCOL_MODE_SPEED)
			{
				CanCommandRouter_SetEncoderState(data_int);
			}
		}
		break;
		case CAN_GET_ENCODER_STATE: 
			CanResponseService_Queue(CAN_GET_ENCODER_STATE, (float)CanCommandRouter_GetEncoderState());
		break;

		case CAN_SET_ENCODER_REVERSE:
		{
			float mode_value = 0.0f;
			(void)TelemetryService_ReadValue(MOTOR_TELEMETRY_MODE, &mode_value);
			if((int)mode_value == PROTOCOL_MODE_DISABLED &&
				(data_int == 0 || data_int == 1))
				(void)RotorCalibrationService_SetReverse(data_int != 0);
		}
		break;
		case CAN_GET_ENCODER_REVERSE:
			CanCommandRouter_SendTelemetry(CAN_GET_ENCODER_REVERSE,
				MOTOR_TELEMETRY_ENCODER_REVERSED, 1.0f);
		break;

		case CAN_SET_CURRENT_CAL:
			(void)ParameterService_WriteMotorParameter(
				MOTOR_PARAMETER_CALIBRATION_CURRENT_A, data);
		break;
		case CAN_GET_CURRENT_CAL:
			CanCommandRouter_SendMotorParameter(CAN_GET_CURRENT_CAL,
				MOTOR_PARAMETER_CALIBRATION_CURRENT_A, 1.0f);
		break;
		
		case CAN_SET_CURRENT_LIMIT:
			(void)ParameterService_WriteMotorParameter(
				MOTOR_PARAMETER_CURRENT_LIMIT_A, data);
		break;
		case CAN_GET_CURRENT_LIMIT:
			CanCommandRouter_SendMotorParameter(CAN_GET_CURRENT_LIMIT,
				MOTOR_PARAMETER_CURRENT_LIMIT_A, 1.0f);
		break;
		
		case CAN_SET_SPEED_LIMIT:
			(void)ParameterService_WriteMotorParameter(
				MOTOR_PARAMETER_SPEED_LIMIT_RAD_S, data * CAN_COMMAND_ROUTER_TWO_PI);
		break;
		case CAN_GET_SPEED_LIMIT:
			CanCommandRouter_SendMotorParameter(CAN_GET_SPEED_LIMIT,
				MOTOR_PARAMETER_SPEED_LIMIT_RAD_S, CAN_COMMAND_ROUTER_ONE_BY_2PI);
		break;
			
		case CAN_SET_SPEED_ACC:
			(void)ParameterService_WriteMotorParameter(
				MOTOR_PARAMETER_SPEED_ACCELERATION_RAD_S2, data * CAN_COMMAND_ROUTER_TWO_PI);
		break;
		case CAN_GET_SPEED_ACC:
			CanCommandRouter_SendMotorParameter(CAN_GET_SPEED_ACC,
				MOTOR_PARAMETER_SPEED_ACCELERATION_RAD_S2, CAN_COMMAND_ROUTER_ONE_BY_2PI);
		break;
		
		case CAN_SET_SPEED_DEC:
			(void)ParameterService_WriteMotorParameter(
				MOTOR_PARAMETER_SPEED_DECELERATION_RAD_S2, data * CAN_COMMAND_ROUTER_TWO_PI);
		break;
		case CAN_GET_SPEED_DEC:
			CanCommandRouter_SendMotorParameter(CAN_GET_SPEED_DEC,
				MOTOR_PARAMETER_SPEED_DECELERATION_RAD_S2, CAN_COMMAND_ROUTER_ONE_BY_2PI);
		break;
		
		case CAN_SET_SPEED_KP:
			(void)ParameterService_WriteMotorParameter(
				MOTOR_PARAMETER_SPEED_KP, data);
		break;
		case CAN_GET_SPEED_KP:
			CanCommandRouter_SendMotorParameter(CAN_GET_SPEED_KP,
				MOTOR_PARAMETER_SPEED_KP, 1.0f);
		break;
		
		case CAN_SET_SPEED_KI:
			(void)ParameterService_WriteMotorParameter(
				MOTOR_PARAMETER_SPEED_KI, data);
		break;
		case CAN_GET_SPEED_KI:
			CanCommandRouter_SendMotorParameter(CAN_GET_SPEED_KI,
				MOTOR_PARAMETER_SPEED_KI, 1.0f);
		break;
		
		case CAN_SET_POS_ACC:
			(void)ParameterService_WriteMotorParameter(
				MOTOR_PARAMETER_POSITION_ACCELERATION_RAD_S2, data * CAN_COMMAND_ROUTER_TWO_PI);
		break;
		case CAN_GET_POS_ACC:
			CanCommandRouter_SendMotorParameter(CAN_GET_POS_ACC,
				MOTOR_PARAMETER_POSITION_ACCELERATION_RAD_S2, CAN_COMMAND_ROUTER_ONE_BY_2PI);
		break;
		
		case CAN_SET_POS_DEC:
			(void)ParameterService_WriteMotorParameter(
				MOTOR_PARAMETER_POSITION_DECELERATION_RAD_S2, data * CAN_COMMAND_ROUTER_TWO_PI);
		break;
		case CAN_GET_POS_DEC:
			CanCommandRouter_SendMotorParameter(CAN_GET_POS_DEC,
				MOTOR_PARAMETER_POSITION_DECELERATION_RAD_S2, CAN_COMMAND_ROUTER_ONE_BY_2PI);
		break;
		
		case CAN_SET_POS_MAXSPEED:
			(void)ParameterService_WriteMotorParameter(
				MOTOR_PARAMETER_POSITION_MAX_SPEED_RAD_S, data * CAN_COMMAND_ROUTER_TWO_PI);
		break;
		case CAN_GET_POS_MAXSPEED:
			CanCommandRouter_SendMotorParameter(CAN_GET_POS_MAXSPEED,
				MOTOR_PARAMETER_POSITION_MAX_SPEED_RAD_S, CAN_COMMAND_ROUTER_ONE_BY_2PI);
		break;
		
		case CAN_SET_POS_KP:
			(void)ParameterService_WriteMotorParameter(
				MOTOR_PARAMETER_POSITION_KP_A_PER_RAD, data);
		break;
		case CAN_GET_POS_KP:
			CanCommandRouter_SendMotorParameter(CAN_GET_POS_KP,
				MOTOR_PARAMETER_POSITION_KP_A_PER_RAD, 1.0f);
		break;

		case CAN_SET_POS_KD:
			(void)ParameterService_WriteMotorParameter(
				MOTOR_PARAMETER_POSITION_KD_A_PER_RAD_S, data);
		break;
		case CAN_GET_POS_KD:
			CanCommandRouter_SendMotorParameter(CAN_GET_POS_KD,
				MOTOR_PARAMETER_POSITION_KD_A_PER_RAD_S, 1.0f);
		break;

		case CAN_SET_POS_KI:
			(void)ParameterService_WriteMotorParameter(
				MOTOR_PARAMETER_POSITION_KI_A_PER_RAD_S, data);
		break;
		case CAN_GET_POS_KI:
			CanCommandRouter_SendMotorParameter(CAN_GET_POS_KI,
				MOTOR_PARAMETER_POSITION_KI_A_PER_RAD_S, 1.0f);
		break;

		case CAN_SET_POS_INTEGRAL_LIMIT:
			(void)ParameterService_WriteMotorParameter(
				MOTOR_PARAMETER_POSITION_INTEGRAL_LIMIT_A, data);
		break;
		case CAN_GET_POS_INTEGRAL_LIMIT:
			CanCommandRouter_SendMotorParameter(CAN_GET_POS_INTEGRAL_LIMIT,
				MOTOR_PARAMETER_POSITION_INTEGRAL_LIMIT_A, 1.0f);
		break;

		case CAN_SET_CASCADE_POS_KP:
			(void)ParameterService_WriteMotorParameter(
				MOTOR_PARAMETER_CASCADE_POSITION_KP_PER_S, data);
		break;
		case CAN_GET_CASCADE_POS_KP:
			CanCommandRouter_SendMotorParameter(CAN_GET_CASCADE_POS_KP,
				MOTOR_PARAMETER_CASCADE_POSITION_KP_PER_S, 1.0f);
		break;

		case CAN_SET_CASCADE_POS_KD:
			(void)ParameterService_WriteMotorParameter(
				MOTOR_PARAMETER_CASCADE_POSITION_KD, data);
		break;
		case CAN_GET_CASCADE_POS_KD:
			CanCommandRouter_SendMotorParameter(CAN_GET_CASCADE_POS_KD,
				MOTOR_PARAMETER_CASCADE_POSITION_KD, 1.0f);
		break;
		
		case CAN_SET_COGGING:
		break;
		case CAN_GET_COGGING:
		break;
	
		case CAN_SET_CAN_BR:
			if (data_int >= 0)
				(void)CanConfigurationService_SetBitrateKbps((uint32_t)data_int);
		break;
		case CAN_GET_CAN_BR:
			CanResponseService_Queue(CAN_GET_CAN_BR,
				(float)CanConfigurationService_GetBitrateKbps());
		break;
		
		case CAN_SET_CAN_HB:
			if (data_int >= 0)
				(void)CanConfigurationService_SetHeartbeatMs((uint32_t)data_int);
		break;
		case CAN_GET_CAN_HB:
			CanResponseService_Queue(CAN_GET_CAN_HB,
				(float)CanConfigurationService_GetHeartbeatMs());
		break;
		
		/*state parameters*/
		case CAN_GET_VBUS:
			CanCommandRouter_SendTelemetry(CAN_GET_VBUS,
				MOTOR_TELEMETRY_BUS_VOLTAGE_V, 1.0f);
		break;
		
		case CAN_GET_IBUS:
			CanCommandRouter_SendTelemetry(CAN_GET_IBUS,
				MOTOR_TELEMETRY_BUS_CURRENT_A, 1.0f);
		break;
		
		case CAN_GET_IA:
			CanCommandRouter_SendTelemetry(CAN_GET_IA,
				MOTOR_TELEMETRY_PHASE_A_CURRENT_A, 1.0f);
		break;
		
		case CAN_GET_IB:
			CanCommandRouter_SendTelemetry(CAN_GET_IB,
				MOTOR_TELEMETRY_PHASE_B_CURRENT_A, 1.0f);
		break;
		
		case CAN_GET_IC:
			CanCommandRouter_SendTelemetry(CAN_GET_IC,
				MOTOR_TELEMETRY_PHASE_C_CURRENT_A, 1.0f);
		break;
		
		case CAN_GET_ID:
			CanCommandRouter_SendTelemetry(CAN_GET_ID,
				MOTOR_TELEMETRY_D_AXIS_CURRENT_A, 1.0f);
		break;
		
		case CAN_GET_IQ:
			CanCommandRouter_SendTelemetry(CAN_GET_IQ,
				MOTOR_TELEMETRY_Q_AXIS_CURRENT_A, 1.0f);
		break;
		
		

		case CAN_GET_SPEED2_FILT:
			CanCommandRouter_SendTelemetry(CAN_GET_SPEED2_FILT,
				MOTOR_TELEMETRY_MECHANICAL_SPEED_RAD_S, 1.0f);
		break;
		
		case CAN_GET_POS2_FILT:
			CanCommandRouter_SendTelemetry(CAN_GET_POS2_FILT,
				MOTOR_TELEMETRY_MECHANICAL_POSITION_RAD, 1.0f);
		break;
		
		case CAN_GET_TEMP:
			CanCommandRouter_SendTelemetry(CAN_GET_TEMP,
				MOTOR_TELEMETRY_TEMPERATURE_C, 1.0f);
		break;

		case CAN_GET_RS:
			CanCommandRouter_SendMotorParameter(CAN_GET_RS,
				MOTOR_PARAMETER_PHASE_RESISTANCE_OHM, 1.0f);
		break;

		case CAN_GET_LD:
			CanCommandRouter_SendMotorParameter(CAN_GET_LD,
				MOTOR_PARAMETER_D_AXIS_INDUCTANCE_H, 1.0f);
		break;

		case CAN_GET_LQ:
			CanCommandRouter_SendMotorParameter(CAN_GET_LQ,
				MOTOR_PARAMETER_Q_AXIS_INDUCTANCE_H, 1.0f);
		break;

		case CAN_GET_FLUX:
			CanCommandRouter_SendMotorParameter(CAN_GET_FLUX,
				MOTOR_PARAMETER_FLUX_WEBER, 1.0f);
		break;		
		
		case CAN_GET_ERROR:
			CanCommandRouter_SendTelemetry(CAN_GET_ERROR,
				MOTOR_TELEMETRY_PRIMARY_ERROR, 1.0f);
		break;
		
		default:break;
	}
}
