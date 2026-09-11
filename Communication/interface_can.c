#include "interface_can.h"

#include <limits.h>
#include <math.h>
#include "fdcan.h"
#include "delay.h"
#include "utils.h"
#include "foc_algorithm.h"
#include "foc_param.h"
#include "foc_param_profile.h"
#include "foc_errhandle.h"
#include "encoder.h"
#include "hw_conf.h"
#include "../hal/api/comm_hw.h"
#include "../hal/api/time_hw.h"
#include "../software/communication/protocol/can_motor_status.h"
#include "foc_friction_identification.h"

CANMsg_TypeDef CANMsg;

extern MotorControl_TypeDef MotorControl;
extern ModeNow_TypeDef ModeLast;
extern FOC_TypeDef FOC;
extern Encoder_TypeDef OnBoard_Encoder;

/** @brief CAN 参数在线路上的数值编码。 */
typedef enum {
	CAN_VALUE_FLOAT32,
	CAN_VALUE_MILLI_I32,
	CAN_VALUE_CENTI_I32,
	CAN_VALUE_MILLI_I16,
} CanValueEncoding;

/**
 * @brief 返回写命令参数的线路编码。
 * @param param_id 参数 ID。
 * @return 位置/速度/电流参数的定点编码，其他参数返回遗留 float32。
 */
static CanValueEncoding CAN_CommandEncoding(CAN_PARAM_ID param_id)
{
	switch (param_id) {
	case CAN_SET_CURRENT:
	case CAN_SET_CURRENT_CAL:
	case CAN_SET_CURRENT_LIMIT:
		return CAN_VALUE_MILLI_I16;
	case CAN_SET_POS:
		return CAN_VALUE_MILLI_I32;
	case CAN_SET_SPEED:
	case CAN_SET_SPEED_LIMIT:
	case CAN_SET_SPEED_ACC:
	case CAN_SET_SPEED_DEC:
	case CAN_SET_POS_ACC:
	case CAN_SET_POS_DEC:
	case CAN_SET_POS_MAXSPEED:
		return CAN_VALUE_CENTI_I32;
	default:
		return CAN_VALUE_FLOAT32;
	}
}

/**
 * @brief 返回回复参数的线路编码。
 * @param param_id 回复参数 ID。
 * @return 位置/速度/电流参数的定点编码，其他参数返回遗留 float32。
 */
static CanValueEncoding CAN_ReplyEncoding(CAN_PARAM_ID param_id)
{
	switch (param_id) {
	case CAN_GET_CURRENT_SET:
	case CAN_GET_CURRENT_CAL:
	case CAN_GET_CURRENT_LIMIT:
	case CAN_GET_IBUS:
	case CAN_GET_IA:
	case CAN_GET_IB:
	case CAN_GET_IC:
	case CAN_GET_ID:
	case CAN_GET_IQ:
	case CAN_GET_FRICTION_COULOMB_POS:
	case CAN_GET_FRICTION_COULOMB_NEG:
	case CAN_GET_FRICTION_RMSE_POS:
	case CAN_GET_FRICTION_RMSE_NEG:
		return CAN_VALUE_MILLI_I16;
	case CAN_GET_POS_SET:
	case CAN_GET_POS2_FILT:
		return CAN_VALUE_MILLI_I32;
	case CAN_GET_SPEED_SET:
	case CAN_GET_SPEED_LIMIT:
	case CAN_GET_SPEED_ACC:
	case CAN_GET_SPEED_DEC:
	case CAN_GET_POS_ACC:
	case CAN_GET_POS_DEC:
	case CAN_GET_POS_MAXSPEED:
	case CAN_GET_SPEED2_FILT:
		return CAN_VALUE_CENTI_I32;
	default:
		return CAN_VALUE_FLOAT32;
	}
}

/**
 * @brief 将浮点 SI 值转换为保留最小值作为无效哨兵的 int32 毫单位。
 * @param value 有限 SI 值。
 * @return 截断并饱和后的线路值；非有限值返回 INT32_MIN。
 */
static int32_t CAN_Milli32(float value)
{
	float scaled;
	if (!isfinite(value)) return INT32_MIN;
	scaled = value * 1000.0f;
	if (scaled >= 2147483648.0f) return INT32_MAX;
	if (scaled <= -2147483648.0f) return -INT32_MAX;
	return (int32_t)scaled;
}

/**
 * @brief 将浮点速度或加速度转换为保留最小值哨兵的 int32 百分一单位。
 * @param value 有限 SI 值，单位 rad/s 或 rad/s²。
 * @return 截断并饱和后的线路值；非有限值返回 INT32_MIN。
 */
static int32_t CAN_Centi32(float value)
{
	float scaled;
	if (!isfinite(value)) return INT32_MIN;
	scaled = value * 100.0f;
	if (scaled >= 2147483648.0f) return INT32_MAX;
	if (scaled <= -2147483648.0f) return -INT32_MAX;
	return (int32_t)scaled;
}

/**
 * @brief 将浮点安培值转换为保留最小值作为无效哨兵的 int16 毫安。
 * @param value 有限安培值。
 * @return 截断并饱和后的线路值；非有限值返回 INT16_MIN。
 */
static int16_t CAN_Milli16(float value)
{
	float scaled;
	if (!isfinite(value)) return INT16_MIN;
	scaled = value * 1000.0f;
	if (scaled >= 32767.0f) return INT16_MAX;
	if (scaled <= -32767.0f) return -INT16_MAX;
	return (int16_t)scaled;
}

/**
	* @brief  FDCAN1 Filter Init  
			  Stdandard ID, Range Mode 
 **/
void FDCAN1_Param_Init(void)
{
	FDCAN_FilterTypeDef FDCAN_Filter;
	CanMotorStatus_Init();
	
	FDCAN_Filter.IdType 		= FDCAN_STANDARD_ID;
	FDCAN_Filter.FilterIndex 	= 0;
	FDCAN_Filter.FilterType		= FDCAN_FILTER_RANGE;
	
	FDCAN_Filter.FilterConfig	= FDCAN_FILTER_TO_RXFIFO0;

	FDCAN_Filter.FilterID1 = (((uint32_t)CANMsg.node_id) << 8);
	FDCAN_Filter.FilterID2 = (((uint32_t)CANMsg.node_id) << 8) + 0xFF;

	if(HAL_FDCAN_ConfigFilter(&hfdcan1, &FDCAN_Filter) != HAL_OK)
	{
		Error_Handler();
	}
	
	if(HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
	{
		Error_Handler();
	}
	
	if(HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
	{
		Error_Handler();
	}
	
	if(HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
	{
		Error_Handler();
	}
	
	CANMsg.baudrate = 1000;
}

/**
	* @brief  Handle CAN heartbeat disconnect protection
 **/
void CAN_DisConnect_Handle(void)
{
	/* Recompute on every supervisor tick: a previous RUN must not keep the
	 * watchdog armed after STOP. Keep any existing fault latched here. */
	CANMsg.can_hb_en = CANMsg.can_hb_set != 0U &&
		(MotorControl.ModeNow == Current_Mode ||
		 MotorControl.ModeNow == Speed_Mode ||
		 MotorControl.ModeNow == Position_Mode ||
		 MotorControl.ModeNow == Position_Impedance_Mode);
	if (!CANMsg.can_hb_en) {
		CANMsg.can_hb_count = 0U;
		return;
	}
	if (CANMsg.can_rx_en) {
		/* Saturate so a sustained disconnect cannot wrap the counter. */
		if (CANMsg.can_hb_count < CANMsg.can_hb_set) ++CANMsg.can_hb_count;
		if (CANMsg.can_hb_count >= CANMsg.can_hb_set &&
			MotorControl.ErrorNow == No_Error)
			Set_ErrorNow(CAN_DisConnect);
	}
}

/**
	* @brief  Set encoder state from CAN parameter value
	* @param  data: encoded encoder state value
 **/
void CAN_SetEncoderState(int data)
{
	(void)data;
}

/**
	* @brief  Switch CAN baudrate when baudrate setting changes
 **/
void CAN_BaudRateSwitching(void)
{
	static uint32_t baudrate_last = 1000;
	
	if(baudrate_last != CANMsg.baudrate)
    {
		if (HAL_FDCAN_Stop(&hfdcan1) != HAL_OK)
		{
			Error_Handler();
		}
		
		if(CANMsg.baudrate <= 1000)
		{
			hfdcan1.Init.DataPrescaler = 10000 / CANMsg.baudrate;
			hfdcan1.Init.NominalPrescaler = 10000 / CANMsg.baudrate;			
		}
		else
		{
			hfdcan1.Init.DataPrescaler = 10000 / CANMsg.baudrate;
			hfdcan1.Init.NominalPrescaler = 10;
		}

		if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
		{
			Error_Handler();
		}

		if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
		{
			Error_Handler();
		}
	}
	
	baudrate_last = CANMsg.baudrate;
}

/**
	* @brief  Get encoded encoder state
	* @retval encoded encoder state value
 **/
int CAN_GetEncoderState(void)
{
	return Encoder_IsOnline(&OnBoard_Encoder) ? 1 : 0;
}

/**
	* @brief  Handle received CAN message
			  update motor control paramters
    * @param  param_id: CAN parameter id 
    * @param  data: CAN parameter data
 **/
void CAN_ReceiveMessage_Update(CAN_PARAM_ID param_id, float data)
{
	/* Handle before float-to-int conversion; NaN/fraction/out-of-range commands
	 * are rejected atomically without changing the previous stream setting. */
	if (param_id == CAN_SET_STATUS_STREAM) {
		bool accepted = CanMotorStatus_Configure(data);
		CAN_SendMessage_Update(CAN_GET_STATUS_STREAM,
			accepted ? (float)CanMotorStatus_Rate() : -1.0f);
		return;
	}
	if (param_id == CAN_GET_STATUS_STREAM) {
		CAN_SendMessage_Update(CAN_GET_STATUS_STREAM, (float)CanMotorStatus_Rate());
		return;
	}
	if (!isfinite(data))
		return;

	int data_int = (int)data;
	
	switch(param_id)
	{
		/*setting parameters*/
		case CAN_SET_MODE:
			if(data_int >= 0 && data_int < (int)MODE_NUM)
				ModeSwitch_Handle((ModeNow_TypeDef)data_int);
		break;
		case CAN_GET_MODE:
			CAN_SendMessage_Update(CAN_GET_MODE, (float)MotorControl.ModeNow);
		break;
		
		case CAN_SET_CURRENT:
			ModeSwitch_Handle(Current_Mode);
			if(fast_abs(data) <= MotorControl.current_limit)
				MotorControl.iqRef = data;
		break;
		case CAN_GET_CURRENT_SET:
			CAN_SendMessage_Update(CAN_GET_CURRENT_SET, MotorControl.iqRef);
		break;
				
		case CAN_SET_SPEED:
			if(MotorControl.ModeNow != Sensorless_Speed_Mode)
				ModeSwitch_Handle(Speed_Mode);
			if(fast_abs(data) <= MotorControl.speed_limit)
				MotorControl.speedRef = data;
		break;
		case CAN_GET_SPEED_SET:
			CAN_SendMessage_Update(CAN_GET_SPEED_SET, MotorControl.speedRef);
		break;
		
		case CAN_SET_POS:
		{
			float position_ref = data;
			if (isfinite(position_ref) &&
				(MotorControl.ModeNow == Position_Mode ||
				 MotorControl.ModeNow == Position_Impedance_Mode ||
				 ModeSwitch_Handle(Position_Mode)))
			{
				MotorControl.posRef = position_ref;
			}
		}
		break;
		case CAN_GET_POS_SET:
			CAN_SendMessage_Update(CAN_GET_POS_SET, MotorControl.posRef);
		break;
		
		
		/*user parameters*/
		case CAN_SET_NODE_ID:
			if(data_int >= 0 && data_int <= 7)
				CANMsg.node_id = data;
		break;
		case CAN_GET_NODE_ID:
			CAN_SendMessage_Update(CAN_GET_NODE_ID, (float)CANMsg.node_id);
		break;
			
		case CAN_SET_POLEPARIS:
			if(data_int >= 2 && data_int <= 30)
				MotorControl.motor_pole_pairs = data;
		break;
		case CAN_GET_POLEPARIS:
			CAN_SendMessage_Update(CAN_GET_POLEPARIS, (float)MotorControl.motor_pole_pairs);
		break;
			
		case CAN_SET_ENCODER_STATE:
			if(MotorControl.ModeNow != Current_Mode && 
			   MotorControl.ModeNow != Speed_Mode   && 
			   MotorControl.ModeNow != Speed_Mode     )
			{
				CAN_SetEncoderState(data_int);
			}
		break;
		case CAN_GET_ENCODER_STATE: 
			CAN_SendMessage_Update(CAN_GET_ENCODER_STATE, (float)CAN_GetEncoderState());
		break;

		case CAN_SET_ENCODER_REVERSE:
			if(MotorControl.ModeNow == Motor_Disable && (data_int == 0 || data_int == 1))
			{
				if (OnBoard_Encoder.reverse != (uint8_t)data_int)
					MotorControl.friction_model_valid = false;
				Encoder_SetReverse(&OnBoard_Encoder, data_int != 0);
			}
		break;
		case CAN_GET_ENCODER_REVERSE:
			CAN_SendMessage_Update(CAN_GET_ENCODER_REVERSE, (float)OnBoard_Encoder.reverse);
		break;

		case CAN_SET_CURRENT_CAL:
			if(data >= 0.0f && data <= CURRENT_CALIB_LIMIT_MAX_A)
				MotorControl.calib_current = data;
		break;
		case CAN_GET_CURRENT_CAL:
			CAN_SendMessage_Update(CAN_GET_CURRENT_CAL, MotorControl.calib_current);
		break;
		
		case CAN_SET_CURRENT_LIMIT:
			if(data >= 0.0f && data <= CURRENT_COMMAND_LIMIT_MAX_A)
			{
				MotorControl.current_limit = data;
				MotorControl.iqRef = constrain(MotorControl.iqRef, -data, data);
			}
		break;
		case CAN_GET_CURRENT_LIMIT:
			CAN_SendMessage_Update(CAN_GET_CURRENT_LIMIT, MotorControl.current_limit);
		break;
		
		case CAN_SET_SPEED_LIMIT:
			if(data > 0.0f && data <= PARAM_MOTOR_SPEED_LIMIT_RPS * _2PI)
			{
				MotorControl.speed_limit = data;
				if (MotorControl.pos_maxspeed > MotorControl.speed_limit)
				{
					MotorControl.pos_maxspeed = MotorControl.speed_limit;
				}
			}
		break;
		case CAN_GET_SPEED_LIMIT:
			CAN_SendMessage_Update(CAN_GET_SPEED_LIMIT, MotorControl.speed_limit);
		break;
			
		case CAN_SET_SPEED_ACC:
			if(data >= 0.0f && data <= 1000.0f * _2PI)
				MotorControl.speedAcc = data;
		break;
		case CAN_GET_SPEED_ACC:
			CAN_SendMessage_Update(CAN_GET_SPEED_ACC, MotorControl.speedAcc);
		break;
		
		case CAN_SET_SPEED_DEC:
			if(data >= 0.0f && data <= 1000.0f * _2PI)
				MotorControl.speedDec = data;
		break;
		case CAN_GET_SPEED_DEC:
			CAN_SendMessage_Update(CAN_GET_SPEED_DEC, MotorControl.speedDec);
		break;
		
		case CAN_SET_SPEED_KP:
			if(data >= 0.01f && data <= 2.0f)
				MotorControl.speed_Kp = data;
		break;
		case CAN_GET_SPEED_KP:
			CAN_SendMessage_Update(CAN_GET_SPEED_KP, MotorControl.speed_Kp);
		break;
		
		case CAN_SET_SPEED_KI:
			if(data >= 0.0f && data <= 2.0f)
				MotorControl.speed_Ki = data;
		break;
		case CAN_GET_SPEED_KI:
			CAN_SendMessage_Update(CAN_GET_SPEED_KI, MotorControl.speed_Ki);
		break;
		
		case CAN_SET_POS_ACC:
			if(data > 0.0f && data <= 200.0f * _2PI)
				MotorControl.posAcc = data;
		break;
		case CAN_GET_POS_ACC:
			CAN_SendMessage_Update(CAN_GET_POS_ACC, MotorControl.posAcc);
		break;
		
		case CAN_SET_POS_DEC:
			if(data > 0.0f && data <= 200.0f * _2PI)
				MotorControl.posDec = data;
		break;
		case CAN_GET_POS_DEC:
			CAN_SendMessage_Update(CAN_GET_POS_DEC, MotorControl.posDec);
		break;
		
		case CAN_SET_POS_MAXSPEED:
			if(data > 0.0f && data <= POSITION_IMPEDANCE_MAX_SPEED_RPS * _2PI &&
			   data <= MotorControl.speed_limit)
			{
				float position_maxspeed = data;
				if (MotorControl.pos_maxspeed != position_maxspeed)
					MotorControl.pos_maxspeed = position_maxspeed;
			}
		break;
		case CAN_GET_POS_MAXSPEED:
			CAN_SendMessage_Update(CAN_GET_POS_MAXSPEED, MotorControl.pos_maxspeed);
		break;
		
		case CAN_SET_POS_KP:
			if(data >= 0.0f && data <= POSITION_IMPEDANCE_KP_MAX_A_PER_RAD)
				MotorControl.pos_Kp = data;
		break;
		case CAN_GET_POS_KP:
			CAN_SendMessage_Update(CAN_GET_POS_KP, MotorControl.pos_Kp);
		break;

		case CAN_SET_POS_KD:
			if(data >= 0.0f && data <= POSITION_IMPEDANCE_KD_MAX_A_PER_RAD_S)
				MotorControl.pos_Kd = data;
		break;
		case CAN_GET_POS_KD:
			CAN_SendMessage_Update(CAN_GET_POS_KD, MotorControl.pos_Kd);
		break;

		case CAN_SET_POS_KI:
			if(data >= 0.0f && data <= POSITION_IMPEDANCE_KI_MAX_A_PER_RAD_S)
				MotorControl.pos_Ki = data;
		break;
		case CAN_GET_POS_KI:
			CAN_SendMessage_Update(CAN_GET_POS_KI, MotorControl.pos_Ki);
		break;

		case CAN_SET_POS_INTEGRAL_LIMIT:
			if(data >= 0.0f && data <= CURRENT_COMMAND_LIMIT_MAX_A)
				MotorControl.pos_integral_limit = data;
		break;
		case CAN_GET_POS_INTEGRAL_LIMIT:
			CAN_SendMessage_Update(CAN_GET_POS_INTEGRAL_LIMIT,
				MotorControl.pos_integral_limit);
		break;

		case CAN_SET_CASCADE_POS_KP:
			if(data >= 0.0f && data <= CASCADE_POSITION_KP_MAX_PER_S)
				MotorControl.cascade_pos_Kp = data;
		break;
		case CAN_GET_CASCADE_POS_KP:
			CAN_SendMessage_Update(CAN_GET_CASCADE_POS_KP,
				MotorControl.cascade_pos_Kp);
		break;

		case CAN_SET_CASCADE_POS_KD:
			if(data >= 0.0f && data <= CASCADE_POSITION_KD_MAX)
				MotorControl.cascade_pos_Kd = data;
		break;
		case CAN_GET_CASCADE_POS_KD:
			CAN_SendMessage_Update(CAN_GET_CASCADE_POS_KD,
				MotorControl.cascade_pos_Kd);
		break;

		case CAN_APPLY_FRICTION_MODEL:
			if (data_int == 1)
				(void)FocFrictionIdentification_ApplyCandidate(&MotorControl);
		break;
		case CAN_GET_FRICTION_STATE:
			CAN_SendMessage_Update(CAN_GET_FRICTION_STATE,
				(float)FocFrictionIdentification_GetState());
		break;
		case CAN_GET_FRICTION_REASON:
			CAN_SendMessage_Update(CAN_GET_FRICTION_REASON,
				(float)FocFrictionIdentification_GetReason());
		break;
		case CAN_GET_FRICTION_COULOMB_POS:
			CAN_SendMessage_Update(CAN_GET_FRICTION_COULOMB_POS,
				FocFrictionIdentification_GetResult()->coulomb_pos_a);
		break;
		case CAN_GET_FRICTION_COULOMB_NEG:
			CAN_SendMessage_Update(CAN_GET_FRICTION_COULOMB_NEG,
				FocFrictionIdentification_GetResult()->coulomb_neg_a);
		break;
		case CAN_GET_FRICTION_VISCOUS_POS:
			CAN_SendMessage_Update(CAN_GET_FRICTION_VISCOUS_POS,
				FocFrictionIdentification_GetResult()->viscous_pos_a_per_rad_s);
		break;
		case CAN_GET_FRICTION_VISCOUS_NEG:
			CAN_SendMessage_Update(CAN_GET_FRICTION_VISCOUS_NEG,
				FocFrictionIdentification_GetResult()->viscous_neg_a_per_rad_s);
		break;
		case CAN_GET_FRICTION_RMSE_POS:
			CAN_SendMessage_Update(CAN_GET_FRICTION_RMSE_POS,
				FocFrictionIdentification_GetResult()->rmse_pos_a);
		break;
		case CAN_GET_FRICTION_RMSE_NEG:
			CAN_SendMessage_Update(CAN_GET_FRICTION_RMSE_NEG,
				FocFrictionIdentification_GetResult()->rmse_neg_a);
		break;
		case CAN_GET_FRICTION_CANDIDATE_VALID:
			CAN_SendMessage_Update(CAN_GET_FRICTION_CANDIDATE_VALID,
				FocFrictionIdentification_GetResult()->valid ? 1.0f : 0.0f);
		break;
		case CAN_GET_FRICTION_MODEL_VALID:
			CAN_SendMessage_Update(CAN_GET_FRICTION_MODEL_VALID,
				MotorControl.friction_model_valid ? 1.0f : 0.0f);
		break;
		
		case CAN_SET_COGGING:
//			if(data_int == 0)
//				MotorControl.isUseAnticogging = false;
//			else if(data_int == 1)
//				MotorControl.isUseAnticogging = true;
		break;
		case CAN_GET_COGGING:
//			CAN_SendMessage_Update(CAN_GET_COGGING, (float)MotorControl.isUseAnticogging);
		break;
	
		case CAN_SET_CAN_BR:
			if(data_int == 100 || data_int == 125  || data_int == 200  || data_int == 250  || data_int == 500 ||
			   data_int ==1000 || data_int == 2000 || data_int == 2500 || data_int == 5000)
				CANMsg.baudrate = data_int;
		break;
		case CAN_GET_CAN_BR:
			CAN_SendMessage_Update(CAN_GET_CAN_HB, (float)CANMsg.baudrate);
		break;
		
		case CAN_SET_CAN_HB:
			if((data_int >= 500 && data <= 1000) || data_int == 0)
				CANMsg.can_hb_set = data_int;
		break;
		case CAN_GET_CAN_HB:
			CAN_SendMessage_Update(CAN_GET_CAN_HB, (float)CANMsg.can_hb_set);
		break;
		
		/*state parameters*/
		case CAN_GET_VBUS:
			CAN_SendMessage_Update(CAN_GET_VBUS, FOC.Vbus_filt);
		break;
		
		case CAN_GET_IBUS:
			CAN_SendMessage_Update(CAN_GET_IBUS, FOC.Ibus_filt);
		break;
		
		case CAN_GET_IA:
			CAN_SendMessage_Update(CAN_GET_IA, FOC.Ia);
		break;
		
		case CAN_GET_IB:
			CAN_SendMessage_Update(CAN_GET_IB, FOC.Ib);
		break;
		
		case CAN_GET_IC:
			CAN_SendMessage_Update(CAN_GET_IC, FOC.Ic);
		break;
		
		case CAN_GET_ID:
			CAN_SendMessage_Update(CAN_GET_ID, FOC.Id);
		break;
		
		case CAN_GET_IQ:
			CAN_SendMessage_Update(CAN_GET_IQ, FOC.Iq);
		break;
		
		

		case CAN_GET_SPEED2_FILT:
			CAN_SendMessage_Update(CAN_GET_SPEED2_FILT, OnBoard_Encoder.vel_mech);
		break;
		
		case CAN_GET_POS2_FILT:
			CAN_SendMessage_Update(CAN_GET_POS2_FILT, OnBoard_Encoder.theta_mech);
		break;
		
		case CAN_GET_TEMP:
			CAN_SendMessage_Update(CAN_GET_TEMP, FOC.temp);
		break;

		case CAN_GET_RS:
			CAN_SendMessage_Update(CAN_GET_RS, MotorControl.motor_phase_resistance);
		break;

		case CAN_GET_LD:
			CAN_SendMessage_Update(CAN_GET_LD, MotorControl.motor_d_inductance);
		break;

		case CAN_GET_LQ:
			CAN_SendMessage_Update(CAN_GET_LQ, MotorControl.motor_q_inductance);
		break;

		case CAN_GET_FLUX:
			CAN_SendMessage_Update(CAN_GET_FLUX, MotorControl.motor_flux);
		break;		
		
		case CAN_GET_ERROR:
			CAN_SendMessage_Update(CAN_GET_ERROR, (float)MotorControl.ErrorNow);
		break;
		
		default:break;
	}
}

/**
	* @brief  Update CAN transmit message data
	* @param  param_id: CAN parameter id
	* @param  data: CAN transmit data
 **/
void CAN_SendMessage_Update(CAN_PARAM_ID param_id, float data)
{
	CANMsg.tx_param_id = param_id;
	CANMsg.tx_data = data;
	CanValueEncoding encoding = CAN_ReplyEncoding(param_id);
	if (encoding == CAN_VALUE_MILLI_I16) {
		uint16_t value = (uint16_t)CAN_Milli16(data);
		CANMsg.tx_data_u8[0] = (uint8_t)(value >> 8);
		CANMsg.tx_data_u8[1] = (uint8_t)value;
		CANMsg.tx_data_u8[2] = CANMsg.tx_data_u8[3] = 0U;
		CANMsg.tx_data_len = 2U;
	} else {
		uint32_t value;
		if (encoding == CAN_VALUE_MILLI_I32)
			value = (uint32_t)CAN_Milli32(data);
		else if (encoding == CAN_VALUE_CENTI_I32)
			value = (uint32_t)CAN_Centi32(data);
		else
			value = FloatToIntBit(data);
		CANMsg.tx_data_u8[0] = (uint8_t)(value >> 24);
		CANMsg.tx_data_u8[1] = (uint8_t)(value >> 16);
		CANMsg.tx_data_u8[2] = (uint8_t)(value >> 8);
		CANMsg.tx_data_u8[3] = (uint8_t)value;
		CANMsg.tx_data_len = 4U;
	}
	
	CANMsg.can_tx_en = true;
}

/**
	* @brief  CAN Rx interrupt Handle  
			  extract param id and data from mail box
 **/
void CANRxIRQHandler(void)
{
	CommHwCanFrame frame;
	uint8_t node_id;
	uint8_t param_id;
	uint32_t u32_data = 0;
	float decoded_data;
	CanValueEncoding encoding;
	
	/* Other nodes' 48-byte status can reach node 7's legacy range filter.
	 * Receive into a full CAN FD buffer; reject status/invalid lengths before
	 * decoding commands or refreshing the control heartbeat. */
	if (!comm_hw_can_receive(&frame) || frame.extended || frame.remote ||
		(frame.length != 2U && frame.length != 4U) || frame.identifier > 0x7FFU ||
		(frame.identifier >= CAN_MOTOR_STATUS_ID_BASE &&
		 frame.identifier < CAN_MOTOR_STATUS_ID_BASE + 8U)) return;
	
	/*high 3 bits*/
	node_id  = frame.identifier >> 8;
	/*low 8 bits*/
	param_id = frame.identifier & 0x0FF;
		
	/*node id matches*/
	if(node_id == CANMsg.node_id)
	{
		encoding = CAN_CommandEncoding((CAN_PARAM_ID)param_id);
		if ((encoding == CAN_VALUE_MILLI_I16 && frame.length != 2U) ||
		    (encoding != CAN_VALUE_MILLI_I16 && frame.length != 4U)) return;
		for (unsigned i = 0; i < frame.length; ++i)
			CANMsg.rx_data_u8[i] = frame.data[i];
		if (encoding == CAN_VALUE_MILLI_I16) {
			int16_t value = (int16_t)(((uint16_t)frame.data[0] << 8) |
				frame.data[1]);
			if (value == INT16_MIN) return;
			decoded_data = (float)value / 1000.0f;
		} else {
			u32_data |= (uint32_t)frame.data[0] << 24;
			u32_data |= (uint32_t)frame.data[1] << 16;
			u32_data |= (uint32_t)frame.data[2] << 8;
			u32_data |= (uint32_t)frame.data[3];
			if ((encoding == CAN_VALUE_MILLI_I32 ||
			     encoding == CAN_VALUE_CENTI_I32) &&
			    (int32_t)u32_data == INT32_MIN) return;
			if (encoding == CAN_VALUE_MILLI_I32)
				decoded_data = (float)(int32_t)u32_data / 1000.0f;
			else if (encoding == CAN_VALUE_CENTI_I32)
				decoded_data = (float)(int32_t)u32_data / 100.0f;
			else
				decoded_data = IntBitToFloat(u32_data);
		}

		CANMsg.can_rx_en = true;
		CANMsg.can_hb_count = 0;
		
		/*clear error if CAN receiver resumed*/
		if(MotorControl.ErrorNow == CAN_DisConnect)
			Set_ErrorNow(No_Error);
		
		CANMsg.rx_param_id = (CAN_PARAM_ID)param_id;
		CANMsg.rx_data     = decoded_data;
		
		CAN_ReceiveMessage_Update(CANMsg.rx_param_id, CANMsg.rx_data);
	}
	
}

/**
	* @brief  CAN Tx function   
			  use ExtId, DLC length 4
 **/
void CAN_SendMessage(void)
{
	if(CANMsg.can_tx_en == false) {
		uint8_t payload[CAN_MOTOR_STATUS_SIZE];
		uint16_t identifier;
		if (CanMotorStatus_Prepare(time_hw_now_ms(), CANMsg.node_id,
			&identifier, payload, sizeof(payload)) && !CANMsg.can_tx_en)
			(void)comm_hw_can_try_send_status(identifier, payload, sizeof(payload));
		return;
	}
	
	FDCAN_TxHeaderTypeDef FDCAN_TxHeader;
	uint32_t ID = CANMsg.node_id << 8 | CANMsg.tx_param_id;
	
	uint8_t send_num = 0;
	
	FDCAN_TxHeader.IdType				 = FDCAN_STANDARD_ID;
	FDCAN_TxHeader.Identifier			 = ID;
	FDCAN_TxHeader.FDFormat				 = FDCAN_FD_CAN;
	FDCAN_TxHeader.DataLength			 = CANMsg.tx_data_len;
	FDCAN_TxHeader.TxFrameType			 = FDCAN_DATA_FRAME;
	FDCAN_TxHeader.BitRateSwitch		 = FDCAN_BRS_ON;
	FDCAN_TxHeader.TxEventFifoControl	 = FDCAN_NO_TX_EVENTS;
	
	while(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &FDCAN_TxHeader, CANMsg.tx_data_u8))
	{
		/* blocked*/
		if(++send_num == 5)
			break;
	}

	CANMsg.can_tx_en = false;
	
}
