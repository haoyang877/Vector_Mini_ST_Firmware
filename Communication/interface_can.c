#include "interface_can.h"

#include "fdcan.h"
#include "delay.h"
#include "utils.h"
#include "foc_algorithm.h"
#include "foc_param.h"
#include "foc_errhandle.h"
#include "encoder.h"

CANMsg_TypeDef CANMsg;

extern MotorControl_TypeDef MotorControl;
extern ModeNow_TypeDef ModeLast;
extern FOC_TypeDef FOC;
extern Encoder_TypeDef External_Encoder;

/**
	* @brief  FDCAN1 Filter Init  
			  Stdandard ID, Range Mode 
 **/
void FDCAN1_Param_Init(void)
{
	FDCAN_FilterTypeDef FDCAN_Filter;
	
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
	if(CANMsg.can_hb_set != 0)
	{
		if(MotorControl.ModeNow  == Current_Mode || 
		   MotorControl.ModeNow  == Speed_Mode   || 
		   MotorControl.ModeNow  == Position_Mode  )
		{
			CANMsg.can_hb_en = true;
		}
	}
	else
	{
		CANMsg.can_hb_en = false;
	}
		
	if(CANMsg.can_rx_en == true && CANMsg.can_hb_en == true)
	{
		/*timeout protect*/
		if(++ CANMsg.can_hb_count >= CANMsg.can_hb_set)
		{
			Set_ErrorNow(CAN_DisConnect);
		}
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
	return Encoder_IsOnline(&External_Encoder) ? 1 : 0;
}

/**
	* @brief  Handle received CAN message
			  update motor control paramters
    * @param  param_id: CAN parameter id 
    * @param  data: CAN parameter data
 **/
void CAN_ReceiveMessage_Update(CAN_PARAM_ID param_id, float data)
{
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
			if(fast_abs(data) <= MotorControl.speed_limit * ONE_BY_2PI)
				MotorControl.speedRef = data * _2PI;
		break;
		case CAN_GET_SPEED_SET:
			CAN_SendMessage_Update(CAN_GET_SPEED_SET, MotorControl.speedRef * ONE_BY_2PI);
		break;
		
		case CAN_SET_POS:
			ModeSwitch_Handle(Position_Mode);
			MotorControl.posRef  = data * _2PI;
		break;
		case CAN_GET_POS_SET:
			CAN_SendMessage_Update(CAN_GET_POS_SET, MotorControl.posRef * ONE_BY_2PI);
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
				Encoder_SetReverse(&External_Encoder, data_int != 0);
		break;
		case CAN_GET_ENCODER_REVERSE:
			CAN_SendMessage_Update(CAN_GET_ENCODER_REVERSE, (float)External_Encoder.reverse);
		break;

		case CAN_SET_CURRENT_CAL:
			if(data >= 0.0f && data <= 30.0f)
				MotorControl.calib_current = data;
		break;
		case CAN_GET_CURRENT_CAL:
			CAN_SendMessage_Update(CAN_GET_CURRENT_CAL, MotorControl.calib_current);
		break;
		
		case CAN_SET_CURRENT_LIMIT:
			if(data >= 0.0f && data <= 60.0f)
				MotorControl.current_limit = data;
		break;
		case CAN_GET_CURRENT_LIMIT:
			CAN_SendMessage_Update(CAN_GET_CURRENT_LIMIT, MotorControl.current_limit);
		break;
		
		case CAN_SET_SPEED_LIMIT:
			if(data >=0.0f && data <= 400.0f)
				MotorControl.speed_limit = data * _2PI;
		break;
		case CAN_GET_SPEED_LIMIT:
			CAN_SendMessage_Update(CAN_GET_SPEED_LIMIT, MotorControl.speed_limit * ONE_BY_2PI);
		break;
			
		case CAN_SET_SPEED_ACC:
			if(data >= 0.0f && data <= 1000.0f)
				MotorControl.speedAcc = data * _2PI;
		break;
		case CAN_GET_SPEED_ACC:
			CAN_SendMessage_Update(CAN_GET_SPEED_ACC, MotorControl.speedAcc * ONE_BY_2PI);
		break;
		
		case CAN_SET_SPEED_DEC:
			if(data >= 0.0f && data <= 1000.0f)
				MotorControl.speedDec = data * _2PI;
		break;
		case CAN_GET_SPEED_DEC:
			CAN_SendMessage_Update(CAN_GET_SPEED_DEC, MotorControl.speedDec * ONE_BY_2PI);
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
			if(data >= 0.0f && data <= 200.0f)
				MotorControl.posAcc = data * _2PI;
		break;
		case CAN_GET_POS_ACC:
			CAN_SendMessage_Update(CAN_GET_POS_ACC, MotorControl.posAcc * ONE_BY_2PI);
		break;
		
		case CAN_SET_POS_DEC:
			if(data >= 0.0f && data <= 200.0f)
				MotorControl.posDec = data * _2PI;		
		break;
		case CAN_GET_POS_DEC:
			CAN_SendMessage_Update(CAN_GET_POS_DEC, MotorControl.posDec * ONE_BY_2PI);	
		break;
		
		case CAN_SET_POS_MAXSPEED:
			if(fast_abs(data) <= MotorControl.speed_limit * ONE_BY_2PI)
				MotorControl.pos_maxspeed = data * _2PI;			
		break;
		case CAN_GET_POS_MAXSPEED:
			CAN_SendMessage_Update(CAN_GET_POS_MAXSPEED, MotorControl.pos_maxspeed * ONE_BY_2PI);		
		break;
		
		case CAN_SET_POS_KP:
			if(data >= 0.01f && data <= 1.0f)
				MotorControl.pos_Kp = data;
		break;
		case CAN_GET_POS_KP:
			CAN_SendMessage_Update(CAN_GET_POS_KP, MotorControl.pos_Kp);
		break;

		case CAN_SET_POS_KD:
			if(data >= 0.0f && data <= 1.0f)
				MotorControl.pos_Kd = data;
		break;
		case CAN_GET_POS_KD:
			CAN_SendMessage_Update(CAN_GET_POS_KD, MotorControl.pos_Kd);
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
			CAN_SendMessage_Update(CAN_GET_SPEED2_FILT, External_Encoder.vel_mech);
		break;
		
		case CAN_GET_POS2_FILT:
			CAN_SendMessage_Update(CAN_GET_POS2_FILT, External_Encoder.theta_mech);
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
	
	uint32_t tx_data_u32;
	
	tx_data_u32 = FloatToIntBit(CANMsg.tx_data);
	
	CANMsg.tx_data_u8[0] = tx_data_u32 >> 24;
	CANMsg.tx_data_u8[1] = tx_data_u32 >> 16;
	CANMsg.tx_data_u8[2] = tx_data_u32 >> 8;
	CANMsg.tx_data_u8[3] = tx_data_u32;
	
	CANMsg.can_tx_en = true;
}

/**
	* @brief  CAN Rx interrupt Handle  
			  extract param id and data from mail box
 **/
void CANRxIRQHandler(void)
{
	FDCAN_RxHeaderTypeDef FDCAN_RxHeader;
	uint8_t node_id;
	uint8_t param_id;
	uint32_t u32_data = 0;
	
	HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &FDCAN_RxHeader, CANMsg.rx_data_u8);
	
	/*high 3 bits*/
	node_id  = FDCAN_RxHeader.Identifier >> 8;
	/*low 8 bits*/
	param_id = FDCAN_RxHeader.Identifier & 0x0FF;
		
	/*node id matches*/
	if(node_id == CANMsg.node_id)
	{
		CANMsg.can_rx_en = true;
		CANMsg.can_hb_count = 0;
		
		/*clear error if CAN receiver resumed*/
		if(MotorControl.ErrorNow == CAN_DisConnect)
			Set_ErrorNow(No_Error);
		
		u32_data |= CANMsg.rx_data_u8[0] << 24;
		u32_data |= CANMsg.rx_data_u8[1] << 16;
		u32_data |= CANMsg.rx_data_u8[2] << 8;
		u32_data |= CANMsg.rx_data_u8[3];
		
		CANMsg.rx_param_id = (CAN_PARAM_ID)param_id;
		CANMsg.rx_data     = IntBitToFloat(u32_data);
		
		CAN_ReceiveMessage_Update(CANMsg.rx_param_id, CANMsg.rx_data);
	}
	
	FDCAN_RxHeader.Identifier = 0;
}

/**
	* @brief  CAN Tx function   
			  use ExtId, DLC length 4
 **/
void CAN_SendMessage(void)
{
	if(CANMsg.can_tx_en == false)
		return;
	
	FDCAN_TxHeaderTypeDef FDCAN_TxHeader;
	uint32_t ID = CANMsg.node_id << 8 | CANMsg.tx_param_id;
	
	uint8_t send_num = 0;
	
	FDCAN_TxHeader.IdType				 = FDCAN_STANDARD_ID;
	FDCAN_TxHeader.Identifier			 = ID;
	FDCAN_TxHeader.FDFormat				 = FDCAN_FD_CAN;
	FDCAN_TxHeader.DataLength			 = 4;
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
