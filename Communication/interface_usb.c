#include "interface_usb.h"

#include "usbd_cdc_if.h"

#include "utils.h"
#include "ring_buffer.h"
#include "foc_algorithm.h"
#include "foc_param.h"
#include "foc_errhandle.h"
#include "encoder.h"
#include "interface_can.h"

USBMsg_TypeDef USBMsg;
USBRxStep_TypeDef USBRXStep = USB_RX_NULL;

extern MotorControl_TypeDef MotorControl;
extern ModeNow_TypeDef ModeLast;
extern Encoder_TypeDef OnBoard_Encoder,External_Encoder;
extern FOC_TypeDef FOC;
extern CANMsg_TypeDef CANMsg;
extern InterfaceParam_TypeDef InterfaceParam;
extern Cyclic_TypeDef Cyclic;

/**
	* @brief  USB ring buffer data analyze
			  use finite state machine
    * @retval USBRxError
 **/
USBRXError_TypeDef USB_CyclicAnalyze(void)
{
	while(1)
	{
		/*written length less than COMMAND_MIN_LENGTH, incomplete data*/
		if(Cyclic_GetLength() < COMMAND_MIN_LENGTH)
		{
			return USB_INCOMPLETE_DATA;
		}
		
		/*head not found, keep moving*/
		if(Cyclic_Read(Cyclic.readindex) != 0x5C)
		{
			Cyclic_AddReadIndex(1);
			continue;
		}
		
		/*head('\')found */
		if(Cyclic_Read(Cyclic.readindex) == 0x5C)
		{
			USBRXStep = USB_RX_HEAD;
		}
		
		if(USBRXStep == USB_RX_HEAD)
		{
			Cyclic_AddReadIndex(1);
			USBRXStep = USB_RX_WRP;
		}
		
		/*write, read, print flag*/
		if(USBRXStep == USB_RX_WRP)
		{
			USBMsg.rx_write_read_print = 0;
			
			if(Cyclic_Read(Cyclic.readindex) == 'w')
			{
				USBMsg.rx_write_read_print = 0;
				Cyclic_AddReadIndex(1);
				USBRXStep = USB_RX_UNDERLINE;
			}
			else if(Cyclic_Read(Cyclic.readindex) == 'r')
			{
				USBMsg.rx_write_read_print = 1;
				Cyclic_AddReadIndex(1);
				USBRXStep = USB_RX_UNDERLINE;
			}
			else if(Cyclic_Read(Cyclic.readindex) == 'p')
			{
				USBMsg.rx_write_read_print = 2;
				Cyclic_AddReadIndex(1);
				USBRXStep = USB_RX_UNDERLINE;
			}
			else
			{
				//error!
				USBRXStep = USB_RX_NULL;
				return USB_SYNTAX_ERROR;
			}
		}
		
		/*underline('_')found*/
		if(USBRXStep == USB_RX_UNDERLINE)
		{
			if(Cyclic_Read(Cyclic.readindex) == '_')
			{
				Cyclic_AddReadIndex(1);
				USBRXStep = USB_RX_PARAM;
			}
			else
			{
				//error!
				USBRXStep = USB_RX_NULL;
				return USB_SYNTAX_ERROR;	
			}
		}
		
		/*parameter analyze*/
		if(USBRXStep == USB_RX_PARAM)
		{
			USBMsg.rx_param_id.str[2] = Cyclic_Read(Cyclic.readindex);
			USBMsg.rx_param_id.str[1] = Cyclic_Read(Cyclic.readindex + 1);
			USBMsg.rx_param_id.str[0] = Cyclic_Read(Cyclic.readindex + 2);
			
			/*read command, go straight to tail*/
			if(USBMsg.rx_write_read_print == 1)
			{
				Cyclic_AddReadIndex(3);
				USBRXStep = USB_RX_TAIL;
			}
			/*write and print command, keep moving*/
			else if(USBMsg.rx_write_read_print == 0 ||USBMsg.rx_write_read_print == 2)
			{
				Cyclic_AddReadIndex(3);
				USBRXStep = USB_RX_EQUAL;
			}
		}
		
		/*equal sign('=')found*/
		if(USBRXStep == USB_RX_EQUAL)
		{
			if(Cyclic_Read(Cyclic.readindex) == '=')
			{
				Cyclic_AddReadIndex(1);
				USBRXStep = USB_RX_SIGN;
			}
			else
			{
				//error!
				USBRXStep = USB_RX_NULL;	
				return USB_SYNTAX_ERROR;		
			}
		}
		
		/*judge the sign of data, positive(+) or negative(-)*/
		if(USBRXStep == USB_RX_SIGN)
		{
			if(Cyclic_Read(Cyclic.readindex) == '-')
			{
				USBMsg.rx_sign = -1.0f;
				Cyclic_AddReadIndex(1);
			}
			else
			{
				USBMsg.rx_sign = 1.0f;
			}
			USBRXStep = USB_RX_INT_DATA;
		}
		
		/*extract integer data*/
		if(USBRXStep == USB_RX_INT_DATA)
		{	
			memset(USBMsg.rx_int_data, 0, 10);
			memset(USBMsg.rx_dec_data, 0, 10);
			USBMsg.int_or_float = 0;
			USBMsg.rx_data = 0.0f;
			USBMsg.rx_int_length = 0;
			USBMsg.rx_float_length = 0;
			int i;
			for(i = 0; Cyclic_Read(Cyclic.readindex + i) != '.'; i++)
			{
				/*no decimal part*/
				if(Cyclic_Read(Cyclic.readindex + i) == '\r')
				{
					USBMsg.rx_int_length = i;
					Cyclic_AddReadIndex(i);
					USBRXStep = USB_RX_TAIL;
					break;
				}
				USBMsg.rx_int_data[i] = Cyclic_Read(Cyclic.readindex + i) - '0';
			}
			
			if(USBRXStep != USB_RX_TAIL)
			{
				USBMsg.rx_int_length = i;
				Cyclic_AddReadIndex(i);
				USBRXStep = USB_RX_POINT;
			}
		}
		
		/*decimal point judgement*/
		if(USBRXStep == USB_RX_POINT)
		{
			if(Cyclic_Read(Cyclic.readindex) == '.')
			{
				USBMsg.int_or_float = 1;
				Cyclic_AddReadIndex(1);
				USBRXStep = USB_RX_FLOAT_DATA;
			}
			else
			{
				USBRXStep = USB_RX_NULL;
				return USB_SYNTAX_ERROR;
			}
		}
		
		/*extract decimal data*/
		if(USBRXStep == USB_RX_FLOAT_DATA)
		{
			int i;
			for(i = 0; Cyclic_Read(Cyclic.readindex + i) != '\r'; i++)
			{
				USBMsg.rx_dec_data[i] = Cyclic_Read(Cyclic.readindex + i) - '0';
			}
			USBMsg.rx_float_length = i;
			Cyclic_AddReadIndex(i);
			USBRXStep = USB_RX_TAIL;
		}
		
		/*tail handle*/
		if(USBRXStep == USB_RX_TAIL)
		{
			if(Cyclic_Read(Cyclic.readindex) == '\r' && Cyclic_Read(Cyclic.readindex + 1) == '\n')
			{
				Cyclic_AddReadIndex(2);
				USBRXStep = USB_RX_NULL;
				for(int i = 0; i < USBMsg.rx_int_length; i++)
				{
					USBMsg.rx_data += (int)USBMsg.rx_int_data[i] * fast_pow(10.0f, USBMsg.rx_int_length - i - 1);
				}
				for(int i = 0; i < USBMsg.rx_float_length; i++)
				{
					USBMsg.rx_data += (int)USBMsg.rx_dec_data[i] * fast_pow(0.1f, i + 1);
				}
				
				USBMsg.rx_data *= USBMsg.rx_sign;
				return USB_NO_ERROR;
			}
			else
			{
				//error!
				USBRXStep = USB_RX_NULL;
				return USB_SYNTAX_ERROR;	
			}
		}
	}
}

/**
	* @brief  Set encoder state from USB parameter value
	* @param  data: encoded encoder state value
	* @retval USB receive error state
 **/
USBRXError_TypeDef USB_SetEncoderState(int data)
{
	int enc1_type,enc1_enable,enc2_type,enc2_enable;
	
	enc2_enable = data % 10;
	data /= 10;
	enc2_type = data % 10;
	data /= 10;
	enc1_enable = data % 10;
	data /= 10;
	enc1_type = data % 10;
	
	if(enc1_enable == 0)
		OnBoard_Encoder.enable = ENCODER_DISABLE;
	else if(enc1_enable == 1)
		OnBoard_Encoder.enable = ENCODER_ENABLE;
	else
		return USB_DATA_OUT_OF_RANGE;
	
	if(enc1_type == 0)
		OnBoard_Encoder.type = TLE5012B;
	else
		return USB_DATA_OUT_OF_RANGE;
	
	if(enc2_enable == 0)
		External_Encoder.enable = ENCODER_DISABLE;
	else if(enc2_enable == 1)
		External_Encoder.enable = ENCODER_ENABLE;
	else
		return USB_DATA_OUT_OF_RANGE;
	
	if(enc2_type == 0)
		External_Encoder.type = TLE5012B;
	else if(enc2_type == 1)
		External_Encoder.type = MT6816;
	else if(enc2_type == 2)
		External_Encoder.type = MT6701;
	else
		return USB_DATA_OUT_OF_RANGE;
	
	return USB_NO_ERROR;
}

/**
	* @brief  Get encoder state and format USB response string
 **/
void USB_GetEncoderState(void)
{
	char enc1_type[10],enc1_enable[10],enc2_type[10],enc2_enable[10];
	
	if(OnBoard_Encoder.type == TLE5012B)
		sprintf(enc1_type, "TLE5012B");
	
	if(OnBoard_Encoder.enable == ENCODER_DISABLE)
		sprintf(enc1_enable, "Disable");
	else if(OnBoard_Encoder.enable == ENCODER_ENABLE)
		sprintf(enc1_enable, "Enable");
	
	if(External_Encoder.type == TLE5012B)
		sprintf(enc2_type, "TLE5012B");
	else if(External_Encoder.type == MT6816)
		sprintf(enc2_type, "MT6816");
	else if(External_Encoder.type == MT6701)
		sprintf(enc2_type, "MT6701");
	
	if(External_Encoder.enable == ENCODER_DISABLE)
		sprintf(enc2_enable, "Disable");
	else if(External_Encoder.enable == ENCODER_ENABLE)
		sprintf(enc2_enable, "Enable");
	
	sprintf(USBMsg.tx_str, "enc1_type=%s-%s.\r\nenc2_type=%s-%s.\r\n", enc1_type, enc1_enable, enc2_type, enc2_enable);
}

/**
	* @brief  USB update receive message
	* @param  w_r_p: write 0, read 1, print 2 flag	
	* @param  param_id: USB paramter id
	* @param  data: USB parameter data
	* @param  int_or_float: int data 0, float data 1 
	* @retval USBRxError 
 **/
USBRXError_TypeDef USB_ReceiveMessage_Update(uint8_t w_r_p, USB_PARAM_ID param_id, float data, uint8_t int_or_float)
{	
	int data_int = (int)data;
	
	/*write*/
	if(w_r_p == 0)
	{
		switch(param_id)
		{
			/*running parameter*/
			case USB_MODE:
				if(int_or_float == 1)
					return USB_DATA_INVALID;
				
				if(data_int >=0 && data_int <= (int)MODE_NUM)
					ModeSwitch_Handle((ModeNow_TypeDef)data_int);
				else
					return USB_DATA_OUT_OF_RANGE;
			break;
			
			case USB_CURRENT_SET:
				ModeSwitch_Handle(Current_Mode);
				if(fast_abs(data) <= MotorControl.current_limit)
					MotorControl.iqRef = data;
				else
					return USB_DATA_OUT_OF_RANGE;
			break;
			
			case USB_SPEED_SET:
				ModeSwitch_Handle(Speed_Mode);
				if(fast_abs(data) <= MotorControl.speed_limit * ONE_BY_2PI)
					MotorControl.speedRef = data * _2PI;
				else
					return USB_DATA_OUT_OF_RANGE;
			break;
			
			case USB_POS_SET:
				ModeSwitch_Handle(Position_Mode);
				MotorControl.posRef = data * _2PI;
			break;
			
			/*user parameters*/
			case USB_NODE_ID:
				if(int_or_float == 1)
					return USB_DATA_INVALID;
				
				if(data_int >= 0 && data_int <= 7)
					CANMsg.node_id = data;
				else
					return USB_DATA_OUT_OF_RANGE;
			break;
			
			case USB_POLEPARIS:
				if(int_or_float == 1)
					return USB_DATA_INVALID;
				
				if(data_int >= 2 && data_int <= 30)
					MotorControl.motor_pole_pairs = data;
				else
					return USB_DATA_OUT_OF_RANGE;
			break;
			
			case USB_ENCODER_STATE:
				if(int_or_float == 1)
					return USB_DATA_INVALID;
				
				if(USB_SetEncoderState(data_int) != USB_NO_ERROR)
					return USB_DATA_OUT_OF_RANGE;
				
				if(MotorControl.ModeNow == Current_Mode || 
				   MotorControl.ModeNow == Speed_Mode   || 
				   MotorControl.ModeNow == Speed_Mode     )
					return USB_WRITE_INVALID;
			break;
			
			case USB_CURRENT_CAL:
				if(data >= 0.0f && data <= 30.0f)
					MotorControl.calib_current = data;
				else
					return USB_DATA_OUT_OF_RANGE;
			break;
			
			case USB_CURRENT_LIMIT:
				if(data >= 0.0f && data <= 60.0f)
					MotorControl.current_limit = data;
				else
					return USB_DATA_OUT_OF_RANGE;
			break;
			
			case USB_SPEED_LIMIT:
				if(data >= 0.0f && data <= 400.0f)
					MotorControl.speed_limit = data * _2PI;
				else
					return USB_DATA_OUT_OF_RANGE;
			break;
			
			case USB_SPEED_ACC:
				if(data >= 0.0f && data <= 1000.0f)
					MotorControl.speedAcc = data * _2PI;
				else
					return USB_DATA_OUT_OF_RANGE;
			break;
			
			case USB_SPEED_DEC:
				if(data >= 0.0f && data <= 1000.0f)
					MotorControl.speedDec = data * _2PI;
				else
					return USB_DATA_OUT_OF_RANGE;
			break;
			
			case USB_SPEED_KP:
				if(data >= 0.01f && data <= 2.0f)
					MotorControl.speed_Kp = data;
				else
					return USB_DATA_OUT_OF_RANGE;
			break;
			
			case USB_SPEED_KI:
				if(data >= 0.0f && data <= 2.0f)
					MotorControl.speed_Ki = data;
				else
					return USB_DATA_OUT_OF_RANGE;
			break;
			
			case USB_POS_ACC:
				if(data >= 0.0f && data <= 200.0f)
					MotorControl.posAcc = data * _2PI;
				else
					return USB_DATA_OUT_OF_RANGE;
			break;
			
			case USB_POS_DEC:
				if(data >= 0.0f && data <= 200.0f)
					MotorControl.posDec = data * _2PI;
				else
					return USB_DATA_OUT_OF_RANGE;
			break;
			
			case USB_POS_MAXSPEED:
				if(fast_abs(data) <= MotorControl.speed_limit * ONE_BY_2PI)
					MotorControl.pos_maxspeed = data * _2PI;	
				else
					return USB_DATA_OUT_OF_RANGE;
			break;
			
			case USB_POS_KP:
				if(data >= 0.01f && data <= 1.0f)
					MotorControl.pos_Kp = data;
				else
					return USB_DATA_OUT_OF_RANGE;
			break;
			
			case USB_POS_KD:
				if(data >= 0.0f && data <= 5.0f)
					MotorControl.pos_Kd = data;
				else
					return USB_DATA_OUT_OF_RANGE;
			break;
			
			case USB_COGGING:
//				if(int_or_float == 1)
//					return USB_DATA_INVALID;
//				
//				if(data_int == 0)
//					 data_int = 0;
//					MotorControl.isUseAnticogging = false;
//				else if(data_int == 1)
//					MotorControl.isReachTargetPos = true;
//				else
//					return USB_DATA_OUT_OF_RANGE;
			break;

			case USB_CAN_BR:
				if(int_or_float == 1)
					return USB_DATA_INVALID;
				
				if(data_int == 100 || data_int == 125  || data_int == 200  || data_int == 250  || data_int == 500 ||
				   data_int ==1000 || data_int == 2000 || data_int == 2500 || data_int == 5000)
					CANMsg.baudrate = data_int;
				else
					return USB_DATA_OUT_OF_RANGE;
			break;
				
			case USB_CAN_HB:
				if(int_or_float == 1)
					return USB_DATA_INVALID;
				
				if((data_int >= 500 && data <= 1000) || data_int == 0)
					CANMsg.can_hb_set = data_int;
				else
					return USB_DATA_OUT_OF_RANGE;
			break;
			
			/*state parameters*/
			case USB_VBUS:
				return USB_WRITE_INVALID;
			
			case USB_IBUS:
				return USB_WRITE_INVALID;
			
			case USB_IA:
				return USB_WRITE_INVALID;
		
			case USB_IB:
				return USB_WRITE_INVALID;
			
			case USB_IC:
				return USB_WRITE_INVALID;
			
			case USB_ID:
				return USB_WRITE_INVALID;
			
			case USB_IQ:
				return USB_WRITE_INVALID;
			
			case USB_SPEED1_FILT:
				return USB_WRITE_INVALID;
			
			case USB_POS1_FILT:
				return USB_WRITE_INVALID;

			case USB_SPEED2_FILT:
				return USB_WRITE_INVALID;
			
			case USB_POS2_FILT:
				return USB_WRITE_INVALID;
			
			case USB_TEMP:
				return USB_WRITE_INVALID;

			case USB_RS:
				return USB_WRITE_INVALID;
			
			case USB_LD:
				return USB_WRITE_INVALID;
			
			case USB_LQ:
				return USB_WRITE_INVALID;
			
			case USB_FLUX:
				return USB_WRITE_INVALID;
			
			case USB_ERROR:
				return USB_WRITE_INVALID;
			
			default:
				return USB_UNKNOWNED_PARAM;
		}
		sprintf(USBMsg.tx_str, "Write Success!\r\n");
	}
	
	/*read*/
	else if(w_r_p == 1)
	{
		switch(param_id)
		{
			/*running parameters*/
			case USB_MODE:
				sprintf(USBMsg.tx_str, "mode=%d\r\n", (int)MotorControl.ModeNow);
			break;
			
			case USB_CURRENT_SET:
				sprintf(USBMsg.tx_str, "i_set=%.2fA\r\n", MotorControl.iqRef);
			break;
			
			case USB_SPEED_SET:
				sprintf(USBMsg.tx_str, "spd_set=%.2fr/s\r\n", MotorControl.speedRef * ONE_BY_2PI);
			break;
			
			case USB_POS_SET:
				sprintf(USBMsg.tx_str, "pos_set=%.2fr\r\n", MotorControl.posRef * ONE_BY_2PI);
			break;
			
			/*user parameters*/
			case USB_NODE_ID:
				sprintf(USBMsg.tx_str, "can_id=%d\r\n", (int)CANMsg.node_id);
			break;
			
			case USB_POLEPARIS:
				sprintf(USBMsg.tx_str, "pol=%d\r\n", (int)MotorControl.motor_pole_pairs);
			break;
			
			case USB_ENCODER_STATE:
				USB_GetEncoderState();
			break;
			
			case USB_CURRENT_CAL:
				sprintf(USBMsg.tx_str, "i_cal=%.2fA\r\n", MotorControl.calib_current);
			break;
			
			case USB_CURRENT_LIMIT:
				sprintf(USBMsg.tx_str, "i_lim=%.2fA\r\n", MotorControl.current_limit);
			break;
			
			case USB_SPEED_LIMIT:
				sprintf(USBMsg.tx_str, "spd_lim=%.2fr/s\r\n", MotorControl.speed_limit * ONE_BY_2PI);
			break;
			
			case USB_SPEED_ACC:
				sprintf(USBMsg.tx_str, "spd_acc=%.2fr/(s，s)\r\n", MotorControl.speedAcc * ONE_BY_2PI);
			break;
			
			case USB_SPEED_DEC:
				sprintf(USBMsg.tx_str, "spd_dec=%.2fr/(s，s)\r\n", MotorControl.speedDec * ONE_BY_2PI);
			break;
			
			case USB_SPEED_KP:
				sprintf(USBMsg.tx_str, "spd_kp=%.2f\r\n", MotorControl.speed_Kp);
			break;
			
			case USB_SPEED_KI:
				sprintf(USBMsg.tx_str, "spd_ki=%.2f\r\n", MotorControl.speed_Ki);
			break;
			
			case USB_POS_ACC:
				sprintf(USBMsg.tx_str, "pos_acc=%.2fr/(s，s)\r\n", MotorControl.posAcc * ONE_BY_2PI);
			break;
			
			case USB_POS_DEC:
				sprintf(USBMsg.tx_str, "pos_dec=%.2fr/(s，s)\r\n", MotorControl.posDec * ONE_BY_2PI);
			break;
			
			case USB_POS_MAXSPEED:
				sprintf(USBMsg.tx_str, "pos_maxspd=%.2fr/s\r\n", MotorControl.pos_maxspeed * ONE_BY_2PI);
			break;
				
			case USB_POS_KP:
				sprintf(USBMsg.tx_str, "pos_kp=%.2f\r\n", MotorControl.pos_Kp);
			break;
			
			case USB_POS_KD:
				sprintf(USBMsg.tx_str, "pos_kd=%.2f\r\n", MotorControl.pos_Kd);
			break;

			case USB_CAN_BR:
				sprintf(USBMsg.tx_str, "can_br=%dkbps\r\n", (int)CANMsg.baudrate);
			break;
				
			case USB_CAN_HB:
				sprintf(USBMsg.tx_str, "can_hb=%dms\r\n", (int)CANMsg.can_hb_set);
			break;
			
			/*state parameters*/
			case USB_VBUS:
				sprintf(USBMsg.tx_str, "vbus=%.2fV\r\n", FOC.Vbus_filt);
			break;
			
			case USB_IBUS:
				sprintf(USBMsg.tx_str, "ibus=%.2fA\r\n", FOC.Ibus_filt);
			break;
			
			case USB_IA:
				sprintf(USBMsg.tx_str, "ia=%.2fA\r\n", FOC.Ia);
			break;
			
			case USB_IB:
				sprintf(USBMsg.tx_str, "ib=%.2fA\r\n", FOC.Ib);
			break;
			
			case USB_IC:
				sprintf(USBMsg.tx_str, "ic=%.2fA\r\n", FOC.Ic);
			break;
			
			case USB_ID:
				sprintf(USBMsg.tx_str, "id=%.2fA\r\n", FOC.Id);
			break;
			
			case USB_IQ:
				sprintf(USBMsg.tx_str, "iq=%.2fA\r\n", FOC.Iq);
			break;
			
			case USB_SPEED1_FILT:
				sprintf(USBMsg.tx_str, "spd1_filt=%.2fr/s\r\n", OnBoard_Encoder.vel);
			break;
			
			case USB_POS1_FILT:
				sprintf(USBMsg.tx_str, "pos1_filt=%.3fr\r\n", OnBoard_Encoder.pos);
			break;

			case USB_SPEED2_FILT:
				sprintf(USBMsg.tx_str, "spd2_filt=%.2fr/s\r\n", External_Encoder.vel);
			break;
			
			case USB_POS2_FILT:
				sprintf(USBMsg.tx_str, "pos2_filt=%.3fr\r\n", External_Encoder.pos);
			break;
			
			case USB_TEMP:
				sprintf(USBMsg.tx_str, "temp=%.2f＜C\r\n", FOC.temp);
			break;

			case USB_RS:
				sprintf(USBMsg.tx_str, "Rs=%.2fmohm\r\n", MotorControl.motor_phase_resistance * 1000.0f);
			break;
			
			case USB_LD:
				sprintf(USBMsg.tx_str, "Ld=%.2fuH\r\n", MotorControl.motor_d_inductance * 1000000.0f);
			break;
			
			case USB_LQ:
				sprintf(USBMsg.tx_str, "Lq=%.2fuH\r\n", MotorControl.motor_q_inductance * 1000000.0f);
			break;
			
			case USB_FLUX:
				sprintf(USBMsg.tx_str, "Flux=%.2fmWb\r\n", MotorControl.motor_flux * 1000.0f);
			break;
			
			case USB_ERROR:
				sprintf(USBMsg.tx_str, "error=%d\r\n", (int)MotorControl.ErrorNow);
			break;
			
			default:return USB_UNKNOWNED_PARAM;
		}
	}
	
	/*print*/
	else if(w_r_p == 2)
	{
		if(data_int >= 5)
			return USB_DATA_INVALID;
		
		switch(param_id)
		{
			/*running parameters*/
			case USB_MODE:
				USBMsg.p_var[data_int].scale = 1.0f;
				USBMsg.p_var[data_int].type = TYPE_INT32;
				USBMsg.p_var[data_int].addr = &MotorControl.ModeNow;
			break;
			
			case USB_CURRENT_SET:
				USBMsg.p_var[data_int].scale = 1.0f;
				USBMsg.p_var[data_int].type = TYPE_FLOAT;
				USBMsg.p_var[data_int].addr = &MotorControl.iqRef;
			break;
			
			case USB_SPEED_SET:
				USBMsg.p_var[data_int].scale = ONE_BY_2PI;
				USBMsg.p_var[data_int].type = TYPE_FLOAT;
				USBMsg.p_var[data_int].addr = &MotorControl.speedRef;
			break;
			
			case USB_POS_SET:
				USBMsg.p_var[data_int].scale = ONE_BY_2PI;
				USBMsg.p_var[data_int].type = TYPE_FLOAT;
				USBMsg.p_var[data_int].addr = &MotorControl.posRef;
			break;
			
			/*user parameters*/
			case USB_NODE_ID:
				USBMsg.p_var[data_int].scale = 1.0f;
				USBMsg.p_var[data_int].type = TYPE_UINT8;
				USBMsg.p_var[data_int].addr = &CANMsg.node_id;
			break;
			
			case USB_POLEPARIS:
				USBMsg.p_var[data_int].scale = 1.0f;
				USBMsg.p_var[data_int].type = TYPE_INT32;
				USBMsg.p_var[data_int].addr = &MotorControl.motor_pole_pairs;
			break;
			
			case USB_ENCODER_STATE:
				USB_GetEncoderState();
			break;
			
			case USB_CURRENT_CAL:
				USBMsg.p_var[data_int].scale = 1.0f;
				USBMsg.p_var[data_int].type = TYPE_FLOAT;
				USBMsg.p_var[data_int].addr = &MotorControl.calib_current;
			break;
			
			case USB_CURRENT_LIMIT:
				USBMsg.p_var[data_int].scale = 1.0f;
				USBMsg.p_var[data_int].type = TYPE_FLOAT;
				USBMsg.p_var[data_int].addr = &MotorControl.current_limit;
			break;
			
			case USB_SPEED_LIMIT:
				USBMsg.p_var[data_int].scale = ONE_BY_2PI;
				USBMsg.p_var[data_int].type = TYPE_FLOAT;
				USBMsg.p_var[data_int].addr = &MotorControl.speed_limit;
			break;
			
			case USB_SPEED_ACC:
				USBMsg.p_var[data_int].scale = ONE_BY_2PI;
				USBMsg.p_var[data_int].type = TYPE_FLOAT;
				USBMsg.p_var[data_int].addr = &MotorControl.speedAcc;
			break;
			
			case USB_SPEED_DEC:
				USBMsg.p_var[data_int].scale = ONE_BY_2PI;
				USBMsg.p_var[data_int].type = TYPE_FLOAT;
				USBMsg.p_var[data_int].addr = &MotorControl.speedDec;
			break;
			
			case USB_SPEED_KP:
				USBMsg.p_var[data_int].scale = 1.0f;
				USBMsg.p_var[data_int].type = TYPE_FLOAT;
				USBMsg.p_var[data_int].addr = &MotorControl.speed_Kp;
			break;
			
			case USB_SPEED_KI:
				USBMsg.p_var[data_int].scale = 1.0f;
				USBMsg.p_var[data_int].type = TYPE_FLOAT;
				USBMsg.p_var[data_int].addr = &MotorControl.speed_Ki;
			break;
			
			case USB_POS_ACC:
				USBMsg.p_var[data_int].scale = ONE_BY_2PI;
				USBMsg.p_var[data_int].type = TYPE_FLOAT;
				USBMsg.p_var[data_int].addr = &MotorControl.posAcc;
			break;
			
			case USB_POS_DEC:
				USBMsg.p_var[data_int].scale = ONE_BY_2PI;
				USBMsg.p_var[data_int].type = TYPE_FLOAT;
				USBMsg.p_var[data_int].addr = &MotorControl.posDec;
			break;
			
			case USB_POS_MAXSPEED:
				USBMsg.p_var[data_int].scale = ONE_BY_2PI;
				USBMsg.p_var[data_int].type = TYPE_FLOAT;
				USBMsg.p_var[data_int].addr = &MotorControl.pos_maxspeed;
			break;
				
			case USB_POS_KP:
				USBMsg.p_var[data_int].scale = 1.0f;
				USBMsg.p_var[data_int].type = TYPE_FLOAT;
				USBMsg.p_var[data_int].addr = &MotorControl.pos_Kp;
			break;
			
			case USB_POS_KD:
				USBMsg.p_var[data_int].scale = 1.0f;
				USBMsg.p_var[data_int].type = TYPE_FLOAT;
				USBMsg.p_var[data_int].addr = &MotorControl.pos_Kd;
			break;

			case USB_CAN_BR:
				USBMsg.p_var[data_int].scale = 1.0f;
				USBMsg.p_var[data_int].type = TYPE_UINT32;
				USBMsg.p_var[data_int].addr = &CANMsg.baudrate;
			break;
				
			case USB_CAN_HB:
				USBMsg.p_var[data_int].scale = 1.0f;
				USBMsg.p_var[data_int].type = TYPE_UINT32;
				USBMsg.p_var[data_int].addr = &CANMsg.can_hb_set;
			break;
			
			/*state parameters*/
			case USB_VBUS:
				USBMsg.p_var[data_int].scale = 1.0f;
				USBMsg.p_var[data_int].type = TYPE_FLOAT;
				USBMsg.p_var[data_int].addr = &FOC.Vbus_filt;
			break;
			
			case USB_IBUS:
				USBMsg.p_var[data_int].scale = 1.0f;
				USBMsg.p_var[data_int].type = TYPE_FLOAT;
				USBMsg.p_var[data_int].addr = &FOC.Ibus_filt;
			break;
			
			case USB_IA:
				USBMsg.p_var[data_int].scale = 1.0f;
				USBMsg.p_var[data_int].type = TYPE_FLOAT;
				USBMsg.p_var[data_int].addr = &FOC.Ia;
			break;
			
			case USB_IB:
				USBMsg.p_var[data_int].scale = 1.0f;
				USBMsg.p_var[data_int].type = TYPE_FLOAT;
				USBMsg.p_var[data_int].addr = &FOC.Ib;
			break;
			
			case USB_IC:
				USBMsg.p_var[data_int].scale = 1.0f;
				USBMsg.p_var[data_int].type = TYPE_FLOAT;
				USBMsg.p_var[data_int].addr = &FOC.Ic;
			break;
			
			case USB_ID:
				USBMsg.p_var[data_int].scale = 1.0f;
				USBMsg.p_var[data_int].type = TYPE_FLOAT;
				USBMsg.p_var[data_int].addr = &FOC.Id_filt;
			break;
			
			case USB_IQ:
				USBMsg.p_var[data_int].scale = 1.0f;
				USBMsg.p_var[data_int].type = TYPE_FLOAT;
				USBMsg.p_var[data_int].addr = &FOC.Iq_filt;
			break;
			
			case USB_SPEED1_FILT:
				USBMsg.p_var[data_int].scale = 1.0f;
				USBMsg.p_var[data_int].type = TYPE_FLOAT;
				USBMsg.p_var[data_int].addr = &OnBoard_Encoder.vel;
			break;
			
			case USB_POS1_FILT:
				USBMsg.p_var[data_int].scale = 1.0f;
				USBMsg.p_var[data_int].type = TYPE_FLOAT;
				USBMsg.p_var[data_int].addr = &OnBoard_Encoder.pos;
			break;
			
			case USB_SPEED2_FILT:
				USBMsg.p_var[data_int].scale = 1.0f;
				USBMsg.p_var[data_int].type = TYPE_FLOAT;
				USBMsg.p_var[data_int].addr = &External_Encoder.vel;
			break;
			
			case USB_POS2_FILT:
				USBMsg.p_var[data_int].scale = 1.0f;
				USBMsg.p_var[data_int].type = TYPE_FLOAT;
				USBMsg.p_var[data_int].addr = &External_Encoder.pos;
			break;
			
			case USB_TEMP:
				USBMsg.p_var[data_int].scale = 1.0f;
				USBMsg.p_var[data_int].type = TYPE_FLOAT;
				USBMsg.p_var[data_int].addr = &FOC.temp;
			break;
	
			case USB_RS:
				USBMsg.p_var[data_int].scale = 1000.0f;
				USBMsg.p_var[data_int].type = TYPE_FLOAT;
				USBMsg.p_var[data_int].addr = &MotorControl.motor_phase_resistance;
			break;

			case USB_LD:
				USBMsg.p_var[data_int].scale = 1000000.0f;
				USBMsg.p_var[data_int].type = TYPE_FLOAT;
				USBMsg.p_var[data_int].addr = &MotorControl.motor_d_inductance;
			break;

			case USB_LQ:
				USBMsg.p_var[data_int].scale = 1000000.0f;
				USBMsg.p_var[data_int].type = TYPE_FLOAT;
				USBMsg.p_var[data_int].addr = &MotorControl.motor_q_inductance;
			break;

			case USB_FLUX:
				USBMsg.p_var[data_int].scale = 1000.0f;
				USBMsg.p_var[data_int].type = TYPE_FLOAT;
				USBMsg.p_var[data_int].addr = &MotorControl.motor_flux;
			break;
			
			case USB_ERROR:
				USBMsg.p_var[data_int].scale = 1.0f;
				USBMsg.p_var[data_int].type = TYPE_FLOAT;
				USBMsg.p_var[data_int].addr = &MotorControl.ErrorNow_f;
			break;
			
			default:return USB_UNKNOWNED_PARAM;
		}
	}
	
	return USB_NO_ERROR;
}

/**
	* @brief  USB receive interrupt handler
	* @param  *data: received data buffer pointer
	* @param  length: received data length
 **/
void USB_RxIRQHandler(uint8_t *data, uint16_t length)
{
	USBRXError_TypeDef USBRXError;
	
	/*write data to ring buffer*/
	Cyclic_Write(data, length);
	/*read data of ring buffer and analyze*/
	USBRXError = USB_CyclicAnalyze();
	
	if(USBRXError == USB_SYNTAX_ERROR)
	{
		sprintf(USBMsg.tx_str, "Syntax error!\r\n");
		USBMsg.tx_en = 1;
		return;		
	}
		
	USBRXError = USB_ReceiveMessage_Update(USBMsg.rx_write_read_print, (USB_PARAM_ID)USBMsg.rx_param_id.combined, USBMsg.rx_data, USBMsg.int_or_float);
	
	switch(USBRXError)
	{
		case USB_WRITE_INVALID:
		{
			sprintf(USBMsg.tx_str, "Write invalid!\r\n");
			USBMsg.tx_en = 1;
			return;		
		}
		case USB_UNKNOWNED_PARAM:
		{
			sprintf(USBMsg.tx_str, "Unknowned parameter!\r\n");
			USBMsg.tx_en = 1;
			return;
		}
		case USB_DATA_INVALID:
		{
			sprintf(USBMsg.tx_str, "Data invalid!\r\n");
			USBMsg.tx_en = 1;
			return;
		}
		case USB_DATA_OUT_OF_RANGE:
		{
			sprintf(USBMsg.tx_str, "Data out of range!\r\n");
			USBMsg.tx_en = 1;
			return;
		}
		default:
		{
			if(USBMsg.rx_write_read_print == 2)
				USBMsg.print_en = 1;
			else
				USBMsg.tx_en = 1;
		}
		break;
	}
}

/**
	* @brief  Send USB response message
 **/
void USB_SendMessage(void)
{
	if(!USBMsg.tx_en)
		return;
	
	CDC_Transmit_FS((uint8_t *)USBMsg.tx_str, strlen(USBMsg.tx_str));
	memset(USBMsg.tx_str, 0, 80);
	USBMsg.tx_en = 0;
}

/**
	* @brief  Get scaled value for USB print profile
	* @param  *var: variable information pointer
	* @retval scaled variable value
 **/
static float print_get_value(VarInfo *var)
{
	float var_output;
	
	switch(var->type)
	{
		case TYPE_FLOAT:
			var_output = (*((float*)var->addr)) * var->scale;
		break;
		
		case TYPE_UINT8:
			var_output = ((float)(*((uint8_t*)var->addr))) * var->scale;
		break;
		
		case TYPE_UINT16:
			var_output = ((float)(*((uint16_t*)var->addr))) * var->scale;
		break;
		
		case TYPE_UINT32:
			var_output = ((float)(*((uint32_t*)var->addr))) * var->scale;
		break;
		
		case TYPE_INT32:
			var_output = ((float)(*((int32_t*)var->addr))) * var->scale;
		break;
		
	}
	
	return var_output;
}

/**
	* @brief  Send USB print profile data
 **/
void USB_PrintProfile(void)
{
	if(!USBMsg.print_en)
		return;
	
	USBMsg.en_channel_num = 5;

	for(int i = 0; i < USBMsg.en_channel_num; i++)
	{
		float var_temp = print_get_value(&USBMsg.p_var[i]);
		USBMsg.print_array[i] = FloatToIntBit(var_temp);
	}
	
	USBMsg.print_array[USBMsg.en_channel_num] = 0x7F800000;
	CDC_Transmit_FS((uint8_t *)&USBMsg.print_array, 4 * (USBMsg.en_channel_num + 1));
}