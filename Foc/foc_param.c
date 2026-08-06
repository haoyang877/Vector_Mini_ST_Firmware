#include "foc_param.h"

#include "common_inc.h"

InterfaceParam_TypeDef InterfaceParam;

extern MotorControl_TypeDef MotorControl;
extern Encoder_TypeDef OnBoard_Encoder,External_Encoder;
extern CANMsg_TypeDef CANMsg;

/**
	* @brief  Return parameters to default value
 **/
void Param_Return_Default(void)
{
	CANMsg.node_id 						= 0x00;
	
	MotorControl.A_Offset 				= 2048;
	MotorControl.B_Offset				= 2048;
	MotorControl.C_Offset 				= 2048;
	
	MotorControl.motor_pole_pairs 		= 21;
	
	/*skip calibration default motor parameters*/
	MotorControl.motor_phase_resistance 	= 1.0f;
	MotorControl.motor_d_inductance 		= 100e-6f;
	MotorControl.motor_q_inductance 		= 100e-6f;
	/*current loop PI derived from R/L, same as calibration*/
	MotorControl.id_Kp 					= MotorControl.motor_d_inductance * 200.0f;
	MotorControl.iq_Kp 					= MotorControl.motor_q_inductance * 200.0f;
	MotorControl.id_Ki 					= MotorControl.motor_phase_resistance * 200.0f;
	MotorControl.iq_Ki 					= MotorControl.motor_phase_resistance * 200.0f;
	/*voltage open-loop mode defaults*/
	MotorControl.ol_voltage 			= 1.0f;
	MotorControl.ol_elec_vel 			= 0.0f;
	MotorControl.ol_theta 				= 0.0f;
	
	OnBoard_Encoder.type		   		= TLE5012B;
	OnBoard_Encoder.enable				= ENCODER_DISABLE;
	OnBoard_Encoder.dir       			= 1;
	OnBoard_Encoder.offset    			= 0;
	OnBoard_Encoder.zero_count			= 0;
	memset(&OnBoard_Encoder.offset_lut, 0, 128 * 4);
	
	External_Encoder.type		   		= MT6701;
	External_Encoder.enable				= ENCODER_ENABLE;
	External_Encoder.dir       			= 1;
	External_Encoder.offset    			= 0;
	External_Encoder.zero_count			= 0;
	memset(&External_Encoder.offset_lut, 0, 128 * 4);
	
	MotorControl.calib_current 			= 4.0f;
	MotorControl.current_limit 			= 30.0f;
	MotorControl.speed_limit   			= 200.0f * _2PI;
	MotorControl.speedAcc      			= 50.0f * _2PI;
	MotorControl.speedDec      			= 50.0f * _2PI;
	MotorControl.speed_Kp 				= 0.05f;
	MotorControl.speed_Ki 				= 0.5f;
	MotorControl.posAcc      			= 10.0f * _2PI;
	MotorControl.posDec      			= 10.0f * _2PI;
	MotorControl.pos_maxspeed			= 5.0f * _2PI;
	MotorControl.pos_Kp 				= 0.05f;
	MotorControl.pos_Kd 				= 0.5f;
	//MotorControl.isUseAnticogging		= 0;
	CANMsg.can_hb_set 					= 500;
	
	MotorControl.ModeNow = Save_Param;
}

/**
	* @brief  Pack parameters in float format,ready to upload to flash
 **/
void Param_Upload(void)
{
	InterfaceParam.node_id				  = (float)CANMsg.node_id;
	
	InterfaceParam.currentoffset_a 		  = (float)MotorControl.A_Offset;
	InterfaceParam.currentoffset_b 		  = (float)MotorControl.B_Offset;
	InterfaceParam.currentoffset_c 		  = (float)MotorControl.C_Offset;
	
	InterfaceParam.motor_pole_pairs 	  = (float)MotorControl.motor_pole_pairs;
	InterfaceParam.motor_phase_resistance = MotorControl.motor_phase_resistance;
	InterfaceParam.motor_d_inductance 	  = MotorControl.motor_d_inductance;
	InterfaceParam.motor_q_inductance 	  = MotorControl.motor_q_inductance;
	InterfaceParam.motor_flux 			  = MotorControl.motor_flux;
	
	InterfaceParam.encoder_type[0] 		  = (float)OnBoard_Encoder.type;
	InterfaceParam.encoder_enable[0]	  = (float)OnBoard_Encoder.enable;
	InterfaceParam.encoder_dir[0] 		  = (float)OnBoard_Encoder.dir;
	InterfaceParam.encoder_offset[0] 	  = (float)OnBoard_Encoder.offset;
	InterfaceParam.encoder_zero_count[0]  = (float)OnBoard_Encoder.zero_count;
	for(int i = 0; i < 128; i++)
	InterfaceParam.offset_lut[i] 		  = (float)OnBoard_Encoder.offset_lut[i];

	InterfaceParam.encoder_type[1] 		  = (float)External_Encoder.type;
	InterfaceParam.encoder_enable[1]	  = (float)External_Encoder.enable;
	InterfaceParam.encoder_dir[1] 		  = (float)External_Encoder.dir;
	InterfaceParam.encoder_offset[1] 	  = (float)External_Encoder.offset;
	InterfaceParam.encoder_zero_count[1]  = (float)External_Encoder.zero_count;
	for(int i = 0; i < 128; i++)
	InterfaceParam.offset_lut[i + 128] 	  = (float)External_Encoder.offset_lut[i];
	
	InterfaceParam.calib_current 		  = MotorControl.calib_current;
	InterfaceParam.current_limit 		  = MotorControl.current_limit;	
	InterfaceParam.id_kp 				  = MotorControl.id_Kp;
	InterfaceParam.id_ki 				  = MotorControl.id_Ki;
	InterfaceParam.iq_kp 				  = MotorControl.iq_Kp;
	InterfaceParam.iq_ki 				  = MotorControl.iq_Ki;
	
	InterfaceParam.speed_limit 			  = MotorControl.speed_limit;
	InterfaceParam.speedAcc 			  = MotorControl.speedAcc;
	InterfaceParam.speedDec 			  = MotorControl.speedDec;
	InterfaceParam.speed_kp 			  = MotorControl.speed_Kp;
	InterfaceParam.speed_ki 			  = MotorControl.speed_Ki;
	
	InterfaceParam.posAcc				  = MotorControl.posAcc;
	InterfaceParam.posDec				  = MotorControl.posDec;
	InterfaceParam.pos_maxspeed			  = MotorControl.pos_maxspeed;
	InterfaceParam.pos_kp 				  = MotorControl.pos_Kp;
	InterfaceParam.pos_kd 				  = MotorControl.pos_Kd ;
	
	InterfaceParam.can_hb 				  = (float)CANMsg.can_hb_set;
}

/**
	* @brief  Unpack downloaded parameters in flash, transform to respective formats
 **/
void Param_Download(void)
{
	/*check if the flash is already written*/
	/*if magic word not detected, return parameters to default value*/
	if(InterfaceParam.magic_word != MAGIC_WORD)
	{
		Param_Return_Default();
	}
	else
	{
		CANMsg.node_id 						= (uint8_t)InterfaceParam.node_id;
		
		MotorControl.A_Offset 				= (uint16_t)InterfaceParam.currentoffset_a;
		MotorControl.B_Offset 				= (uint16_t)InterfaceParam.currentoffset_b;
		MotorControl.C_Offset 				= (uint16_t)InterfaceParam.currentoffset_c;
		
		MotorControl.motor_pole_pairs 		= (int32_t)InterfaceParam.motor_pole_pairs;
		MotorControl.motor_phase_resistance = InterfaceParam.motor_phase_resistance;
		MotorControl.motor_d_inductance 	= InterfaceParam.motor_d_inductance;
		MotorControl.motor_q_inductance 	= InterfaceParam.motor_q_inductance;
		MotorControl.motor_flux 			= InterfaceParam.motor_flux;
		
		OnBoard_Encoder.type		   		= (Encoder_Type)InterfaceParam.encoder_type[0];
		OnBoard_Encoder.enable				= (Encoder_Enable)InterfaceParam.encoder_enable[0];
		OnBoard_Encoder.dir       			= (int32_t)InterfaceParam.encoder_dir[0];
		OnBoard_Encoder.offset    			= (int32_t)InterfaceParam.encoder_offset[0];
		OnBoard_Encoder.zero_count			= (int32_t)InterfaceParam.encoder_zero_count[0];
		for(int i = 0; i < 128; i++)
		OnBoard_Encoder.offset_lut[i] 		= (int32_t)InterfaceParam.offset_lut[i];
		
		External_Encoder.type		   		= (Encoder_Type)InterfaceParam.encoder_type[1];
		External_Encoder.enable				= (Encoder_Enable)InterfaceParam.encoder_enable[1];
		External_Encoder.dir       			= (int32_t)InterfaceParam.encoder_dir[1];
		External_Encoder.offset    			= (int32_t)InterfaceParam.encoder_offset[1];
		External_Encoder.zero_count			= (int32_t)InterfaceParam.encoder_zero_count[1];
		for(int i = 0; i < 128; i++)
		External_Encoder.offset_lut[i] 		= (int32_t)InterfaceParam.offset_lut[i + 128];
		
		MotorControl.calib_current 			= InterfaceParam.calib_current;
		MotorControl.current_limit 			= InterfaceParam.current_limit;
		MotorControl.id_Kp 					= InterfaceParam.id_kp;
		MotorControl.id_Ki 					= InterfaceParam.id_ki;
		MotorControl.iq_Kp 					= InterfaceParam.iq_kp;
		MotorControl.iq_Ki 					= InterfaceParam.iq_ki;

		MotorControl.speed_limit   			= InterfaceParam.speed_limit;
		MotorControl.speedAcc      			= InterfaceParam.speedAcc;
		MotorControl.speedDec      			= InterfaceParam.speedDec;
		MotorControl.speed_Kp 				= InterfaceParam.speed_kp;
		MotorControl.speed_Ki 				= InterfaceParam.speed_ki;
		
		MotorControl.posAcc					= InterfaceParam.posAcc;
		MotorControl.posDec					= InterfaceParam.posDec;
		MotorControl.pos_maxspeed			= InterfaceParam.pos_maxspeed;
		MotorControl.pos_Kp 				= InterfaceParam.pos_kp;
		MotorControl.pos_Kd 				= InterfaceParam.pos_kd;
			
		CANMsg.can_hb_set 					= (int32_t)InterfaceParam.can_hb;
	}
}
