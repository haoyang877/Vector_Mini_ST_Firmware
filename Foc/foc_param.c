#include "foc_param.h"

#include "common_inc.h"
#include "foc_param_profile.h"

InterfaceParam_TypeDef InterfaceParam;

extern MotorControl_TypeDef MotorControl;
extern Encoder_TypeDef OnBoard_Encoder,External_Encoder;
extern CANMsg_TypeDef CANMsg;

/**
	* @brief  Return parameters to default value
 **/
void Param_Return_Default(void)
{
	CANMsg.node_id 						= PARAM_HW_CAN_NODE_ID;
	
	MotorControl.A_Offset 				= PARAM_HW_CURRENT_OFFSET_A_COUNTS;
	MotorControl.B_Offset				= PARAM_HW_CURRENT_OFFSET_B_COUNTS;
	MotorControl.C_Offset 				= PARAM_HW_CURRENT_OFFSET_C_COUNTS;
	
	MotorControl.motor_pole_pairs 		= PARAM_MOTOR_POLE_PAIRS;
	MotorControl.motor_phase_resistance 	= PARAM_MOTOR_PHASE_RESISTANCE_OHM;
	MotorControl.motor_d_inductance 		= PARAM_MOTOR_D_INDUCTANCE_H;
	MotorControl.motor_q_inductance 		= PARAM_MOTOR_Q_INDUCTANCE_H;
	MotorControl.motor_flux 				= PARAM_MOTOR_FLUX_WB;
	/*current loop PI derived from R/L, same as calibration*/
	MotorControl.id_Kp 					= MotorControl.motor_d_inductance * PARAM_MOTOR_CURRENT_LOOP_BANDWIDTH_RAD_S;
	MotorControl.iq_Kp 					= MotorControl.motor_q_inductance * PARAM_MOTOR_CURRENT_LOOP_BANDWIDTH_RAD_S;
	MotorControl.id_Ki 					= MotorControl.motor_phase_resistance * PARAM_MOTOR_CURRENT_LOOP_BANDWIDTH_RAD_S;
	MotorControl.iq_Ki 					= MotorControl.motor_phase_resistance * PARAM_MOTOR_CURRENT_LOOP_BANDWIDTH_RAD_S;
	/*voltage open-loop mode defaults*/
	MotorControl.ol_voltage 			= PARAM_APP_OPEN_LOOP_VOLTAGE_V;
	MotorControl.ol_elec_vel 			= PARAM_APP_OPEN_LOOP_ELEC_VEL_RAD_S;
	MotorControl.ol_theta 				= PARAM_APP_OPEN_LOOP_THETA_RAD;
	
	OnBoard_Encoder.type		   		= PARAM_HW_ONBOARD_ENCODER_TYPE;
	OnBoard_Encoder.enable				= PARAM_HW_ONBOARD_ENCODER_ENABLE;
	OnBoard_Encoder.dir       			= PARAM_HW_ONBOARD_ENCODER_DIR;
	OnBoard_Encoder.offset    			= PARAM_APP_ENCODER_OFFSET_COUNTS;
	OnBoard_Encoder.zero_count			= PARAM_APP_ENCODER_ZERO_COUNT;
	OnBoard_Encoder.calib_flag			= PARAM_APP_ENCODER_CALIB_FLAG;
	memset(&OnBoard_Encoder.offset_lut, 0, 128 * 4);
	
	External_Encoder.type		   		= PARAM_HW_EXTERNAL_ENCODER_TYPE;
	External_Encoder.enable				= PARAM_HW_EXTERNAL_ENCODER_ENABLE;
	External_Encoder.dir       			= PARAM_HW_EXTERNAL_ENCODER_DIR;
	External_Encoder.offset    			= PARAM_APP_ENCODER_OFFSET_COUNTS;
	External_Encoder.zero_count			= PARAM_APP_ENCODER_ZERO_COUNT;
	External_Encoder.calib_flag			= PARAM_APP_ENCODER_CALIB_FLAG;
	memset(&External_Encoder.offset_lut, 0, 128 * 4);
	
	MotorControl.calib_current 			= PARAM_MOTOR_CALIB_CURRENT_A;
	MotorControl.current_limit 			= PARAM_MOTOR_CURRENT_LIMIT_A;
	MotorControl.vqRef 					= 0.0f;
	MotorControl.speed_limit   			= PARAM_MOTOR_SPEED_LIMIT_RPS * _2PI;
	MotorControl.speedAcc      			= PARAM_APP_SPEED_ACCEL_RPS2 * _2PI;
	MotorControl.speedDec      			= PARAM_APP_SPEED_DECEL_RPS2 * _2PI;
	MotorControl.speed_Kp 				= PARAM_APP_SPEED_KP;
	MotorControl.speed_Ki 				= PARAM_APP_SPEED_KI;
	MotorControl.posAcc      			= PARAM_APP_POSITION_ACCEL_RPS2 * _2PI;
	MotorControl.posDec      			= PARAM_APP_POSITION_DECEL_RPS2 * _2PI;
	MotorControl.pos_maxspeed			= PARAM_APP_POSITION_MAX_SPEED_RPS * _2PI;
	MotorControl.pos_Kp 				= PARAM_APP_POSITION_KP;
	MotorControl.pos_Kd 				= PARAM_APP_POSITION_KD;
	//MotorControl.isUseAnticogging		= 0;
	CANMsg.can_hb_set 					= PARAM_HW_CAN_HEARTBEAT_MS;
	
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
	InterfaceParam.encoder_calib_flag[0]  = (float)OnBoard_Encoder.calib_flag;
	for(int i = 0; i < 128; i++)
	InterfaceParam.offset_lut[i] 		  = (float)OnBoard_Encoder.offset_lut[i];

	InterfaceParam.encoder_type[1] 		  = (float)External_Encoder.type;
	InterfaceParam.encoder_enable[1]	  = (float)External_Encoder.enable;
	InterfaceParam.encoder_dir[1] 		  = (float)External_Encoder.dir;
	InterfaceParam.encoder_offset[1] 	  = (float)External_Encoder.offset;
	InterfaceParam.encoder_zero_count[1]  = (float)External_Encoder.zero_count;
	InterfaceParam.encoder_calib_flag[1]  = (float)External_Encoder.calib_flag;
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
		OnBoard_Encoder.calib_flag			= (uint8_t)InterfaceParam.encoder_calib_flag[0];
		for(int i = 0; i < 128; i++)
		OnBoard_Encoder.offset_lut[i] 		= (int32_t)InterfaceParam.offset_lut[i];
		
		External_Encoder.type		   		= (Encoder_Type)InterfaceParam.encoder_type[1];
		External_Encoder.enable				= (Encoder_Enable)InterfaceParam.encoder_enable[1];
		External_Encoder.dir       			= (int32_t)InterfaceParam.encoder_dir[1];
		External_Encoder.offset    			= (int32_t)InterfaceParam.encoder_offset[1];
		External_Encoder.zero_count			= (int32_t)InterfaceParam.encoder_zero_count[1];
		External_Encoder.calib_flag			= (uint8_t)InterfaceParam.encoder_calib_flag[1];
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
