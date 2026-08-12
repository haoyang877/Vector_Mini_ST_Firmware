#include "foc_param.h"

#include "common_inc.h"
#include "foc_param_profile.h"

extern MotorControl_TypeDef MotorControl;
extern Encoder_TypeDef External_Encoder;
extern CANMsg_TypeDef CANMsg;

/**
	* @brief  Return parameters to default value
 **/
void Param_Return_Default(void)
{
	CANMsg.node_id						= PARAM_HW_CAN_NODE_ID;
	
	MotorControl.A_Offset 				= PARAM_HW_CURRENT_OFFSET_A_COUNTS;
	MotorControl.B_Offset				= PARAM_HW_CURRENT_OFFSET_B_COUNTS;
	MotorControl.C_Offset 				= PARAM_HW_CURRENT_OFFSET_C_COUNTS;
	
	MotorControl.motor_pole_pairs 		= PARAM_MOTOR_POLE_PAIRS;
	MotorControl.motor_phase_resistance 	= PARAM_MOTOR_PHASE_RESISTANCE_OHM;
	MotorControl.motor_d_inductance 		= PARAM_MOTOR_D_INDUCTANCE_H;
	MotorControl.motor_q_inductance 		= PARAM_MOTOR_Q_INDUCTANCE_H;
	MotorControl.motor_flux 				= PARAM_MOTOR_FLUX_WB;
	MotorControl.id_Kp 					= MotorControl.motor_d_inductance * PARAM_MOTOR_CURRENT_LOOP_BANDWIDTH_RAD_S;
	MotorControl.iq_Kp 					= MotorControl.motor_q_inductance * PARAM_MOTOR_CURRENT_LOOP_BANDWIDTH_RAD_S;
	MotorControl.id_Ki 					= MotorControl.motor_phase_resistance * PARAM_MOTOR_CURRENT_LOOP_BANDWIDTH_RAD_S;
	MotorControl.iq_Ki 					= MotorControl.motor_phase_resistance * PARAM_MOTOR_CURRENT_LOOP_BANDWIDTH_RAD_S;
	MotorControl.ol_voltage 			= PARAM_APP_OPEN_LOOP_VOLTAGE_V;
	MotorControl.ol_elec_vel 			= PARAM_APP_OPEN_LOOP_ELEC_VEL_RAD_S;
	MotorControl.ol_theta 				= PARAM_APP_OPEN_LOOP_THETA_RAD;
	
	External_Encoder.type 				= PARAM_HW_EXTERNAL_ENCODER_TYPE;
	External_Encoder.enable				= PARAM_HW_EXTERNAL_ENCODER_ENABLE;
	External_Encoder.dir 					= PARAM_HW_EXTERNAL_ENCODER_DIR;
	External_Encoder.offset 				= PARAM_APP_ENCODER_OFFSET_COUNTS;
	External_Encoder.zero_count			= PARAM_APP_ENCODER_ZERO_COUNT;
	External_Encoder.calib_flag			= PARAM_APP_ENCODER_CALIB_FLAG;
	memset(External_Encoder.offset_lut, 0, sizeof(External_Encoder.offset_lut));
	
	MotorControl.calib_current 			= PARAM_MOTOR_CALIB_CURRENT_A;
	MotorControl.current_limit 			= PARAM_MOTOR_CURRENT_LIMIT_A;
	MotorControl.vqRef 					= 0.0f;
	MotorControl.speed_limit 			= PARAM_MOTOR_SPEED_LIMIT_RPS * _2PI;
	MotorControl.speedAcc 				= PARAM_APP_SPEED_ACCEL_RPS2 * _2PI;
	MotorControl.speedDec 				= PARAM_APP_SPEED_DECEL_RPS2 * _2PI;
	MotorControl.speed_Kp 				= PARAM_APP_SPEED_KP;
	MotorControl.speed_Ki 				= PARAM_APP_SPEED_KI;
	MotorControl.posAcc 				= PARAM_APP_POSITION_ACCEL_RPS2 * _2PI;
	MotorControl.posDec 				= PARAM_APP_POSITION_DECEL_RPS2 * _2PI;
	MotorControl.pos_maxspeed			= PARAM_APP_POSITION_MAX_SPEED_RPS * _2PI;
	MotorControl.pos_Kp 				= PARAM_APP_POSITION_KP;
	MotorControl.pos_Kd 				= PARAM_APP_POSITION_KD;
	CANMsg.can_hb_set 				= PARAM_HW_CAN_HEARTBEAT_MS;
	
	MotorControl.ModeNow = Save_Param;
}

/**
	* @brief  Pack runtime parameters for flash storage
 **/
void Param_Upload(InterfaceParam_TypeDef *param)
{
	memset(param, 0, sizeof(*param));
	param->node_id							= (float)CANMsg.node_id;
	param->currentoffset_a 					= (float)MotorControl.A_Offset;
	param->currentoffset_b 					= (float)MotorControl.B_Offset;
	param->currentoffset_c 					= (float)MotorControl.C_Offset;
	param->motor_pole_pairs 				= (float)MotorControl.motor_pole_pairs;
	param->motor_phase_resistance 			= MotorControl.motor_phase_resistance;
	param->motor_d_inductance 				= MotorControl.motor_d_inductance;
	param->motor_q_inductance 				= MotorControl.motor_q_inductance;
	param->motor_flux 						= MotorControl.motor_flux;
	param->encoder_type 					= (float)External_Encoder.type;
	param->encoder_enable					= (float)External_Encoder.enable;
	param->encoder_dir 						= (float)External_Encoder.dir;
	param->encoder_offset 					= (float)External_Encoder.offset;
	param->encoder_zero_count 				= (float)External_Encoder.zero_count;
	param->encoder_calib_flag 				= (float)External_Encoder.calib_flag;
	for (uint32_t i = 0U; i < ENCODER_OFFSET_LUT_SIZE; ++i)
		param->offset_lut[i] = External_Encoder.offset_lut[i];
	param->calib_current 					= MotorControl.calib_current;
	param->current_limit 					= MotorControl.current_limit;
	param->id_kp 							= MotorControl.id_Kp;
	param->id_ki 							= MotorControl.id_Ki;
	param->iq_kp 							= MotorControl.iq_Kp;
	param->iq_ki 							= MotorControl.iq_Ki;
	param->speed_limit 					= MotorControl.speed_limit;
	param->speedAcc 						= MotorControl.speedAcc;
	param->speedDec 						= MotorControl.speedDec;
	param->speed_kp 						= MotorControl.speed_Kp;
	param->speed_ki 						= MotorControl.speed_Ki;
	param->posAcc							= MotorControl.posAcc;
	param->posDec							= MotorControl.posDec;
	param->pos_maxspeed					= MotorControl.pos_maxspeed;
	param->pos_kp 							= MotorControl.pos_Kp;
	param->pos_kd 							= MotorControl.pos_Kd;
	param->can_hb 							= (float)CANMsg.can_hb_set;
}

/**
	* @brief  Unpack parameters read from flash
 **/
void Param_Download(const InterfaceParam_TypeDef *param)
{
	if (param->magic_word != MAGIC_WORD)
	{
		Param_Return_Default();
		return;
	}

	CANMsg.node_id 						= (uint8_t)param->node_id;
	MotorControl.A_Offset 				= (uint16_t)param->currentoffset_a;
	MotorControl.B_Offset 				= (uint16_t)param->currentoffset_b;
	MotorControl.C_Offset 				= (uint16_t)param->currentoffset_c;
	MotorControl.motor_pole_pairs 		= (int32_t)param->motor_pole_pairs;
	MotorControl.motor_phase_resistance = param->motor_phase_resistance;
	MotorControl.motor_d_inductance 		= param->motor_d_inductance;
	MotorControl.motor_q_inductance 		= param->motor_q_inductance;
	MotorControl.motor_flux 				= param->motor_flux;
	External_Encoder.type 				= (Encoder_Type)param->encoder_type;
	External_Encoder.enable				= (Encoder_Enable)param->encoder_enable;
	External_Encoder.dir 					= (int32_t)param->encoder_dir;
	External_Encoder.offset 				= (int32_t)param->encoder_offset;
	External_Encoder.zero_count			= (int32_t)param->encoder_zero_count;
	External_Encoder.calib_flag			= (uint8_t)param->encoder_calib_flag;
	for (uint32_t i = 0U; i < ENCODER_OFFSET_LUT_SIZE; ++i)
		External_Encoder.offset_lut[i] = param->offset_lut[i];
	MotorControl.calib_current 			= param->calib_current;
	MotorControl.current_limit 			= param->current_limit;
	MotorControl.id_Kp 					= param->id_kp;
	MotorControl.id_Ki 					= param->id_ki;
	MotorControl.iq_Kp 					= param->iq_kp;
	MotorControl.iq_Ki 					= param->iq_ki;
	MotorControl.speed_limit 			= param->speed_limit;
	MotorControl.speedAcc 				= param->speedAcc;
	MotorControl.speedDec 				= param->speedDec;
	MotorControl.speed_Kp 				= param->speed_kp;
	MotorControl.speed_Ki 				= param->speed_ki;
	MotorControl.posAcc				= param->posAcc;
	MotorControl.posDec				= param->posDec;
	MotorControl.pos_maxspeed			= param->pos_maxspeed;
	MotorControl.pos_Kp 				= param->pos_kp;
	MotorControl.pos_Kd 				= param->pos_kd;
	CANMsg.can_hb_set 				= (int32_t)param->can_hb;
}
