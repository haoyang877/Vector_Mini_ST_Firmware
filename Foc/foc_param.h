#ifndef __FOC_PARAM_H__
#define __FOC_PARAM_H__

#include "main.h"

typedef struct
{
	float node_id;
	
	float currentoffset_a,currentoffset_b,currentoffset_c;
	
	float motor_pole_pairs;
	float motor_phase_resistance;
	float motor_d_inductance;
	float motor_q_inductance;
	float motor_flux;
	
	float encoder_type[2];
	float encoder_enable[2];
	float encoder_dir[2];
	float encoder_offset[2];
	float encoder_zero_count[2];
	float encoder_calib_flag[2];
	float offset_lut[256];
	
	float id_kp;
	float id_ki;
	float iq_kp;
	float iq_ki;
	
	float speedAcc;
	float speedDec;
	float speed_kp;
	float speed_ki;
	
	float posAcc;
	float posDec;
	float pos_maxspeed;
	float pos_kp;
	float pos_kd;
	
	float calib_current;
	float current_limit;
	float speed_limit;

	float can_hb;
	
	uint32_t magic_word;
}InterfaceParam_TypeDef;

void Param_Return_Default(void);
void Param_Upload(void);
void Param_Download(void);

#endif
