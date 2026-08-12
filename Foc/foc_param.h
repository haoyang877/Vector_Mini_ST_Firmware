#ifndef __FOC_PARAM_H__
#define __FOC_PARAM_H__

#include "main.h"
#include "encoder.h"

typedef struct
{
	float node_id;
	float currentoffset_a;
	float currentoffset_b;
	float currentoffset_c;
	float motor_pole_pairs;
	float motor_phase_resistance;
	float motor_d_inductance;
	float motor_q_inductance;
	float motor_flux;
	float encoder_type;
	float encoder_enable;
	float encoder_dir;
	float encoder_offset;
	float encoder_zero_count;
	float encoder_calib_flag;
	int16_t offset_lut[ENCODER_OFFSET_LUT_SIZE];
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
} InterfaceParam_TypeDef;

void Param_Return_Default(void);
void Param_Upload(InterfaceParam_TypeDef *param);
void Param_Download(const InterfaceParam_TypeDef *param);

#endif
