#ifndef __FOC_PARAM_H__
#define __FOC_PARAM_H__

#include "main.h"
#include "encoder.h"

#define PARAM_SCHEMA_VERSION 6U
#define PARAM_SCHEMA_VERSION_LEGACY_IMPEDANCE 5U
#define PARAM_SCHEMA_VERSION_LEGACY_CASCADE 4U

/* Runtime/persisted position-impedance gain limits. */
#define POSITION_IMPEDANCE_KP_MAX_A_PER_RAD       50.0f
#define POSITION_IMPEDANCE_KD_MAX_A_PER_RAD_S     10.0f
#define POSITION_IMPEDANCE_KI_MAX_A_PER_RAD_S     10.0f
#define POSITION_IMPEDANCE_MAX_SPEED_RPS          0.125f

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
	uint16_t encoder_electrical_zero_q15;
	uint16_t encoder_mechanical_zero_q15;
	uint8_t encoder_calib_flag;
	uint8_t encoder_reverse;
	uint8_t encoder_reserved[2];
	int16_t encoder_linearization_lut_q15[ENCODER_OFFSET_LUT_SIZE];
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
	uint32_t schema_version;
	uint32_t magic_word;
	/* Appended in schema v5 so the v4 schema/magic offsets remain readable. */
	float pos_ki;
	/* Appended in schema v6; identifies the current-sense scaling in Flash. */
	uint32_t current_sense_shunt_milliohm;
} InterfaceParam_TypeDef;

void Param_Return_Default(void);
void Param_Upload(InterfaceParam_TypeDef *param);
void Param_Download(const InterfaceParam_TypeDef *param);

#endif
