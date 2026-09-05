#ifndef PRODUCT_ENCODER_PROFILES_H
#define PRODUCT_ENCODER_PROFILES_H

#include <stdint.h>

typedef struct
{
	uint16_t profile_id;
	uint32_t counts_per_revolution;
	uint16_t speed_loop_divider;
	float speed_sample_period_s;
	uint16_t default_electrical_zero_q15;
	uint16_t default_mechanical_zero_q15;
	uint8_t default_calibration_flag;
	uint8_t default_reverse;
} EncoderProfile;

const EncoderProfile *EncoderProfile_GetActive(void);

#endif
