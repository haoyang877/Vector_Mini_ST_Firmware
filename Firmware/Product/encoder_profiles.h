#ifndef PRODUCT_ENCODER_PROFILES_H
#define PRODUCT_ENCODER_PROFILES_H

#include <stdint.h>

#define ENCODER_PROFILE_TLE5012B_16BIT 1U

#ifndef ACTIVE_ENCODER_PROFILE
#define ACTIVE_ENCODER_PROFILE ENCODER_PROFILE_TLE5012B_16BIT
#endif

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
