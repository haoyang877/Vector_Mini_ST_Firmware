#include "encoder_profiles.h"

#include "control_loop_config.h"
#include "vector_mini_st_profile.h"

static const EncoderProfile ActiveEncoderProfile =
{
	.profile_id = ACTIVE_ENCODER_PROFILE,
	.counts_per_revolution = 65536UL,
	.speed_loop_divider = SPEED_LOOP_DIVIDER,
	.speed_sample_period_s = SPEED_LOOP_PERIOD_S,
	.default_electrical_zero_q15 = PARAM_APP_ENCODER_ELECTRICAL_ZERO_Q15,
	.default_mechanical_zero_q15 = PARAM_APP_ENCODER_MECHANICAL_ZERO_Q15,
	.default_calibration_flag = PARAM_APP_ENCODER_CALIB_FLAG,
	.default_reverse = PARAM_APP_ENCODER_REVERSE
};

const EncoderProfile *EncoderProfile_GetActive(void)
{
#if ACTIVE_ENCODER_PROFILE != ENCODER_PROFILE_TLE5012B_16BIT
#error "Unsupported ACTIVE_ENCODER_PROFILE"
#endif
	return &ActiveEncoderProfile;
}
