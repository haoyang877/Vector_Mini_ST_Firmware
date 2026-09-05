#ifndef PRODUCT_MECHANICAL_LOAD_PROFILES_H
#define PRODUCT_MECHANICAL_LOAD_PROFILES_H

#include <stdbool.h>
#include <stdint.h>

#include "control_tuning_profile.h"

#define MECHANICAL_LOAD_PROFILE_NO_DAMPER             0U
#define MECHANICAL_LOAD_PROFILE_DAMPING_RING_1P5NM    1U
#define FRICTION_IDENTIFICATION_SPEED_POINT_COUNT      4U

typedef struct
{
	uint16_t profile_id;
	bool damping_ring_present;
	float default_speed_limit_rps;
	float default_position_max_speed_rps;
	SensorlessStartupTuning encoder_calibration_startup;
	float encoder_calibration_speed_stable_timeout_s;
	uint32_t encoder_calibration_mechanical_turns;
	float encoder_calibration_startup_timeout_s;
	float encoder_calibration_stop_current_ramp_time_s;
	float encoder_electrical_zero_min_align_current_a;
	bool position_friction_feedforward_enabled;
	float friction_positive_current_a;
	float friction_negative_current_a;
	float breakaway_positive_current_a;
	float breakaway_negative_current_a;
	float friction_current_slew_rate_a_per_s;
	float friction_position_enter_rad;
	float friction_position_exit_rad;
	float friction_reference_speed_rad_s;
	float friction_stop_speed_rad_s;
	float friction_move_speed_rad_s;
	float friction_stuck_time_s;
	float friction_landing_position_rad;
	float friction_landing_speed_rad_s;
	float friction_recovery_delay_s;
	float friction_recovery_pulse_time_s;
	float friction_recovery_cooldown_s;
	float friction_identification_speed_points_rad_s[FRICTION_IDENTIFICATION_SPEED_POINT_COUNT];
	uint32_t friction_identification_speed_point_count;
	float friction_identification_stable_time_s;
	float friction_identification_track_timeout_s;
	float friction_identification_sample_timeout_s;
	float friction_identification_stop_hold_time_s;
	float friction_identification_stop_timeout_s;
	float friction_identification_speed_tolerance_ratio;
	float friction_identification_minimum_speed_tolerance_rad_s;
	float friction_identification_stop_speed_rad_s;
	float friction_identification_sample_turns;
	float friction_identification_minimum_sample_time_s;
	float friction_identification_current_ratio_max;
	float friction_identification_saturation_time_s;
	float friction_identification_rmse_floor_a;
	float friction_identification_rmse_ratio_max;
	float cogging_identification_speed_rad_s;
	uint32_t cogging_identification_turns;
	float cogging_identification_stable_time_s;
	float cogging_identification_stage_timeout_s;
	float cogging_identification_speed_tolerance_ratio;
	uint16_t cogging_identification_min_samples_per_bin;
	float cogging_identification_max_current_a;
} MechanicalLoadProfile;

const MechanicalLoadProfile *MechanicalLoadProfile_GetActive(void);
const MechanicalLoadProfile *MechanicalLoadProfile_GetById(uint16_t profile_id);

#endif
