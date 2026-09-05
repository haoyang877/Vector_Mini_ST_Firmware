#ifndef PRODUCT_MECHANICAL_LOAD_PROFILES_H
#define PRODUCT_MECHANICAL_LOAD_PROFILES_H

#include <stdbool.h>
#include <stdint.h>

#include "control_tuning_profile.h"

#define MECHANICAL_LOAD_PROFILE_NO_DAMPER             0U
#define MECHANICAL_LOAD_PROFILE_DAMPING_RING_1P5NM    1U

typedef struct
{
	uint16_t profile_id;
	bool damping_ring_present;
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
} MechanicalLoadProfile;

const MechanicalLoadProfile *MechanicalLoadProfile_GetActive(void);
const MechanicalLoadProfile *MechanicalLoadProfile_GetById(uint16_t profile_id);

#endif
