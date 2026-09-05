#include "mechanical_load_profiles.h"

#include "vector_mini_st_profile.h"

static const MechanicalLoadProfile NoDamperProfile =
{
	.profile_id = MECHANICAL_LOAD_PROFILE_NO_DAMPER,
	.damping_ring_present = false,
	.default_speed_limit_rps = 372.0f / 60.0f,
	.default_position_max_speed_rps = 0.125f,
	.encoder_calibration_startup =
	{
		.align_current_ramp_time_s = 0.50f,
		.align_hold_time_s = 0.30f,
		.align_current_a = 2.0f,
		.startup_iq_initial_a = 0.15f,
		.startup_iq_a = 0.50f,
		.startup_iq_ramp_time_s = 2.00f,
		.startup_id_a = 0.50f,
		.minimum_current_limit_a = 0.0f,
		.minimum_electrical_velocity_rad_s = 250.0f,
		.target_electrical_velocity_rad_s = 420.0f,
		.startup_ramp_time_s = 2.00f,
		.speed_lock_time_s = 0.20f,
		.speed_lock_filter_alpha = 1.0f,
		.observer_lock_ratio = 0.25f,
		.angle_handoff_time_s = 0.10f,
		.lock_timeout_s = 3.00f,
		.id_ramp_down_time_s = 0.50f,
		.observer_loss_time_s = 0.20f
	},
	.encoder_calibration_speed_stable_timeout_s = 5.00f,
	.encoder_calibration_mechanical_turns = 10U,
	.encoder_calibration_startup_timeout_s = 8.0f,
	.encoder_calibration_stop_current_ramp_time_s = 0.50f,
	.encoder_electrical_zero_min_align_current_a = 0.0f,
	.position_friction_feedforward_enabled = false,
	.friction_positive_current_a = 0.0f,
	.friction_negative_current_a = 0.0f,
	.breakaway_positive_current_a = 0.0f,
	.breakaway_negative_current_a = 0.0f,
	.friction_current_slew_rate_a_per_s = 15.0f,
	.friction_position_enter_rad = 0.001f,
	.friction_position_exit_rad = 0.003f,
	.friction_reference_speed_rad_s = 0.03f,
	.friction_stop_speed_rad_s = 0.02f,
	.friction_move_speed_rad_s = 0.05f,
	.friction_stuck_time_s = 0.05f,
	.friction_landing_position_rad = 0.006f,
	.friction_landing_speed_rad_s = 0.10f,
	.friction_recovery_delay_s = 0.20f,
	.friction_recovery_pulse_time_s = 0.05f,
	.friction_recovery_cooldown_s = 0.20f,
	.friction_identification_speed_points_rad_s =
		{0.62831853f, 1.25663706f, 2.51327412f, 3.14159265f},
	.friction_identification_speed_point_count = 4U,
	.friction_identification_stable_time_s = 0.75f,
	.friction_identification_track_timeout_s = 8.0f,
	.friction_identification_sample_timeout_s = 15.0f,
	.friction_identification_stop_hold_time_s = 0.30f,
	.friction_identification_stop_timeout_s = 5.0f,
	.friction_identification_speed_tolerance_ratio = 0.05f,
	.friction_identification_minimum_speed_tolerance_rad_s = 0.08f,
	.friction_identification_stop_speed_rad_s = 0.12f,
	.friction_identification_sample_turns = 1.0f,
	.friction_identification_minimum_sample_time_s = 0.50f,
	.friction_identification_current_ratio_max = 0.90f,
	.friction_identification_saturation_time_s = 0.25f,
	.friction_identification_rmse_floor_a = 0.05f,
	.friction_identification_rmse_ratio_max = 0.25f
};

static const MechanicalLoadProfile DampingRing1p5NmProfile =
{
	.profile_id = MECHANICAL_LOAD_PROFILE_DAMPING_RING_1P5NM,
	.damping_ring_present = true,
	.default_speed_limit_rps = 0.50f,
	.default_position_max_speed_rps = 0.50f,
	.encoder_calibration_startup =
	{
		.align_current_ramp_time_s = 0.80f,
		.align_hold_time_s = 0.50f,
		.align_current_a = 3.50f,
		.startup_iq_initial_a = 3.50f,
		.startup_iq_a = 4.50f,
		.startup_iq_ramp_time_s = 1.00f,
		.startup_id_a = 0.50f,
		.minimum_current_limit_a = 5.50f,
		.minimum_electrical_velocity_rad_s = 250.0f,
		.target_electrical_velocity_rad_s = 420.0f,
		.startup_ramp_time_s = 2.00f,
		.speed_lock_time_s = 0.20f,
		.speed_lock_filter_alpha = 0.01f,
		.observer_lock_ratio = 0.25f,
		.angle_handoff_time_s = 0.10f,
		.lock_timeout_s = 5.00f,
		.id_ramp_down_time_s = 0.50f,
		.observer_loss_time_s = 0.20f
	},
	.encoder_calibration_speed_stable_timeout_s = 10.00f,
	.encoder_calibration_mechanical_turns = 5U,
	.encoder_calibration_startup_timeout_s = 12.0f,
	.encoder_calibration_stop_current_ramp_time_s = 0.10f,
	.encoder_electrical_zero_min_align_current_a = 4.50f,
	.position_friction_feedforward_enabled = true,
	.friction_positive_current_a = 1.55f,
	.friction_negative_current_a = 1.50f,
	.breakaway_positive_current_a = 1.85f,
	.breakaway_negative_current_a = 1.80f,
	.friction_current_slew_rate_a_per_s = 15.0f,
	.friction_position_enter_rad = 0.001f,
	.friction_position_exit_rad = 0.003f,
	.friction_reference_speed_rad_s = 0.03f,
	.friction_stop_speed_rad_s = 0.02f,
	.friction_move_speed_rad_s = 0.05f,
	.friction_stuck_time_s = 0.05f,
	.friction_landing_position_rad = 0.006f,
	.friction_landing_speed_rad_s = 0.10f,
	.friction_recovery_delay_s = 0.20f,
	.friction_recovery_pulse_time_s = 0.05f,
	.friction_recovery_cooldown_s = 0.20f,
	.friction_identification_speed_points_rad_s =
		{0.62831853f, 1.25663706f, 2.51327412f, 3.14159265f},
	.friction_identification_speed_point_count = 4U,
	.friction_identification_stable_time_s = 0.75f,
	.friction_identification_track_timeout_s = 8.0f,
	.friction_identification_sample_timeout_s = 15.0f,
	.friction_identification_stop_hold_time_s = 0.30f,
	.friction_identification_stop_timeout_s = 5.0f,
	.friction_identification_speed_tolerance_ratio = 0.05f,
	.friction_identification_minimum_speed_tolerance_rad_s = 0.08f,
	.friction_identification_stop_speed_rad_s = 0.12f,
	.friction_identification_sample_turns = 1.0f,
	.friction_identification_minimum_sample_time_s = 0.50f,
	.friction_identification_current_ratio_max = 0.90f,
	.friction_identification_saturation_time_s = 0.25f,
	.friction_identification_rmse_floor_a = 0.05f,
	.friction_identification_rmse_ratio_max = 0.25f
};

static const MechanicalLoadProfile *const SupportedProfiles[] =
{
	&NoDamperProfile,
	&DampingRing1p5NmProfile
};

const MechanicalLoadProfile *MechanicalLoadProfile_GetActive(void)
{
#if ACTIVE_MECHANICAL_LOAD_PROFILE > MECHANICAL_LOAD_PROFILE_DAMPING_RING_1P5NM
#error "Unsupported ACTIVE_MECHANICAL_LOAD_PROFILE"
#endif
	return SupportedProfiles[ACTIVE_MECHANICAL_LOAD_PROFILE];
}

const MechanicalLoadProfile *MechanicalLoadProfile_GetById(uint16_t profile_id)
{
	if (profile_id > MECHANICAL_LOAD_PROFILE_DAMPING_RING_1P5NM)
		return 0;
	return SupportedProfiles[profile_id];
}
