#include "control_tuning_profile.h"

static const ControlTuningProfile ActiveControlTuningProfile =
{
	.profile_id = CONTROL_TUNING_PROFILE_HT8115_4_VECTOR_MINI_ST,
	.sensorless_startup =
	{
		.align_current_ramp_time_s = 0.50f,
		.align_hold_time_s = 0.30f,
		.align_current_a = 2.0f,
		.startup_iq_initial_a = 0.15f,
		.startup_iq_a = 0.50f,
		.startup_iq_ramp_time_s = 2.00f,
		.startup_id_a = 0.5f,
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
	.sensorless_observer_max_electrical_velocity_rad_s = 5000.0f,
	.sensorless_speed_feedback_lpf_alpha = 0.1042f,
	.flux_observer_gamma = 800000.0f,
	/*
	 * The voltage-model observer needs the effective stator resistance seen at
	 * the PWM fundamental.  On this inverter/motor it is lower than the DC
	 * phase resistance used to tune the current PI loop.  Keep the two models
	 * independent so motor identification and current-loop gains remain
	 * physically meaningful.
	 */
	.flux_observer_resistance_scale = 0.4199475f, /* 0.800 / 1.905 */
	.flux_observer_max_correction_step_rad = 0.2f,
	.flux_observer_minimum_flux_weber = 1.0e-6f,
	.flux_observer_velocity_lpf_alpha = 0.1f,
	.flux_observer_angle_wrap_threshold_rad = 4.0f,
	.encoder_linearization_align_time_s = 1.5f,
	.encoder_linearization_ramp_time_s = 3.0f,
	.encoder_linearization_speed_electrical_rad_s =
		2.0f * 3.14159265358979323846f * 20.0f,
	.encoder_linearization_timeout_factor = 1.5f,
	.encoder_linearization_unlock_timeout_s = 1.0f,
	.encoder_calibration_speed_mechanical_rad_s = 20.0f,
	.encoder_calibration_speed_error_ratio = 0.20f,
	.encoder_calibration_speed_stable_time_s = 0.50f,
	.encoder_calibration_speed_stable_timeout_s = 5.00f,
	.encoder_calibration_align_sample_time_s = 0.10f,
	.encoder_calibration_mechanical_turns = 10U,
	.encoder_calibration_verify_mechanical_turns = 1U,
	.encoder_calibration_min_samples_per_bin = 16U,
	.encoder_calibration_lut_build_bins_per_cycle = 8U,
	.encoder_calibration_find_origin_timeout_s = 1.50f,
	.encoder_calibration_sample_timeout_s = 12.00f,
	.encoder_calibration_verify_timeout_s = 4.00f,
	.encoder_calibration_max_rms_residual_q15 = 256U,
	.encoder_calibration_max_peak_residual_q15 = 1024U,
	.encoder_calibration_startup_timeout_s = 8.0f,
	.encoder_calibration_stop_speed_margin = 1.05f,
	.encoder_calibration_stop_deceleration_time_s = 1.00f,
	.encoder_calibration_stop_deceleration_timeout_s = 3.00f,
	.encoder_calibration_stop_speed_tolerance_ratio = 0.10f,
	.encoder_calibration_stop_current_ramp_time_s = 0.50f,
	.encoder_electrical_zero_current_ramp_time_s = 0.50f,
	.encoder_electrical_zero_hold_time_s = 1.00f,
	.encoder_direction_align_time_s = 1.00f,
	.encoder_direction_speed_electrical_rad_s = 20.0f
};

const ControlTuningProfile *ControlTuningProfile_GetActive(void)
{
	return &ActiveControlTuningProfile;
}
