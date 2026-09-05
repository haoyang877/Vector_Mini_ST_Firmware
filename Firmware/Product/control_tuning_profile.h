#ifndef PRODUCT_CONTROL_TUNING_PROFILE_H
#define PRODUCT_CONTROL_TUNING_PROFILE_H

#include <stdint.h>

typedef struct
{
	float align_current_ramp_time_s;
	float align_hold_time_s;
	float align_current_a;
	float startup_iq_initial_a;
	float startup_iq_a;
	float startup_iq_ramp_time_s;
	float startup_id_a;
	float minimum_current_limit_a;
	float minimum_electrical_velocity_rad_s;
	float target_electrical_velocity_rad_s;
	float startup_ramp_time_s;
	float speed_lock_time_s;
	float speed_lock_filter_alpha;
	float observer_lock_ratio;
	float angle_handoff_time_s;
	float lock_timeout_s;
	float id_ramp_down_time_s;
	float observer_loss_time_s;
} SensorlessStartupTuning;

typedef struct
{
	SensorlessStartupTuning sensorless_startup;
	float sensorless_observer_max_electrical_velocity_rad_s;
	float sensorless_speed_feedback_lpf_alpha;
	float flux_observer_gamma;
	float flux_observer_max_correction_step_rad;
	float flux_observer_minimum_flux_weber;
	float flux_observer_velocity_lpf_alpha;
	float flux_observer_angle_wrap_threshold_rad;
	float encoder_linearization_align_time_s;
	float encoder_linearization_ramp_time_s;
	float encoder_linearization_speed_electrical_rad_s;
	float encoder_linearization_timeout_factor;
	float encoder_linearization_unlock_timeout_s;
	float encoder_calibration_speed_mechanical_rad_s;
	float encoder_calibration_speed_error_ratio;
	float encoder_calibration_speed_stable_time_s;
	float encoder_calibration_speed_stable_timeout_s;
	float encoder_calibration_align_sample_time_s;
	uint32_t encoder_calibration_mechanical_turns;
	uint32_t encoder_calibration_verify_mechanical_turns;
	uint16_t encoder_calibration_min_samples_per_bin;
	uint16_t encoder_calibration_lut_build_bins_per_cycle;
	float encoder_calibration_find_origin_timeout_s;
	float encoder_calibration_sample_timeout_s;
	float encoder_calibration_verify_timeout_s;
	uint16_t encoder_calibration_max_rms_residual_q15;
	uint16_t encoder_calibration_max_peak_residual_q15;
	float encoder_calibration_startup_timeout_s;
	float encoder_calibration_stop_speed_margin;
	float encoder_calibration_stop_deceleration_time_s;
	float encoder_calibration_stop_deceleration_timeout_s;
	float encoder_calibration_stop_speed_tolerance_ratio;
	float encoder_calibration_stop_current_ramp_time_s;
	float encoder_electrical_zero_current_ramp_time_s;
	float encoder_electrical_zero_hold_time_s;
} ControlTuningProfile;

const ControlTuningProfile *ControlTuningProfile_GetActive(void);

#endif
