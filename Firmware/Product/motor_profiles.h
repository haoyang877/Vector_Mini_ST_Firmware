#ifndef PRODUCT_MOTOR_PROFILES_H
#define PRODUCT_MOTOR_PROFILES_H

#include <stdint.h>

typedef struct
{
	uint16_t profile_id;
	uint8_t pole_pairs;
	float phase_resistance_ohm;
	float d_axis_inductance_h;
	float q_axis_inductance_h;
	float flux_weber;
	float phase_resistance_min_ohm;
	float phase_resistance_max_ohm;
	float inductance_min_h;
	float inductance_max_h;
	float flux_min_weber;
	float flux_max_weber;
	float calibration_current_a;
	float current_limit_a;
	float speed_limit_rps;
	float current_loop_bandwidth_rad_s;
	float open_loop_voltage_v;
	float open_loop_electrical_velocity_rad_s;
	float open_loop_initial_theta_rad;
	float speed_acceleration_rps2;
	float speed_deceleration_rps2;
	float speed_kp;
	float speed_ki;
	float position_acceleration_rps2;
	float position_deceleration_rps2;
	float position_max_speed_rps;
	float position_kp_a_per_rad;
	float position_kd_a_per_rad_s;
	float position_ki_a_per_rad_s;
	float position_integral_limit_a;
	float position_error_window_rad;
	float cascade_position_kp_per_s;
	float cascade_position_kd;
	float phase_resistance_test_current_low_a;
	float phase_resistance_test_current_high_a;
	float phase_resistance_test_current_max_a;
	float phase_resistance_test_current_min_a;
	float phase_resistance_current_tolerance_a;
	float phase_resistance_q_current_tolerance_a;
	float phase_resistance_voltage_tolerance_v;
	float phase_resistance_voltage_min_delta_v;
	float phase_resistance_voltage_filter_alpha;
	uint32_t phase_resistance_ramp_time_ms;
	uint32_t phase_resistance_settle_time_ms;
	uint32_t phase_resistance_sample_time_ms;
	uint32_t phase_resistance_pause_time_ms;
	uint32_t phase_resistance_timeout_ms;
	float phase_resistance_balance_warning_pct;
	float phase_resistance_balance_fault_pct;
	float speed_limit_max_rad_s;
	float speed_ramp_max_rad_s2;
	float position_ramp_max_rad_s2;
	float position_speed_limit_rps;
	float position_kp_limit_a_per_rad;
	float position_kd_limit_a_per_rad_s;
	float position_ki_limit_a_per_rad_s;
	float cascade_position_kp_limit_per_s;
	float cascade_position_kd_limit;
} MotorProfile;

const MotorProfile *MotorProfile_GetActive(void);

#endif
