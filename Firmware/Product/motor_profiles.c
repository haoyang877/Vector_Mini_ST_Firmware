#include "motor_profiles.h"

#include "vector_mini_st_profile.h"

#define MOTOR_PROFILE_TWO_PI 6.28318530717958647692f
#define MOTOR_LIMIT_CASCADE_POSITION_KP_PER_S       50.0f
#define MOTOR_LIMIT_CASCADE_POSITION_KD             10.0f
#define MOTOR_LIMIT_POSITION_KP_A_PER_RAD            50.0f
#define MOTOR_LIMIT_POSITION_KD_A_PER_RAD_S          10.0f
#define MOTOR_LIMIT_POSITION_KI_A_PER_RAD_S          10.0f
#define MOTOR_LIMIT_POSITION_SPEED_RPS               (372.0f / 60.0f)

static const MotorProfile ActiveMotorProfile =
{
	.profile_id = ACTIVE_MOTOR_PROFILE,
	.pole_pairs = PARAM_MOTOR_POLE_PAIRS,
	.phase_resistance_ohm = PARAM_MOTOR_PHASE_RESISTANCE_OHM,
	.d_axis_inductance_h = PARAM_MOTOR_D_INDUCTANCE_H,
	.q_axis_inductance_h = PARAM_MOTOR_Q_INDUCTANCE_H,
	.flux_weber = PARAM_MOTOR_FLUX_WB,
	.phase_resistance_min_ohm = PARAM_MOTOR_PHASE_RESISTANCE_MIN_OHM,
	.phase_resistance_max_ohm = PARAM_MOTOR_PHASE_RESISTANCE_MAX_OHM,
	.inductance_min_h = PARAM_MOTOR_INDUCTANCE_MIN_H,
	.inductance_max_h = PARAM_MOTOR_INDUCTANCE_MAX_H,
	.flux_min_weber = PARAM_MOTOR_FLUX_MIN_WB,
	.flux_max_weber = PARAM_MOTOR_FLUX_MAX_WB,
	.calibration_current_a = PARAM_MOTOR_CALIB_CURRENT_A,
	.current_limit_a = PARAM_MOTOR_CURRENT_LIMIT_A,
	.speed_limit_rps = PARAM_MOTOR_SPEED_LIMIT_RPS,
	.current_loop_bandwidth_rad_s = PARAM_MOTOR_CURRENT_LOOP_BANDWIDTH_RAD_S,
	.open_loop_voltage_v = PARAM_APP_OPEN_LOOP_VOLTAGE_V,
	.open_loop_electrical_velocity_rad_s = PARAM_APP_OPEN_LOOP_ELEC_VEL_RAD_S,
	.open_loop_initial_theta_rad = PARAM_APP_OPEN_LOOP_THETA_RAD,
	.speed_acceleration_rps2 = PARAM_APP_SPEED_ACCEL_RPS2,
	.speed_deceleration_rps2 = PARAM_APP_SPEED_DECEL_RPS2,
	.speed_kp = PARAM_APP_SPEED_KP,
	.speed_ki = PARAM_APP_SPEED_KI,
	.position_acceleration_rps2 = PARAM_APP_POSITION_ACCEL_RPS2,
	.position_deceleration_rps2 = PARAM_APP_POSITION_DECEL_RPS2,
	.position_max_speed_rps = PARAM_APP_POSITION_MAX_SPEED_RPS,
	.position_kp_a_per_rad = PARAM_APP_POSITION_KP,
	.position_kd_a_per_rad_s = PARAM_APP_POSITION_KD,
	.position_ki_a_per_rad_s = PARAM_APP_POSITION_KI,
	.position_integral_limit_a = PARAM_APP_POSITION_INTEGRAL_LIMIT_A,
	.position_error_window_rad = PARAM_APP_POSITION_ERROR_WINDOW_RAD,
	.cascade_position_kp_per_s = PARAM_APP_CASCADE_POSITION_KP,
	.cascade_position_kd = PARAM_APP_CASCADE_POSITION_KD,
	.phase_resistance_test_current_low_a = PARAM_MOTOR_PHASE_RESISTANCE_TEST_CURRENT_LOW_A,
	.phase_resistance_test_current_high_a = PARAM_MOTOR_PHASE_RESISTANCE_TEST_CURRENT_HIGH_A,
	.phase_resistance_test_current_max_a = PARAM_MOTOR_PHASE_RESISTANCE_TEST_CURRENT_MAX_A,
	.phase_resistance_test_current_min_a = PARAM_MOTOR_PHASE_RESISTANCE_TEST_CURRENT_MIN_A,
	.phase_resistance_current_tolerance_a = PARAM_MOTOR_PHASE_RESISTANCE_CURRENT_TOLERANCE_A,
	.phase_resistance_q_current_tolerance_a = PARAM_MOTOR_PHASE_RESISTANCE_Q_CURRENT_TOLERANCE_A,
	.phase_resistance_voltage_tolerance_v = PARAM_MOTOR_PHASE_RESISTANCE_VOLTAGE_TOLERANCE_V,
	.phase_resistance_voltage_min_delta_v = PARAM_MOTOR_PHASE_RESISTANCE_VOLTAGE_MIN_DELTA_V,
	.phase_resistance_voltage_filter_alpha = PARAM_MOTOR_PHASE_RESISTANCE_VOLTAGE_FILTER_ALPHA,
	.phase_resistance_ramp_time_ms = PARAM_MOTOR_PHASE_RESISTANCE_RAMP_TIME_MS,
	.phase_resistance_settle_time_ms = PARAM_MOTOR_PHASE_RESISTANCE_SETTLE_TIME_MS,
	.phase_resistance_sample_time_ms = PARAM_MOTOR_PHASE_RESISTANCE_SAMPLE_TIME_MS,
	.phase_resistance_pause_time_ms = PARAM_MOTOR_PHASE_RESISTANCE_PAUSE_TIME_MS,
	.phase_resistance_timeout_ms = PARAM_MOTOR_PHASE_RESISTANCE_TIMEOUT_MS,
	.phase_resistance_balance_warning_pct = PARAM_MOTOR_PHASE_RESISTANCE_BALANCE_WARNING_PCT,
	.phase_resistance_balance_fault_pct = PARAM_MOTOR_PHASE_RESISTANCE_BALANCE_FAULT_PCT,
	.speed_limit_max_rad_s = 400.0f * MOTOR_PROFILE_TWO_PI,
	.speed_ramp_max_rad_s2 = 1000.0f * MOTOR_PROFILE_TWO_PI,
	.position_ramp_max_rad_s2 = 200.0f * MOTOR_PROFILE_TWO_PI,
	.position_speed_limit_rps = MOTOR_LIMIT_POSITION_SPEED_RPS,
	.position_kp_limit_a_per_rad = MOTOR_LIMIT_POSITION_KP_A_PER_RAD,
	.position_kd_limit_a_per_rad_s = MOTOR_LIMIT_POSITION_KD_A_PER_RAD_S,
	.position_ki_limit_a_per_rad_s = MOTOR_LIMIT_POSITION_KI_A_PER_RAD_S,
	.cascade_position_kp_limit_per_s = MOTOR_LIMIT_CASCADE_POSITION_KP_PER_S,
	.cascade_position_kd_limit = MOTOR_LIMIT_CASCADE_POSITION_KD
};

const MotorProfile *MotorProfile_GetActive(void)
{
	return &ActiveMotorProfile;
}
