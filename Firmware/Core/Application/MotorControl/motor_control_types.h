#ifndef CORE_APPLICATION_MOTOR_CONTROL_MOTOR_CONTROL_TYPES_H
#define CORE_APPLICATION_MOTOR_CONTROL_MOTOR_CONTROL_TYPES_H

#include <stdbool.h>
#include <stdint.h>
#include "Core/Config/product_config.h"

typedef enum
{
	MOTOR_FAULT_NONE = 0,
	MOTOR_FAULT_CURRENT_OFFSET,
	MOTOR_FAULT_ENCODER,
	MOTOR_FAULT_POLE_PAIRS,
	MOTOR_FAULT_CAN_DISCONNECTED,
	MOTOR_FAULT_PHASE_RESISTANCE,
	MOTOR_FAULT_PHASE_INDUCTANCE,
	MOTOR_FAULT_OVER_CURRENT,
	MOTOR_FAULT_OVER_VOLTAGE,
	MOTOR_FAULT_UNDER_VOLTAGE,
	MOTOR_FAULT_HIGH_TEMPERATURE,
	MOTOR_FAULT_ENCODER_NOT_CALIBRATED,
	MOTOR_FAULT_INVALID_PARAMETER,
	MOTOR_FAULT_SENSORLESS,
	MOTOR_FAULT_PARAMETER_STORE,
	MOTOR_FAULT_POWER_STAGE,
	MOTOR_FAULT_FRICTION_IDENTIFICATION,
	MOTOR_FAULT_ENCODER_DIRECTION,
	MOTOR_FAULT_COGGING_IDENTIFICATION
} MotorFaultCode;

typedef struct
{
	uint16_t phase_a_current_offset_adc;
	uint16_t phase_b_current_offset_adc;
	uint16_t phase_c_current_offset_adc;
	int32_t pole_pairs;
	float phase_resistance_ohm;
	float d_axis_inductance_h;
	float q_axis_inductance_h;
	float flux_weber;
	bool use_sensorless_feedback;
	float calibration_current_a;
	float current_limit_a;
	float speed_limit_rad_s;
	float d_axis_current_kp;
	float d_axis_current_ki;
	float q_axis_current_kp;
	float q_axis_current_ki;
	float speed_acceleration_rad_s2;
	float speed_deceleration_rad_s2;
	float speed_kp;
	float speed_ki;
	float position_error_window_rad;
	float position_acceleration_rad_s2;
	float position_deceleration_rad_s2;
	float position_max_speed_rad_s;
	float position_kp_a_per_rad;
	float position_kd_a_per_rad_s;
	float position_ki_a_per_rad_s;
	float position_integral_limit_a;
	float cascade_position_kp_per_s;
	float cascade_position_kd;
	float open_loop_voltage_v;
	float open_loop_electrical_velocity_rad_s;
	float friction_coulomb_pos_a;
	float friction_coulomb_neg_a;
	float friction_viscous_pos_a_per_rad_s;
	float friction_viscous_neg_a_per_rad_s;
	bool friction_model_valid;
} MotorConfiguration;

typedef struct
{
	float d_axis_current_reference_a;
	float q_axis_current_reference_a;
	float q_axis_voltage_reference_v;
	float speed_reference_rad_s;
	float position_reference_rad;
} MotorCommand;

/*
 * Single-writer control demand derived from the accepted MotorCommand.
 * Control modes and service procedures may update this object, but must never
 * rewrite MotorCommand: the latter remains the accepted Application command.
 * Only the command mailbox and the safety/lifecycle arbiter may replace it.
 */
typedef struct
{
	float d_axis_current_a;
	float q_axis_current_a;
	float q_axis_voltage_v;
	float speed_rad_s;
	float position_rad;
} MotorControlTargets;

typedef struct
{
	MotorFaultCode primary_fault;
	float action_telemetry;
	float fault_telemetry;
	float phase_resistance_vector_ohm[3];
	float phase_a_resistance_ohm;
	float phase_b_resistance_ohm;
	float phase_c_resistance_ohm;
	float phase_resistance_spread_pct;
	float phase_resistance_design_error_pct;
	bool phase_resistance_valid;
	bool phase_resistance_warning;
	bool phase_resistance_balanced;
	bool phase_resistance_matches_design;
	float speed_command_ramp_rad_s;
	bool has_reached_position;
	float position_command_ramp_rad;
	float position_velocity_filtered_rad_s;
	float open_loop_electrical_angle_rad;
} MotorRuntimeState;

/* All runtime loop timing is derived once from ProductConfig at the
 * composition/runtime boundary.  Consumers retain neither board clocks nor
 * independently rounded copies of these divisors. */
typedef struct
{
	float current_period_s;
	float speed_period_s;
	float position_period_s;
	float cascade_position_period_s;
	uint16_t current_divider;
	uint16_t speed_divider;
	uint16_t position_divider;
	uint16_t cascade_position_divider;
} MotorControlLoopSchedule;

static inline bool MotorControlLoopSchedule_Derive(
	MotorControlLoopSchedule *schedule,
	uint32_t current_frequency_hz,
	uint32_t speed_frequency_hz,
	uint32_t position_frequency_hz,
	uint32_t cascade_position_frequency_hz)
{
	MotorControlLoopSchedule derived;
	uint32_t speed_divider;
	uint32_t position_divider;
	uint32_t cascade_position_divider;

	if (schedule == 0 || current_frequency_hz == 0U ||
		speed_frequency_hz == 0U || position_frequency_hz == 0U ||
		cascade_position_frequency_hz == 0U ||
		speed_frequency_hz > current_frequency_hz ||
		position_frequency_hz > current_frequency_hz ||
		cascade_position_frequency_hz > current_frequency_hz ||
		current_frequency_hz % speed_frequency_hz != 0U ||
		current_frequency_hz % position_frequency_hz != 0U ||
		current_frequency_hz % cascade_position_frequency_hz != 0U)
		return false;

	speed_divider = current_frequency_hz / speed_frequency_hz;
	position_divider = current_frequency_hz / position_frequency_hz;
	cascade_position_divider =
		current_frequency_hz / cascade_position_frequency_hz;
	if (speed_divider > UINT16_MAX || position_divider > UINT16_MAX ||
		cascade_position_divider > UINT16_MAX)
		return false;

	derived.current_period_s = 1.0f / (float)current_frequency_hz;
	derived.speed_period_s = 1.0f / (float)speed_frequency_hz;
	derived.position_period_s = 1.0f / (float)position_frequency_hz;
	derived.cascade_position_period_s =
		1.0f / (float)cascade_position_frequency_hz;
	derived.current_divider = 1U;
	derived.speed_divider = (uint16_t)speed_divider;
	derived.position_divider = (uint16_t)position_divider;
	derived.cascade_position_divider =
		(uint16_t)cascade_position_divider;
	*schedule = derived;
	return true;
}

typedef struct
{
	MotorConfiguration configuration;
	MotorCommand command;
	MotorControlTargets targets;
	MotorRuntimeState runtime;
	MotorControlLoopSchedule schedule;
	const ProductControlConfig *control_config;
	const ProductCommissioningTuningConfig *commissioning_tuning;
} MotorControlContext;

#endif
