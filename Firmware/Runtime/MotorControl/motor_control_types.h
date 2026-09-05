#ifndef RUNTIME_MOTOR_CONTROL_TYPES_H
#define RUNTIME_MOTOR_CONTROL_TYPES_H

#include <stdbool.h>
#include <stdint.h>
#include "control_tuning_profile.h"
#include "mechanical_load_profiles.h"

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
	MOTOR_FAULT_POWER_STAGE
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
	bool phase_resistance_valid;
	bool phase_resistance_warning;
	bool phase_resistance_balanced;
	float speed_command_ramp_rad_s;
	bool has_reached_position;
	float position_command_ramp_rad;
	float position_velocity_filtered_rad_s;
	float open_loop_electrical_angle_rad;
} MotorRuntimeState;

typedef struct
{
	MotorConfiguration configuration;
	MotorCommand command;
	MotorControlTargets targets;
	MotorRuntimeState runtime;
	const ControlTuningProfile *tuning_profile;
	const MechanicalLoadProfile *mechanical_load_profile;
} MotorControlContext;

#endif
