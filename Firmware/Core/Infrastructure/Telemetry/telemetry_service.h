#ifndef CORE_INFRASTRUCTURE_TELEMETRY_SERVICE_H
#define CORE_INFRASTRUCTURE_TELEMETRY_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

#define MOTOR_TELEMETRY_FAULT_CODE_COUNT 19U

typedef enum
{
	MOTOR_TELEMETRY_MODE = 0,
	MOTOR_TELEMETRY_PRIMARY_ERROR,
	MOTOR_TELEMETRY_ACTIVE_FAULTS,
	MOTOR_TELEMETRY_LATCHED_FAULTS,
	MOTOR_TELEMETRY_CURRENT_REFERENCE_A,
	MOTOR_TELEMETRY_SPEED_REFERENCE_RAD_S,
	MOTOR_TELEMETRY_POSITION_REFERENCE_RAD,
	MOTOR_TELEMETRY_BUS_VOLTAGE_V,
	MOTOR_TELEMETRY_BUS_CURRENT_A,
	MOTOR_TELEMETRY_PHASE_A_CURRENT_A,
	MOTOR_TELEMETRY_PHASE_B_CURRENT_A,
	MOTOR_TELEMETRY_PHASE_C_CURRENT_A,
	MOTOR_TELEMETRY_D_AXIS_CURRENT_A,
	MOTOR_TELEMETRY_Q_AXIS_CURRENT_A,
	MOTOR_TELEMETRY_D_AXIS_CURRENT_FILTERED_A,
	MOTOR_TELEMETRY_Q_AXIS_CURRENT_FILTERED_A,
	MOTOR_TELEMETRY_MECHANICAL_SPEED_RAD_S,
	MOTOR_TELEMETRY_MECHANICAL_POSITION_RAD,
	MOTOR_TELEMETRY_TEMPERATURE_C,
	MOTOR_TELEMETRY_ENCODER_ONLINE,
	MOTOR_TELEMETRY_ENCODER_REVERSED,
	MOTOR_TELEMETRY_POLE_PAIRS,
	MOTOR_TELEMETRY_CALIBRATION_CURRENT_A,
	MOTOR_TELEMETRY_CURRENT_LIMIT_A,
	MOTOR_TELEMETRY_SPEED_LIMIT_RAD_S,
	MOTOR_TELEMETRY_SPEED_ACCELERATION_RAD_S2,
	MOTOR_TELEMETRY_SPEED_DECELERATION_RAD_S2,
	MOTOR_TELEMETRY_SPEED_KP,
	MOTOR_TELEMETRY_SPEED_KI,
	MOTOR_TELEMETRY_POSITION_ACCELERATION_RAD_S2,
	MOTOR_TELEMETRY_POSITION_DECELERATION_RAD_S2,
	MOTOR_TELEMETRY_POSITION_MAX_SPEED_RAD_S,
	MOTOR_TELEMETRY_POSITION_KP_A_PER_RAD,
	MOTOR_TELEMETRY_POSITION_KD_A_PER_RAD_S,
	MOTOR_TELEMETRY_POSITION_KI_A_PER_RAD_S,
	MOTOR_TELEMETRY_POSITION_INTEGRAL_LIMIT_A,
	MOTOR_TELEMETRY_CASCADE_POSITION_KP_PER_S,
	MOTOR_TELEMETRY_CASCADE_POSITION_KD,
	MOTOR_TELEMETRY_PHASE_RESISTANCE_OHM,
	MOTOR_TELEMETRY_D_AXIS_INDUCTANCE_H,
	MOTOR_TELEMETRY_Q_AXIS_INDUCTANCE_H,
	MOTOR_TELEMETRY_FLUX_WEBER,
	MOTOR_TELEMETRY_COMMISSIONING_STAGE,
	MOTOR_TELEMETRY_COMMISSIONING_PROGRESS_PERCENT,
	MOTOR_TELEMETRY_PHASE_RESISTANCE_SPREAD_PERCENT,
	MOTOR_TELEMETRY_PHASE_RESISTANCE_DESIGN_ERROR_PERCENT
} MotorTelemetryId;

typedef struct
{
	uint32_t mode;
	uint32_t primary_error;
	uint32_t active_faults;
	uint32_t latched_faults;
	uint32_t fault_event_sequence;
	uint32_t fault_occurrence_count[MOTOR_TELEMETRY_FAULT_CODE_COUNT];
	uint32_t fast_loop_invocation_count;
	uint32_t fast_loop_maximum_cycles;
	uint32_t fast_loop_deadline_cycles;
	uint32_t fast_loop_deadline_overrun_count;
	uint32_t primary_fault_occurrence_count;
	uint32_t primary_fault_first_event_sequence;
	uint32_t primary_fault_latest_event_sequence;
	uint32_t primary_fault_first_time_ms;
	uint32_t primary_fault_latest_time_ms;
	float fault_bus_voltage_v;
	float fault_phase_a_current_a;
	float fault_phase_b_current_a;
	float fault_phase_c_current_a;
	float fault_temperature_c;
	float fault_mechanical_position_rad;
	float fault_mechanical_speed_rad_s;
	float current_reference_a;
	float speed_reference_rad_s;
	float position_reference_rad;
	float d_axis_current_target_a;
	float q_axis_current_target_a;
	float speed_target_rad_s;
	float position_target_rad;
	float bus_voltage_v;
	float bus_current_a;
	float phase_a_current_a;
	float phase_b_current_a;
	float phase_c_current_a;
	float d_axis_current_a;
	float q_axis_current_a;
	float d_axis_current_filtered_a;
	float q_axis_current_filtered_a;
	float mechanical_speed_rad_s;
	float mechanical_position_rad;
	float temperature_c;
	uint32_t encoder_online;
	uint32_t encoder_reversed;
	float pole_pairs;
	float calibration_current_a;
	float current_limit_a;
	float speed_limit_rad_s;
	float speed_acceleration_rad_s2;
	float speed_deceleration_rad_s2;
	float speed_kp;
	float speed_ki;
	float position_acceleration_rad_s2;
	float position_deceleration_rad_s2;
	float position_max_speed_rad_s;
	float position_kp_a_per_rad;
	float position_kd_a_per_rad_s;
	float position_ki_a_per_rad_s;
	float position_integral_limit_a;
	float cascade_position_kp_per_s;
	float cascade_position_kd;
	float phase_resistance_ohm;
	float d_axis_inductance_h;
	float q_axis_inductance_h;
	float flux_weber;
	uint32_t commissioning_stage;
	uint32_t commissioning_progress_percent;
	float phase_resistance_spread_percent;
	float phase_resistance_design_error_percent;
} MotorTelemetrySnapshot;

typedef struct
{
	volatile MotorTelemetrySnapshot buffers[2];
	volatile uint32_t sequence[2];
	volatile uint8_t published_buffer;
	volatile bool is_available;
} TelemetryServiceContext;

bool TelemetryService_Initialize(TelemetryServiceContext *context);
void TelemetryService_Publish(TelemetryServiceContext *context,
	const MotorTelemetrySnapshot *snapshot);
bool TelemetryService_ReadSnapshot(const TelemetryServiceContext *context,
	MotorTelemetrySnapshot *snapshot);
bool TelemetryService_ReadValue(const TelemetryServiceContext *context,
	MotorTelemetryId telemetry, float *value);

#endif
