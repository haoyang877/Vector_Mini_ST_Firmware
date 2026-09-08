#ifndef __POSITION_CASCADE_H__
#define __POSITION_CASCADE_H__

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
	POSITION_SERVO_PHASE_MOVE = 0,
	POSITION_SERVO_PHASE_SETTLE,
	POSITION_SERVO_PHASE_HOLD
} PositionServoPhase_TypeDef;

typedef struct
{
	float update_period_s;
	uint16_t call_divider;
	float target_position;
	float position_error_window; /* Trajectory finish tolerance only, rad. */
	float hold_enter_position; /* Position qualification window, rad. */
	float hold_exit_position; /* Wider hold/recovery hysteresis, rad. */
	float velocity_filter_hz; /* 0 bypasses the software feedback filter. */
	float following_error_limit; /* Soft governor range in rad; 0 disables. */
	float stiction_integral_rate; /* A/s added to PI only after confirmed stall; 0 disables. */
	float acceleration;
	float deceleration;
	float maximum_speed;
	float speed_limit;
	float jerk_limit;
	float position_kp;
	float position_kd;
	float speed_kp;
	float speed_ki;
	float acceleration_feedforward_gain;
	float current_limit;
	bool friction_feedforward_enabled;
	float friction_coulomb_positive;
	float friction_coulomb_negative;
	float friction_viscous_positive;
	float friction_viscous_negative;
	float friction_breakaway_ratio;
	float friction_attack_slew_rate;
	float friction_fast_release_slew_rate;
	float friction_release_slew_rate;
	float friction_reference_speed;
	float friction_stop_speed;
	float friction_move_speed;
	float friction_breakaway_distance;
	float friction_stuck_time;
} PositionCascadeConfig_TypeDef;

typedef struct
{
	float position_reference;
	float speed_reference;
	float acceleration_reference;
	float speed_feedback;
	float iq_reference;
	float feedback_current;
	float acceleration_feedforward_current;
	float friction_feedforward_current;
	float hold_current;
	PositionServoPhase_TypeDef phase;
	bool target_reached;
} PositionCascadeOutput_TypeDef;

typedef struct
{
	float position_reference;
	float trajectory_speed_reference;
	float speed_command;
	float speed_feedback; /* Continuous, filtered velocity used by this servo. */
	float acceleration_reference;
	float feedback_current;
	float acceleration_feedforward_current;
	float friction_feedforward_current;
	float hold_current;
	PositionServoPhase_TypeDef phase;
	bool target_reached;
	bool current_saturated;
	bool friction_landing_active;
	bool settle_recovery_active;
	bool hold_candidate_active;
	bool trajectory_limited;
	bool stiction_integrating; /* Bounded static-error integral applied this update. */
} PositionCascadeTelemetry_TypeDef;

/** Reset all mode-3 position-servo trajectory and controller state. */
void PositionCascade_Reset(void);
/**
 * Execute one mode-3 position-servo call. The internal servo work is divided
 * down from the FOC interrupt to CASCADE_POSITION_LOOP_FREQ.
 */
bool PositionCascade_Update(const PositionCascadeConfig_TypeDef *config,
	float measured_position, float measured_speed,
	PositionCascadeOutput_TypeDef *output);
/** Copy a coherent mode-3 diagnostic snapshot without exposing private state. */
bool PositionCascade_GetTelemetry(PositionCascadeTelemetry_TypeDef *telemetry);
/** Defer optional telemetry after full configuration validation or a slow servo update. */
bool PositionCascade_ShouldDeferTelemetry(void);

#endif
