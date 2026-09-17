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
	float velocity_filter_hz; /* 0 bypasses the base filter; HOLD shaping is separate. */
	float hold_velocity_filter_hz; /* 0 bypasses HOLD shaping; runtime tuning in Hz. */
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
	float speed_feedback; /* Actual control velocity, including optional HOLD filter.
	                       * HOLD exit uses the original feedback before that filter. */
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

/* Minimal current-command/feedback view for the high-rate motor adapter.
 * Rich diagnostics remain available through GetTelemetry / the existing API. */
typedef struct
{
    float position_reference;
    float speed_reference;
    float speed_feedback;
    float iq_reference;
    bool target_reached;
} PositionCascadeControlOutput_TypeDef;

/** Same state transition, validation and timing as Update, with compact output. */
bool PositionCascade_UpdateControl(const PositionCascadeConfig_TypeDef *config,
    float measured_position, float measured_speed, PositionCascadeControlOutput_TypeDef *output);
/** Same validated-configuration contract as UpdateTarget, with compact output.
 * All APIs share one controller instance and require the same serial owner. */
bool PositionCascade_UpdateTargetControl(float target_position, float measured_position,
    float measured_speed, PositionCascadeControlOutput_TypeDef *output);

/** Reset all mode-3 position-servo trajectory and controller state. */
void PositionCascade_Reset(void);
/**
 * Execute one mode-3 position-servo call. The internal servo work is divided
 * down from the FOC interrupt to CASCADE_POSITION_LOOP_FREQ.
 */
bool PositionCascade_Update(const PositionCascadeConfig_TypeDef *config,
	float measured_position, float measured_speed,
	PositionCascadeOutput_TypeDef *output);
/** Borrow the last validated settings, or NULL after Reset/before configuration.
 * Read-only; valid until the next Update/Reset. Calls require one serial owner.
 * The adapter must detect live tuning changes before using UpdateTarget. */
const PositionCascadeConfig_TypeDef *PositionCascade_GetConfiguration(void);
/** Execute a fast tick with unchanged validated tuning and a fresh target.
 * Checks finite target/feedback every call; preserves target-change handling
 * and the existing divider. Fails if no configuration has been validated. */
bool PositionCascade_UpdateTarget(float target_position, float measured_position,
	float measured_speed, PositionCascadeOutput_TypeDef *output);
/** Copy a coherent mode-3 diagnostic snapshot without exposing private state. */
bool PositionCascade_GetTelemetry(PositionCascadeTelemetry_TypeDef *telemetry);
/** Defer optional telemetry after full configuration validation or a slow servo update. */
bool PositionCascade_ShouldDeferTelemetry(void);

#endif
