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
	float position_error_window; /* 仅轨迹结束容差，rad；HOLD资格使用独立窗口。 */
	float hold_enter_position; /* 主动保持入口及前馈捕获窗口，rad。 */
	float hold_exit_position; /* 主动保持退出及摩擦恢复滞环，rad。 */
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
	float acceleration_feedforward_gain; /* J/Kt，A/(rad/s²)；只乘轨迹加速度，0关闭。 */
	float current_limit;
    float thermal_output_limit_a; /* 实际可用电流，A；不作为PI增益尺度。 */
    bool thermal_limit_active;
	bool friction_feedforward_enabled;
	bool passive_hold_enabled; /* 仅保留布局兼容；恢复主动保持后忽略。 */
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
    /* 仅保留布局兼容；独立支撑配置被忽略，不参与主动保持。 */
    float support_current_limit_a;
    float support_adjust_rate_a_per_s;
    float support_release_rate_a_per_s;
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
    float support_current_a; /* 兼容字段恒为0；hold_current仍为PI.Ui，单位A。 */
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
    /* 以下为旧被动停靠/支撑的兼容字段，均恒为false或0。 */
	bool passive_hold;
    float support_current_a;
    bool support_adjusting;
    bool support_limited;
    bool support_refining;
    uint32_t passive_confirm_ticks;
    float qualification_position_rad;
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
