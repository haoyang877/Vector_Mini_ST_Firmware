#ifndef CORE_SERVICES_MOTION_CONTROL_POSITION_IMPEDANCE_H
#define CORE_SERVICES_MOTION_CONTROL_POSITION_IMPEDANCE_H

#include <stdbool.h>
#include <stdint.h>

#include "trapezoidal_trajectory.h"

typedef struct
{
    float target_position;
    float position_error_window;
    float acceleration;
    float deceleration;
    float maximum_speed;
    float speed_limit_rad_s;
    float kp;
    float kd;
    float ki;
    float integral_limit;
    float output_limit;
    float kp_limit;
    float kd_limit;
    float ki_limit;
    float maximum_speed_limit_rad_s;
    float maximum_current_limit_a;
	bool friction_feedforward_enabled;
	float friction_positive_current;
	float friction_negative_current;
	float breakaway_positive_current;
	float breakaway_negative_current;
	float friction_current_slew_rate;
	float friction_position_enter;
	float friction_position_exit;
	float friction_reference_speed;
	float friction_stop_speed;
	float friction_move_speed;
	float friction_stuck_time;
	float friction_landing_position;
	float friction_landing_speed;
	float friction_recovery_delay;
	float friction_recovery_pulse_time;
	float friction_recovery_cooldown;
    float sample_period_s;
    uint16_t loop_divider;
} PositionImpedanceConfig;

typedef struct
{
    float position_reference;
    float speed_reference;
    float velocity_feedback;
    float iq_reference;
	float friction_current;
    bool target_reached;
} PositionImpedanceOutput;

typedef struct
{
    TrapezoidalTrajectoryContext trajectory;
    bool initialized;
    bool integral_transport_active;
    float integral;
    float velocity_filtered;
    float last_position;
    float last_target;
    float last_acceleration;
    float last_deceleration;
    float last_maximum_speed;
    float position_reference;
    float speed_reference;
    float iq_reference;
	float friction_current;
    uint32_t hold_counter;
	uint32_t friction_stuck_counter;
	uint32_t friction_recovery_delay_counter;
	uint32_t friction_recovery_pulse_counter;
	uint32_t friction_recovery_cooldown_counter;
	uint16_t loop_counter;
	int8_t friction_direction;
	uint8_t friction_state;
    bool target_reached;
} PositionImpedanceContext;

void PositionImpedance_Reset(PositionImpedanceContext *context);
bool PositionImpedance_Update(PositionImpedanceContext *context,
    const PositionImpedanceConfig *config,
    float measured_position, PositionImpedanceOutput *output);

#endif
