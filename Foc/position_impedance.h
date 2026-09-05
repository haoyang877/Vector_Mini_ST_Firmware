#ifndef __POSITION_IMPEDANCE_H__
#define __POSITION_IMPEDANCE_H__

#include <stdbool.h>

typedef struct
{
	float target_position;
	float position_error_window;
	float acceleration;
	float deceleration;
	float maximum_speed;
	float speed_limit;
	float kp;
	float kd;
	float ki;
	float integral_limit;
	float output_limit;
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
} PositionImpedanceConfig_TypeDef;

typedef struct
{
	float position_reference;
	float speed_reference;
	float velocity_feedback;
	float iq_reference;
	float friction_current;
	bool target_reached;
} PositionImpedanceOutput_TypeDef;

void PositionImpedance_Reset(void);
bool PositionImpedance_Update(const PositionImpedanceConfig_TypeDef *config,
	float measured_position, PositionImpedanceOutput_TypeDef *output);

#endif
