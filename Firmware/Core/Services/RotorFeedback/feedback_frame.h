#ifndef CORE_SERVICES_ROTOR_FEEDBACK_FEEDBACK_FRAME_H
#define CORE_SERVICES_ROTOR_FEEDBACK_FEEDBACK_FRAME_H

#include <stdbool.h>

#include "feedback_router.h"

/* Hardware-independent, control-facing feedback snapshot. */
typedef struct
{
	float electrical_angle_rad;
	float electrical_velocity_rad_s;
	float motor_velocity_rad_s;
	float motor_position_rad;
	float output_position_rad;
	float output_velocity_rad_s;
	float control_position_rad;
	float control_velocity_rad_s;
	float calibration_reference_rad;
	FeedbackRouterSignalMask valid_signals;
	FeedbackRouterHealth health;
	bool uses_output_position;
} RotorFeedbackFrame;

#endif
