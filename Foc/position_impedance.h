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
} PositionImpedanceConfig_TypeDef;

typedef struct
{
	float position_reference;
	float speed_reference;
	float velocity_feedback;
	float iq_reference;
	bool target_reached;
} PositionImpedanceOutput_TypeDef;

void PositionImpedance_Reset(void);
bool PositionImpedance_Update(const PositionImpedanceConfig_TypeDef *config,
	float measured_position, PositionImpedanceOutput_TypeDef *output);

#endif
