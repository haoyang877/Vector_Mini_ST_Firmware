#ifndef __POSITION_CASCADE_H__
#define __POSITION_CASCADE_H__

#include <stdbool.h>

typedef struct
{
	float target_position;
	float position_error_window;
	float acceleration;
	float deceleration;
	float maximum_speed;
	float position_kp;
	float position_kd;
	float speed_kp;
	float speed_ki;
	float current_limit;
} PositionCascadeConfig_TypeDef;

typedef struct
{
	float position_reference;
	float speed_reference;
	float speed_feedback;
	float iq_reference;
	bool target_reached;
} PositionCascadeOutput_TypeDef;

void PositionCascade_Reset(void);
bool PositionCascade_Update(const PositionCascadeConfig_TypeDef *config,
	float measured_position, float measured_speed,
	PositionCascadeOutput_TypeDef *output);

#endif
