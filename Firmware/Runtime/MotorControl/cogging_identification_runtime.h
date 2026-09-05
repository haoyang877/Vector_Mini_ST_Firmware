#ifndef RUNTIME_COGGING_IDENTIFICATION_H
#define RUNTIME_COGGING_IDENTIFICATION_H

#include <stdbool.h>
#include <stdint.h>

#include "control_mode_runtime.h"
#include "current_control_runtime.h"
#include "encoder.h"
#include "motor_control_types.h"
#include "pi_controller.h"

#define COGGING_COMPENSATION_MAP_SIZE ENCODER_COGGING_MAP_SIZE

typedef struct MotorStateContext MotorStateContext;

typedef enum
{
	COGGING_IDENT_IDLE = 0,
	COGGING_IDENT_WAIT_CW,
	COGGING_IDENT_SAMPLE_CW,
	COGGING_IDENT_WAIT_CCW,
	COGGING_IDENT_SAMPLE_CCW,
	COGGING_IDENT_BUILD,
	COGGING_IDENT_COMPLETE,
	COGGING_IDENT_FAILED
} CoggingIdentificationState;

typedef struct
{
	CoggingIdentificationState state;
	int32_t current_sum_ma[2][COGGING_COMPENSATION_MAP_SIZE];
	uint16_t sample_count[2][COGGING_COMPENSATION_MAP_SIZE];
	int64_t start_shadow_q15;
	int32_t map_sum_ma;
	float stable_speed_sum_rad_s;
	uint32_t stable_ticks;
	uint32_t stage_ticks;
	uint16_t build_index;
	uint8_t build_pass;
	bool completion_reported;
} CoggingIdentificationRuntimeContext;

void CoggingIdentificationRuntime_Reset(
	CoggingIdentificationRuntimeContext *context);
void CoggingIdentificationRuntime_ExecuteStep(
	CoggingIdentificationRuntimeContext *context, MotionControlContext *motion,
	CurrentControlContext *current_control, MotorControlContext *motor,
	PiController *speed_controller, EncoderContext *encoder,
	MotorStateContext *motor_state);
void CoggingIdentificationRuntime_Cancel(
	CoggingIdentificationRuntimeContext *context,
	CurrentControlContext *current_control, MotorControlContext *motor,
	PiController *speed_controller);

#endif
