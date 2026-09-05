#ifndef RUNTIME_ENCODER_DIRECTION_CALIBRATION_H
#define RUNTIME_ENCODER_DIRECTION_CALIBRATION_H

#include <stdbool.h>
#include <stdint.h>

#include "current_control_runtime.h"
#include "encoder.h"
#include "motor_control_types.h"

typedef struct MotorStateContext MotorStateContext;

typedef enum
{
	ENCODER_DIRECTION_CALIBRATION_IDLE = 0,
	ENCODER_DIRECTION_CALIBRATION_ALIGN,
	ENCODER_DIRECTION_CALIBRATION_ROTATE,
	ENCODER_DIRECTION_CALIBRATION_COMPLETE
} EncoderDirectionCalibrationState;

typedef struct
{
	EncoderDirectionCalibrationState state;
	uint32_t loop_count;
	float drive_phase_rad;
	uint16_t previous_raw_q15;
	int64_t raw_travel_q15;
	bool completion_reported;
} EncoderDirectionCalibrationContext;

void EncoderDirectionCalibrationRuntime_Reset(
	EncoderDirectionCalibrationContext *context);
void EncoderDirectionCalibrationRuntime_ExecuteStep(
	EncoderDirectionCalibrationContext *context,
	CurrentControlContext *current_control, MotorControlContext *motor,
	EncoderContext *encoder, MotorStateContext *motor_state);

#endif
