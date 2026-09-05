#ifndef RUNTIME_ELECTRICAL_ZERO_CALIBRATION_H
#define RUNTIME_ELECTRICAL_ZERO_CALIBRATION_H

#include <stdbool.h>
#include <stdint.h>

#include "current_control_runtime.h"
#include "encoder.h"
#include "motor_control_types.h"

typedef struct
{
	uint32_t loop_count;
	uint32_t sample_count;
	uint16_t sample_anchor;
	int64_t unwrapped_sum;
	bool completion_reported;
} ElectricalZeroCalibrationContext;

void ElectricalZeroCalibrationRuntime_ExecuteStep(
	ElectricalZeroCalibrationContext *context,
	CurrentControlContext *current_control, MotorControlContext *motor,
	EncoderContext *encoder);
void ElectricalZeroCalibrationRuntime_Reset(
	ElectricalZeroCalibrationContext *context);

#endif
