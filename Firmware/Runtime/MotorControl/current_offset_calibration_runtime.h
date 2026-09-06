#ifndef RUNTIME_CURRENT_OFFSET_CALIBRATION_H
#define RUNTIME_CURRENT_OFFSET_CALIBRATION_H

#include <stdint.h>

#include "current_control_runtime.h"
#include "motor_control_types.h"

typedef struct MotorStateContext MotorStateContext;

typedef struct
{
	uint32_t offset_count;
	uint32_t phase_a_offset_sum;
	uint32_t phase_b_offset_sum;
	uint32_t phase_c_offset_sum;
	uint16_t phase_a_offset_adc;
	uint16_t phase_b_offset_adc;
	uint16_t phase_c_offset_adc;
	bool result_is_ready;
} CurrentOffsetCalibrationContext;

typedef enum
{
	CURRENT_OFFSET_CALIBRATION_RUNNING = 0,
	CURRENT_OFFSET_CALIBRATION_COMPLETE
} CurrentOffsetCalibrationStatus;

CurrentOffsetCalibrationStatus CurrentOffsetCalibrationRuntime_ExecuteStep(
	CurrentOffsetCalibrationContext *context,
	CurrentControlContext *current_control,
	uint32_t sample_count, MotorStateContext *motor_state);
bool CurrentOffsetCalibrationRuntime_ReadResult(
	const CurrentOffsetCalibrationContext *context,
	uint16_t *phase_a_offset_adc, uint16_t *phase_b_offset_adc,
	uint16_t *phase_c_offset_adc);
void CurrentOffsetCalibrationRuntime_Reset(
	CurrentOffsetCalibrationContext *context);

#endif
