#ifndef CORE_APPLICATION_MOTOR_CONTROL_CURRENT_OFFSET_CALIBRATION_RUNTIME_H
#define CORE_APPLICATION_MOTOR_CONTROL_CURRENT_OFFSET_CALIBRATION_RUNTIME_H

#include <stdint.h>

#include "current_control_runtime.h"
#include "measurement_model.h"
#include "motor_control_types.h"

typedef struct MotorStateContext MotorStateContext;

typedef struct
{
	uint32_t frame_count;
	uint32_t offset_sum[MEASUREMENT_MODEL_PHASE_COUNT];
	uint32_t observation_count[MEASUREMENT_MODEL_PHASE_COUNT];
	uint16_t offset_adc[MEASUREMENT_MODEL_PHASE_COUNT];
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
	const MeasurementCurrentSenseConfig *current_sense,
	uint32_t sample_count, MotorStateContext *motor_state);
bool CurrentOffsetCalibrationRuntime_ReadResult(
	const CurrentOffsetCalibrationContext *context,
	uint16_t *phase_a_offset_adc, uint16_t *phase_b_offset_adc,
	uint16_t *phase_c_offset_adc);
void CurrentOffsetCalibrationRuntime_Reset(
	CurrentOffsetCalibrationContext *context);

#endif
