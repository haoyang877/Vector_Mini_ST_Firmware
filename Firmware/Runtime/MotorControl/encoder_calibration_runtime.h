#ifndef RUNTIME_ENCODER_CALIBRATION_H
#define RUNTIME_ENCODER_CALIBRATION_H

#include <stdbool.h>
#include <stdint.h>
#include "current_control_runtime.h"
#include "motor_control_types.h"
#include "encoder.h"
#include "sensorless_runtime.h"
#include "board_profile.h"
#include "current_offset_calibration_runtime.h"
#include "electrical_zero_calibration_runtime.h"

#define OFFSET_LUT_NUM              ENCODER_OFFSET_LUT_SIZE
#define MAX_MOTOR_POLE_PAIRS        20U
#define COGGING_MAP_NUM             5000U

typedef enum
{
	CS_NULL = 0,
	CS_ADC_OFFSET_START,
	CS_ADC_OFFSET_LOOP,
	CS_ADC_OFFSET_END,
	CS_MOTOR_R_START,
	CS_MOTOR_RA_LOOP,
	CS_MOTOR_RB_LOOP,
	CS_MOTOR_RC_LOOP,
	CS_MOTOR_R_END,
	CS_MOTOR_L_START,
	CS_MOTOR_LD_LOOP,
	CS_MOTOR_LQ_LOOP,
	CS_MOTOR_L_END,
	CS_MOTOR_FLUX_START,
	CS_MOTOR_FLUX_LOOP,
	CS_MOTOR_FLUX_END,
	CS_ANTICOGGING_START,
	CS_ANTICOGGING_CW_TEMP,
	CS_ANTICOGGING_CW_SAMPLE,
	CS_ANTICOGGING_CCW_TEMP,
	CS_ANTICOGGING_CCW_SAMPLE,
	CS_ANTICOGGING_END,
	CS_OBS_ALIGN_ORIGIN,
	CS_OBS_WAIT_CLOSED_LOOP,
	CS_OBS_SPEED_STABLE,
	CS_OBS_FIND_ORIGIN,
	CS_OBS_SAMPLE_CW,
	CS_OBS_BUILD_LUT,
	CS_OBS_VERIFY_CW,
	CS_OBS_STOP_DECEL,
	CS_OBS_STOP_CURRENT,
	CS_OBS_CLEAR_SAMPLES,
	CS_ENC_OFFSET_CLEAR_SAMPLES,
	CS_ENC_OFFSET_ALIGN,
	CS_ENC_OFFSET_ALIGN_LOOP,
	CS_ENC_OFFSET_RAMP_CW,
	CS_ENC_OFFSET_SAMPLE_CW,
	CS_ENC_OFFSET_BUILD_LUT
} CalibrationStep;

typedef struct
{
	uint32_t loop_count;
	float drive_phase;
	float drive_omega;
	float observer_theta_last;
	float observer_theta_unwrapped;
	float sample_theta_start;
	uint32_t observer_unlock_ticks;
	bool sampling_started;
} EncoderLinearizationContext;

typedef struct
{
	uint16_t index;
	uint8_t stage;
	int32_t previous;
	int32_t shift;
	int64_t sum;
} EncoderLutBuildContext;

typedef struct
{
	uint32_t state_ticks;
	uint32_t stage_ticks;
	uint32_t observer_position_epoch;
	uint16_t origin_anchor_q15;
	int64_t origin_sum_q15;
	uint32_t origin_sample_count;
	uint16_t origin_q15;
	uint16_t previous_directed_q15;
	float previous_observer_position;
	float observer_position_origin;
	float sample_previous_observer_position;
	float sample_valid_electrical_travel;
	float verify_previous_observer_position;
	float verify_valid_electrical_travel;
	float stop_start_speed;
	float stop_current_ref;
	uint64_t residual_squared_sum;
	uint32_t residual_sample_count;
	uint32_t residual_peak_abs_q15;
	bool origin_negative_seen;
} ObserverCalibrationContext;

typedef struct
{
	int32_t position_error_sum_storage[ENCODER_OFFSET_LUT_SIZE];
	uint16_t calibration_samples_storage[ENCODER_OFFSET_LUT_SIZE];
	int16_t candidate_linearization_lut_storage[ENCODER_OFFSET_LUT_SIZE];
	int32_t *position_error_sum;
	uint16_t *calibration_samples;
	int16_t *candidate_linearization_lut;
	uint16_t sample_clear_index;
	CalibrationStep step;
	bool encoder_offset_sampling_started;
	bool observer_origin_negative_seen;
	EncoderLinearizationContext encoder_linearization;
	EncoderLutBuildContext lut_build;
	ObserverCalibrationContext observer_calibration;
} MotorCalibrationContext;

typedef struct MotorStateContext MotorStateContext;

void MotorCalibration_Reset(MotorCalibrationContext *context);
void CalibrationRuntime_RunEncoderLinearization(MotorCalibrationContext *context,
	CurrentControlContext *CurrentControl, MotorControlContext *MotorControl,
	EncoderContext *Encoder, MotorStateContext *motor_state);
void CalibrationRuntime_RunEncoderObserver(MotorCalibrationContext *context,
	CurrentControlContext *CurrentControl, MotorControlContext *MotorControl,
	PiController *SpeedController, EncoderContext *Encoder,
	FluxObserverContext *Fluxobserver, SensorlessStartupContext *Startup,
	MotorStateContext *motor_state);
#endif
