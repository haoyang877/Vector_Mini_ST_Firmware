#include "Core/Application/MotorControl/encoder_calibration_runtime.h"

#include <stdbool.h>
#include <limits.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include "fast_math.h"
#include "motor_state_runtime.h"
#include "control_tuning_access.h"
#include "sensorless_runtime.h"

#include <control_mode_runtime.h>

#define ENCODER_CALIB_CLEAR_ENTRIES_PER_CYCLE 16U

void MotorCalibration_Reset(MotorCalibrationContext *context)
{
	if (context != NULL)
	{
		/* Sample storage is cleared incrementally when a calibration starts.
		 * Reset only metadata here so service exit stays inside the ISR budget. */
		memset((uint8_t *)context + offsetof(MotorCalibrationContext,
			position_error_sum), 0,
			sizeof(*context) - offsetof(MotorCalibrationContext,
				position_error_sum));
	}
}

static void Encoder_Calib_ReleaseSamples(MotorCalibrationContext *context)
{
	context->position_error_sum = NULL;
	context->calibration_samples = NULL;
	context->candidate_linearization_lut = NULL;
}

static int16_t Encoder_Calib_Q15Difference(uint16_t target_q15, uint16_t source_q15)
{
	int32_t difference = (int32_t)target_q15 - (int32_t)source_q15;

	if (difference > ENCODER_Q15_HALF_TURN)
		difference -= (int32_t)ENCODER_Q15_CPR;
	else if (difference < -ENCODER_Q15_HALF_TURN)
		difference += (int32_t)ENCODER_Q15_CPR;

	return (int16_t)difference;
}

static bool Encoder_Calib_AllocateSamples(MotorCalibrationContext *context)
{
	context->position_error_sum = context->position_error_sum_storage;
	context->calibration_samples = context->calibration_samples_storage;
	context->candidate_linearization_lut = context->candidate_linearization_lut_storage;
	return true;
}

static bool Encoder_Calib_ClearSampleChunk(MotorCalibrationContext *context)
{
	uint16_t end_index = (uint16_t)(context->sample_clear_index +
		ENCODER_CALIB_CLEAR_ENTRIES_PER_CYCLE);
	uint16_t index;

	if (end_index > ENCODER_OFFSET_LUT_SIZE)
		end_index = ENCODER_OFFSET_LUT_SIZE;
	for (index = context->sample_clear_index; index < end_index; ++index)
	{
		context->position_error_sum[index] = 0;
		context->calibration_samples[index] = 0U;
		context->candidate_linearization_lut[index] = 0;
	}
	context->sample_clear_index = end_index;
	return end_index >= ENCODER_OFFSET_LUT_SIZE;
}

static uint16_t Encoder_Calib_ApplyCandidateLut(
	const MotorCalibrationContext *context, uint16_t directed_q15)
{
	uint16_t lut_index = directed_q15 >> 6;
	uint16_t fraction = directed_q15 & 0x003FU;
	int32_t correction_a = context->candidate_linearization_lut[lut_index];
	int32_t correction_b = context->candidate_linearization_lut[(lut_index + 1U) & (ENCODER_OFFSET_LUT_SIZE - 1U)];
	int32_t correction = correction_a + (((correction_b - correction_a) * fraction) >> 6);

	return (uint16_t)((int32_t)directed_q15 - correction);
}

static void Encoder_ObserverCalib_Abort(MotorCalibrationContext *context,
	CurrentControlContext *CurrentControl,
	MotorControlContext *MotorControl, PiController *SpeedController,
	SensorlessStartupContext *Startup, MotorStateContext *motor_state)
{
	Encoder_Calib_ReleaseSamples(context);
	SensorlessStartup_Reset(Startup);
	CurrentControlRuntime_ResetControllers(CurrentControl);
	PI_Controller_Reset(SpeedController);
	MotorControl->targets.speed_rad_s = 0.0f;
	MotorControl->runtime.speed_command_ramp_rad_s = 0.0f;
	MotorControl->targets.d_axis_current_a = 0.0f;
	MotorControl->targets.q_axis_current_a = 0.0f;
	context->step = CS_NULL;
	MotorLifecycle_ReportServiceFailed(motor_state);
	MotorState_DisableMotorDrive(motor_state);
}

static void Encoder_ObserverCalib_Finish(MotorCalibrationContext *context,
	CurrentControlContext *CurrentControl, MotorControlContext *MotorControl,
	PiController *SpeedController, SensorlessStartupContext *Startup,
	MotorStateContext *motor_state)
{
	SensorlessStartup_Reset(Startup);
	CurrentControlRuntime_ResetControllers(CurrentControl);
	PI_Controller_Reset(SpeedController);
	MotorControl->targets.speed_rad_s = 0.0f;
	MotorControl->runtime.speed_command_ramp_rad_s = 0.0f;
	MotorControl->targets.d_axis_current_a = 0.0f;
	MotorControl->targets.q_axis_current_a = 0.0f;
	context->step = CS_NULL;
	CurrentControlRuntime_ApplyHighSideZeroVector(CurrentControl);
	MotorLifecycle_ReportServiceComplete(motor_state, true);
}

typedef enum
{
	ENCODER_LUT_BUILD_RUNNING = 0,
	ENCODER_LUT_BUILD_COMPLETE,
	ENCODER_LUT_BUILD_FAILED
} EncoderLutBuildStatus;

static void Encoder_Calib_BeginLutBuild(MotorCalibrationContext *context)
{
	memset(&context->lut_build, 0, sizeof(context->lut_build));
}

static EncoderLutBuildStatus Encoder_Calib_BuildLutStep(
	MotorCalibrationContext *context, uint16_t minimum_samples,
	uint16_t bins_per_cycle)
{
	EncoderLutBuildContext *build = &context->lut_build;
	uint16_t bins_processed = 0U;

	if (context->position_error_sum == NULL ||
		context->calibration_samples == NULL ||
		context->candidate_linearization_lut == NULL ||
		bins_per_cycle == 0U)
		return ENCODER_LUT_BUILD_FAILED;

	while (bins_processed < bins_per_cycle &&
		build->index < ENCODER_OFFSET_LUT_SIZE)
	{
		uint16_t index = build->index;
		if (build->stage == 0U)
		{
			if (context->calibration_samples[index] < minimum_samples)
				return ENCODER_LUT_BUILD_FAILED;
		}
		else if (build->stage == 1U)
		{
			int32_t correction = (int16_t)(context->position_error_sum[index] /
				(int32_t)context->calibration_samples[index]);
			if (index > 0U)
			{
				while (correction - build->previous > ENCODER_Q15_HALF_TURN)
					correction -= (int32_t)ENCODER_Q15_CPR;
				while (correction - build->previous < -ENCODER_Q15_HALF_TURN)
					correction += (int32_t)ENCODER_Q15_CPR;
			}
			context->position_error_sum[index] = correction;
			build->previous = correction;
		}
		else if (build->stage == 2U)
			build->sum += context->position_error_sum[index];
		else
		{
			int32_t correction = context->position_error_sum[index] -
				build->shift;
			if (correction < INT16_MIN || correction > INT16_MAX)
				return ENCODER_LUT_BUILD_FAILED;
			context->candidate_linearization_lut[index] = (int16_t)correction;
		}
		build->index++;
		bins_processed++;
	}

	if (build->index < ENCODER_OFFSET_LUT_SIZE)
		return ENCODER_LUT_BUILD_RUNNING;
	if (build->stage >= 3U)
		return ENCODER_LUT_BUILD_COMPLETE;

	build->index = 0U;
	build->stage++;
	if (build->stage == 2U)
		build->sum = 0;
	else if (build->stage == 3U)
	{
		build->sum /= (int64_t)ENCODER_OFFSET_LUT_SIZE;
		build->shift = 0;
		while (build->sum > INT16_MAX)
		{
			build->sum -= (int32_t)ENCODER_Q15_CPR;
			build->shift += (int32_t)ENCODER_Q15_CPR;
		}
		while (build->sum < INT16_MIN)
		{
			build->sum += (int32_t)ENCODER_Q15_CPR;
			build->shift -= (int32_t)ENCODER_Q15_CPR;
		}
	}
	return ENCODER_LUT_BUILD_RUNNING;
}


static void Encoder_Calib_CommitCandidateLut(
	const MotorCalibrationContext *context, EncoderContext *Encoder)
{
	uint16_t current_linearized_q15;

	memcpy(Encoder->linearization_lut_q15, context->candidate_linearization_lut,
		sizeof(Encoder->linearization_lut_q15));
	current_linearized_q15 = Encoder_Calib_ApplyCandidateLut(context, Encoder->directed_q15);
	Encoder->linearized_q15 = current_linearized_q15;
	Encoder->previous_linearized_q15 = current_linearized_q15;
	Encoder->shadow_q15 = current_linearized_q15;
	Encoder->mechanical_zero_shadow_q15 = 0;
	Encoder->electrical_zero_q15 = 0U;
	Encoder->mechanical_zero_q15 = 0U;
	Encoder->calib_flag &= (uint8_t)~(ENC_CALIB_ELECTRICAL_ZERO |
		ENC_CALIB_MECHANICAL_ZERO | ENC_CALIB_COGGING);
	memset(Encoder->cogging_compensation_map_ma, 0,
		sizeof(Encoder->cogging_compensation_map_ma));
	Encoder->calib_flag |= ENC_CALIB_LINEARIZED;
	Encoder_ResetVelocity(Encoder);
}

static bool Encoder_ObserverCalib_IsStable(MotorControlContext *MotorControl,
	FluxObserverContext *Fluxobserver,
	SensorlessStartupContext *Startup, uint32_t position_epoch)
{
	float target_electrical_speed = SENSORLESS_ENCODER_CALIB_SPEED_MEC_RAD_S *
		(float)MotorControl->configuration.pole_pairs;
	float filtered_electrical_speed = Startup->speed_feedback *
		(float)MotorControl->configuration.pole_pairs;

	return Startup->state == SENSORLESS_STARTUP_CLOSED_LOOP &&
		FluxObserver_GetPositionEpoch(Fluxobserver) == position_epoch &&
		FastMath_Abs(filtered_electrical_speed - target_electrical_speed) <=
			FastMath_Abs(target_electrical_speed) * SENSORLESS_ENCODER_CALIB_SPEED_ERROR_RATIO &&
		FastMath_Abs(MotorControl->runtime.speed_command_ramp_rad_s - SENSORLESS_ENCODER_CALIB_SPEED_MEC_RAD_S) <=
			SENSORLESS_ENCODER_CALIB_SPEED_MEC_RAD_S * SENSORLESS_ENCODER_CALIB_SPEED_ERROR_RATIO;
}

static bool Encoder_ObserverCalib_IsTracking(FluxObserverContext *Fluxobserver,
	SensorlessStartupContext *Startup, uint32_t position_epoch)
{
	return Startup->state == SENSORLESS_STARTUP_CLOSED_LOOP &&
		FluxObserver_GetPositionEpoch(Fluxobserver) == position_epoch;
}

static float Encoder_ObserverCalib_GetStopSpeed(const MotorControlContext *MotorControl)
{
	float minimum_mechanical_speed = MotorControl->commissioning_tuning->angle.
		startup.minimum_electrical_velocity_rad_s /
		(float)MotorControl->configuration.pole_pairs;
	float stop_speed = minimum_mechanical_speed * SENSORLESS_ENCODER_CALIB_STOP_SPEED_MARGIN;

	return stop_speed < SENSORLESS_ENCODER_CALIB_SPEED_MEC_RAD_S ?
		stop_speed : SENSORLESS_ENCODER_CALIB_SPEED_MEC_RAD_S;
}

static void Encoder_Calib_Abort(MotorCalibrationContext *context,
	CurrentControlContext *CurrentControl)
{
	Encoder_Calib_ReleaseSamples(context);
	context->step = CS_NULL;
	CurrentControlRuntime_ApplyHighSideZeroVector(CurrentControl);
}

/**
 * @brief Build the 1024-point linearization LUT from a controlled rotating
 *        current vector and one forward mechanical sweep.
 *
 * The commanded electrical phase is the calibration reference.  This service
 * deliberately does not depend on the sensorless observer: the observer model
 * may not yet be trustworthy on a newly commissioned motor.  A later
 * electrical-zero calibration removes the constant rotor/load phase offset.
 */
void CalibrationRuntime_RunEncoderLinearization(MotorCalibrationContext *context,
	CurrentControlContext *CurrentControl, MotorControlContext *MotorControl,
	EncoderContext *Encoder, MotorStateContext *motor_state)
{
	float time = (float)context->encoder_linearization.loop_count *
		MotorControl->schedule.current_period_s;
	float theta_relative;
	float required_theta;

	if (!Encoder_IsOnline(Encoder))
	{
		MotorState_RaiseFault(motor_state, MOTOR_FAULT_ENCODER);
		Encoder_Calib_Abort(context, CurrentControl);
		return;
	}
	if (MotorControl->configuration.pole_pairs <= 0)
	{
		MotorState_RaiseFault(motor_state, MOTOR_FAULT_POLE_PAIRS);
		Encoder_Calib_Abort(context, CurrentControl);
		return;
	}
	if (MotorControl->configuration.phase_resistance_ohm <= 0.0f || MotorControl->configuration.flux_weber <= 0.0f ||
		MotorControl->configuration.calibration_current_a <= 0.0f ||
		MotorControl->configuration.calibration_current_a >
			MotorControl->configuration.current_limit_a)
	{
		MotorState_RaiseFault(motor_state, MOTOR_FAULT_INVALID_PARAMETER);
		Encoder_Calib_Abort(context, CurrentControl);
		return;
	}

	required_theta = MATH_TWO_PI * (float)MotorControl->configuration.pole_pairs;

	switch (context->step)
	{
		case CS_NULL:
			if (!Encoder_Calib_AllocateSamples(context))
			{
				MotorState_RaiseFault(motor_state, MOTOR_FAULT_ENCODER);
				Encoder_Calib_Abort(context, CurrentControl);
				return;
			}
			context->encoder_linearization.loop_count = 0U;
			context->sample_clear_index = 0U;
			context->step = CS_ENC_OFFSET_CLEAR_SAMPLES;
			break;

		case CS_ENC_OFFSET_CLEAR_SAMPLES:
			CurrentControlRuntime_ApplyHighSideZeroVector(CurrentControl);
			if (Encoder_Calib_ClearSampleChunk(context))
				context->step = CS_ENC_OFFSET_ALIGN;
			break;

		case CS_ENC_OFFSET_ALIGN:
			context->encoder_linearization.loop_count = 0U;
			context->encoder_linearization.drive_phase = 0.0f;
			context->encoder_linearization.drive_omega = 0.0f;
			context->encoder_linearization.sample_theta_start = 0.0f;
			context->encoder_linearization.sampling_started = false;
			context->encoder_linearization.observer_unlock_ticks = 0U;
			CurrentControlRuntime_ResetControllers(CurrentControl);
			MotorControl->targets.d_axis_current_a = 0.0f;
			MotorControl->targets.q_axis_current_a = 0.0f;
			CurrentControlRuntime_RunClosedLoop(CurrentControl, MotorControl, 0.0f, 0.0f);
			context->step = CS_ENC_OFFSET_ALIGN_LOOP;
			break;

		case CS_ENC_OFFSET_ALIGN_LOOP:
			MotorControl->targets.d_axis_current_a =
				MotorControl->configuration.calibration_current_a *
				FastMath_Clamp(time / ENCODER_LINEARIZATION_ALIGN_TIME_S, 0.0f, 1.0f);
			MotorControl->targets.q_axis_current_a = 0.0f;
			CurrentControlRuntime_RunClosedLoop(CurrentControl, MotorControl, 0.0f, 0.0f);
			if (time >= ENCODER_LINEARIZATION_ALIGN_TIME_S)
			{
				context->encoder_linearization.loop_count = 0U;
				context->step = CS_ENC_OFFSET_RAMP_CW;
			}
			break;

		case CS_ENC_OFFSET_RAMP_CW:
		case CS_ENC_OFFSET_SAMPLE_CW:
			if (context->step == CS_ENC_OFFSET_RAMP_CW)
			{
				context->encoder_linearization.drive_omega +=
					(ENCODER_LINEARIZATION_SPEED_ELEC_RAD_S /
					 ENCODER_LINEARIZATION_RAMP_TIME_S) *
					MotorControl->schedule.current_period_s;
				if (context->encoder_linearization.drive_omega >=
					ENCODER_LINEARIZATION_SPEED_ELEC_RAD_S)
				{
					context->encoder_linearization.drive_omega =
						ENCODER_LINEARIZATION_SPEED_ELEC_RAD_S;
					context->encoder_linearization.observer_unlock_ticks = 0U;
					context->step = CS_ENC_OFFSET_SAMPLE_CW;
				}
			}

			context->encoder_linearization.drive_phase +=
				context->encoder_linearization.drive_omega *
				MotorControl->schedule.current_period_s;
			MotorControl->targets.d_axis_current_a = MotorControl->configuration.calibration_current_a;
			MotorControl->targets.q_axis_current_a = 0.0f;
			CurrentControlRuntime_RunClosedLoop(CurrentControl, MotorControl,
				context->encoder_linearization.drive_phase,
				context->encoder_linearization.drive_omega);

			if (context->step != CS_ENC_OFFSET_SAMPLE_CW)
				break;

			if (Encoder->vel_elec <
				ENCODER_LINEARIZATION_SPEED_ELEC_RAD_S * 0.5f)
			{
				if (++context->encoder_linearization.observer_unlock_ticks >=
					(uint32_t)(ENCODER_LINEARIZATION_UNLOCK_TIMEOUT_S /
						MotorControl->schedule.current_period_s))
				{
					MotorState_RaiseFault(motor_state, MOTOR_FAULT_ENCODER);
					Encoder_Calib_Abort(context, CurrentControl);
				}
				break;
			}
			context->encoder_linearization.observer_unlock_ticks = 0U;

			if (!context->encoder_linearization.sampling_started)
			{
				context->encoder_linearization.sampling_started = true;
				context->encoder_linearization.sample_theta_start = context->encoder_linearization.drive_phase;
			}

			theta_relative = context->encoder_linearization.drive_phase -
				context->encoder_linearization.sample_theta_start;
			if (Encoder->read_status == ENCODER_READ_OK &&
				theta_relative >= 0.0f && theta_relative < required_theta)
			{
				uint16_t lut_index = Encoder->directed_q15 >> 6;
				uint16_t reference_q15 = (uint16_t)(theta_relative * ((float)ENCODER_Q15_CPR / required_theta));
				int16_t correction_q15 = (int16_t)(uint16_t)(Encoder->directed_q15 - reference_q15);
				if (context->calibration_samples[lut_index] < UINT16_MAX)
				{
					context->position_error_sum[lut_index] += correction_q15;
					context->calibration_samples[lut_index]++;
				}
			}
			else if (theta_relative >= required_theta)
			{
				Encoder_Calib_BeginLutBuild(context);
				context->step = CS_ENC_OFFSET_BUILD_LUT;
			}
			else if (time >= ENCODER_LINEARIZATION_TIMEOUT_FACTOR *
				required_theta / ENCODER_LINEARIZATION_SPEED_ELEC_RAD_S +
				ENCODER_LINEARIZATION_RAMP_TIME_S +
				ENCODER_LINEARIZATION_ALIGN_TIME_S)
			{
				MotorState_RaiseFault(motor_state, MOTOR_FAULT_ENCODER);
				Encoder_Calib_Abort(context, CurrentControl);
			}
			break;

		case CS_ENC_OFFSET_BUILD_LUT:
		{
			EncoderLutBuildStatus build_status;
			CurrentControlRuntime_ApplyHighSideZeroVector(CurrentControl);
			build_status = Encoder_Calib_BuildLutStep(context, 1U,
				SENSORLESS_ENCODER_CALIB_LUT_BUILD_BINS_PER_CYCLE);
			if (build_status == ENCODER_LUT_BUILD_FAILED)
			{
				MotorState_RaiseFault(motor_state, MOTOR_FAULT_ENCODER);
				Encoder_Calib_Abort(context, CurrentControl);
			}
			else if (build_status == ENCODER_LUT_BUILD_COMPLETE)
			{
				Encoder_Calib_CommitCandidateLut(context, Encoder);
				Encoder_Calib_ReleaseSamples(context);
				context->step = CS_NULL;
				MotorLifecycle_ReportServiceComplete(motor_state, true);
			}
			break;
		}

		default:
			Encoder_Calib_Abort(context, CurrentControl);
			break;
	}

	context->encoder_linearization.loop_count++;
}

/**
 * @brief Mode 13: calibrate the encoder linearization LUT from the sensorless observer.
 *        The static phase=0 alignment provides the mechanical reference origin for this power cycle.
 */
void CalibrationRuntime_RunEncoderObserver(MotorCalibrationContext *context,
	CurrentControlContext *CurrentControl, MotorControlContext *MotorControl,
	PiController *SpeedController, EncoderContext *Encoder,
	FluxObserverContext *Fluxobserver, SensorlessStartupContext *Startup,
	MotorStateContext *motor_state)
{
	float required_electrical_theta;
	float observer_position;
	float relative_theta;
	uint16_t reference_q15;

	if (!Encoder_IsOnline(Encoder))
	{
		MotorState_RaiseFault(motor_state, MOTOR_FAULT_ENCODER);
		Encoder_ObserverCalib_Abort(context, CurrentControl, MotorControl, SpeedController, Startup, motor_state);
		return;
	}
	if (MotorControl->configuration.pole_pairs <= 0)
	{
		MotorState_RaiseFault(motor_state, MOTOR_FAULT_POLE_PAIRS);
		Encoder_ObserverCalib_Abort(context, CurrentControl, MotorControl, SpeedController, Startup, motor_state);
		return;
	}
	if (MotorControl->configuration.phase_resistance_ohm <= 0.0f ||
		MotorControl->configuration.d_axis_inductance_h <= 0.0f ||
		MotorControl->configuration.q_axis_inductance_h <= 0.0f ||
		MotorControl->configuration.flux_weber <= 0.0f || MotorControl->configuration.current_limit_a <= 0.0f)
	{
		MotorState_RaiseFault(motor_state, MOTOR_FAULT_INVALID_PARAMETER);
		Encoder_ObserverCalib_Abort(context, CurrentControl, MotorControl, SpeedController, Startup, motor_state);
		return;
	}
	if (MotorControl->commissioning_tuning == 0 ||
		MotorControl->configuration.current_limit_a < MotorControl->
			commissioning_tuning->angle.startup.minimum_current_limit_a)
	{
		MotorState_RaiseFault(motor_state, MOTOR_FAULT_INVALID_PARAMETER);
		Encoder_ObserverCalib_Abort(context, CurrentControl, MotorControl,
			SpeedController, Startup, motor_state);
		return;
	}

	if (context->step == CS_NULL)
	{
		if (!Encoder_Calib_AllocateSamples(context))
		{
			MotorState_RaiseFault(motor_state, MOTOR_FAULT_ENCODER);
			Encoder_ObserverCalib_Abort(context, CurrentControl, MotorControl, SpeedController, Startup, motor_state);
			return;
		}
		context->sample_clear_index = 0U;
		context->step = CS_OBS_CLEAR_SAMPLES;
	}

	if (context->step == CS_OBS_CLEAR_SAMPLES)
	{
		CurrentControlRuntime_ApplyHighSideZeroVector(CurrentControl);
		if (!Encoder_Calib_ClearSampleChunk(context))
			return;

		SensorlessStartup_Reset(Startup);
		CurrentControlRuntime_ResetControllers(CurrentControl);
		PI_Controller_Reset(SpeedController);
		context->observer_calibration.state_ticks = 0U;
		context->observer_calibration.stage_ticks = 0U;
		context->observer_calibration.observer_position_epoch = 0U;
		context->observer_calibration.origin_anchor_q15 = 0U;
		context->observer_calibration.origin_sum_q15 = 0;
		context->observer_calibration.origin_sample_count = 0U;
		context->observer_calibration.origin_q15 = 0U;
		context->observer_calibration.previous_directed_q15 = 0U;
		context->observer_calibration.previous_observer_position = 0.0f;
		context->observer_calibration.observer_position_origin = 0.0f;
		context->observer_calibration.sample_previous_observer_position = 0.0f;
		context->observer_calibration.sample_valid_electrical_travel = 0.0f;
		context->observer_calibration.verify_previous_observer_position = 0.0f;
		context->observer_calibration.verify_valid_electrical_travel = 0.0f;
		context->observer_calibration.stop_start_speed = SENSORLESS_ENCODER_CALIB_SPEED_MEC_RAD_S;
		context->observer_calibration.stop_current_ref = 0.0f;
		context->observer_calibration.origin_negative_seen = false;
		context->observer_calibration.residual_squared_sum = 0U;
		context->observer_calibration.residual_sample_count = 0U;
		context->observer_calibration.residual_peak_abs_q15 = 0U;
		Encoder_Calib_BeginLutBuild(context);
		context->step = CS_OBS_ALIGN_ORIGIN;
	}

	if (context->step == CS_OBS_STOP_CURRENT)
	{
		float current_ratio;

		if (!Encoder_ObserverCalib_IsTracking(Fluxobserver, Startup, context->observer_calibration.observer_position_epoch))
		{
			/* The LUT is committed before stopping. A high-friction rotor may stop
			 * below the observer threshold; finish without discarding valid data. */
			Encoder_ObserverCalib_Finish(context, CurrentControl, MotorControl,
				SpeedController, Startup, motor_state);
			return;
		}

		current_ratio = FastMath_Clamp(((float)context->observer_calibration.state_ticks + 1.0f) * MotorControl->schedule.current_period_s /
			SENSORLESS_ENCODER_CALIB_STOP_CURRENT_RAMP_TIME_S, 0.0f, 1.0f);
		MotorControl->targets.d_axis_current_a = 0.0f;
		MotorControl->targets.q_axis_current_a = context->observer_calibration.stop_current_ref * (1.0f - current_ratio);
		CurrentControlRuntime_RunClosedLoop(CurrentControl, MotorControl, FluxObserver_GetElectricalAngle(Fluxobserver),
			FluxObserver_GetElectricalVelocity(Fluxobserver));
	}
	else
	{
		if (context->step == CS_OBS_STOP_DECEL)
			Startup->speed_pi_output_max = 0.0f;
		else
			Startup->speed_pi_output_max = 1.0f;

		if (context->step == CS_OBS_STOP_DECEL)
		{
			float decel_ratio = FastMath_Clamp((float)context->observer_calibration.state_ticks * MotorControl->schedule.current_period_s /
				SENSORLESS_ENCODER_CALIB_STOP_DECEL_TIME_S, 0.0f, 1.0f);
			float stop_speed = Encoder_ObserverCalib_GetStopSpeed(MotorControl);

			MotorControl->targets.speed_rad_s = context->observer_calibration.stop_start_speed +
				(stop_speed - context->observer_calibration.stop_start_speed) * decel_ratio;
			MotorControl->runtime.speed_command_ramp_rad_s = MotorControl->targets.speed_rad_s;
		}
		else
		{
			MotorControl->targets.speed_rad_s = SENSORLESS_ENCODER_CALIB_SPEED_MEC_RAD_S;
		}

		ControlModeRuntime_RunSensorlessSpeed(CurrentControl, MotorControl,
			SpeedController, Fluxobserver, Startup,
			&MotorControl->commissioning_tuning->angle.startup,
			motor_state);
		if (MotorControl->runtime.primary_fault != MOTOR_FAULT_NONE)
		{
			Encoder_ObserverCalib_Abort(context, CurrentControl, MotorControl, SpeedController, Startup, motor_state);
			return;
		}
	}

	required_electrical_theta = MATH_TWO_PI * (float)MotorControl->configuration.pole_pairs;
	observer_position = FluxObserver_GetUnwrappedElectricalPosition(Fluxobserver);

	switch (context->step)
	{
		case CS_OBS_ALIGN_ORIGIN:
			if (Startup->state == SENSORLESS_STARTUP_ALIGN &&
				Startup->state_ticks >= (uint32_t)(((MotorControl->commissioning_tuning->
					angle.startup.align_current_ramp_time_s +
					MotorControl->commissioning_tuning->angle.startup.align_hold_time_s) -
					SENSORLESS_ENCODER_CALIB_ALIGN_SAMPLE_TIME_S) /
					MotorControl->schedule.current_period_s))
			{
				int32_t unwrapped_q15;

				if (context->observer_calibration.origin_sample_count == 0U)
					context->observer_calibration.origin_anchor_q15 = Encoder->directed_q15;
				unwrapped_q15 = (int32_t)context->observer_calibration.origin_anchor_q15 +
					Encoder_Calib_Q15Difference(Encoder->directed_q15, context->observer_calibration.origin_anchor_q15);
				context->observer_calibration.origin_sum_q15 += unwrapped_q15;
				context->observer_calibration.origin_sample_count++;
			}

			if (Startup->state != SENSORLESS_STARTUP_ALIGN)
			{
				if (context->observer_calibration.origin_sample_count == 0U)
				{
					MotorState_RaiseFault(motor_state, MOTOR_FAULT_ENCODER);
					Encoder_ObserverCalib_Abort(context, CurrentControl, MotorControl, SpeedController, Startup, motor_state);
					return;
				}

				context->observer_calibration.origin_q15 = (uint16_t)(context->observer_calibration.origin_sum_q15 / (int64_t)context->observer_calibration.origin_sample_count);
				context->observer_calibration.state_ticks = 0U;
				context->observer_calibration.stage_ticks = 0U;
				context->step = CS_OBS_WAIT_CLOSED_LOOP;
			}
			break;

		case CS_OBS_WAIT_CLOSED_LOOP:
			if (Startup->state == SENSORLESS_STARTUP_CLOSED_LOOP)
			{
				context->observer_calibration.observer_position_epoch = FluxObserver_GetPositionEpoch(Fluxobserver);
				context->observer_calibration.state_ticks = 0U;
				context->observer_calibration.stage_ticks = 0U;
				context->step = CS_OBS_SPEED_STABLE;
			}
			else if (++context->observer_calibration.stage_ticks >= (uint32_t)(SENSORLESS_ENCODER_CALIB_STARTUP_TIMEOUT_S / MotorControl->schedule.current_period_s))
			{
				MotorState_RaiseFault(motor_state, MOTOR_FAULT_SENSORLESS);
				Encoder_ObserverCalib_Abort(context, CurrentControl, MotorControl, SpeedController, Startup, motor_state);
			}
			break;

		case CS_OBS_SPEED_STABLE:
			if (FluxObserver_GetPositionEpoch(Fluxobserver) != context->observer_calibration.observer_position_epoch)
			{
				MotorState_RaiseFault(motor_state, MOTOR_FAULT_SENSORLESS);
				Encoder_ObserverCalib_Abort(context, CurrentControl, MotorControl, SpeedController, Startup, motor_state);
				return;
			}

			if (Encoder_ObserverCalib_IsStable(MotorControl, Fluxobserver, Startup,
				context->observer_calibration.observer_position_epoch))
			{
				if (++context->observer_calibration.state_ticks >= (uint32_t)(SENSORLESS_ENCODER_CALIB_SPEED_STABLE_TIME_S / MotorControl->schedule.current_period_s))
				{
					context->observer_calibration.previous_directed_q15 = Encoder->directed_q15;
					context->observer_calibration.previous_observer_position = observer_position;
					context->observer_calibration.origin_negative_seen = false;
					context->observer_calibration.state_ticks = 0U;
					context->observer_calibration.stage_ticks = 0U;
					context->step = CS_OBS_FIND_ORIGIN;
				}
			}
			else
			{
				context->observer_calibration.state_ticks = 0U;
			}
			if (++context->observer_calibration.stage_ticks >= (uint32_t)(SENSORLESS_ENCODER_CALIB_SPEED_STABLE_TIMEOUT_S / MotorControl->schedule.current_period_s))
			{
				MotorState_RaiseFault(motor_state, MOTOR_FAULT_SENSORLESS);
				Encoder_ObserverCalib_Abort(context, CurrentControl, MotorControl, SpeedController, Startup, motor_state);
			}
			break;

		case CS_OBS_FIND_ORIGIN:
		{
			int16_t previous_relative = Encoder_Calib_Q15Difference(context->observer_calibration.previous_directed_q15, context->observer_calibration.origin_q15);
			int16_t current_relative = Encoder_Calib_Q15Difference(Encoder->directed_q15, context->observer_calibration.origin_q15);

			if (!Encoder_ObserverCalib_IsTracking(Fluxobserver, Startup, context->observer_calibration.observer_position_epoch))
			{
				MotorState_RaiseFault(motor_state, MOTOR_FAULT_SENSORLESS);
				Encoder_ObserverCalib_Abort(context, CurrentControl, MotorControl, SpeedController, Startup, motor_state);
				return;
			}

			if (current_relative < -64)
				context->observer_calibration.origin_negative_seen = true;

			if (context->observer_calibration.origin_negative_seen && previous_relative < 0 && current_relative >= 0)
			{
				float crossing_fraction = (float)(-previous_relative) /
					(float)(current_relative - previous_relative);

				context->observer_calibration.observer_position_origin = context->observer_calibration.previous_observer_position + crossing_fraction *
					(observer_position - context->observer_calibration.previous_observer_position);
				context->observer_calibration.sample_previous_observer_position = observer_position;
				context->observer_calibration.sample_valid_electrical_travel = 0.0f;
				context->observer_calibration.stage_ticks = 0U;
				context->step = CS_OBS_SAMPLE_CW;
				break;
			}

			context->observer_calibration.previous_directed_q15 = Encoder->directed_q15;
			context->observer_calibration.previous_observer_position = observer_position;
			if (++context->observer_calibration.stage_ticks >= (uint32_t)(SENSORLESS_ENCODER_CALIB_FIND_ORIGIN_TIMEOUT_S / MotorControl->schedule.current_period_s))
			{
				MotorState_RaiseFault(motor_state, MOTOR_FAULT_SENSORLESS);
				Encoder_ObserverCalib_Abort(context, CurrentControl, MotorControl, SpeedController, Startup, motor_state);
			}
			break;
		}

		case CS_OBS_SAMPLE_CW:
			relative_theta = observer_position - context->observer_calibration.observer_position_origin;
			if (!Encoder_ObserverCalib_IsTracking(Fluxobserver, Startup, context->observer_calibration.observer_position_epoch))
			{
				MotorState_RaiseFault(motor_state, MOTOR_FAULT_SENSORLESS);
				Encoder_ObserverCalib_Abort(context, CurrentControl, MotorControl, SpeedController, Startup, motor_state);
				return;
			}

			if (Encoder_ObserverCalib_IsStable(MotorControl, Fluxobserver, Startup,
				context->observer_calibration.observer_position_epoch))
			{
				float observer_delta = observer_position - context->observer_calibration.sample_previous_observer_position;

				if (observer_delta > 0.0f)
					context->observer_calibration.sample_valid_electrical_travel += observer_delta;

				if (relative_theta >= 0.0f)
				{
					uint16_t lut_index = Encoder->directed_q15 >> (16U - ENCODER_OFFSET_LUT_BITS);
					int32_t correction_q15;

					reference_q15 = (uint16_t)(relative_theta *
						((float)ENCODER_Q15_CPR / required_electrical_theta));
					correction_q15 = Encoder_Calib_Q15Difference(Encoder->directed_q15, reference_q15);
					if (context->calibration_samples[lut_index] != 0U)
					{
						int32_t average_q15 = context->position_error_sum[lut_index] /
							(int32_t)context->calibration_samples[lut_index];
						while (correction_q15 - average_q15 > ENCODER_Q15_HALF_TURN)
							correction_q15 -= (int32_t)ENCODER_Q15_CPR;
						while (correction_q15 - average_q15 < -ENCODER_Q15_HALF_TURN)
							correction_q15 += (int32_t)ENCODER_Q15_CPR;
					}

					if (context->calibration_samples[lut_index] < UINT16_MAX)
					{
						context->position_error_sum[lut_index] += correction_q15;
						context->calibration_samples[lut_index]++;
					}
				}
			}
			context->observer_calibration.sample_previous_observer_position = observer_position;

			if (context->observer_calibration.sample_valid_electrical_travel >=
				(float)SENSORLESS_ENCODER_CALIB_MECH_TURNS * required_electrical_theta)
			{
				Encoder_Calib_BeginLutBuild(context);
				context->step = CS_OBS_BUILD_LUT;
				break;
			}
			if (++context->observer_calibration.stage_ticks >= (uint32_t)(SENSORLESS_ENCODER_CALIB_SAMPLE_TIMEOUT_S / MotorControl->schedule.current_period_s))
			{
				MotorState_RaiseFault(motor_state, MOTOR_FAULT_SENSORLESS);
				Encoder_ObserverCalib_Abort(context, CurrentControl, MotorControl, SpeedController, Startup, motor_state);
			}
			break;

		case CS_OBS_BUILD_LUT:
		{
			EncoderLutBuildStatus build_status;
			if (!Encoder_ObserverCalib_IsTracking(Fluxobserver, Startup, context->observer_calibration.observer_position_epoch))
			{
				MotorState_RaiseFault(motor_state, MOTOR_FAULT_SENSORLESS);
				Encoder_ObserverCalib_Abort(context, CurrentControl, MotorControl, SpeedController, Startup, motor_state);
				return;
			}
			build_status = Encoder_Calib_BuildLutStep(context,
				SENSORLESS_ENCODER_CALIB_MIN_SAMPLES_PER_BIN,
				SENSORLESS_ENCODER_CALIB_LUT_BUILD_BINS_PER_CYCLE);
			if (build_status == ENCODER_LUT_BUILD_FAILED)
			{
				MotorState_RaiseFault(motor_state, MOTOR_FAULT_ENCODER);
				Encoder_ObserverCalib_Abort(context, CurrentControl, MotorControl,
					SpeedController, Startup, motor_state);
				return;
			}
			if (build_status == ENCODER_LUT_BUILD_COMPLETE)
			{
					context->observer_calibration.verify_previous_observer_position = observer_position;
					context->observer_calibration.verify_valid_electrical_travel = 0.0f;
					context->observer_calibration.residual_squared_sum = 0U;
					context->observer_calibration.residual_sample_count = 0U;
					context->observer_calibration.residual_peak_abs_q15 = 0U;
					context->observer_calibration.stage_ticks = 0U;
					context->step = CS_OBS_VERIFY_CW;
			}
			break;
		}

		case CS_OBS_VERIFY_CW:
			relative_theta = observer_position - context->observer_calibration.observer_position_origin;
			if (!Encoder_ObserverCalib_IsTracking(Fluxobserver, Startup, context->observer_calibration.observer_position_epoch))
			{
				MotorState_RaiseFault(motor_state, MOTOR_FAULT_SENSORLESS);
				Encoder_ObserverCalib_Abort(context, CurrentControl, MotorControl, SpeedController, Startup, motor_state);
				return;
			}

			if (Encoder_ObserverCalib_IsStable(MotorControl, Fluxobserver, Startup,
				context->observer_calibration.observer_position_epoch))
			{
				float observer_delta = observer_position - context->observer_calibration.verify_previous_observer_position;

				if (observer_delta > 0.0f)
					context->observer_calibration.verify_valid_electrical_travel += observer_delta;

				if (relative_theta >= 0.0f)
				{
					int16_t residual_q15;
					int32_t residual_abs_q15;
					uint16_t linearized_q15;

					reference_q15 = (uint16_t)(relative_theta *
						((float)ENCODER_Q15_CPR / required_electrical_theta));
					linearized_q15 = Encoder_Calib_ApplyCandidateLut(context, Encoder->directed_q15);
					residual_q15 = Encoder_Calib_Q15Difference(linearized_q15, reference_q15);
					residual_abs_q15 = residual_q15 >= 0 ? residual_q15 : -(int32_t)residual_q15;
					context->observer_calibration.residual_squared_sum += (uint64_t)((int64_t)residual_q15 *
						(int64_t)residual_q15);
					if ((uint32_t)residual_abs_q15 > context->observer_calibration.residual_peak_abs_q15)
						context->observer_calibration.residual_peak_abs_q15 = (uint32_t)residual_abs_q15;
					context->observer_calibration.residual_sample_count++;
				}
			}
			context->observer_calibration.verify_previous_observer_position = observer_position;

			if (context->observer_calibration.verify_valid_electrical_travel >=
				(float)SENSORLESS_ENCODER_CALIB_VERIFY_MECH_TURNS * required_electrical_theta)
			{
				uint64_t max_rms_squared = (uint64_t)SENSORLESS_ENCODER_CALIB_MAX_RMS_RESIDUAL_Q15 *
					(uint64_t)SENSORLESS_ENCODER_CALIB_MAX_RMS_RESIDUAL_Q15;

				if (context->observer_calibration.residual_sample_count == 0U || context->observer_calibration.residual_peak_abs_q15 >
					SENSORLESS_ENCODER_CALIB_MAX_PEAK_RESIDUAL_Q15 ||
					context->observer_calibration.residual_squared_sum > (uint64_t)context->observer_calibration.residual_sample_count * max_rms_squared)
				{
					MotorState_RaiseFault(motor_state, MOTOR_FAULT_ENCODER);
					Encoder_ObserverCalib_Abort(context, CurrentControl, MotorControl, SpeedController, Startup, motor_state);
					return;
				}

				Encoder_Calib_CommitCandidateLut(context, Encoder);
				Encoder_Calib_ReleaseSamples(context);
				context->observer_calibration.stop_start_speed = MotorControl->runtime.speed_command_ramp_rad_s;
				if (context->observer_calibration.stop_start_speed < Encoder_ObserverCalib_GetStopSpeed(MotorControl))
					context->observer_calibration.stop_start_speed = Encoder_ObserverCalib_GetStopSpeed(MotorControl);
				context->observer_calibration.state_ticks = 0U;
				context->step = CS_OBS_STOP_DECEL;
				break;
			}
			if (++context->observer_calibration.stage_ticks >= (uint32_t)(SENSORLESS_ENCODER_CALIB_VERIFY_TIMEOUT_S / MotorControl->schedule.current_period_s))
			{
				MotorState_RaiseFault(motor_state, MOTOR_FAULT_SENSORLESS);
				Encoder_ObserverCalib_Abort(context, CurrentControl, MotorControl, SpeedController, Startup, motor_state);
			}
			break;

		case CS_OBS_STOP_DECEL:
		{
			float stop_speed = Encoder_ObserverCalib_GetStopSpeed(MotorControl);
			float observer_mech_vel = FluxObserver_GetElectricalVelocity(Fluxobserver) /
				(float)MotorControl->configuration.pole_pairs;
			bool speed_reached = FastMath_Abs(observer_mech_vel) <= stop_speed *
				(1.0f + SENSORLESS_ENCODER_CALIB_STOP_SPEED_TOLERANCE_RATIO);

			context->observer_calibration.state_ticks++;
			if (context->observer_calibration.state_ticks >= (uint32_t)(SENSORLESS_ENCODER_CALIB_STOP_DECEL_TIME_S /
				MotorControl->schedule.current_period_s) && speed_reached)
			{
				context->observer_calibration.stop_current_ref = FastMath_Min(MotorControl->targets.q_axis_current_a, 0.0f);
				context->observer_calibration.state_ticks = 0U;
				context->step = CS_OBS_STOP_CURRENT;
			}
			else if (context->observer_calibration.state_ticks >= (uint32_t)(SENSORLESS_ENCODER_CALIB_STOP_DECEL_TIMEOUT_S /
				MotorControl->schedule.current_period_s))
			{
				MotorState_RaiseFault(motor_state, MOTOR_FAULT_SENSORLESS);
				Encoder_ObserverCalib_Abort(context, CurrentControl, MotorControl, SpeedController, Startup, motor_state);
			}
			break;
		}

		case CS_OBS_STOP_CURRENT:
			if (++context->observer_calibration.state_ticks >= (uint32_t)(SENSORLESS_ENCODER_CALIB_STOP_CURRENT_RAMP_TIME_S /
				MotorControl->schedule.current_period_s))
			{
				Encoder_ObserverCalib_Finish(context, CurrentControl, MotorControl,
					SpeedController, Startup, motor_state);
			}
			break;

		default:
			MotorState_RaiseFault(motor_state, MOTOR_FAULT_ENCODER);
			Encoder_ObserverCalib_Abort(context, CurrentControl, MotorControl, SpeedController, Startup, motor_state);
			break;
	}

}

