#include "friction_identification_runtime.h"

#include <math.h>
#include <string.h>

#include "fast_math.h"
#include "motor_state_runtime.h"

static void StopOutput(CurrentControlContext *current_control,
	MotorControlContext *motor, PiController *speed_controller)
{
	if (motor != 0)
	{
		motor->targets.speed_rad_s = 0.0f;
		motor->targets.d_axis_current_a = 0.0f;
		motor->targets.q_axis_current_a = 0.0f;
		motor->runtime.speed_command_ramp_rad_s = 0.0f;
	}
	if (speed_controller != 0)
		PI_Controller_Reset(speed_controller);
	if (current_control != 0)
	{
		CurrentControlRuntime_ResetControllers(current_control);
		CurrentControlRuntime_ApplyHighSideZeroVector(current_control);
	}
}

static bool BuildConfig(FrictionIdentificationConfig *config,
	const EncoderContext *encoder, const MechanicalLoadProfile *profile)
{
	uint32_t index;
	if (config == 0 || encoder == 0 || profile == 0 ||
		profile->friction_identification_speed_point_count == 0U ||
		profile->friction_identification_speed_point_count >
		FRICTION_IDENTIFICATION_MAX_SPEED_POINTS ||
		encoder->velocity_sample_period_s <= 0.0f)
		return false;
	memset(config, 0, sizeof(*config));
	config->update_period_s = encoder->velocity_sample_period_s;
	config->speed_point_count = profile->friction_identification_speed_point_count;
	for (index = 0U; index < config->speed_point_count; index++)
		config->speed_points_rad_s[index] =
			profile->friction_identification_speed_points_rad_s[index];
	config->stable_time_s = profile->friction_identification_stable_time_s;
	config->track_timeout_s = profile->friction_identification_track_timeout_s;
	config->sample_timeout_s = profile->friction_identification_sample_timeout_s;
	config->stop_hold_time_s = profile->friction_identification_stop_hold_time_s;
	config->stop_timeout_s = profile->friction_identification_stop_timeout_s;
	config->speed_tolerance_ratio =
		profile->friction_identification_speed_tolerance_ratio;
	config->minimum_speed_tolerance_rad_s =
		profile->friction_identification_minimum_speed_tolerance_rad_s;
	config->stop_speed_rad_s = profile->friction_identification_stop_speed_rad_s;
	config->sample_turns = profile->friction_identification_sample_turns;
	config->minimum_sample_time_s =
		profile->friction_identification_minimum_sample_time_s;
	config->saturation_time_s = profile->friction_identification_saturation_time_s;
	config->rmse_floor_a = profile->friction_identification_rmse_floor_a;
	config->rmse_ratio_max = profile->friction_identification_rmse_ratio_max;
	return true;
}

static bool RuntimeConfigIsValid(const MotorControlContext *motor,
	const EncoderContext *encoder, const MechanicalLoadProfile *profile)
{
	uint32_t last_index;
	if (motor == 0 || encoder == 0 || profile == 0 ||
		profile->friction_identification_speed_point_count == 0U ||
		profile->friction_identification_speed_point_count >
		FRICTION_IDENTIFICATION_MAX_SPEED_POINTS)
		return false;
	last_index = profile->friction_identification_speed_point_count - 1U;
	return isfinite(motor->configuration.current_limit_a) &&
		motor->configuration.current_limit_a > 0.0f &&
		isfinite(motor->configuration.speed_limit_rad_s) &&
		motor->configuration.speed_limit_rad_s >=
			profile->friction_identification_speed_points_rad_s[last_index] &&
		isfinite(motor->configuration.speed_kp) && motor->configuration.speed_kp > 0.0f &&
		isfinite(motor->configuration.speed_ki) && motor->configuration.speed_ki >= 0.0f &&
		Encoder_IsOnline(encoder) &&
		(encoder->calib_flag & ENC_CALIB_ALL) == ENC_CALIB_ALL;
}

FrictionIdentificationState FrictionIdentificationRuntime_Run(
	FrictionIdentificationRuntimeContext *context,
	MotionControlContext *motion, CurrentControlContext *current_control,
	MotorControlContext *motor, PiController *speed_controller,
	EncoderContext *encoder,
	const MechanicalLoadProfile *load_profile)
{
	FrictionIdentificationConfig config;
	FrictionIdentificationInput input;
	FrictionIdentificationState state;
	if (context == 0 || motion == 0 || current_control == 0 || motor == 0 ||
		speed_controller == 0 || encoder == 0 || load_profile == 0)
		return FRICTION_IDENT_FAILED;
	if (!RuntimeConfigIsValid(motor, encoder, load_profile))
	{
		FrictionIdentification_Fail(&context->core,
			FRICTION_IDENT_REASON_INVALID_CONFIG);
		StopOutput(current_control, motor, speed_controller);
		return FRICTION_IDENT_FAILED;
	}
	if (!context->started)
	{
		if (!BuildConfig(&config, encoder, load_profile) ||
			!FrictionIdentification_Init(&context->core, &config) ||
			!FrictionIdentification_Start(&context->core))
		{
			StopOutput(current_control, motor, speed_controller);
			return FRICTION_IDENT_FAILED;
		}
		context->started = true;
		motion->speed_loop_count = 0U;
		motor->runtime.speed_command_ramp_rad_s = 0.0f;
		PI_Controller_Reset(speed_controller);
	}

	motor->targets.speed_rad_s =
		FrictionIdentification_GetTargetSpeed(&context->core);
	ControlModeRuntime_RunSpeed(motion, current_control, motor,
		speed_controller, encoder);
	if (motion->speed_loop_count != 0U)
		return FrictionIdentification_GetState(&context->core);

	input.measured_speed_rad_s = Encoder_GetMecVel(encoder);
	input.ramped_speed_reference_rad_s = motor->runtime.speed_command_ramp_rad_s;
	input.iq_a = current_control->filtered_q_axis_current_a;
	input.mechanical_position_rad = Encoder_GetMecPos(encoder);
	input.current_saturated = FastMath_Abs(motor->targets.q_axis_current_a) >=
		motor->configuration.current_limit_a *
		load_profile->friction_identification_current_ratio_max;
	FrictionIdentification_Update(&context->core, &input);
	motor->targets.speed_rad_s =
		FrictionIdentification_GetTargetSpeed(&context->core);
	state = FrictionIdentification_GetState(&context->core);
	if (state == FRICTION_IDENT_COMPLETE || state == FRICTION_IDENT_FAILED)
		StopOutput(current_control, motor, speed_controller);
	return state;
}

void FrictionIdentificationRuntime_Cancel(
	FrictionIdentificationRuntimeContext *context,
	CurrentControlContext *current_control, MotorControlContext *motor,
	PiController *speed_controller)
{
	if (context == 0) return;
	if (context->started)
	{
		FrictionIdentification_Abort(&context->core);
		StopOutput(current_control, motor, speed_controller);
	}
	context->started = false;
}

static bool ReadStatus(void *opaque, FrictionIdentificationPortStatus *status)
{
	FrictionIdentificationRuntimeContext *context =
		(FrictionIdentificationRuntimeContext *)opaque;
	const FrictionIdentificationResult *result;
	BspCriticalSectionToken irq_state;
	if (context == 0 || status == 0 || !context->port_initialized) return false;
	irq_state = context->critical_section.enter(context->critical_section.context);
	result = FrictionIdentification_GetResult(&context->core);
	status->state = (uint8_t)FrictionIdentification_GetState(&context->core);
	status->reason = (uint8_t)FrictionIdentification_GetReason(&context->core);
	status->point_index = (uint8_t)FrictionIdentification_GetPointIndex(&context->core);
	status->sample_count = (uint8_t)FrictionIdentification_GetSampleCount(&context->core);
	status->progress_percent = FrictionIdentification_GetProgressPercent(&context->core);
	status->candidate_coulomb_pos_a = result->coulomb_pos_a;
	status->candidate_coulomb_neg_a = result->coulomb_neg_a;
	status->candidate_viscous_pos_a_per_rad_s = result->viscous_pos_a_per_rad_s;
	status->candidate_viscous_neg_a_per_rad_s = result->viscous_neg_a_per_rad_s;
	status->candidate_rmse_pos_a = result->rmse_pos_a;
	status->candidate_rmse_neg_a = result->rmse_neg_a;
	status->candidate_valid = result->valid;
	status->active_coulomb_pos_a = context->motor->configuration.friction_coulomb_pos_a;
	status->active_coulomb_neg_a = context->motor->configuration.friction_coulomb_neg_a;
	status->active_viscous_pos_a_per_rad_s =
		context->motor->configuration.friction_viscous_pos_a_per_rad_s;
	status->active_viscous_neg_a_per_rad_s =
		context->motor->configuration.friction_viscous_neg_a_per_rad_s;
	status->active_model_valid = context->motor->configuration.friction_model_valid;
	context->critical_section.exit(context->critical_section.context, irq_state);
	return true;
}

static bool ReadSample(void *opaque, uint8_t index,
	FrictionIdentificationPortSample *sample)
{
	FrictionIdentificationRuntimeContext *context =
		(FrictionIdentificationRuntimeContext *)opaque;
	const FrictionIdentificationSample *samples;
	BspCriticalSectionToken irq_state;
	if (context == 0 || sample == 0 || !context->port_initialized ||
		index >= FrictionIdentification_GetSampleCount(&context->core)) return false;
	irq_state = context->critical_section.enter(context->critical_section.context);
	samples = FrictionIdentification_GetSamples(&context->core);
	sample->target_speed_rad_s = samples[index].target_speed_rad_s;
	sample->mean_speed_rad_s = samples[index].mean_speed_rad_s;
	sample->mean_iq_a = samples[index].mean_iq_a;
	sample->sample_count = samples[index].sample_count;
	context->critical_section.exit(context->critical_section.context, irq_state);
	return true;
}

static bool ApplyCandidate(void *opaque)
{
	FrictionIdentificationRuntimeContext *context =
		(FrictionIdentificationRuntimeContext *)opaque;
	const FrictionIdentificationResult *result;
	FrictionIdentificationResult candidate;
	BspCriticalSectionToken irq_state;
	if (context == 0 || !context->port_initialized ||
		MotorLifecycle_GetDeviceState(context->motor_state) != DEVICE_STATE_STANDBY ||
		FrictionIdentification_GetState(&context->core) != FRICTION_IDENT_COMPLETE)
		return false;
	irq_state = context->critical_section.enter(context->critical_section.context);
	result = FrictionIdentification_GetResult(&context->core);
	if (!result->valid)
	{
		context->critical_section.exit(context->critical_section.context, irq_state);
		return false;
	}
	candidate = *result;
	context->critical_section.exit(context->critical_section.context, irq_state);
	return MotorServiceAdapter_StageFrictionModel(
		context->configuration_adapter, candidate.coulomb_pos_a,
		candidate.coulomb_neg_a, candidate.viscous_pos_a_per_rad_s,
		candidate.viscous_neg_a_per_rad_s);
}

FrictionIdentificationPort FrictionIdentificationRuntime_CreatePort(
	FrictionIdentificationRuntimeContext *context, MotorControlContext *motor,
	MotorConfigurationAdapterContext *configuration_adapter,
	const BspCriticalSectionPort *critical_section,
	MotorStateContext *motor_state)
{
	FrictionIdentificationPort port = {0};
	if (context == 0 || motor == 0 || configuration_adapter == 0 ||
		critical_section == 0 || motor_state == 0 ||
		critical_section->enter == 0 || critical_section->exit == 0) return port;
	context->motor = motor;
	context->configuration_adapter = configuration_adapter;
	context->motor_state = motor_state;
	context->critical_section = *critical_section;
	context->port_initialized = true;
	port.context = context;
	port.read_status = ReadStatus;
	port.read_sample = ReadSample;
	port.apply_candidate = ApplyCandidate;
	return port;
}
