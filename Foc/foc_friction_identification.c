#include "foc_friction_identification.h"

#include <math.h>
#include <string.h>

#include "foc_errhandle.h"
#include "foc_param_profile.h"
#include "foc_run.h"
#include "hw_conf.h"
#include "utils.h"

#if PARAM_FRICTION_IDENT_SPEED_POINT_COUNT != 4U
#error "Update the FOC friction adapter when changing the profile point count"
#endif

static FrictionIdentification_TypeDef identification;
static uint32_t speed_loop_divider;
static bool session_started;

static FrictionIdentificationConfig_TypeDef BuildConfig(void)
{
	FrictionIdentificationConfig_TypeDef config;

	memset(&config, 0, sizeof(config));
	config.update_period_s = Speed_Ts;
	config.speed_point_count = PARAM_FRICTION_IDENT_SPEED_POINT_COUNT;
	config.speed_points_rad_s[0] = PARAM_FRICTION_IDENT_SPEED_0_RPS * _2PI;
	config.speed_points_rad_s[1] = PARAM_FRICTION_IDENT_SPEED_1_RPS * _2PI;
	config.speed_points_rad_s[2] = PARAM_FRICTION_IDENT_SPEED_2_RPS * _2PI;
	config.speed_points_rad_s[3] = PARAM_FRICTION_IDENT_SPEED_3_RPS * _2PI;
	config.stable_time_s = PARAM_FRICTION_IDENT_STABLE_TIME_S;
	config.track_timeout_s = PARAM_FRICTION_IDENT_TRACK_TIMEOUT_S;
	config.sample_timeout_s = PARAM_FRICTION_IDENT_SAMPLE_TIMEOUT_S;
	config.stop_hold_time_s = PARAM_FRICTION_IDENT_STOP_HOLD_TIME_S;
	config.stop_timeout_s = PARAM_FRICTION_IDENT_STOP_TIMEOUT_S;
	config.speed_tolerance_ratio = PARAM_FRICTION_IDENT_SPEED_TOLERANCE_RATIO;
	config.minimum_speed_tolerance_rad_s = PARAM_FRICTION_IDENT_MIN_SPEED_TOL_RAD_S;
	config.stop_speed_rad_s = PARAM_FRICTION_IDENT_STOP_SPEED_RAD_S;
	config.sample_turns = PARAM_FRICTION_IDENT_SAMPLE_TURNS;
	config.minimum_sample_time_s = PARAM_FRICTION_IDENT_MIN_SAMPLE_TIME_S;
	config.saturation_time_s = PARAM_FRICTION_IDENT_SATURATION_TIME_S;
	config.rmse_floor_a = PARAM_FRICTION_IDENT_RMSE_FLOOR_A;
	config.rmse_ratio_max = PARAM_FRICTION_IDENT_RMSE_RATIO_MAX;
	return config;
}

static void StopOutput(MotorControl_TypeDef *motor,
	PI_Controller_TypeDef *speed_controller)
{
	if (motor == NULL || speed_controller == NULL)
		return;
	motor->speedRef = 0.0f;
	motor->speedShadow = 0.0f;
	motor->idRef = 0.0f;
	motor->iqRef = 0.0f;
	PI_Controller_Reset(speed_controller);
}

static bool RuntimeConfigIsValid(const MotorControl_TypeDef *motor)
{
	const float maximum_speed = PARAM_FRICTION_IDENT_SPEED_3_RPS * _2PI;

	return motor != NULL && isfinite(motor->current_limit) &&
		motor->current_limit > 0.0f && isfinite(motor->speed_limit) &&
		motor->speed_limit >= maximum_speed && isfinite(motor->speed_Kp) &&
		motor->speed_Kp > 0.0f && isfinite(motor->speed_Ki) &&
		motor->speed_Ki >= 0.0f;
}

static void Fail(MotorControl_TypeDef *motor,
	PI_Controller_TypeDef *speed_controller,
	FrictionIdentificationReason_TypeDef reason, bool report_module_error)
{
	FrictionIdentification_Fail(&identification, reason);
	StopOutput(motor, speed_controller);
	if (report_module_error)
		Set_ErrorNow(FrictionIdentification_Error);
}

void FocFrictionIdentification_Init(void)
{
	FrictionIdentificationConfig_TypeDef config = BuildConfig();

	(void)FrictionIdentification_Init(&identification, &config);
	speed_loop_divider = 0U;
	session_started = false;
}

bool FocFrictionIdentification_Start(MotorControl_TypeDef *motor,
	Encoder_TypeDef *encoder, PI_Controller_TypeDef *speed_controller)
{
	FocFrictionIdentification_Init();
	if (encoder == NULL || speed_controller == NULL ||
		!RuntimeConfigIsValid(motor) || !Encoder_IsOnline(encoder) ||
		(encoder->calib_flag & ENC_CALIB_ALL) != ENC_CALIB_ALL ||
		!FrictionIdentification_Start(&identification))
	{
		FrictionIdentification_Fail(&identification,
			FRICTION_IDENT_REASON_INVALID_CONFIG);
		return false;
	}
	StopOutput(motor, speed_controller);
	motor->speedRef = FrictionIdentification_GetTargetSpeed(&identification);
	session_started = true;
	return true;
}

void FocFrictionIdentification_Abort(MotorControl_TypeDef *motor,
	PI_Controller_TypeDef *speed_controller)
{
	FrictionIdentification_Abort(&identification);
	StopOutput(motor, speed_controller);
	session_started = false;
}

void FocFrictionIdentification_Task(FOC_TypeDef *foc, MotorControl_TypeDef *motor,
	PI_Controller_TypeDef *speed_controller, Encoder_TypeDef *encoder)
{
	FrictionIdentificationInput_TypeDef input;
	FrictionIdentificationState_TypeDef state;

	if (foc == NULL || speed_controller == NULL || encoder == NULL ||
		!RuntimeConfigIsValid(motor))
	{
		Fail(motor, speed_controller, FRICTION_IDENT_REASON_INVALID_CONFIG, true);
		return;
	}
	if (!Encoder_IsOnline(encoder))
	{
		Set_ErrorNow(Encoder_Error);
		Fail(motor, speed_controller, FRICTION_IDENT_REASON_SAFETY_FAULT, false);
		return;
	}
	if ((encoder->calib_flag & ENC_CALIB_ALL) != ENC_CALIB_ALL)
	{
		Set_ErrorNow(Encoder_NotCalibrated);
		Fail(motor, speed_controller, FRICTION_IDENT_REASON_SAFETY_FAULT, false);
		return;
	}
	/* Own the lifecycle here so mode 19 also works when ModeNow is changed by
	 * a debugger or another integration path that bypasses ModeSwitch_Handle. */
	if (!session_started &&
		!FocFrictionIdentification_Start(motor, encoder, speed_controller))
	{
		Set_ErrorNow(FrictionIdentification_Error);
		return;
	}

	if (foc->Vbus_filt > 30.0f)
		Set_ErrorNow(Over_Voltage);
	else if (foc->Vbus_filt < 10.0f)
		Set_ErrorNow(Under_Voltage);
	else if (foc->temp >= 100.0f)
		Set_ErrorNow(High_Temprature);
	if (motor->ErrorNow != No_Error)
	{
		Fail(motor, speed_controller, FRICTION_IDENT_REASON_SAFETY_FAULT, false);
		return;
	}

	Task_Speed_Mode(foc, motor, speed_controller, encoder);
	if (++speed_loop_divider < SPEED_LOOP_DIVIDER)
		return;
	speed_loop_divider = 0U;

	input.measured_speed_rad_s = Encoder_GetMecVel(encoder);
	input.ramped_speed_reference_rad_s = motor->speedShadow;
	input.iq_a = foc->Iq_filt;
	input.mechanical_position_rad = encoder->theta_mech;
	input.current_saturated = fast_abs(motor->iqRef) >=
		motor->current_limit * PARAM_FRICTION_IDENT_CURRENT_RATIO_MAX;
	FrictionIdentification_Update(&identification, &input);
	motor->speedRef = FrictionIdentification_GetTargetSpeed(&identification);

	state = FrictionIdentification_GetState(&identification);
	if (state == FRICTION_IDENT_FAILED)
	{
		StopOutput(motor, speed_controller);
		if (motor->ErrorNow == No_Error)
			Set_ErrorNow(FrictionIdentification_Error);
	}
	else if (state == FRICTION_IDENT_COMPLETE)
	{
		StopOutput(motor, speed_controller);
		Set_ModeNow(Motor_Disable);
	}
}

bool FocFrictionIdentification_ApplyCandidate(MotorControl_TypeDef *motor)
{
	const FrictionIdentificationResult_TypeDef *result =
		FrictionIdentification_GetResult(&identification);

	if (motor == NULL || result == NULL || !result->valid ||
		FrictionIdentification_GetState(&identification) != FRICTION_IDENT_COMPLETE ||
		motor->ModeNow != Motor_Disable)
		return false;

	motor->friction_coulomb_pos_a = result->coulomb_pos_a;
	motor->friction_coulomb_neg_a = result->coulomb_neg_a;
	motor->friction_viscous_pos_a_per_rad_s =
		result->viscous_pos_a_per_rad_s;
	motor->friction_viscous_neg_a_per_rad_s =
		result->viscous_neg_a_per_rad_s;
	motor->friction_model_valid = true;
	return true;
}

FrictionIdentificationState_TypeDef FocFrictionIdentification_GetState(void)
{
	return FrictionIdentification_GetState(&identification);
}

FrictionIdentificationReason_TypeDef FocFrictionIdentification_GetReason(void)
{
	return FrictionIdentification_GetReason(&identification);
}

uint32_t FocFrictionIdentification_GetPointIndex(void)
{
	return FrictionIdentification_GetPointIndex(&identification);
}

float FocFrictionIdentification_GetProgressPercent(void)
{
	return FrictionIdentification_GetProgressPercent(&identification);
}

uint32_t FocFrictionIdentification_GetSampleCount(void)
{
	return FrictionIdentification_GetSampleCount(&identification);
}

const FrictionIdentificationResult_TypeDef *FocFrictionIdentification_GetResult(void)
{
	return FrictionIdentification_GetResult(&identification);
}

const FrictionIdentificationSample_TypeDef *FocFrictionIdentification_GetSamples(void)
{
	return FrictionIdentification_GetSamples(&identification);
}
