#include "friction_identification.h"

#include <math.h>
#include <string.h>

#define FRICTION_TWO_PI 6.28318530717958647692f

static float AbsValue(float value) { return value >= 0.0f ? value : -value; }
static float MaxValue(float first, float second) { return first >= second ? first : second; }

static bool IsActive(const FrictionIdentificationContext *identification)
{
	return identification->state == FRICTION_IDENT_TRACKING ||
		identification->state == FRICTION_IDENT_SAMPLING ||
		identification->state == FRICTION_IDENT_STOPPING;
}

static bool ConfigIsValid(const FrictionIdentificationConfig *config)
{
	uint32_t index;
	if (config == NULL || !isfinite(config->update_period_s) ||
		config->update_period_s <= 0.0f || config->speed_point_count < 2U ||
		config->speed_point_count > FRICTION_IDENTIFICATION_MAX_SPEED_POINTS ||
		!isfinite(config->stable_time_s) || config->stable_time_s <= 0.0f ||
		!isfinite(config->track_timeout_s) ||
		config->track_timeout_s <= config->stable_time_s ||
		!isfinite(config->sample_timeout_s) || config->sample_timeout_s <= 0.0f ||
		!isfinite(config->stop_hold_time_s) || config->stop_hold_time_s <= 0.0f ||
		!isfinite(config->stop_timeout_s) ||
		config->stop_timeout_s <= config->stop_hold_time_s ||
		!isfinite(config->speed_tolerance_ratio) || config->speed_tolerance_ratio <= 0.0f ||
		!isfinite(config->minimum_speed_tolerance_rad_s) ||
		config->minimum_speed_tolerance_rad_s <= 0.0f ||
		!isfinite(config->stop_speed_rad_s) || config->stop_speed_rad_s <= 0.0f ||
		!isfinite(config->sample_turns) || config->sample_turns <= 0.0f ||
		!isfinite(config->minimum_sample_time_s) || config->minimum_sample_time_s <= 0.0f ||
		!isfinite(config->saturation_time_s) || config->saturation_time_s <= 0.0f ||
		!isfinite(config->rmse_floor_a) || config->rmse_floor_a < 0.0f ||
		!isfinite(config->rmse_ratio_max) || config->rmse_ratio_max <= 0.0f)
		return false;
	for (index = 0U; index < config->speed_point_count; ++index)
	{
		if (!isfinite(config->speed_points_rad_s[index]) ||
			config->speed_points_rad_s[index] <= 0.0f ||
			(index > 0U && config->speed_points_rad_s[index] <=
			 config->speed_points_rad_s[index - 1U]))
			return false;
	}
	return true;
}

static uint32_t SecondsToTicks(const FrictionIdentificationContext *identification,
	float seconds)
{
	float ticks = seconds / identification->config.update_period_s;
	if (!isfinite(ticks) || ticks <= 1.0f) return 1U;
	if (ticks >= 4294967040.0f) return UINT32_MAX;
	return (uint32_t)(ticks + 0.5f);
}

static float TargetForPoint(const FrictionIdentificationContext *identification,
	uint32_t point_index)
{
	float direction = (point_index & 1U) == 0U ? 1.0f : -1.0f;
	return direction * identification->config.speed_points_rad_s[point_index / 2U];
}

static float SpeedTolerance(const FrictionIdentificationContext *identification,
	float target_speed)
{
	return MaxValue(AbsValue(target_speed) * identification->config.speed_tolerance_ratio,
		identification->config.minimum_speed_tolerance_rad_s);
}

static bool FitDirection(FrictionIdentificationContext *identification, bool positive,
	float *coulomb_a, float *viscous_a_per_rad_s, float *rmse_a)
{
	float sum_x = 0.0f, sum_y = 0.0f, sum_xx = 0.0f, sum_xy = 0.0f;
	float error_squared_sum = 0.0f, denominator, intercept, slope, mean_y;
	uint32_t count = 0U, index;
	uint32_t sample_count = FrictionIdentification_GetSampleCount(identification);

	for (index = positive ? 0U : 1U; index < sample_count; index += 2U)
	{
		float x = AbsValue(identification->samples[index].mean_speed_rad_s);
		float y = positive ? identification->samples[index].mean_iq_a :
			-identification->samples[index].mean_iq_a;
		if (!isfinite(x) || !isfinite(y) || x <= 0.0f || y <= 0.0f ||
			identification->samples[index].sample_count == 0U)
			return false;
		sum_x += x; sum_y += y; sum_xx += x * x; sum_xy += x * y; count++;
	}
	if (count < 2U) return false;
	denominator = (float)count * sum_xx - sum_x * sum_x;
	if (!isfinite(denominator) || denominator <= 1e-9f) return false;
	slope = ((float)count * sum_xy - sum_x * sum_y) / denominator;
	intercept = (sum_y - slope * sum_x) / (float)count;
	if (!isfinite(intercept) || !isfinite(slope)) return false;
	if (slope < 0.0f) { slope = 0.0f; intercept = sum_y / (float)count; }
	if (intercept < 0.0f) { intercept = 0.0f; slope = sum_xx > 0.0f ? sum_xy / sum_xx : 0.0f; }
	if (!isfinite(intercept) || !isfinite(slope) || intercept < 0.0f || slope < 0.0f)
		return false;
	for (index = positive ? 0U : 1U; index < sample_count; index += 2U)
	{
		float x = AbsValue(identification->samples[index].mean_speed_rad_s);
		float y = positive ? identification->samples[index].mean_iq_a :
			-identification->samples[index].mean_iq_a;
		float error = y - (intercept + slope * x);
		error_squared_sum += error * error;
	}
	*rmse_a = sqrtf(error_squared_sum / (float)count);
	mean_y = sum_y / (float)count;
	if (!isfinite(*rmse_a) || *rmse_a > MaxValue(identification->config.rmse_floor_a,
		identification->config.rmse_ratio_max * mean_y))
		return false;
	*coulomb_a = intercept;
	*viscous_a_per_rad_s = slope;
	return true;
}

static bool Fit(FrictionIdentificationContext *identification)
{
	bool positive_valid = FitDirection(identification, true,
		&identification->result.coulomb_pos_a,
		&identification->result.viscous_pos_a_per_rad_s,
		&identification->result.rmse_pos_a);
	bool negative_valid = FitDirection(identification, false,
		&identification->result.coulomb_neg_a,
		&identification->result.viscous_neg_a_per_rad_s,
		&identification->result.rmse_neg_a);
	identification->result.valid = positive_valid && negative_valid;
	return identification->result.valid;
}

bool FrictionIdentification_Init(FrictionIdentificationContext *identification,
	const FrictionIdentificationConfig *config)
{
	if (identification == NULL) return false;
	memset(identification, 0, sizeof(*identification));
	if (!ConfigIsValid(config))
	{
		identification->state = FRICTION_IDENT_FAILED;
		identification->reason = FRICTION_IDENT_REASON_INVALID_CONFIG;
		return false;
	}
	identification->config = *config;
	identification->state = FRICTION_IDENT_IDLE;
	return true;
}

void FrictionIdentification_Reset(FrictionIdentificationContext *identification)
{
	FrictionIdentificationConfig config;
	if (identification == NULL) return;
	config = identification->config;
	memset(identification, 0, sizeof(*identification));
	identification->config = config;
	identification->state = ConfigIsValid(&config) ? FRICTION_IDENT_IDLE : FRICTION_IDENT_FAILED;
	if (identification->state == FRICTION_IDENT_FAILED)
		identification->reason = FRICTION_IDENT_REASON_INVALID_CONFIG;
}

bool FrictionIdentification_Start(FrictionIdentificationContext *identification)
{
	if (identification == NULL || !ConfigIsValid(&identification->config))
	{
		if (identification != NULL)
			FrictionIdentification_Fail(identification, FRICTION_IDENT_REASON_INVALID_CONFIG);
		return false;
	}
	FrictionIdentification_Reset(identification);
	identification->state = FRICTION_IDENT_TRACKING;
	identification->target_speed_rad_s = TargetForPoint(identification, 0U);
	return true;
}

void FrictionIdentification_Abort(FrictionIdentificationContext *identification)
{
	if (identification != NULL && IsActive(identification))
		FrictionIdentification_Fail(identification, FRICTION_IDENT_REASON_CANCELLED);
}

void FrictionIdentification_Fail(FrictionIdentificationContext *identification,
	FrictionIdentificationReason reason)
{
	if (identification == NULL) return;
	identification->state = FRICTION_IDENT_FAILED;
	identification->reason = reason;
	identification->result.valid = false;
	identification->target_speed_rad_s = 0.0f;
}

void FrictionIdentification_Update(FrictionIdentificationContext *identification,
	const FrictionIdentificationInput *input)
{
	float target_speed, tolerance;
	uint32_t minimum_sample_ticks, total_sample_count;
	if (identification == NULL || input == NULL || !IsActive(identification)) return;
	if (!isfinite(input->measured_speed_rad_s) ||
		!isfinite(input->ramped_speed_reference_rad_s) || !isfinite(input->iq_a) ||
		!isfinite(input->mechanical_position_rad))
	{
		FrictionIdentification_Fail(identification, FRICTION_IDENT_REASON_INVALID_SAMPLE);
		return;
	}
	identification->state_ticks++;
	identification->saturation_ticks = input->current_saturated ?
		identification->saturation_ticks + 1U : 0U;
	if (identification->saturation_ticks >= SecondsToTicks(identification,
		identification->config.saturation_time_s))
	{
		FrictionIdentification_Fail(identification, FRICTION_IDENT_REASON_CURRENT_SATURATION);
		return;
	}
	if (identification->state == FRICTION_IDENT_STOPPING)
	{
		if (AbsValue(input->measured_speed_rad_s) <= identification->config.stop_speed_rad_s &&
			AbsValue(input->ramped_speed_reference_rad_s) <= identification->config.stop_speed_rad_s)
			identification->stable_ticks++;
		else identification->stable_ticks = 0U;
		if (identification->stable_ticks >= SecondsToTicks(identification,
			identification->config.stop_hold_time_s))
		{
			identification->stable_ticks = 0U;
			identification->state_ticks = 0U;
			identification->saturation_ticks = 0U;
			total_sample_count = FrictionIdentification_GetSampleCount(identification);
			if (++identification->point_index >= total_sample_count)
			{
				identification->target_speed_rad_s = 0.0f;
				if (!Fit(identification))
				{
					FrictionIdentification_Fail(identification, FRICTION_IDENT_REASON_FIT_REJECTED);
					return;
				}
				identification->state = FRICTION_IDENT_COMPLETE;
				identification->reason = FRICTION_IDENT_REASON_NONE;
				return;
			}
			identification->state = FRICTION_IDENT_TRACKING;
			identification->target_speed_rad_s = TargetForPoint(identification,
				identification->point_index);
			return;
		}
		if (identification->state_ticks >= SecondsToTicks(identification,
			identification->config.stop_timeout_s))
			FrictionIdentification_Fail(identification, FRICTION_IDENT_REASON_STOP_TIMEOUT);
		return;
	}
	target_speed = identification->target_speed_rad_s;
	tolerance = SpeedTolerance(identification, target_speed);
	if (identification->state == FRICTION_IDENT_TRACKING)
	{
		if (AbsValue(input->ramped_speed_reference_rad_s - target_speed) <= tolerance)
		{
			identification->stable_ticks++;
			identification->tracking_mean_speed +=
				(input->measured_speed_rad_s - identification->tracking_mean_speed) /
				(float)identification->stable_ticks;
		}
		else { identification->stable_ticks = 0U; identification->tracking_mean_speed = 0.0f; }
		if (identification->stable_ticks >= SecondsToTicks(identification,
			identification->config.stable_time_s))
		{
			if (AbsValue(identification->tracking_mean_speed - target_speed) <= tolerance)
			{
				identification->state = FRICTION_IDENT_SAMPLING;
				identification->state_ticks = 0U;
				identification->sample_count = 0U;
				identification->sample_mean_speed = 0.0f;
				identification->sample_mean_iq = 0.0f;
				identification->sample_start_position = input->mechanical_position_rad;
			}
			else { identification->stable_ticks = 0U; identification->tracking_mean_speed = 0.0f; }
		}
		else if (identification->state_ticks >= SecondsToTicks(identification,
			identification->config.track_timeout_s))
			FrictionIdentification_Fail(identification, FRICTION_IDENT_REASON_TRACK_TIMEOUT);
		return;
	}
	identification->sample_count++;
	identification->sample_mean_speed +=
		(input->measured_speed_rad_s - identification->sample_mean_speed) /
		(float)identification->sample_count;
	identification->sample_mean_iq +=
		(input->iq_a - identification->sample_mean_iq) /
		(float)identification->sample_count;
	minimum_sample_ticks = SecondsToTicks(identification,
		identification->config.minimum_sample_time_s);
	if (identification->sample_count >= minimum_sample_ticks &&
		AbsValue(input->mechanical_position_rad - identification->sample_start_position) >=
			identification->config.sample_turns * FRICTION_TWO_PI)
	{
		FrictionIdentificationSample *sample =
			&identification->samples[identification->point_index];
		if (AbsValue(identification->sample_mean_speed - target_speed) > tolerance)
		{
			identification->sample_count = 0U;
			identification->sample_mean_speed = 0.0f;
			identification->sample_mean_iq = 0.0f;
			identification->sample_start_position = input->mechanical_position_rad;
			return;
		}
		sample->target_speed_rad_s = target_speed;
		sample->mean_speed_rad_s = identification->sample_mean_speed;
		sample->mean_iq_a = identification->sample_mean_iq;
		sample->sample_count = identification->sample_count;
		identification->state = FRICTION_IDENT_STOPPING;
		identification->state_ticks = 0U;
		identification->stable_ticks = 0U;
		identification->saturation_ticks = 0U;
		identification->target_speed_rad_s = 0.0f;
		return;
	}
	if (identification->state_ticks >= SecondsToTicks(identification,
		identification->config.sample_timeout_s))
		FrictionIdentification_Fail(identification, FRICTION_IDENT_REASON_SAMPLE_TIMEOUT);
}

float FrictionIdentification_GetTargetSpeed(const FrictionIdentificationContext *identification)
{ return identification != NULL ? identification->target_speed_rad_s : 0.0f; }

float FrictionIdentification_GetProgressPercent(const FrictionIdentificationContext *identification)
{
	uint32_t sample_count;
	if (identification == NULL || identification->state == FRICTION_IDENT_IDLE ||
		identification->state == FRICTION_IDENT_FAILED) return 0.0f;
	if (identification->state == FRICTION_IDENT_COMPLETE) return 100.0f;
	sample_count = FrictionIdentification_GetSampleCount(identification);
	return sample_count > 0U ?
		100.0f * (float)identification->point_index / (float)sample_count : 0.0f;
}

uint32_t FrictionIdentification_GetSampleCount(const FrictionIdentificationContext *identification)
{
	if (identification == NULL || identification->config.speed_point_count >
		FRICTION_IDENTIFICATION_MAX_SPEED_POINTS) return 0U;
	return 2U * identification->config.speed_point_count;
}

FrictionIdentificationState FrictionIdentification_GetState(
	const FrictionIdentificationContext *identification)
{ return identification != NULL ? identification->state : FRICTION_IDENT_FAILED; }

FrictionIdentificationReason FrictionIdentification_GetReason(
	const FrictionIdentificationContext *identification)
{ return identification != NULL ? identification->reason : FRICTION_IDENT_REASON_INVALID_CONFIG; }

uint32_t FrictionIdentification_GetPointIndex(const FrictionIdentificationContext *identification)
{ return identification != NULL ? identification->point_index : 0U; }

const FrictionIdentificationResult *FrictionIdentification_GetResult(
	const FrictionIdentificationContext *identification)
{ return identification != NULL ? &identification->result : NULL; }

const FrictionIdentificationSample *FrictionIdentification_GetSamples(
	const FrictionIdentificationContext *identification)
{ return identification != NULL ? identification->samples : NULL; }
