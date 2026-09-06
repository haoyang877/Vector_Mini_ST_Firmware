#ifndef CORE_SERVICES_IDENTIFICATION_FRICTION_IDENTIFICATION_H
#define CORE_SERVICES_IDENTIFICATION_FRICTION_IDENTIFICATION_H

#include <stdbool.h>
#include <stdint.h>

#define FRICTION_IDENTIFICATION_MAX_SPEED_POINTS 4U
#define FRICTION_IDENTIFICATION_MAX_SAMPLES \
	(2U * FRICTION_IDENTIFICATION_MAX_SPEED_POINTS)

typedef enum
{
	FRICTION_IDENT_IDLE = 0,
	FRICTION_IDENT_TRACKING,
	FRICTION_IDENT_SAMPLING,
	FRICTION_IDENT_STOPPING,
	FRICTION_IDENT_COMPLETE,
	FRICTION_IDENT_FAILED
} FrictionIdentificationState;

typedef enum
{
	FRICTION_IDENT_REASON_NONE = 0,
	FRICTION_IDENT_REASON_CANCELLED,
	FRICTION_IDENT_REASON_INVALID_CONFIG,
	FRICTION_IDENT_REASON_TRACK_TIMEOUT,
	FRICTION_IDENT_REASON_SAMPLE_TIMEOUT,
	FRICTION_IDENT_REASON_STOP_TIMEOUT,
	FRICTION_IDENT_REASON_CURRENT_SATURATION,
	FRICTION_IDENT_REASON_INVALID_SAMPLE,
	FRICTION_IDENT_REASON_FIT_REJECTED,
	FRICTION_IDENT_REASON_SAFETY_FAULT
} FrictionIdentificationReason;

typedef struct
{
	float update_period_s;
	float speed_points_rad_s[FRICTION_IDENTIFICATION_MAX_SPEED_POINTS];
	uint32_t speed_point_count;
	float stable_time_s;
	float track_timeout_s;
	float sample_timeout_s;
	float stop_hold_time_s;
	float stop_timeout_s;
	float speed_tolerance_ratio;
	float minimum_speed_tolerance_rad_s;
	float stop_speed_rad_s;
	float sample_turns;
	float minimum_sample_time_s;
	float saturation_time_s;
	float rmse_floor_a;
	float rmse_ratio_max;
} FrictionIdentificationConfig;

typedef struct
{
	float measured_speed_rad_s;
	float ramped_speed_reference_rad_s;
	float iq_a;
	float mechanical_position_rad;
	bool current_saturated;
} FrictionIdentificationInput;

typedef struct
{
	float target_speed_rad_s;
	float mean_speed_rad_s;
	float mean_iq_a;
	uint32_t sample_count;
} FrictionIdentificationSample;

typedef struct
{
	float coulomb_pos_a;
	float coulomb_neg_a;
	float viscous_pos_a_per_rad_s;
	float viscous_neg_a_per_rad_s;
	float rmse_pos_a;
	float rmse_neg_a;
	bool valid;
} FrictionIdentificationResult;

typedef struct
{
	FrictionIdentificationConfig config;
	FrictionIdentificationState state;
	FrictionIdentificationReason reason;
	FrictionIdentificationResult result;
	FrictionIdentificationSample samples[FRICTION_IDENTIFICATION_MAX_SAMPLES];
	uint32_t point_index;
	uint32_t state_ticks;
	uint32_t stable_ticks;
	uint32_t saturation_ticks;
	uint32_t sample_count;
	float tracking_mean_speed;
	float sample_mean_speed;
	float sample_mean_iq;
	float sample_start_position;
	float target_speed_rad_s;
} FrictionIdentificationContext;

bool FrictionIdentification_Init(FrictionIdentificationContext *identification,
	const FrictionIdentificationConfig *config);
void FrictionIdentification_Reset(FrictionIdentificationContext *identification);
bool FrictionIdentification_Start(FrictionIdentificationContext *identification);
void FrictionIdentification_Abort(FrictionIdentificationContext *identification);
void FrictionIdentification_Fail(FrictionIdentificationContext *identification,
	FrictionIdentificationReason reason);
void FrictionIdentification_Update(FrictionIdentificationContext *identification,
	const FrictionIdentificationInput *input);
float FrictionIdentification_GetTargetSpeed(
	const FrictionIdentificationContext *identification);
float FrictionIdentification_GetProgressPercent(
	const FrictionIdentificationContext *identification);
uint32_t FrictionIdentification_GetSampleCount(
	const FrictionIdentificationContext *identification);
FrictionIdentificationState FrictionIdentification_GetState(
	const FrictionIdentificationContext *identification);
FrictionIdentificationReason FrictionIdentification_GetReason(
	const FrictionIdentificationContext *identification);
uint32_t FrictionIdentification_GetPointIndex(
	const FrictionIdentificationContext *identification);
const FrictionIdentificationResult *FrictionIdentification_GetResult(
	const FrictionIdentificationContext *identification);
const FrictionIdentificationSample *FrictionIdentification_GetSamples(
	const FrictionIdentificationContext *identification);

#endif
