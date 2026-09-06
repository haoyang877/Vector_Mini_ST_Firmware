#ifndef CORE_SERVICES_ROTOR_FEEDBACK_SECONDARY_ANGLE_TRACKER_H
#define CORE_SERVICES_ROTOR_FEEDBACK_SECONDARY_ANGLE_TRACKER_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Lightweight unwrapped-position state for the one angle source which is not
 * the calibrated motor-rotor encoder.  It deliberately owns no LUT or
 * persistent calibration data.
 */
typedef struct
{
	uint32_t previous_single_turn_position_u32;
	int64_t unwrapped_position_u32;
	float position_rad;
	float velocity_rad_s;
	bool has_position;
	bool fresh;
} SecondaryAngleTrackerContext;

/* Hardware-independent sample presented by the application adapter. */
typedef struct
{
	uint32_t single_turn_position_u32;
	int32_t turn_count;
	float velocity_rad_s;
	bool turn_count_valid;
	bool velocity_valid;
} SecondaryAngleSample;

void SecondaryAngleTracker_Reset(SecondaryAngleTrackerContext *context);
void SecondaryAngleTracker_Invalidate(SecondaryAngleTrackerContext *context);
bool SecondaryAngleTracker_Update(SecondaryAngleTrackerContext *context,
	const SecondaryAngleSample *sample, float sample_period_s);

#endif
