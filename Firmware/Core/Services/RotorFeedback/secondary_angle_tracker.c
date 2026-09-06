#include "secondary_angle_tracker.h"

#include <float.h>
#include <limits.h>
#include <string.h>

#define SECONDARY_ANGLE_TURN_COUNTS INT64_C(4294967296)
#define SECONDARY_ANGLE_RAD_PER_COUNT \
	(6.28318530717958647692f / 4294967296.0f)

static bool SecondaryAngleTracker_IsFinite(float value)
{
	return value == value && value <= FLT_MAX && value >= -FLT_MAX;
}

void SecondaryAngleTracker_Reset(SecondaryAngleTrackerContext *context)
{
	if (context != 0)
		memset(context, 0, sizeof(*context));
}

void SecondaryAngleTracker_Invalidate(SecondaryAngleTrackerContext *context)
{
	if (context != 0)
		context->fresh = false;
}

bool SecondaryAngleTracker_Update(SecondaryAngleTrackerContext *context,
	const SecondaryAngleSample *sample, float sample_period_s)
{
	int64_t next_unwrapped;
	int64_t delta;
	bool has_turn_count;

	if (context == 0 || sample == 0 ||
		!SecondaryAngleTracker_IsFinite(sample_period_s) ||
		sample_period_s <= 0.0f ||
		(sample->velocity_valid &&
		 !SecondaryAngleTracker_IsFinite(sample->velocity_rad_s)))
	{
		SecondaryAngleTracker_Invalidate(context);
		return false;
	}

	has_turn_count = sample->turn_count_valid;
	if (has_turn_count)
	{
		next_unwrapped = (int64_t)sample->turn_count *
			SECONDARY_ANGLE_TURN_COUNTS +
			(int64_t)sample->single_turn_position_u32;
	}
	else if (!context->has_position)
	{
		next_unwrapped = (int64_t)sample->single_turn_position_u32;
	}
	else
	{
		delta = (int64_t)sample->single_turn_position_u32 -
			(int64_t)context->previous_single_turn_position_u32;
		if (delta > INT32_MAX)
			delta -= SECONDARY_ANGLE_TURN_COUNTS;
		else if (delta < INT32_MIN)
			delta += SECONDARY_ANGLE_TURN_COUNTS;
		next_unwrapped = context->unwrapped_position_u32 + delta;
	}

	delta = context->has_position ?
		next_unwrapped - context->unwrapped_position_u32 : 0;
	context->previous_single_turn_position_u32 =
		sample->single_turn_position_u32;
	context->unwrapped_position_u32 = next_unwrapped;
	context->position_rad = (float)next_unwrapped *
		SECONDARY_ANGLE_RAD_PER_COUNT;
	context->velocity_rad_s =
		sample->velocity_valid ?
			sample->velocity_rad_s :
			(float)delta * SECONDARY_ANGLE_RAD_PER_COUNT /
				sample_period_s;
	context->has_position = true;
	context->fresh = true;
	return true;
}
