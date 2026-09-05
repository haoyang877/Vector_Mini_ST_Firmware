#include "phase_resistance.h"

#include <math.h>
#include <string.h>

#define PHASE_RESISTANCE_STEP_RAMP 1U
#define PHASE_RESISTANCE_STEP_SETTLE 2U
#define PHASE_RESISTANCE_STEP_SAMPLE 3U
#define PHASE_RESISTANCE_STEP_PAUSE 4U
#define PHASE_RESISTANCE_STEP_DONE 5U
#define PHASE_RESISTANCE_STEP_FAILED 6U

#define PHASE_RESISTANCE_TWO_PI 6.28318530717958647692f
#define PHASE_RESISTANCE_CURRENT_DELTA_MIN_RATIO 0.25f

static const uint8_t PhaseResistanceDirectionPair[PHASE_RESISTANCE_VECTOR_COUNT]
	[PHASE_RESISTANCE_CURRENT_LEVEL_COUNT] =
{
	{0U, 3U},
	{2U, 5U},
	{4U, 1U}
};

static float PhaseResistance_Min(float first, float second)
{
	return first < second ? first : second;
}

static float PhaseResistance_Max(float first, float second)
{
	return first > second ? first : second;
}

static uint8_t PhaseResistance_GetCurrentLevelIndex(
	const PhaseResistanceContext *context)
{
	if ((context->direction_index & 1U) == 0U)
		return context->current_point_index;
	return (PHASE_RESISTANCE_CURRENT_LEVEL_COUNT - 1U) -
		context->current_point_index;
}

static float PhaseResistance_GetTestCurrent(
	const PhaseResistanceContext *context)
{
	if (PhaseResistance_GetCurrentLevelIndex(context) == 0U)
		return context->config.test_current_low;
	return context->config.test_current_high;
}

static void PhaseResistance_ResetSampling(PhaseResistanceContext *context)
{
	context->stable_count = 0U;
	context->sample_count = 0U;
	context->filtered_voltage_d = 0.0f;
	context->filtered_voltage_q = 0.0f;
	context->previous_voltage_d = 0.0f;
	context->previous_voltage_q = 0.0f;
	context->sample_current_sum = 0.0f;
	context->sample_voltage_sum = 0.0f;
}

static void PhaseResistance_ResetSampleWindow(PhaseResistanceContext *context)
{
	context->sample_count = 0U;
	context->sample_current_sum = 0.0f;
	context->sample_voltage_sum = 0.0f;
}

static void PhaseResistance_BeginPoint(PhaseResistanceContext *context)
{
	context->ramp_count = 0U;
	context->pause_count = 0U;
	context->timeout_count = 0U;
	PhaseResistance_ResetSampling(context);
	context->step = PHASE_RESISTANCE_STEP_RAMP;
}

static bool PhaseResistance_ConfigIsValid(const PhaseResistanceConfig *config)
{
	return config != NULL &&
		config->test_current_low >= config->minimum_test_current &&
		config->test_current_high > config->test_current_low &&
		config->test_current_high <= config->maximum_test_current &&
		config->minimum_test_current > 0.0f &&
		config->ramp_ticks > 0U &&
		config->settle_ticks > 0U &&
		config->sample_ticks > 0U &&
		config->pause_ticks > 0U &&
		config->timeout_ticks >= config->settle_ticks &&
		config->current_tolerance >= 0.0f &&
		config->q_current_tolerance >= 0.0f &&
		config->voltage_tolerance >= 0.0f &&
		config->voltage_min_delta >= 0.0f &&
		config->voltage_filter > 0.0f && config->voltage_filter <= 1.0f &&
		config->path_compensation_ohm >= 0.0f &&
		config->balance_warning_pct >= 0.0f &&
		config->balance_fault_pct >= config->balance_warning_pct;
}

/**
 * Stability requires filtered d-axis current to reach the active target,
 * q-axis current to remain small, and the filtered d/q voltage to stop moving.
 */
static bool PhaseResistance_IsStable(PhaseResistanceContext *context,
	const PhaseResistanceSample *sample, float test_current)
{
	float current_error = fabsf(sample->id_filt - test_current);
	float voltage_delta;
	float voltage_magnitude;
	bool current_stable;
	bool voltage_stable;

	context->filtered_voltage_d += (sample->vd - context->filtered_voltage_d) *
		context->config.voltage_filter;
	context->filtered_voltage_q += (sample->vq - context->filtered_voltage_q) *
		context->config.voltage_filter;
	voltage_delta = fabsf(context->filtered_voltage_d - context->previous_voltage_d) +
		fabsf(context->filtered_voltage_q - context->previous_voltage_q);
	voltage_magnitude = sqrtf(context->filtered_voltage_d * context->filtered_voltage_d +
		context->filtered_voltage_q * context->filtered_voltage_q);

	current_stable = current_error <= test_current * context->config.current_tolerance &&
		fabsf(sample->iq_filt) <= test_current * context->config.q_current_tolerance;
	voltage_stable = voltage_delta <= PhaseResistance_Max(
		context->config.voltage_min_delta,
		voltage_magnitude * context->config.voltage_tolerance);

	context->previous_voltage_d = context->filtered_voltage_d;
	context->previous_voltage_q = context->filtered_voltage_q;
	return current_stable && voltage_stable;
}

static bool PhaseResistance_StorePoint(PhaseResistanceContext *context)
{
	uint8_t current_level_index;

	if (context->sample_count == 0U)
		return false;

	current_level_index = PhaseResistance_GetCurrentLevelIndex(context);
	context->direction_current[context->direction_index][current_level_index] =
		context->sample_current_sum / (float)context->sample_count;
	context->direction_voltage[context->direction_index][current_level_index] =
		context->sample_voltage_sum / (float)context->sample_count;
	return true;
}

/**
 * Fit the two current levels for every direction, then pair opposite vectors
 * before converting the three equivalent values to individual phase values.
 */
static bool PhaseResistance_CalculateResult(PhaseResistanceContext *context)
{
	float resistance_min;
	float resistance_max;
	float resistance_mean;
	float minimum_current_delta = (context->config.test_current_high -
		context->config.test_current_low) * PHASE_RESISTANCE_CURRENT_DELTA_MIN_RATIO;
	uint8_t direction_index;
	uint8_t vector_index;

	for (direction_index = 0U;
		direction_index < PHASE_RESISTANCE_DIRECTION_COUNT; direction_index++)
	{
		float current_low = context->direction_current[direction_index][0U];
		float current_high = context->direction_current[direction_index][1U];
		float voltage_low = context->direction_voltage[direction_index][0U];
		float voltage_high = context->direction_voltage[direction_index][1U];
		float current_delta = current_high - current_low;
		float resistance;

		if (current_delta < minimum_current_delta)
			return false;

		resistance = (voltage_high - voltage_low) / current_delta;
		if (resistance <= 0.0f)
			return false;

		context->result.direction_resistance[direction_index] = resistance;
		context->result.direction_voltage_offset[direction_index] = voltage_low -
			resistance * current_low;
	}

	for (vector_index = 0U; vector_index < PHASE_RESISTANCE_VECTOR_COUNT;
		vector_index++)
	{
		uint8_t first_direction = PhaseResistanceDirectionPair[vector_index][0U];
		uint8_t second_direction = PhaseResistanceDirectionPair[vector_index][1U];

		context->result.vector_resistance[vector_index] =
			(context->result.direction_resistance[first_direction] +
			context->result.direction_resistance[second_direction]) * 0.5f;
		if (context->result.vector_resistance[vector_index] <=
			context->config.path_compensation_ohm)
			return false;
	}

	context->result.phase_a_resistance_ohm =
		(5.0f * context->result.vector_resistance[0U] -
		context->result.vector_resistance[1U] -
		context->result.vector_resistance[2U]) / 3.0f -
		context->config.path_compensation_ohm;
	context->result.phase_b_resistance_ohm =
		(5.0f * context->result.vector_resistance[1U] -
		context->result.vector_resistance[0U] -
		context->result.vector_resistance[2U]) / 3.0f -
		context->config.path_compensation_ohm;
	context->result.phase_c_resistance_ohm =
		(5.0f * context->result.vector_resistance[2U] -
		context->result.vector_resistance[0U] -
		context->result.vector_resistance[1U]) / 3.0f -
		context->config.path_compensation_ohm;
	if (context->result.phase_a_resistance_ohm <= 0.0f ||
		context->result.phase_b_resistance_ohm <= 0.0f ||
		context->result.phase_c_resistance_ohm <= 0.0f)
		return false;

	resistance_min = PhaseResistance_Min(context->result.phase_a_resistance_ohm,
		PhaseResistance_Min(context->result.phase_b_resistance_ohm,
		context->result.phase_c_resistance_ohm));
	resistance_max = PhaseResistance_Max(context->result.phase_a_resistance_ohm,
		PhaseResistance_Max(context->result.phase_b_resistance_ohm,
		context->result.phase_c_resistance_ohm));
	resistance_mean = (context->result.phase_a_resistance_ohm +
		context->result.phase_b_resistance_ohm + context->result.phase_c_resistance_ohm) / 3.0f;
	if (resistance_mean <= 0.0f)
		return false;

	context->result.spread_pct =
		(resistance_max - resistance_min) * 100.0f / resistance_mean;
	context->result.warning = context->result.spread_pct >
		context->config.balance_warning_pct;
	context->result.balanced = context->result.spread_pct <=
		context->config.balance_fault_pct;
	context->result.valid = true;
	return true;
}

void PhaseResistance_Init(PhaseResistanceContext *context)
{
	if (context == NULL)
		return;

	memset(context, 0, sizeof(*context));
	context->step = PHASE_RESISTANCE_STEP_FAILED;
}

bool PhaseResistance_Start(PhaseResistanceContext *context,
	const PhaseResistanceConfig *config)
{
	if (context == NULL || !PhaseResistance_ConfigIsValid(config))
		return false;

	PhaseResistance_Init(context);
	context->config = *config;
	PhaseResistance_BeginPoint(context);
	return true;
}

void PhaseResistance_GetCommand(const PhaseResistanceContext *context,
	PhaseResistanceCommand *command)
{
	float test_current;

	if (command == NULL)
		return;

	memset(command, 0, sizeof(*command));
	command->reset_current_controller = true;
	if (context == NULL)
		return;

	command->electrical_angle = (float)context->direction_index *
		PHASE_RESISTANCE_TWO_PI / (float)PHASE_RESISTANCE_DIRECTION_COUNT;
	test_current = PhaseResistance_GetTestCurrent(context);
	if (context->step == PHASE_RESISTANCE_STEP_RAMP)
	{
		command->inject_current = true;
		command->reset_current_controller = false;
		command->id_ref = test_current * (float)(context->ramp_count + 1U) /
			(float)context->config.ramp_ticks;
	}
	else if (context->step == PHASE_RESISTANCE_STEP_SETTLE ||
		context->step == PHASE_RESISTANCE_STEP_SAMPLE)
	{
		command->inject_current = true;
		command->reset_current_controller = false;
		command->id_ref = test_current;
	}
}

PhaseResistanceCoreStatus PhaseResistance_InputSample(
	PhaseResistanceContext *context,
	const PhaseResistanceSample *sample)
{
	bool stable;
	float test_current;

	if (context == NULL)
		return PHASE_RESISTANCE_CORE_INVALID_RESULT;

	test_current = PhaseResistance_GetTestCurrent(context);
	switch (context->step)
	{
		case PHASE_RESISTANCE_STEP_RAMP:
			context->ramp_count++;
			if (context->ramp_count >= context->config.ramp_ticks)
				context->step = PHASE_RESISTANCE_STEP_SETTLE;
			return PHASE_RESISTANCE_CORE_RUNNING;

		case PHASE_RESISTANCE_STEP_SETTLE:
			if (sample == NULL)
				break;
			stable = PhaseResistance_IsStable(context, sample, test_current);
			context->stable_count = stable ? context->stable_count + 1U : 0U;
			context->timeout_count++;
			if (context->stable_count >= context->config.settle_ticks)
			{
				PhaseResistance_ResetSampleWindow(context);
				context->step = PHASE_RESISTANCE_STEP_SAMPLE;
			}
			else if (context->timeout_count >= context->config.timeout_ticks)
			{
				context->step = PHASE_RESISTANCE_STEP_FAILED;
				return PHASE_RESISTANCE_CORE_SETTLE_TIMEOUT;
			}
			return PHASE_RESISTANCE_CORE_RUNNING;

		case PHASE_RESISTANCE_STEP_SAMPLE:
			if (sample == NULL || !PhaseResistance_IsStable(context, sample, test_current))
			{
				context->stable_count = 0U;
				PhaseResistance_ResetSampleWindow(context);
				context->step = PHASE_RESISTANCE_STEP_SETTLE;
				return PHASE_RESISTANCE_CORE_RUNNING;
			}
			else
			{
				float current_magnitude = sqrtf(sample->id * sample->id +
					sample->iq * sample->iq);

				if (current_magnitude <= 0.0f)
				{
					context->stable_count = 0U;
					PhaseResistance_ResetSampleWindow(context);
					context->step = PHASE_RESISTANCE_STEP_SETTLE;
					return PHASE_RESISTANCE_CORE_RUNNING;
				}
				context->sample_current_sum += current_magnitude;
				context->sample_voltage_sum += (sample->vd * sample->id +
					sample->vq * sample->iq) / current_magnitude;
				context->sample_count++;
			}
			if (context->sample_count < context->config.sample_ticks)
				return PHASE_RESISTANCE_CORE_RUNNING;
			if (!PhaseResistance_StorePoint(context))
				break;
			context->pause_count = 0U;
			context->step = PHASE_RESISTANCE_STEP_PAUSE;
			return PHASE_RESISTANCE_CORE_RUNNING;

		case PHASE_RESISTANCE_STEP_PAUSE:
			context->pause_count++;
			if (context->pause_count < context->config.pause_ticks)
				return PHASE_RESISTANCE_CORE_RUNNING;
			context->current_point_index++;
			if (context->current_point_index >= PHASE_RESISTANCE_CURRENT_LEVEL_COUNT)
			{
				context->current_point_index = 0U;
				context->direction_index++;
				if (context->direction_index >= PHASE_RESISTANCE_DIRECTION_COUNT)
				{
					if (!PhaseResistance_CalculateResult(context))
						break;
					context->step = PHASE_RESISTANCE_STEP_DONE;
					return PHASE_RESISTANCE_CORE_DONE;
				}
			}
			PhaseResistance_BeginPoint(context);
			return PHASE_RESISTANCE_CORE_RUNNING;

		case PHASE_RESISTANCE_STEP_DONE:
			return PHASE_RESISTANCE_CORE_DONE;

		case PHASE_RESISTANCE_STEP_FAILED:
			return PHASE_RESISTANCE_CORE_INVALID_RESULT;

		default:
			break;
	}

	context->step = PHASE_RESISTANCE_STEP_FAILED;
	return PHASE_RESISTANCE_CORE_INVALID_RESULT;
}

bool PhaseResistance_GetResult(const PhaseResistanceContext *context,
	PhaseResistanceResult *result)
{
	if (context == NULL || result == NULL || !context->result.valid)
		return false;

	*result = context->result;
	return true;
}

void PhaseResistance_Cancel(PhaseResistanceContext *context)
{
	PhaseResistance_Init(context);
}
