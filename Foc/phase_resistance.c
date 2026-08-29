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

static float PhaseResistance_Min(float first, float second)
{
	return first < second ? first : second;
}

static float PhaseResistance_Max(float first, float second)
{
	return first > second ? first : second;
}

static void PhaseResistance_ResetSampling(PhaseResistanceContext_TypeDef *context)
{
	context->stable_count = 0U;
	context->sample_count = 0U;
	context->timeout_count = 0U;
	context->filtered_voltage_d = 0.0f;
	context->filtered_voltage_q = 0.0f;
	context->previous_voltage_d = 0.0f;
	context->previous_voltage_q = 0.0f;
	context->power_sum = 0.0f;
	context->current_square_sum = 0.0f;
}

static void PhaseResistance_BeginVector(PhaseResistanceContext_TypeDef *context)
{
	context->ramp_count = 0U;
	context->pause_count = 0U;
	PhaseResistance_ResetSampling(context);
	context->step = PHASE_RESISTANCE_STEP_RAMP;
}

static bool PhaseResistance_ConfigIsValid(const PhaseResistanceConfig_TypeDef *config)
{
	return config != NULL &&
		config->test_current >= config->minimum_test_current &&
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
		config->balance_limit_pct >= 0.0f;
}

/**
 * Stability requires filtered d-axis current to reach the configured target,
 * q-axis current to remain small, and the filtered d/q voltage to stop moving.
 */
static bool PhaseResistance_IsStable(PhaseResistanceContext_TypeDef *context,
	const PhaseResistanceSample_TypeDef *sample)
{
	float current_error = fabsf(sample->id_filt - context->config.test_current);
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

	current_stable = current_error <= context->config.test_current *
		context->config.current_tolerance &&
		fabsf(sample->iq_filt) <= context->config.test_current *
		context->config.q_current_tolerance;
	voltage_stable = voltage_delta <= PhaseResistance_Max(
		context->config.voltage_min_delta,
		voltage_magnitude * context->config.voltage_tolerance);

	context->previous_voltage_d = context->filtered_voltage_d;
	context->previous_voltage_q = context->filtered_voltage_q;
	return current_stable && voltage_stable;
}

/**
 * Convert the three equivalent vector resistances into individual phase
 * resistances. The configured path compensation is subtracted afterwards.
 */
static bool PhaseResistance_CalculateResult(PhaseResistanceContext_TypeDef *context)
{
	float resistance_0 = context->result.vector_resistance[0];
	float resistance_1 = context->result.vector_resistance[1];
	float resistance_2 = context->result.vector_resistance[2];
	float resistance_min;
	float resistance_max;
	float resistance_mean;

	if (resistance_0 <= context->config.path_compensation_ohm ||
		resistance_1 <= context->config.path_compensation_ohm ||
		resistance_2 <= context->config.path_compensation_ohm)
		return false;

	context->result.phase_resistance_a =
		(5.0f * resistance_0 - resistance_1 - resistance_2) / 3.0f -
		context->config.path_compensation_ohm;
	context->result.phase_resistance_b =
		(5.0f * resistance_1 - resistance_0 - resistance_2) / 3.0f -
		context->config.path_compensation_ohm;
	context->result.phase_resistance_c =
		(5.0f * resistance_2 - resistance_0 - resistance_1) / 3.0f -
		context->config.path_compensation_ohm;
	if (context->result.phase_resistance_a <= 0.0f ||
		context->result.phase_resistance_b <= 0.0f ||
		context->result.phase_resistance_c <= 0.0f)
		return false;

	resistance_min = PhaseResistance_Min(context->result.phase_resistance_a,
		PhaseResistance_Min(context->result.phase_resistance_b,
		context->result.phase_resistance_c));
	resistance_max = PhaseResistance_Max(context->result.phase_resistance_a,
		PhaseResistance_Max(context->result.phase_resistance_b,
		context->result.phase_resistance_c));
	resistance_mean = (context->result.phase_resistance_a +
		context->result.phase_resistance_b + context->result.phase_resistance_c) / 3.0f;
	if (resistance_mean <= 0.0f)
		return false;

	context->result.spread_pct =
		(resistance_max - resistance_min) * 100.0f / resistance_mean;
	context->result.valid = true;
	context->result.balanced = context->result.spread_pct <=
		context->config.balance_limit_pct;
	return true;
}

void PhaseResistance_Init(PhaseResistanceContext_TypeDef *context)
{
	if (context != NULL)
		memset(context, 0, sizeof(*context));
}

bool PhaseResistance_Start(PhaseResistanceContext_TypeDef *context,
	const PhaseResistanceConfig_TypeDef *config)
{
	if (context == NULL || !PhaseResistance_ConfigIsValid(config))
		return false;

	PhaseResistance_Init(context);
	context->config = *config;
	PhaseResistance_BeginVector(context);
	return true;
}

void PhaseResistance_GetCommand(const PhaseResistanceContext_TypeDef *context,
	PhaseResistanceCommand_TypeDef *command)
{
	if (command == NULL)
		return;

	memset(command, 0, sizeof(*command));
	command->reset_current_controller = true;
	if (context == NULL)
		return;

	command->electrical_angle = (float)context->vector_index *
		PHASE_RESISTANCE_TWO_PI / (float)PHASE_RESISTANCE_VECTOR_COUNT;
	if (context->step == PHASE_RESISTANCE_STEP_RAMP)
	{
		command->inject_current = true;
		command->reset_current_controller = false;
		command->id_ref = context->config.test_current *
			(float)(context->ramp_count + 1U) / (float)context->config.ramp_ticks;
	}
	else if (context->step == PHASE_RESISTANCE_STEP_SETTLE ||
		context->step == PHASE_RESISTANCE_STEP_SAMPLE)
	{
		command->inject_current = true;
		command->reset_current_controller = false;
		command->id_ref = context->config.test_current;
	}
}

PhaseResistanceCoreStatus_TypeDef PhaseResistance_InputSample(
	PhaseResistanceContext_TypeDef *context,
	const PhaseResistanceSample_TypeDef *sample)
{
	bool stable;

	if (context == NULL)
		return PHASE_RESISTANCE_CORE_INVALID_RESULT;

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
			stable = PhaseResistance_IsStable(context, sample);
			context->stable_count = stable ? context->stable_count + 1U : 0U;
			context->timeout_count++;
			if (context->stable_count >= context->config.settle_ticks)
			{
				context->sample_count = 0U;
				context->power_sum = 0.0f;
				context->current_square_sum = 0.0f;
				context->step = PHASE_RESISTANCE_STEP_SAMPLE;
			}
			else if (context->timeout_count >= context->config.timeout_ticks)
			{
				context->step = PHASE_RESISTANCE_STEP_FAILED;
				return PHASE_RESISTANCE_CORE_SETTLE_TIMEOUT;
			}
			return PHASE_RESISTANCE_CORE_RUNNING;

		case PHASE_RESISTANCE_STEP_SAMPLE:
			if (sample == NULL || !PhaseResistance_IsStable(context, sample))
				break;
			context->power_sum += sample->vd * sample->id + sample->vq * sample->iq;
			context->current_square_sum += sample->id * sample->id + sample->iq * sample->iq;
			context->sample_count++;
			if (context->sample_count < context->config.sample_ticks)
				return PHASE_RESISTANCE_CORE_RUNNING;
			if (context->current_square_sum <= 0.0f)
				break;
			context->result.vector_resistance[context->vector_index] =
				context->power_sum / context->current_square_sum;
			context->pause_count = 0U;
			context->step = PHASE_RESISTANCE_STEP_PAUSE;
			return PHASE_RESISTANCE_CORE_RUNNING;

		case PHASE_RESISTANCE_STEP_PAUSE:
			context->pause_count++;
			if (context->pause_count < context->config.pause_ticks)
				return PHASE_RESISTANCE_CORE_RUNNING;
			context->vector_index++;
			if (context->vector_index >= PHASE_RESISTANCE_VECTOR_COUNT)
			{
				if (!PhaseResistance_CalculateResult(context))
					break;
				context->step = PHASE_RESISTANCE_STEP_DONE;
				return PHASE_RESISTANCE_CORE_DONE;
			}
			PhaseResistance_BeginVector(context);
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

bool PhaseResistance_GetResult(const PhaseResistanceContext_TypeDef *context,
	PhaseResistanceResult_TypeDef *result)
{
	if (context == NULL || result == NULL || !context->result.valid)
		return false;

	*result = context->result;
	return true;
}

void PhaseResistance_Cancel(PhaseResistanceContext_TypeDef *context)
{
	PhaseResistance_Init(context);
}
