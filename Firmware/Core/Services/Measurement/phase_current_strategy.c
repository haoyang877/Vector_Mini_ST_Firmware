#include "phase_current_strategy.h"

#include <float.h>
#include <stddef.h>
#include <string.h>

static const PhaseCurrentDcLinkSectorMapping s_default_dc_link_sector_map[
	PHASE_CURRENT_DC_LINK_SECTOR_COUNT] =
{
	{ PHASE_CURRENT_DC_LINK_MAP_POSITIVE_A,
		PHASE_CURRENT_DC_LINK_MAP_NEGATIVE_C },
	{ PHASE_CURRENT_DC_LINK_MAP_NEGATIVE_C,
		PHASE_CURRENT_DC_LINK_MAP_POSITIVE_B },
	{ PHASE_CURRENT_DC_LINK_MAP_POSITIVE_B,
		PHASE_CURRENT_DC_LINK_MAP_NEGATIVE_A },
	{ PHASE_CURRENT_DC_LINK_MAP_NEGATIVE_A,
		PHASE_CURRENT_DC_LINK_MAP_POSITIVE_C },
	{ PHASE_CURRENT_DC_LINK_MAP_POSITIVE_C,
		PHASE_CURRENT_DC_LINK_MAP_NEGATIVE_B },
	{ PHASE_CURRENT_DC_LINK_MAP_NEGATIVE_B,
		PHASE_CURRENT_DC_LINK_MAP_POSITIVE_A }
};

static bool PhaseCurrent_IsSupportedTopology(PhaseCurrentTopology topology)
{
	return topology == PHASE_CURRENT_TOPOLOGY_INLINE_3_SHUNT ||
		topology == PHASE_CURRENT_TOPOLOGY_LOW_SIDE_3_SHUNT ||
		topology == PHASE_CURRENT_TOPOLOGY_LOW_SIDE_2_SHUNT ||
		topology == PHASE_CURRENT_TOPOLOGY_DC_LINK_1_SHUNT;
}

static bool PhaseCurrent_IsFinite(float value)
{
	return value == value && value <= FLT_MAX && value >= -FLT_MAX;
}

static uint8_t PhaseCurrent_CountBits(uint8_t value)
{
	uint8_t count = 0U;

	while (value != 0U)
	{
		count = (uint8_t)(count + (value & 1U));
		value = (uint8_t)(value >> 1U);
	}
	return count;
}

static bool PhaseCurrent_DecodeMapping(
	PhaseCurrentDcLinkSampleMapping mapping, uint8_t *phase_index,
	float *sign)
{
	if (phase_index == NULL || sign == NULL)
		return false;

	switch (mapping)
	{
	case PHASE_CURRENT_DC_LINK_MAP_POSITIVE_A:
		*phase_index = PHASE_CURRENT_PHASE_A;
		*sign = 1.0f;
		return true;
	case PHASE_CURRENT_DC_LINK_MAP_NEGATIVE_A:
		*phase_index = PHASE_CURRENT_PHASE_A;
		*sign = -1.0f;
		return true;
	case PHASE_CURRENT_DC_LINK_MAP_POSITIVE_B:
		*phase_index = PHASE_CURRENT_PHASE_B;
		*sign = 1.0f;
		return true;
	case PHASE_CURRENT_DC_LINK_MAP_NEGATIVE_B:
		*phase_index = PHASE_CURRENT_PHASE_B;
		*sign = -1.0f;
		return true;
	case PHASE_CURRENT_DC_LINK_MAP_POSITIVE_C:
		*phase_index = PHASE_CURRENT_PHASE_C;
		*sign = 1.0f;
		return true;
	case PHASE_CURRENT_DC_LINK_MAP_NEGATIVE_C:
		*phase_index = PHASE_CURRENT_PHASE_C;
		*sign = -1.0f;
		return true;
	default:
		return false;
	}
}

static bool PhaseCurrent_SectorMappingIsValid(
	const PhaseCurrentDcLinkSectorMapping *mapping)
{
	uint8_t first_phase;
	uint8_t second_phase;
	float sign;

	if (mapping == NULL ||
		!PhaseCurrent_DecodeMapping(mapping->first_window, &first_phase, &sign) ||
		!PhaseCurrent_DecodeMapping(mapping->second_window, &second_phase, &sign))
	{
		return false;
	}

	return first_phase != second_phase;
}

static void PhaseCurrent_ClearSample(PhaseCurrentSample *sample,
	uint32_t sequence)
{
	memset(sample, 0, sizeof(*sample));
	sample->quality = PHASE_CURRENT_QUALITY_INVALID;
	sample->sequence = sequence;
}

static void PhaseCurrent_SetSampleValues(PhaseCurrentSample *sample,
	const float phase_current_a[PHASE_CURRENT_PHASE_COUNT],
	PhaseCurrentQuality quality)
{
	sample->ia_a = phase_current_a[PHASE_CURRENT_PHASE_A];
	sample->ib_a = phase_current_a[PHASE_CURRENT_PHASE_B];
	sample->ic_a = phase_current_a[PHASE_CURRENT_PHASE_C];
	sample->valid_phase_mask = PHASE_CURRENT_VALID_ALL;
	sample->quality = quality;
}

static PhaseCurrentStatus PhaseCurrent_CheckSequence(
	PhaseCurrentStrategy *strategy, uint32_t sequence,
	PhaseCurrentSample *sample)
{
	uint32_t delta;

	if (!strategy->has_last_sequence)
	{
		strategy->last_sequence = sequence;
		strategy->has_last_sequence = true;
		return PHASE_CURRENT_STATUS_OK;
	}

	delta = sequence - strategy->last_sequence;
	if (delta == 0U)
	{
		sample->quality = PHASE_CURRENT_QUALITY_STALE;
		return PHASE_CURRENT_STATUS_DUPLICATE_SEQUENCE;
	}
	if (delta >= 0x80000000UL)
	{
		sample->quality = PHASE_CURRENT_QUALITY_STALE;
		return PHASE_CURRENT_STATUS_OUT_OF_ORDER_SEQUENCE;
	}

	/* Every observed frame is consumed, including a frame with bad payload. */
	strategy->last_sequence = sequence;
	return PHASE_CURRENT_STATUS_OK;
}

static PhaseCurrentStatus PhaseCurrent_ProcessDirect(
	const PhaseCurrentDirectFrame *frame, bool reconstruct_missing,
	PhaseCurrentSample *sample)
{
	float phase_current_a[PHASE_CURRENT_PHASE_COUNT] = { 0.0f, 0.0f, 0.0f };
	uint8_t phase;
	uint8_t missing_phase = PHASE_CURRENT_PHASE_A;
	uint8_t expected_count = reconstruct_missing ? 2U : 3U;

	if ((frame->valid_phase_mask & (uint8_t)~PHASE_CURRENT_VALID_ALL) != 0U ||
		(frame->saturated_phase_mask & (uint8_t)~PHASE_CURRENT_VALID_ALL) != 0U ||
		(frame->saturated_phase_mask &
			(uint8_t)~frame->valid_phase_mask) != 0U ||
		PhaseCurrent_CountBits(frame->valid_phase_mask) != expected_count)
	{
		return PHASE_CURRENT_STATUS_INVALID_PHASE_MASK;
	}

	if ((frame->saturated_phase_mask & frame->valid_phase_mask) != 0U)
	{
		sample->quality = PHASE_CURRENT_QUALITY_SATURATED;
		return PHASE_CURRENT_STATUS_SATURATED;
	}

	for (phase = PHASE_CURRENT_PHASE_A;
		phase < PHASE_CURRENT_PHASE_COUNT; phase++)
	{
		uint8_t phase_mask = (uint8_t)(1U << phase);

		if ((frame->valid_phase_mask & phase_mask) != 0U)
		{
			if (!PhaseCurrent_IsFinite(frame->phase_current_a[phase]))
				return PHASE_CURRENT_STATUS_NONFINITE_SAMPLE;
			phase_current_a[phase] = frame->phase_current_a[phase];
		}
		else
		{
			missing_phase = phase;
		}
	}

	if (reconstruct_missing)
	{
		float sum = phase_current_a[PHASE_CURRENT_PHASE_A] +
			phase_current_a[PHASE_CURRENT_PHASE_B] +
			phase_current_a[PHASE_CURRENT_PHASE_C];

		if (!PhaseCurrent_IsFinite(sum))
			return PHASE_CURRENT_STATUS_NONFINITE_SAMPLE;
		phase_current_a[missing_phase] = -sum;
		PhaseCurrent_SetSampleValues(sample, phase_current_a,
			PHASE_CURRENT_QUALITY_KCL_RECONSTRUCTED);
	}
	else
	{
		PhaseCurrent_SetSampleValues(sample, phase_current_a,
			PHASE_CURRENT_QUALITY_DIRECT);
	}

	return PHASE_CURRENT_STATUS_OK;
}

static PhaseCurrentStatus PhaseCurrent_ProcessDcLink(
	const PhaseCurrentStrategy *strategy, const PhaseCurrentDcLinkFrame *frame,
	PhaseCurrentSample *sample)
{
	const PhaseCurrentDcLinkSectorMapping *sector_mapping;
	float phase_current_a[PHASE_CURRENT_PHASE_COUNT] = { 0.0f, 0.0f, 0.0f };
	uint8_t observed_phase_mask = PHASE_CURRENT_VALID_NONE;
	uint8_t missing_phase = PHASE_CURRENT_PHASE_A;
	uint8_t window;
	uint8_t phase;

	if (frame->sector < 1U ||
		frame->sector > PHASE_CURRENT_DC_LINK_SECTOR_COUNT)
	{
		return PHASE_CURRENT_STATUS_INVALID_SECTOR;
	}

	sector_mapping = &strategy->config.dc_link_sector_map[frame->sector - 1U];
	if (!PhaseCurrent_SectorMappingIsValid(sector_mapping))
		return PHASE_CURRENT_STATUS_INVALID_WINDOW_MAPPING;

	for (window = 0U; window < PHASE_CURRENT_DC_LINK_WINDOW_COUNT; window++)
	{
		const PhaseCurrentDcLinkWindow *observation = &frame->windows[window];
		PhaseCurrentDcLinkWindowId expected_window =
			(window == 0U) ?
			PHASE_CURRENT_DC_LINK_WINDOW_FIRST_ACTIVE_VECTOR :
			PHASE_CURRENT_DC_LINK_WINDOW_SECOND_ACTIVE_VECTOR;
		PhaseCurrentDcLinkSampleMapping mapping =
			(window == 0U) ? sector_mapping->first_window :
			sector_mapping->second_window;
		uint8_t phase_mask;
		float sign;

		if (observation->id != expected_window || !observation->valid)
			return PHASE_CURRENT_STATUS_INVALID_WINDOW;
		if (observation->saturated)
		{
			sample->quality = PHASE_CURRENT_QUALITY_SATURATED;
			return PHASE_CURRENT_STATUS_SATURATED;
		}
		if (!PhaseCurrent_IsFinite(observation->current_a))
			return PHASE_CURRENT_STATUS_NONFINITE_SAMPLE;
		if (!PhaseCurrent_DecodeMapping(mapping, &phase, &sign))
			return PHASE_CURRENT_STATUS_INVALID_WINDOW_MAPPING;

		phase_mask = (uint8_t)(1U << phase);
		if ((observed_phase_mask & phase_mask) != 0U)
			return PHASE_CURRENT_STATUS_INVALID_WINDOW_MAPPING;
		phase_current_a[phase] = sign * observation->current_a;
		observed_phase_mask = (uint8_t)(observed_phase_mask | phase_mask);
	}

	for (phase = PHASE_CURRENT_PHASE_A;
		phase < PHASE_CURRENT_PHASE_COUNT; phase++)
	{
		if ((observed_phase_mask & (uint8_t)(1U << phase)) == 0U)
			missing_phase = phase;
	}

	phase_current_a[missing_phase] =
		-(phase_current_a[PHASE_CURRENT_PHASE_A] +
		  phase_current_a[PHASE_CURRENT_PHASE_B] +
		  phase_current_a[PHASE_CURRENT_PHASE_C]);
	if (!PhaseCurrent_IsFinite(phase_current_a[missing_phase]))
		return PHASE_CURRENT_STATUS_NONFINITE_SAMPLE;

	PhaseCurrent_SetSampleValues(sample, phase_current_a,
		PHASE_CURRENT_QUALITY_DC_LINK_RECONSTRUCTED);
	return PHASE_CURRENT_STATUS_OK;
}

PhaseCurrentStatus PhaseCurrentStrategy_MakeDefaultConfig(
	PhaseCurrentTopology topology, PhaseCurrentStrategyConfig *config)
{
	if (config == NULL)
		return PHASE_CURRENT_STATUS_NULL_ARGUMENT;
	memset(config, 0, sizeof(*config));
	if (!PhaseCurrent_IsSupportedTopology(topology))
		return PHASE_CURRENT_STATUS_UNSUPPORTED_TOPOLOGY;

	config->topology = topology;
	memcpy(config->dc_link_sector_map, s_default_dc_link_sector_map,
		sizeof(config->dc_link_sector_map));
	return PHASE_CURRENT_STATUS_OK;
}

void PhaseCurrentStrategy_Reset(PhaseCurrentStrategy *strategy)
{
	if (strategy != NULL)
		memset(strategy, 0, sizeof(*strategy));
}

PhaseCurrentStatus PhaseCurrentStrategy_Configure(
	PhaseCurrentStrategy *strategy, const PhaseCurrentStrategyConfig *config)
{
	PhaseCurrentStrategyConfig config_copy;
	uint8_t sector;

	if (strategy == NULL || config == NULL)
		return PHASE_CURRENT_STATUS_NULL_ARGUMENT;
	config_copy = *config;
	if (!PhaseCurrent_IsSupportedTopology(config_copy.topology))
	{
		PhaseCurrentStrategy_Reset(strategy);
		return PHASE_CURRENT_STATUS_UNSUPPORTED_TOPOLOGY;
	}

	if (config_copy.topology == PHASE_CURRENT_TOPOLOGY_DC_LINK_1_SHUNT)
	{
		for (sector = 0U; sector < PHASE_CURRENT_DC_LINK_SECTOR_COUNT; sector++)
		{
			if (!PhaseCurrent_SectorMappingIsValid(
				&config_copy.dc_link_sector_map[sector]))
			{
				PhaseCurrentStrategy_Reset(strategy);
				return PHASE_CURRENT_STATUS_INVALID_WINDOW_MAPPING;
			}
		}
	}

	PhaseCurrentStrategy_Reset(strategy);
	strategy->config = config_copy;
	strategy->is_configured = true;
	return PHASE_CURRENT_STATUS_OK;
}

PhaseCurrentStatus PhaseCurrentStrategy_GetDcLinkMapping(
	const PhaseCurrentStrategy *strategy, uint8_t sector,
	PhaseCurrentDcLinkWindowId window,
	PhaseCurrentDcLinkSampleMapping *mapping)
{
	const PhaseCurrentDcLinkSectorMapping *sector_mapping;

	if (strategy == NULL || mapping == NULL)
		return PHASE_CURRENT_STATUS_NULL_ARGUMENT;
	if (!strategy->is_configured)
		return PHASE_CURRENT_STATUS_NOT_CONFIGURED;
	if (strategy->config.topology != PHASE_CURRENT_TOPOLOGY_DC_LINK_1_SHUNT)
		return PHASE_CURRENT_STATUS_UNSUPPORTED_TOPOLOGY;
	if (sector < 1U || sector > PHASE_CURRENT_DC_LINK_SECTOR_COUNT)
		return PHASE_CURRENT_STATUS_INVALID_SECTOR;

	sector_mapping = &strategy->config.dc_link_sector_map[sector - 1U];
	if (!PhaseCurrent_SectorMappingIsValid(sector_mapping))
		return PHASE_CURRENT_STATUS_INVALID_WINDOW_MAPPING;
	if (window == PHASE_CURRENT_DC_LINK_WINDOW_FIRST_ACTIVE_VECTOR)
		*mapping = sector_mapping->first_window;
	else if (window == PHASE_CURRENT_DC_LINK_WINDOW_SECOND_ACTIVE_VECTOR)
		*mapping = sector_mapping->second_window;
	else
		return PHASE_CURRENT_STATUS_INVALID_WINDOW;

	return PHASE_CURRENT_STATUS_OK;
}

PhaseCurrentStatus PhaseCurrentStrategy_Process(
	PhaseCurrentStrategy *strategy, const PhaseCurrentAcquisition *acquisition,
	PhaseCurrentSample *sample)
{
	PhaseCurrentStatus status;

	if (strategy == NULL || acquisition == NULL || sample == NULL)
		return PHASE_CURRENT_STATUS_NULL_ARGUMENT;

	PhaseCurrent_ClearSample(sample, acquisition->sequence);
	if (!strategy->is_configured)
		return PHASE_CURRENT_STATUS_NOT_CONFIGURED;

	status = PhaseCurrent_CheckSequence(strategy, acquisition->sequence, sample);
	if (status != PHASE_CURRENT_STATUS_OK)
		return status;

	switch (strategy->config.topology)
	{
	case PHASE_CURRENT_TOPOLOGY_INLINE_3_SHUNT:
	case PHASE_CURRENT_TOPOLOGY_LOW_SIDE_3_SHUNT:
		return PhaseCurrent_ProcessDirect(&acquisition->data.direct, false,
			sample);
	case PHASE_CURRENT_TOPOLOGY_LOW_SIDE_2_SHUNT:
		return PhaseCurrent_ProcessDirect(&acquisition->data.direct, true,
			sample);
	case PHASE_CURRENT_TOPOLOGY_DC_LINK_1_SHUNT:
		return PhaseCurrent_ProcessDcLink(strategy, &acquisition->data.dc_link,
			sample);
	default:
		return PHASE_CURRENT_STATUS_UNSUPPORTED_TOPOLOGY;
	}
}
