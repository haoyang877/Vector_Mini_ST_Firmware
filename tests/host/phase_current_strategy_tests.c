#include "phase_current_strategy.h"

#include <stdint.h>
#include <string.h>

#define TEST_CHECK(condition) do { if (!(condition)) return __LINE__; } while (0)
#define TEST_EPSILON_A (0.00001f)

static float PhaseCurrentTest_Abs(float value)
{
	return value >= 0.0f ? value : -value;
}

static bool PhaseCurrentTest_Near(float actual, float expected)
{
	return PhaseCurrentTest_Abs(actual - expected) <= TEST_EPSILON_A;
}

static float PhaseCurrentTest_Nan(void)
{
	union
	{
		uint32_t bits;
		float value;
	} value;

	value.bits = 0x7FC00000UL;
	return value.value;
}

static PhaseCurrentStatus PhaseCurrentTest_Configure(
	PhaseCurrentStrategy *strategy, PhaseCurrentTopology topology)
{
	PhaseCurrentStrategyConfig config;
	PhaseCurrentStatus status;

	PhaseCurrentStrategy_Reset(strategy);
	status = PhaseCurrentStrategy_MakeDefaultConfig(topology, &config);
	if (status != PHASE_CURRENT_STATUS_OK)
		return status;
	return PhaseCurrentStrategy_Configure(strategy, &config);
}

static void PhaseCurrentTest_SetDirectFrame(PhaseCurrentAcquisition *acquisition,
	uint32_t sequence, float ia, float ib, float ic, uint8_t valid_mask)
{
	memset(acquisition, 0, sizeof(*acquisition));
	acquisition->sequence = sequence;
	acquisition->data.direct.phase_current_a[PHASE_CURRENT_PHASE_A] = ia;
	acquisition->data.direct.phase_current_a[PHASE_CURRENT_PHASE_B] = ib;
	acquisition->data.direct.phase_current_a[PHASE_CURRENT_PHASE_C] = ic;
	acquisition->data.direct.valid_phase_mask = valid_mask;
}

static int PhaseCurrentTest_CheckSample(const PhaseCurrentSample *sample,
	float ia, float ib, float ic, PhaseCurrentQuality quality,
	uint32_t sequence)
{
	TEST_CHECK(PhaseCurrentTest_Near(sample->ia_a, ia));
	TEST_CHECK(PhaseCurrentTest_Near(sample->ib_a, ib));
	TEST_CHECK(PhaseCurrentTest_Near(sample->ic_a, ic));
	TEST_CHECK(sample->valid_phase_mask == PHASE_CURRENT_VALID_ALL);
	TEST_CHECK(sample->quality == quality);
	TEST_CHECK(sample->sequence == sequence);
	return 0;
}

static int PhaseCurrentTest_ThreeShuntTopologies(void)
{
	const PhaseCurrentTopology topologies[] =
	{
		PHASE_CURRENT_TOPOLOGY_INLINE_3_SHUNT,
		PHASE_CURRENT_TOPOLOGY_LOW_SIDE_3_SHUNT
	};
	PhaseCurrentStrategy strategy;
	PhaseCurrentAcquisition acquisition;
	PhaseCurrentSample sample;
	uint8_t topology_index;

	for (topology_index = 0U;
		topology_index < (uint8_t)(sizeof(topologies) / sizeof(topologies[0]));
		topology_index++)
	{
		TEST_CHECK(PhaseCurrentTest_Configure(&strategy,
			topologies[topology_index]) == PHASE_CURRENT_STATUS_OK);
		PhaseCurrentTest_SetDirectFrame(&acquisition, 10U, 3.0f, -1.0f,
			-2.0f, PHASE_CURRENT_VALID_ALL);
		TEST_CHECK(PhaseCurrentStrategy_Process(&strategy, &acquisition,
			&sample) == PHASE_CURRENT_STATUS_OK);
		TEST_CHECK(PhaseCurrentTest_CheckSample(&sample, 3.0f, -1.0f, -2.0f,
			PHASE_CURRENT_QUALITY_DIRECT, 10U) == 0);
	}
	return 0;
}

static int PhaseCurrentTest_TwoShuntMissingEveryPhase(void)
{
	const uint8_t valid_masks[] =
	{
		PHASE_CURRENT_VALID_B | PHASE_CURRENT_VALID_C,
		PHASE_CURRENT_VALID_A | PHASE_CURRENT_VALID_C,
		PHASE_CURRENT_VALID_A | PHASE_CURRENT_VALID_B
	};
	PhaseCurrentStrategy strategy;
	PhaseCurrentAcquisition acquisition;
	PhaseCurrentSample sample;
	uint8_t missing_phase;

	TEST_CHECK(PhaseCurrentTest_Configure(&strategy,
		PHASE_CURRENT_TOPOLOGY_LOW_SIDE_2_SHUNT) == PHASE_CURRENT_STATUS_OK);
	for (missing_phase = 0U; missing_phase < PHASE_CURRENT_PHASE_COUNT;
		missing_phase++)
	{
		PhaseCurrentTest_SetDirectFrame(&acquisition,
			(uint32_t)(20U + missing_phase), 3.0f, -1.0f, -2.0f,
			valid_masks[missing_phase]);
		TEST_CHECK(PhaseCurrentStrategy_Process(&strategy, &acquisition,
			&sample) == PHASE_CURRENT_STATUS_OK);
		TEST_CHECK(PhaseCurrentTest_CheckSample(&sample, 3.0f, -1.0f, -2.0f,
			PHASE_CURRENT_QUALITY_KCL_RECONSTRUCTED,
			(uint32_t)(20U + missing_phase)) == 0);
	}
	return 0;
}

static float PhaseCurrentTest_DcLinkObservation(
	PhaseCurrentDcLinkSampleMapping mapping, float ia, float ib, float ic)
{
	switch (mapping)
	{
	case PHASE_CURRENT_DC_LINK_MAP_POSITIVE_A:
		return ia;
	case PHASE_CURRENT_DC_LINK_MAP_NEGATIVE_A:
		return -ia;
	case PHASE_CURRENT_DC_LINK_MAP_POSITIVE_B:
		return ib;
	case PHASE_CURRENT_DC_LINK_MAP_NEGATIVE_B:
		return -ib;
	case PHASE_CURRENT_DC_LINK_MAP_POSITIVE_C:
		return ic;
	case PHASE_CURRENT_DC_LINK_MAP_NEGATIVE_C:
		return -ic;
	default:
		return 0.0f;
	}
}

static int PhaseCurrentTest_AllSixDcLinkSectors(void)
{
	static const PhaseCurrentDcLinkSectorMapping expected_mapping[
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
	PhaseCurrentStrategy strategy;
	PhaseCurrentAcquisition acquisition;
	PhaseCurrentSample sample;
	PhaseCurrentDcLinkSampleMapping first_mapping;
	PhaseCurrentDcLinkSampleMapping second_mapping;
	uint8_t sector;

	TEST_CHECK(PhaseCurrentTest_Configure(&strategy,
		PHASE_CURRENT_TOPOLOGY_DC_LINK_1_SHUNT) == PHASE_CURRENT_STATUS_OK);
	for (sector = 1U; sector <= PHASE_CURRENT_DC_LINK_SECTOR_COUNT; sector++)
	{
		TEST_CHECK(PhaseCurrentStrategy_GetDcLinkMapping(&strategy, sector,
			PHASE_CURRENT_DC_LINK_WINDOW_FIRST_ACTIVE_VECTOR,
			&first_mapping) == PHASE_CURRENT_STATUS_OK);
		TEST_CHECK(PhaseCurrentStrategy_GetDcLinkMapping(&strategy, sector,
			PHASE_CURRENT_DC_LINK_WINDOW_SECOND_ACTIVE_VECTOR,
			&second_mapping) == PHASE_CURRENT_STATUS_OK);
		TEST_CHECK(first_mapping == expected_mapping[sector - 1U].first_window);
		TEST_CHECK(second_mapping == expected_mapping[sector - 1U].second_window);

		memset(&acquisition, 0, sizeof(acquisition));
		acquisition.sequence = (uint32_t)(100U + sector);
		acquisition.data.dc_link.sector = sector;
		acquisition.data.dc_link.windows[0].id =
			PHASE_CURRENT_DC_LINK_WINDOW_FIRST_ACTIVE_VECTOR;
		acquisition.data.dc_link.windows[0].valid = true;
		acquisition.data.dc_link.windows[0].current_a =
			PhaseCurrentTest_DcLinkObservation(first_mapping,
				3.0f, -1.0f, -2.0f);
		acquisition.data.dc_link.windows[1].id =
			PHASE_CURRENT_DC_LINK_WINDOW_SECOND_ACTIVE_VECTOR;
		acquisition.data.dc_link.windows[1].valid = true;
		acquisition.data.dc_link.windows[1].current_a =
			PhaseCurrentTest_DcLinkObservation(second_mapping,
				3.0f, -1.0f, -2.0f);

		TEST_CHECK(PhaseCurrentStrategy_Process(&strategy, &acquisition,
			&sample) == PHASE_CURRENT_STATUS_OK);
		TEST_CHECK(PhaseCurrentTest_CheckSample(&sample, 3.0f, -1.0f, -2.0f,
			PHASE_CURRENT_QUALITY_DC_LINK_RECONSTRUCTED,
			(uint32_t)(100U + sector)) == 0);
	}
	return 0;
}

static int PhaseCurrentTest_InvalidAndSaturatedInputs(void)
{
	PhaseCurrentStrategy strategy;
	PhaseCurrentStrategyConfig config;
	PhaseCurrentAcquisition acquisition;
	PhaseCurrentSample sample;

	TEST_CHECK(PhaseCurrentTest_Configure(&strategy,
		PHASE_CURRENT_TOPOLOGY_INLINE_3_SHUNT) == PHASE_CURRENT_STATUS_OK);
	PhaseCurrentTest_SetDirectFrame(&acquisition, 1U, 1.0f, 2.0f, -3.0f,
		PHASE_CURRENT_VALID_A | PHASE_CURRENT_VALID_B);
	TEST_CHECK(PhaseCurrentStrategy_Process(&strategy, &acquisition, &sample) ==
		PHASE_CURRENT_STATUS_INVALID_PHASE_MASK);
	TEST_CHECK(sample.valid_phase_mask == PHASE_CURRENT_VALID_NONE);

	PhaseCurrentTest_SetDirectFrame(&acquisition, 2U, 1.0f, 2.0f, -3.0f,
		PHASE_CURRENT_VALID_ALL);
	acquisition.data.direct.saturated_phase_mask = PHASE_CURRENT_VALID_B;
	TEST_CHECK(PhaseCurrentStrategy_Process(&strategy, &acquisition, &sample) ==
		PHASE_CURRENT_STATUS_SATURATED);
	TEST_CHECK(sample.quality == PHASE_CURRENT_QUALITY_SATURATED);

	PhaseCurrentTest_SetDirectFrame(&acquisition, 3U,
		PhaseCurrentTest_Nan(), 2.0f, -3.0f, PHASE_CURRENT_VALID_ALL);
	TEST_CHECK(PhaseCurrentStrategy_Process(&strategy, &acquisition, &sample) ==
		PHASE_CURRENT_STATUS_NONFINITE_SAMPLE);

	TEST_CHECK(PhaseCurrentTest_Configure(&strategy,
		PHASE_CURRENT_TOPOLOGY_LOW_SIDE_2_SHUNT) == PHASE_CURRENT_STATUS_OK);
	PhaseCurrentTest_SetDirectFrame(&acquisition, 1U, 1.0f, 2.0f, -3.0f,
		PHASE_CURRENT_VALID_A);
	TEST_CHECK(PhaseCurrentStrategy_Process(&strategy, &acquisition, &sample) ==
		PHASE_CURRENT_STATUS_INVALID_PHASE_MASK);
	PhaseCurrentTest_SetDirectFrame(&acquisition, 2U, 1.0f, 2.0f, -3.0f,
		PHASE_CURRENT_VALID_ALL);
	TEST_CHECK(PhaseCurrentStrategy_Process(&strategy, &acquisition, &sample) ==
		PHASE_CURRENT_STATUS_INVALID_PHASE_MASK);

	TEST_CHECK(PhaseCurrentStrategy_MakeDefaultConfig(
		PHASE_CURRENT_TOPOLOGY_DC_LINK_1_SHUNT, &config) ==
		PHASE_CURRENT_STATUS_OK);
	config.dc_link_sector_map[0].second_window =
		PHASE_CURRENT_DC_LINK_MAP_NEGATIVE_A;
	TEST_CHECK(PhaseCurrentStrategy_Configure(&strategy, &config) ==
		PHASE_CURRENT_STATUS_INVALID_WINDOW_MAPPING);
	TEST_CHECK(!strategy.is_configured);
	return 0;
}

static void PhaseCurrentTest_SetValidDcLinkFrame(
	PhaseCurrentAcquisition *acquisition, uint32_t sequence, uint8_t sector)
{
	memset(acquisition, 0, sizeof(*acquisition));
	acquisition->sequence = sequence;
	acquisition->data.dc_link.sector = sector;
	acquisition->data.dc_link.windows[0].id =
		PHASE_CURRENT_DC_LINK_WINDOW_FIRST_ACTIVE_VECTOR;
	acquisition->data.dc_link.windows[0].current_a = 3.0f;
	acquisition->data.dc_link.windows[0].valid = true;
	acquisition->data.dc_link.windows[1].id =
		PHASE_CURRENT_DC_LINK_WINDOW_SECOND_ACTIVE_VECTOR;
	acquisition->data.dc_link.windows[1].current_a = 2.0f;
	acquisition->data.dc_link.windows[1].valid = true;
}

static int PhaseCurrentTest_DcLinkFailures(void)
{
	PhaseCurrentStrategy strategy;
	PhaseCurrentAcquisition acquisition;
	PhaseCurrentSample sample;

	TEST_CHECK(PhaseCurrentTest_Configure(&strategy,
		PHASE_CURRENT_TOPOLOGY_DC_LINK_1_SHUNT) == PHASE_CURRENT_STATUS_OK);

	PhaseCurrentTest_SetValidDcLinkFrame(&acquisition, 1U, 0U);
	TEST_CHECK(PhaseCurrentStrategy_Process(&strategy, &acquisition, &sample) ==
		PHASE_CURRENT_STATUS_INVALID_SECTOR);
	PhaseCurrentTest_SetValidDcLinkFrame(&acquisition, 2U, 7U);
	TEST_CHECK(PhaseCurrentStrategy_Process(&strategy, &acquisition, &sample) ==
		PHASE_CURRENT_STATUS_INVALID_SECTOR);

	PhaseCurrentTest_SetValidDcLinkFrame(&acquisition, 3U, 1U);
	acquisition.data.dc_link.windows[0].valid = false;
	TEST_CHECK(PhaseCurrentStrategy_Process(&strategy, &acquisition, &sample) ==
		PHASE_CURRENT_STATUS_INVALID_WINDOW);
	PhaseCurrentTest_SetValidDcLinkFrame(&acquisition, 4U, 1U);
	acquisition.data.dc_link.windows[1].id =
		PHASE_CURRENT_DC_LINK_WINDOW_FIRST_ACTIVE_VECTOR;
	TEST_CHECK(PhaseCurrentStrategy_Process(&strategy, &acquisition, &sample) ==
		PHASE_CURRENT_STATUS_INVALID_WINDOW);
	PhaseCurrentTest_SetValidDcLinkFrame(&acquisition, 5U, 1U);
	acquisition.data.dc_link.windows[1].saturated = true;
	TEST_CHECK(PhaseCurrentStrategy_Process(&strategy, &acquisition, &sample) ==
		PHASE_CURRENT_STATUS_SATURATED);
	TEST_CHECK(sample.quality == PHASE_CURRENT_QUALITY_SATURATED);
	return 0;
}

static int PhaseCurrentTest_SequenceRules(void)
{
	PhaseCurrentStrategy strategy;
	PhaseCurrentAcquisition acquisition;
	PhaseCurrentSample sample;

	TEST_CHECK(PhaseCurrentTest_Configure(&strategy,
		PHASE_CURRENT_TOPOLOGY_INLINE_3_SHUNT) == PHASE_CURRENT_STATUS_OK);
	PhaseCurrentTest_SetDirectFrame(&acquisition, 50U, 1.0f, 2.0f, -3.0f,
		PHASE_CURRENT_VALID_ALL);
	TEST_CHECK(PhaseCurrentStrategy_Process(&strategy, &acquisition, &sample) ==
		PHASE_CURRENT_STATUS_OK);
	TEST_CHECK(sample.sequence == 50U);
	TEST_CHECK(PhaseCurrentStrategy_Process(&strategy, &acquisition, &sample) ==
		PHASE_CURRENT_STATUS_DUPLICATE_SEQUENCE);
	TEST_CHECK(sample.quality == PHASE_CURRENT_QUALITY_STALE);

	acquisition.sequence = 49U;
	TEST_CHECK(PhaseCurrentStrategy_Process(&strategy, &acquisition, &sample) ==
		PHASE_CURRENT_STATUS_OUT_OF_ORDER_SEQUENCE);
	TEST_CHECK(sample.quality == PHASE_CURRENT_QUALITY_STALE);

	TEST_CHECK(PhaseCurrentTest_Configure(&strategy,
		PHASE_CURRENT_TOPOLOGY_INLINE_3_SHUNT) == PHASE_CURRENT_STATUS_OK);
	acquisition.sequence = 0xFFFFFFFFUL;
	TEST_CHECK(PhaseCurrentStrategy_Process(&strategy, &acquisition, &sample) ==
		PHASE_CURRENT_STATUS_OK);
	acquisition.sequence = 0U;
	TEST_CHECK(PhaseCurrentStrategy_Process(&strategy, &acquisition, &sample) ==
		PHASE_CURRENT_STATUS_OK);
	TEST_CHECK(sample.sequence == 0U);
	return 0;
}

int PhaseCurrentStrategy_RunHostTests(void)
{
	int result;

	result = PhaseCurrentTest_ThreeShuntTopologies();
	if (result != 0)
		return result;
	result = PhaseCurrentTest_TwoShuntMissingEveryPhase();
	if (result != 0)
		return result;
	result = PhaseCurrentTest_AllSixDcLinkSectors();
	if (result != 0)
		return result;
	result = PhaseCurrentTest_InvalidAndSaturatedInputs();
	if (result != 0)
		return result;
	result = PhaseCurrentTest_DcLinkFailures();
	if (result != 0)
		return result;
	return PhaseCurrentTest_SequenceRules();
}
