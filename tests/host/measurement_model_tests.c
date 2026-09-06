#include "measurement_model.h"

#include <string.h>

#define TEST_CHECK(condition) do { if (!(condition)) return __LINE__; } while (0)
#define TEST_EPSILON_A (0.0001f)

static float MeasurementModelTest_Abs(float value)
{
	return value >= 0.0f ? value : -value;
}

static bool MeasurementModelTest_Near(float actual, float expected)
{
	return MeasurementModelTest_Abs(actual - expected) <= TEST_EPSILON_A;
}

static MeasurementModelConfig MeasurementModelTest_CreateCommonConfig(void)
{
	MeasurementModelConfig config;

	memset(&config, 0, sizeof(config));
	config.bus_voltage_v_per_count = 0.01f;
	config.bus_voltage_filter_alpha = 0.25f;
	config.overcurrent_trip_a = 10.0f;
	config.overvoltage_trip_v = 30.0f;
	config.undervoltage_trip_v = 10.0f;
	config.overcurrent_confirm_cycles = 1U;
	config.voltage_confirm_cycles = 1U;
	return config;
}

static MeasurementModelConfig MeasurementModelTest_CreateLegacyConfig(void)
{
	MeasurementModelConfig config = MeasurementModelTest_CreateCommonConfig();

	config.phase_channel_index[0] = 0U;
	config.phase_channel_index[1] = 1U;
	config.phase_channel_index[2] = 2U;
	config.maximum_valid_offset_adc[0] = 4095U;
	config.maximum_valid_offset_adc[1] = 4095U;
	config.maximum_valid_offset_adc[2] = 4095U;
	config.current_a_per_count[0] = -0.01f;
	config.current_a_per_count[1] = -0.02f;
	config.current_a_per_count[2] = -0.03f;
	return config;
}

static void MeasurementModelTest_SetChannel(
	MeasurementCurrentChannelConfig *channel, uint8_t acquisition_index,
	MeasurementCurrentChannelRole role, uint16_t offset_adc,
	float current_a_per_count)
{
	memset(channel, 0, sizeof(*channel));
	channel->acquisition_index = acquisition_index;
	channel->role = role;
	channel->offset_adc = offset_adc;
	channel->maximum_valid_offset_adc = 4095U;
	channel->current_a_per_count = current_a_per_count;
}

static MeasurementModelConfig MeasurementModelTest_CreateThreeShuntConfig(void)
{
	MeasurementModelConfig config = MeasurementModelTest_CreateCommonConfig();

	config.current_sense.topology =
		PHASE_CURRENT_TOPOLOGY_LOW_SIDE_3_SHUNT;
	config.current_sense.physical_channel_count = 3U;
	/* Descriptor order and acquisition order deliberately differ. */
	MeasurementModelTest_SetChannel(&config.current_sense.channels[0], 2U,
		MEASUREMENT_CURRENT_CHANNEL_ROLE_PHASE_B, 100, 0.02f);
	MeasurementModelTest_SetChannel(&config.current_sense.channels[1], 0U,
		MEASUREMENT_CURRENT_CHANNEL_ROLE_PHASE_C, 100, -0.01f);
	MeasurementModelTest_SetChannel(&config.current_sense.channels[2], 1U,
		MEASUREMENT_CURRENT_CHANNEL_ROLE_PHASE_A, 100, 0.01f);
	return config;
}

static MeasurementModelConfig MeasurementModelTest_CreateTwoShuntConfig(void)
{
	MeasurementModelConfig config = MeasurementModelTest_CreateCommonConfig();

	config.current_sense.topology =
		PHASE_CURRENT_TOPOLOGY_LOW_SIDE_2_SHUNT;
	config.current_sense.physical_channel_count = 2U;
	MeasurementModelTest_SetChannel(&config.current_sense.channels[0], 1U,
		MEASUREMENT_CURRENT_CHANNEL_ROLE_PHASE_A, 100, 0.01f);
	/* Selected endpoints occupy non-compact BSP slots 1 and 2. */
	MeasurementModelTest_SetChannel(&config.current_sense.channels[1], 2U,
		MEASUREMENT_CURRENT_CHANNEL_ROLE_PHASE_B, 100, 0.02f);
	return config;
}

static MeasurementModelConfig MeasurementModelTest_CreateOneShuntConfig(void)
{
	MeasurementModelConfig config = MeasurementModelTest_CreateCommonConfig();

	config.current_sense.topology =
		PHASE_CURRENT_TOPOLOGY_DC_LINK_1_SHUNT;
	config.current_sense.physical_channel_count = 1U;
	MeasurementModelTest_SetChannel(&config.current_sense.channels[0], 2U,
		MEASUREMENT_CURRENT_CHANNEL_ROLE_DC_LINK, 2048, 0.01f);
	return config;
}

static void MeasurementModelTest_SetDirectInput(
	MeasurementModelAcquisitionInput *input, uint32_t sequence,
	uint8_t sample_count, uint8_t valid_phase_mask)
{
	memset(input, 0, sizeof(*input));
	input->current.sequence = sequence;
	input->current.sample_count = sample_count;
	input->current.valid_phase_mask = valid_phase_mask;
	input->bus_voltage_adc = 1000U;
}

static int MeasurementModelTest_LegacyCompatibility(void)
{
	MeasurementModelContext context;
	MeasurementModelConfig config = MeasurementModelTest_CreateLegacyConfig();
	MeasurementModelInput input;
	MeasurementModelOutput output;

	MeasurementModel_Reset(&context);
	TEST_CHECK(MeasurementModel_Configure(&context, &config));
	TEST_CHECK(!context.uses_acquisition_pipeline);
	memset(&input, 0, sizeof(input));
	input.phase_a_adc = 100U;
	input.phase_b_adc = 100U;
	input.phase_c_adc = 100U;
	input.bus_voltage_adc = 1000U;
	TEST_CHECK(MeasurementModel_Update(&context, &input, &output));
	TEST_CHECK(output.phase_a_current_a == -1.0f);
	TEST_CHECK(output.phase_b_current_a == -2.0f);
	TEST_CHECK(output.phase_c_current_a == -3.0f);
	TEST_CHECK(output.bus_voltage_filtered_v == 2.5f);
	TEST_CHECK(output.phase_current_quality == PHASE_CURRENT_QUALITY_DIRECT);
	TEST_CHECK(MeasurementModel_Update(&context, &input, &output));
	TEST_CHECK(output.bus_voltage_filtered_v == 4.375f);
	return 0;
}

static int MeasurementModelTest_ThreeShuntAcquisitionMapping(void)
{
	MeasurementModelContext context;
	MeasurementModelConfig config =
		MeasurementModelTest_CreateThreeShuntConfig();
	MeasurementModelAcquisitionInput input;
	MeasurementModelOutput output;

	MeasurementModel_Reset(&context);
	TEST_CHECK(MeasurementModel_Configure(&context, &config));
	TEST_CHECK(context.uses_acquisition_pipeline);
	MeasurementModelTest_SetDirectInput(&input, 10U, 3U,
		PHASE_CURRENT_VALID_ALL);
	/* Acquisition 0 -> C, 1 -> A, 2 -> B. */
	input.current.raw_observation_adc[0] = 300U;
	input.current.raw_observation_adc[1] = 400U;
	input.current.raw_observation_adc[2] = 50U;
	TEST_CHECK(MeasurementModel_UpdateAcquisition(&context, &input, &output) ==
		PHASE_CURRENT_STATUS_OK);
	TEST_CHECK(MeasurementModelTest_Near(output.phase_a_current_a, 3.0f));
	TEST_CHECK(MeasurementModelTest_Near(output.phase_b_current_a, -1.0f));
	TEST_CHECK(MeasurementModelTest_Near(output.phase_c_current_a, -2.0f));
	TEST_CHECK(output.phase_current_quality == PHASE_CURRENT_QUALITY_DIRECT);
	TEST_CHECK(output.current_sequence == 10U);
	TEST_CHECK(output.bus_voltage_filtered_v == 2.5f);
	return 0;
}

static int MeasurementModelTest_TwoShuntKclReconstruction(void)
{
	MeasurementModelContext context;
	MeasurementModelConfig config =
		MeasurementModelTest_CreateTwoShuntConfig();
	MeasurementModelAcquisitionInput input;
	MeasurementModelOutput output;

	MeasurementModel_Reset(&context);
	TEST_CHECK(MeasurementModel_Configure(&context, &config));
	MeasurementModelTest_SetDirectInput(&input, 20U, 2U,
		PHASE_CURRENT_VALID_A | PHASE_CURRENT_VALID_B);
	input.current.raw_observation_adc[2] = 50U;
	input.current.raw_observation_adc[1] = 400U;
	TEST_CHECK(MeasurementModel_UpdateAcquisition(&context, &input, &output) ==
		PHASE_CURRENT_STATUS_OK);
	TEST_CHECK(MeasurementModelTest_Near(output.phase_a_current_a, 3.0f));
	TEST_CHECK(MeasurementModelTest_Near(output.phase_b_current_a, -1.0f));
	TEST_CHECK(MeasurementModelTest_Near(output.phase_c_current_a, -2.0f));
	TEST_CHECK(output.phase_current_quality ==
		PHASE_CURRENT_QUALITY_KCL_RECONSTRUCTED);
	return 0;
}

static int32_t MeasurementModelTest_DcLinkMilliampObservation(
	PhaseCurrentDcLinkSampleMapping mapping)
{
	switch (mapping)
	{
		case PHASE_CURRENT_DC_LINK_MAP_POSITIVE_A: return 3000;
		case PHASE_CURRENT_DC_LINK_MAP_NEGATIVE_A: return -3000;
		case PHASE_CURRENT_DC_LINK_MAP_POSITIVE_B: return -1000;
		case PHASE_CURRENT_DC_LINK_MAP_NEGATIVE_B: return 1000;
		case PHASE_CURRENT_DC_LINK_MAP_POSITIVE_C: return -2000;
		case PHASE_CURRENT_DC_LINK_MAP_NEGATIVE_C: return 2000;
		default: return 0;
	}
}

static int MeasurementModelTest_OneShuntAllSectors(void)
{
	MeasurementModelContext context;
	MeasurementModelConfig config =
		MeasurementModelTest_CreateOneShuntConfig();
	MeasurementModelAcquisitionInput input;
	MeasurementModelOutput output;
	PhaseCurrentStrategyConfig strategy_config;
	uint8_t sector;

	TEST_CHECK(PhaseCurrentStrategy_MakeDefaultConfig(
		PHASE_CURRENT_TOPOLOGY_DC_LINK_1_SHUNT, &strategy_config) ==
		PHASE_CURRENT_STATUS_OK);
	MeasurementModel_Reset(&context);
	TEST_CHECK(MeasurementModel_Configure(&context, &config));
	for (sector = 1U; sector <= PHASE_CURRENT_DC_LINK_SECTOR_COUNT; sector++)
	{
		int32_t first_milliamps = MeasurementModelTest_DcLinkMilliampObservation(
			strategy_config.dc_link_sector_map[sector - 1U].first_window);
		int32_t second_milliamps = MeasurementModelTest_DcLinkMilliampObservation(
			strategy_config.dc_link_sector_map[sector - 1U].second_window);

		memset(&input, 0, sizeof(input));
		input.current.sequence = (uint32_t)(100U + sector);
		input.current.sample_count = 2U;
		input.current.sector = sector;
		input.current.windows[0] =
			PHASE_CURRENT_DC_LINK_WINDOW_FIRST_ACTIVE_VECTOR;
		input.current.windows[1] =
			PHASE_CURRENT_DC_LINK_WINDOW_SECOND_ACTIVE_VECTOR;
		input.current.raw_observation_adc[0] =
			(uint32_t)(2048 + first_milliamps / 10);
		input.current.raw_observation_adc[1] =
			(uint32_t)(2048 + second_milliamps / 10);
		input.bus_voltage_adc = 1000U;
		TEST_CHECK(MeasurementModel_UpdateAcquisition(&context, &input,
			&output) == PHASE_CURRENT_STATUS_OK);
		TEST_CHECK(MeasurementModelTest_Near(output.phase_a_current_a, 3.0f));
		TEST_CHECK(MeasurementModelTest_Near(output.phase_b_current_a, -1.0f));
		TEST_CHECK(MeasurementModelTest_Near(output.phase_c_current_a, -2.0f));
		TEST_CHECK(output.phase_current_quality ==
			PHASE_CURRENT_QUALITY_DC_LINK_RECONSTRUCTED);
	}
	return 0;
}

static int MeasurementModelTest_ProtectionConfirmation(void)
{
	MeasurementModelContext context;
	MeasurementModelConfig config =
		MeasurementModelTest_CreateThreeShuntConfig();
	MeasurementModelAcquisitionInput input;
	MeasurementModelOutput output;

	config.bus_voltage_filter_alpha = 1.0f;
	config.overcurrent_confirm_cycles = 2U;
	config.voltage_confirm_cycles = 2U;
	MeasurementModel_Reset(&context);
	TEST_CHECK(MeasurementModel_Configure(&context, &config));
	MeasurementModelTest_SetDirectInput(&input, 1U, 3U,
		PHASE_CURRENT_VALID_ALL);
	input.protection_is_active = true;
	input.bus_voltage_adc = 4000U;
	input.current.raw_observation_adc[0] = 100U;
	input.current.raw_observation_adc[1] = 1200U;
	input.current.raw_observation_adc[2] = 100U;
	TEST_CHECK(MeasurementModel_UpdateAcquisition(&context, &input, &output) ==
		PHASE_CURRENT_STATUS_OK);
	TEST_CHECK((output.faults & MEASUREMENT_FAULT_OVER_CURRENT) == 0U);
	TEST_CHECK((output.faults & MEASUREMENT_FAULT_OVER_VOLTAGE) == 0U);
	input.current.sequence = 2U;
	TEST_CHECK(MeasurementModel_UpdateAcquisition(&context, &input, &output) ==
		PHASE_CURRENT_STATUS_OK);
	TEST_CHECK((output.faults & MEASUREMENT_FAULT_OVER_CURRENT) != 0U);
	TEST_CHECK((output.faults & MEASUREMENT_FAULT_OVER_VOLTAGE) != 0U);

	input.current.sequence = 3U;
	input.bus_voltage_adc = 500U;
	input.current.raw_observation_adc[1] = 100U;
	TEST_CHECK(MeasurementModel_UpdateAcquisition(&context, &input, &output) ==
		PHASE_CURRENT_STATUS_OK);
	TEST_CHECK((output.faults & MEASUREMENT_FAULT_UNDER_VOLTAGE) == 0U);
	input.current.sequence = 4U;
	TEST_CHECK(MeasurementModel_UpdateAcquisition(&context, &input, &output) ==
		PHASE_CURRENT_STATUS_OK);
	TEST_CHECK((output.faults & MEASUREMENT_FAULT_UNDER_VOLTAGE) != 0U);
	return 0;
}

static int MeasurementModelTest_InvalidConfig(void)
{
	MeasurementModelContext context;
	MeasurementModelConfig config =
		MeasurementModelTest_CreateThreeShuntConfig();

	MeasurementModel_Reset(&context);
	config.current_sense.channels[1].acquisition_index =
		config.current_sense.channels[0].acquisition_index;
	TEST_CHECK(!MeasurementModel_Configure(&context, &config));
	config = MeasurementModelTest_CreateTwoShuntConfig();
	config.current_sense.channels[1].role =
		MEASUREMENT_CURRENT_CHANNEL_ROLE_PHASE_A;
	TEST_CHECK(!MeasurementModel_Configure(&context, &config));
	config = MeasurementModelTest_CreateOneShuntConfig();
	config.current_sense.channels[0].role =
		MEASUREMENT_CURRENT_CHANNEL_ROLE_PHASE_A;
	TEST_CHECK(!MeasurementModel_Configure(&context, &config));
	return 0;
}

static int MeasurementModelTest_InvalidFramesFailClosed(void)
{
	MeasurementModelContext context;
	MeasurementModelConfig config =
		MeasurementModelTest_CreateThreeShuntConfig();
	MeasurementModelAcquisitionInput input;
	MeasurementModelOutput output;

	MeasurementModel_Reset(&context);
	TEST_CHECK(MeasurementModel_Configure(&context, &config));
	MeasurementModelTest_SetDirectInput(&input, 50U, 2U,
		PHASE_CURRENT_VALID_ALL);
	TEST_CHECK(MeasurementModel_UpdateAcquisition(&context, &input, &output) ==
		PHASE_CURRENT_STATUS_INVALID_SAMPLE_COUNT);
	TEST_CHECK((output.faults & MEASUREMENT_FAULT_CURRENT_ACQUISITION) != 0U);
	TEST_CHECK(output.phase_a_current_a == 0.0f);
	/* Bad sequence 50 was consumed and cannot be replayed after correction. */
	input.current.sample_count = 3U;
	TEST_CHECK(MeasurementModel_UpdateAcquisition(&context, &input, &output) ==
		PHASE_CURRENT_STATUS_DUPLICATE_SEQUENCE);
	TEST_CHECK(output.phase_current_quality == PHASE_CURRENT_QUALITY_STALE);

	input.current.sequence = 49U;
	TEST_CHECK(MeasurementModel_UpdateAcquisition(&context, &input, &output) ==
		PHASE_CURRENT_STATUS_OUT_OF_ORDER_SEQUENCE);
	input.current.sequence = 51U;
	input.current.valid_phase_mask =
		PHASE_CURRENT_VALID_A | PHASE_CURRENT_VALID_B;
	TEST_CHECK(MeasurementModel_UpdateAcquisition(&context, &input, &output) ==
		PHASE_CURRENT_STATUS_INVALID_PHASE_MASK);
	input.current.sequence = 52U;
	input.current.valid_phase_mask = PHASE_CURRENT_VALID_ALL;
	input.current.saturated_observation_mask = 1U;
	TEST_CHECK(MeasurementModel_UpdateAcquisition(&context, &input, &output) ==
		PHASE_CURRENT_STATUS_SATURATED);
	TEST_CHECK(output.phase_current_quality == PHASE_CURRENT_QUALITY_SATURATED);

	/* A valid frame after invalid payloads remains usable and ordered. */
	input.current.sequence = 53U;
	input.current.saturated_observation_mask = 0U;
	input.current.raw_observation_adc[0] = 300U;
	input.current.raw_observation_adc[1] = 400U;
	input.current.raw_observation_adc[2] = 50U;
	TEST_CHECK(MeasurementModel_UpdateAcquisition(&context, &input, &output) ==
		PHASE_CURRENT_STATUS_OK);
	TEST_CHECK(MeasurementModelTest_Near(output.phase_a_current_a, 3.0f));
	return 0;
}

static int MeasurementModelTest_OneShuntWindowFailures(void)
{
	MeasurementModelContext context;
	MeasurementModelConfig config =
		MeasurementModelTest_CreateOneShuntConfig();
	MeasurementModelAcquisitionInput input;
	MeasurementModelOutput output;

	MeasurementModel_Reset(&context);
	TEST_CHECK(MeasurementModel_Configure(&context, &config));
	memset(&input, 0, sizeof(input));
	input.current.sequence = 1U;
	input.current.sample_count = 2U;
	input.current.windows[0] =
		PHASE_CURRENT_DC_LINK_WINDOW_FIRST_ACTIVE_VECTOR;
	input.current.windows[1] =
		PHASE_CURRENT_DC_LINK_WINDOW_SECOND_ACTIVE_VECTOR;
	input.current.raw_observation_adc[0] = 2348U;
	input.current.raw_observation_adc[1] = 2248U;
	TEST_CHECK(MeasurementModel_UpdateAcquisition(&context, &input, &output) ==
		PHASE_CURRENT_STATUS_INVALID_SECTOR);

	input.current.sequence = 2U;
	input.current.sector = 1U;
	input.current.windows[1] =
		PHASE_CURRENT_DC_LINK_WINDOW_FIRST_ACTIVE_VECTOR;
	TEST_CHECK(MeasurementModel_UpdateAcquisition(&context, &input, &output) ==
		PHASE_CURRENT_STATUS_INVALID_WINDOW);
	return 0;
}

static int MeasurementModelTest_OffsetAndSequenceWrap(void)
{
	MeasurementModelContext context;
	MeasurementModelConfig config =
		MeasurementModelTest_CreateThreeShuntConfig();
	MeasurementModelAcquisitionInput input;
	MeasurementModelOutput output;

	config.current_sense.channels[0].offset_adc = 4096U;
	MeasurementModel_Reset(&context);
	TEST_CHECK(MeasurementModel_Configure(&context, &config));
	MeasurementModelTest_SetDirectInput(&input, UINT32_MAX, 3U,
		PHASE_CURRENT_VALID_ALL);
	TEST_CHECK(MeasurementModel_UpdateAcquisition(&context, &input, &output) ==
		PHASE_CURRENT_STATUS_OK);
	TEST_CHECK((output.faults & MEASUREMENT_FAULT_CURRENT_OFFSET) != 0U);
	TEST_CHECK(output.phase_a_current_a == 0.0f);

	config = MeasurementModelTest_CreateThreeShuntConfig();
	TEST_CHECK(MeasurementModel_Configure(&context, &config));
	input.current.sequence = UINT32_MAX;
	input.current.raw_observation_adc[0] = 300U;
	input.current.raw_observation_adc[1] = 400U;
	input.current.raw_observation_adc[2] = 50U;
	TEST_CHECK(MeasurementModel_UpdateAcquisition(&context, &input, &output) ==
		PHASE_CURRENT_STATUS_OK);
	input.current.sequence = 0U;
	TEST_CHECK(MeasurementModel_UpdateAcquisition(&context, &input, &output) ==
		PHASE_CURRENT_STATUS_OK);
	TEST_CHECK(output.current_sequence == 0U);
	return 0;
}

int MeasurementModel_RunHostTests(void)
{
	int result;

	result = MeasurementModelTest_LegacyCompatibility();
	if (result != 0)
		return result;
	result = MeasurementModelTest_ThreeShuntAcquisitionMapping();
	if (result != 0)
		return result;
	result = MeasurementModelTest_TwoShuntKclReconstruction();
	if (result != 0)
		return result;
	result = MeasurementModelTest_OneShuntAllSectors();
	if (result != 0)
		return result;
	result = MeasurementModelTest_ProtectionConfirmation();
	if (result != 0)
		return result;
	result = MeasurementModelTest_InvalidConfig();
	if (result != 0)
		return result;
	result = MeasurementModelTest_InvalidFramesFailClosed();
	if (result != 0)
		return result;
	result = MeasurementModelTest_OneShuntWindowFailures();
	if (result != 0)
		return result;
	return MeasurementModelTest_OffsetAndSequenceWrap();
}
