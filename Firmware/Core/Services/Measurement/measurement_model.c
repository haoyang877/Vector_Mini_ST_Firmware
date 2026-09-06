#include "measurement_model.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static float Measurement_Abs(float value)
{
	return value >= 0.0f ? value : -value;
}

static bool Measurement_CommonConfigIsValid(
	const MeasurementModelConfig *config)
{
	return config != NULL &&
		isfinite(config->bus_voltage_v_per_count) &&
		config->bus_voltage_v_per_count > 0.0f &&
		isfinite(config->bus_voltage_filter_alpha) &&
		config->bus_voltage_filter_alpha > 0.0f &&
		config->bus_voltage_filter_alpha <= 1.0f &&
		isfinite(config->overcurrent_trip_a) &&
		config->overcurrent_trip_a > 0.0f &&
		isfinite(config->overvoltage_trip_v) &&
		isfinite(config->undervoltage_trip_v) &&
		config->overvoltage_trip_v > config->undervoltage_trip_v &&
		config->overcurrent_confirm_cycles > 0U &&
		config->voltage_confirm_cycles > 0U;
}

static bool Measurement_LegacyCurrentConfigIsValid(
	const MeasurementModelConfig *config)
{
	uint8_t channel;

	for (channel = 0U; channel < MEASUREMENT_MODEL_PHASE_COUNT; channel++)
	{
		uint8_t other;

		if (config->phase_channel_index[channel] >=
				MEASUREMENT_MODEL_PHASE_COUNT ||
			config->minimum_valid_offset_adc[channel] >
				config->maximum_valid_offset_adc[channel] ||
			!isfinite(config->current_a_per_count[channel]) ||
			config->current_a_per_count[channel] == 0.0f)
		{
			return false;
		}
		for (other = (uint8_t)(channel + 1U);
			other < MEASUREMENT_MODEL_PHASE_COUNT; other++)
		{
			if (config->phase_channel_index[channel] ==
				config->phase_channel_index[other])
			{
				return false;
			}
		}
	}
	return true;
}

static uint8_t Measurement_RequiredPhysicalChannelCount(
	PhaseCurrentTopology topology)
{
	switch (topology)
	{
		case PHASE_CURRENT_TOPOLOGY_INLINE_3_SHUNT:
		case PHASE_CURRENT_TOPOLOGY_LOW_SIDE_3_SHUNT:
			return 3U;
		case PHASE_CURRENT_TOPOLOGY_LOW_SIDE_2_SHUNT:
			return 2U;
		case PHASE_CURRENT_TOPOLOGY_DC_LINK_1_SHUNT:
			return 1U;
		default:
			return 0U;
	}
}

static bool Measurement_ChannelRoleToPhase(
	MeasurementCurrentChannelRole role, uint8_t *phase)
{
	if (phase == NULL)
		return false;
	switch (role)
	{
		case MEASUREMENT_CURRENT_CHANNEL_ROLE_PHASE_A:
			*phase = PHASE_CURRENT_PHASE_A;
			return true;
		case MEASUREMENT_CURRENT_CHANNEL_ROLE_PHASE_B:
			*phase = PHASE_CURRENT_PHASE_B;
			return true;
		case MEASUREMENT_CURRENT_CHANNEL_ROLE_PHASE_C:
			*phase = PHASE_CURRENT_PHASE_C;
			return true;
		default:
			return false;
	}
}

static bool Measurement_AcquisitionCurrentConfigIsValid(
	const MeasurementCurrentSenseConfig *config)
{
	uint8_t required_channel_count;
	uint8_t role_mask = PHASE_CURRENT_VALID_NONE;
	uint8_t channel;

	if (config == NULL)
		return false;
	required_channel_count = Measurement_RequiredPhysicalChannelCount(
		config->topology);
	if (required_channel_count == 0U ||
		config->physical_channel_count != required_channel_count)
	{
		return false;
	}

	for (channel = 0U; channel < required_channel_count; channel++)
	{
		const MeasurementCurrentChannelConfig *channel_config =
			&config->channels[channel];
		uint8_t other;
		uint8_t phase;

		if (channel_config->acquisition_index >=
				MEASUREMENT_MODEL_MAX_CURRENT_OBSERVATIONS ||
			channel_config->minimum_valid_offset_adc >
				channel_config->maximum_valid_offset_adc ||
			!isfinite(channel_config->current_a_per_count) ||
			channel_config->current_a_per_count == 0.0f)
		{
			return false;
		}

		if (config->topology == PHASE_CURRENT_TOPOLOGY_DC_LINK_1_SHUNT)
		{
			if (channel_config->role !=
				MEASUREMENT_CURRENT_CHANNEL_ROLE_DC_LINK)
			{
				return false;
			}
		}
		else
		{
			uint8_t phase_mask;

			if (!Measurement_ChannelRoleToPhase(channel_config->role, &phase))
				return false;
			phase_mask = (uint8_t)(1U << phase);
			if ((role_mask & phase_mask) != 0U)
				return false;
			role_mask = (uint8_t)(role_mask | phase_mask);
		}

		for (other = (uint8_t)(channel + 1U);
			other < required_channel_count; other++)
		{
			if (channel_config->acquisition_index ==
				config->channels[other].acquisition_index)
			{
				return false;
			}
		}
	}

	if ((config->topology == PHASE_CURRENT_TOPOLOGY_INLINE_3_SHUNT ||
		 config->topology == PHASE_CURRENT_TOPOLOGY_LOW_SIDE_3_SHUNT) &&
		role_mask != PHASE_CURRENT_VALID_ALL)
	{
		return false;
	}
	return true;
}

static bool Measurement_ConfigIsValid(const MeasurementModelConfig *config)
{
	if (!Measurement_CommonConfigIsValid(config))
		return false;
	if (config->current_sense.topology == PHASE_CURRENT_TOPOLOGY_INVALID)
		return Measurement_LegacyCurrentConfigIsValid(config);
	return Measurement_AcquisitionCurrentConfigIsValid(
		&config->current_sense);
}

static bool Measurement_LegacyOffsetsAreValid(
	const MeasurementModelConfig *config)
{
	return config->phase_a_offset_adc >=
			config->minimum_valid_offset_adc[0] &&
		config->phase_a_offset_adc <= config->maximum_valid_offset_adc[0] &&
		config->phase_b_offset_adc >=
			config->minimum_valid_offset_adc[1] &&
		config->phase_b_offset_adc <= config->maximum_valid_offset_adc[1] &&
		config->phase_c_offset_adc >=
			config->minimum_valid_offset_adc[2] &&
		config->phase_c_offset_adc <= config->maximum_valid_offset_adc[2];
}

static bool Measurement_AcquisitionOffsetsAreValid(
	const MeasurementCurrentSenseConfig *config)
{
	uint8_t channel;

	for (channel = 0U; channel < config->physical_channel_count; channel++)
	{
		const MeasurementCurrentChannelConfig *channel_config =
			&config->channels[channel];

		if (channel_config->offset_adc <
				channel_config->minimum_valid_offset_adc ||
			channel_config->offset_adc >
				channel_config->maximum_valid_offset_adc)
		{
			return false;
		}
	}
	return true;
}

static void Measurement_BeginCycle(MeasurementModelContext *context)
{
	context->output.faults = MEASUREMENT_FAULT_NONE;
	context->output.phase_current_status = PHASE_CURRENT_STATUS_OK;
}

static void Measurement_ProcessBusVoltage(MeasurementModelContext *context,
	uint32_t bus_voltage_adc, bool protection_is_active)
{
	const MeasurementModelConfig *config = &context->config;

	context->output.bus_voltage_v = bus_voltage_adc *
		config->bus_voltage_v_per_count;
	context->output.bus_voltage_filtered_v +=
		config->bus_voltage_filter_alpha *
		(context->output.bus_voltage_v -
		 context->output.bus_voltage_filtered_v);

	if (!protection_is_active)
		return;

	if (context->output.bus_voltage_filtered_v > config->overvoltage_trip_v)
	{
		if (context->overvoltage_count < config->voltage_confirm_cycles)
			context->overvoltage_count++;
		if (context->overvoltage_count >= config->voltage_confirm_cycles)
		{
			context->output.faults = (MeasurementFaultFlags)
				(context->output.faults | MEASUREMENT_FAULT_OVER_VOLTAGE);
		}
	}
	else
	{
		context->overvoltage_count = 0U;
	}

	if (context->output.bus_voltage_filtered_v < config->undervoltage_trip_v)
	{
		if (++context->undervoltage_count >= config->voltage_confirm_cycles)
		{
			context->output.faults = (MeasurementFaultFlags)
				(context->output.faults | MEASUREMENT_FAULT_UNDER_VOLTAGE);
			context->undervoltage_count = 0U;
		}
	}
	else
	{
		context->undervoltage_count = 0U;
	}
}

static void Measurement_ProcessOvercurrent(MeasurementModelContext *context)
{
	const MeasurementModelConfig *config = &context->config;
	bool has_overcurrent =
		Measurement_Abs(context->output.phase_a_current_a) >
			config->overcurrent_trip_a ||
		Measurement_Abs(context->output.phase_b_current_a) >
			config->overcurrent_trip_a ||
		Measurement_Abs(context->output.phase_c_current_a) >
			config->overcurrent_trip_a;

	if (has_overcurrent)
	{
		if (context->overcurrent_count < config->overcurrent_confirm_cycles)
			context->overcurrent_count++;
		if (context->overcurrent_count >= config->overcurrent_confirm_cycles)
		{
			context->output.faults = (MeasurementFaultFlags)
				(context->output.faults | MEASUREMENT_FAULT_OVER_CURRENT);
		}
	}
	else
	{
		context->overcurrent_count = 0U;
	}
}

static void Measurement_ClearPhaseCurrents(MeasurementModelContext *context)
{
	context->output.phase_a_current_a = 0.0f;
	context->output.phase_b_current_a = 0.0f;
	context->output.phase_c_current_a = 0.0f;
}

static PhaseCurrentStatus Measurement_CheckAcquisitionSequence(
	MeasurementModelContext *context, uint32_t sequence)
{
	uint32_t delta;

	if (!context->has_last_acquisition_sequence)
	{
		context->last_acquisition_sequence = sequence;
		context->has_last_acquisition_sequence = true;
		return PHASE_CURRENT_STATUS_OK;
	}

	delta = sequence - context->last_acquisition_sequence;
	if (delta == 0U)
		return PHASE_CURRENT_STATUS_DUPLICATE_SEQUENCE;
	if (delta >= UINT32_C(0x80000000))
		return PHASE_CURRENT_STATUS_OUT_OF_ORDER_SEQUENCE;

	/* Consume every newer sequence before inspecting its payload. */
	context->last_acquisition_sequence = sequence;
	return PHASE_CURRENT_STATUS_OK;
}

static PhaseCurrentStatus Measurement_FailAcquisition(
	MeasurementModelContext *context, PhaseCurrentStatus status,
	uint32_t sequence, MeasurementModelOutput *output)
{
	Measurement_ClearPhaseCurrents(context);
	context->output.faults = (MeasurementFaultFlags)
		(context->output.faults | MEASUREMENT_FAULT_CURRENT_ACQUISITION);
	context->output.phase_current_quality =
		(status == PHASE_CURRENT_STATUS_DUPLICATE_SEQUENCE ||
		 status == PHASE_CURRENT_STATUS_OUT_OF_ORDER_SEQUENCE) ?
		PHASE_CURRENT_QUALITY_STALE :
		(status == PHASE_CURRENT_STATUS_SATURATED ?
		 PHASE_CURRENT_QUALITY_SATURATED : PHASE_CURRENT_QUALITY_INVALID);
	context->output.phase_current_status = status;
	context->output.current_sequence = sequence;
	*output = context->output;
	return status;
}

static uint8_t Measurement_ConfiguredDirectPhaseMask(
	const MeasurementCurrentSenseConfig *config)
{
	uint8_t phase_mask = PHASE_CURRENT_VALID_NONE;
	uint8_t channel;

	for (channel = 0U; channel < config->physical_channel_count; channel++)
	{
		uint8_t phase;

		if (Measurement_ChannelRoleToPhase(config->channels[channel].role,
			&phase))
		{
			phase_mask = (uint8_t)(phase_mask | (uint8_t)(1U << phase));
		}
	}
	return phase_mask;
}

static uint8_t Measurement_ConfiguredAcquisitionMask(
	const MeasurementCurrentSenseConfig *config)
{
	uint8_t acquisition_mask = 0U;
	uint8_t channel;

	for (channel = 0U; channel < config->physical_channel_count; channel++)
	{
		acquisition_mask = (uint8_t)(acquisition_mask |
			(uint8_t)(1U << config->channels[channel].acquisition_index));
	}
	return acquisition_mask;
}

static PhaseCurrentStatus Measurement_BuildPhaseAcquisition(
	const MeasurementModelContext *context,
	const MeasurementCurrentAcquisitionInput *input,
	PhaseCurrentAcquisition *acquisition)
{
	const MeasurementCurrentSenseConfig *config =
		&context->config.current_sense;
	uint8_t allowed_observation_mask;
	uint8_t expected_sample_count;
	uint8_t channel;

	memset(acquisition, 0, sizeof(*acquisition));
	acquisition->sequence = input->sequence;
	expected_sample_count =
		(config->topology == PHASE_CURRENT_TOPOLOGY_DC_LINK_1_SHUNT) ?
		PHASE_CURRENT_DC_LINK_WINDOW_COUNT : config->physical_channel_count;
	if (input->sample_count != expected_sample_count ||
		input->sample_count == 0U ||
		input->sample_count > MEASUREMENT_MODEL_MAX_CURRENT_OBSERVATIONS)
	{
		return PHASE_CURRENT_STATUS_INVALID_SAMPLE_COUNT;
	}
	allowed_observation_mask =
		(config->topology == PHASE_CURRENT_TOPOLOGY_DC_LINK_1_SHUNT) ?
		(uint8_t)((1U << PHASE_CURRENT_DC_LINK_WINDOW_COUNT) - 1U) :
		Measurement_ConfiguredAcquisitionMask(config);
	if ((input->saturated_observation_mask &
			(uint8_t)~allowed_observation_mask) != 0U)
	{
		return PHASE_CURRENT_STATUS_INVALID_SAMPLE_COUNT;
	}

	if (config->topology == PHASE_CURRENT_TOPOLOGY_DC_LINK_1_SHUNT)
	{
		const MeasurementCurrentChannelConfig *channel_config =
			&config->channels[0];
		uint8_t window;

		if (input->valid_phase_mask != PHASE_CURRENT_VALID_NONE)
			return PHASE_CURRENT_STATUS_INVALID_PHASE_MASK;
		acquisition->data.dc_link.sector = input->sector;
		for (window = 0U; window < PHASE_CURRENT_DC_LINK_WINDOW_COUNT;
			window++)
		{
			float current_a =
				((float)input->raw_observation_adc[window] -
				 (float)channel_config->offset_adc) *
				channel_config->current_a_per_count;

			if (!isfinite(current_a))
				return PHASE_CURRENT_STATUS_NONFINITE_SAMPLE;
			acquisition->data.dc_link.windows[window].current_a = current_a;
			acquisition->data.dc_link.windows[window].id =
				input->windows[window];
			acquisition->data.dc_link.windows[window].valid = true;
			acquisition->data.dc_link.windows[window].saturated =
				(input->saturated_observation_mask & (uint8_t)(1U << window)) !=
				0U;
		}
		return PHASE_CURRENT_STATUS_OK;
	}

	if ((input->valid_phase_mask & (uint8_t)~PHASE_CURRENT_VALID_ALL) != 0U ||
		input->valid_phase_mask != Measurement_ConfiguredDirectPhaseMask(config))
	{
		return PHASE_CURRENT_STATUS_INVALID_PHASE_MASK;
	}
	acquisition->data.direct.valid_phase_mask = input->valid_phase_mask;
	for (channel = 0U; channel < config->physical_channel_count; channel++)
	{
		const MeasurementCurrentChannelConfig *channel_config =
			&config->channels[channel];
		uint8_t acquisition_index = channel_config->acquisition_index;
		uint8_t phase;
		float current_a;

		if (!Measurement_ChannelRoleToPhase(channel_config->role, &phase))
			return PHASE_CURRENT_STATUS_INVALID_PHASE_MASK;
		current_a =
			((float)input->raw_observation_adc[acquisition_index] -
			 (float)channel_config->offset_adc) *
			channel_config->current_a_per_count;
		if (!isfinite(current_a))
			return PHASE_CURRENT_STATUS_NONFINITE_SAMPLE;
		acquisition->data.direct.phase_current_a[phase] = current_a;
		if ((input->saturated_observation_mask &
			(uint8_t)(1U << acquisition_index)) != 0U)
		{
			acquisition->data.direct.saturated_phase_mask = (uint8_t)
				(acquisition->data.direct.saturated_phase_mask |
				 (uint8_t)(1U << phase));
		}
	}
	return PHASE_CURRENT_STATUS_OK;
}

void MeasurementModel_Reset(MeasurementModelContext *context)
{
	if (context != NULL)
		memset(context, 0, sizeof(*context));
}

bool MeasurementModel_Configure(MeasurementModelContext *context,
	const MeasurementModelConfig *config)
{
	PhaseCurrentStrategy strategy;
	PhaseCurrentStrategyConfig strategy_config;

	if (context == NULL || !Measurement_ConfigIsValid(config))
		return false;

	if (config->current_sense.topology != PHASE_CURRENT_TOPOLOGY_INVALID)
	{
		PhaseCurrentStrategy_Reset(&strategy);
		if (PhaseCurrentStrategy_MakeDefaultConfig(
				config->current_sense.topology, &strategy_config) !=
				PHASE_CURRENT_STATUS_OK ||
			PhaseCurrentStrategy_Configure(&strategy, &strategy_config) !=
				PHASE_CURRENT_STATUS_OK)
		{
			return false;
		}
		context->phase_current_strategy = strategy;
		context->uses_acquisition_pipeline = true;
	}
	else
	{
		PhaseCurrentStrategy_Reset(&context->phase_current_strategy);
		context->uses_acquisition_pipeline = false;
	}
	context->config = *config;
	context->has_last_acquisition_sequence = false;
	context->last_acquisition_sequence = 0U;
	context->is_configured = true;
	return true;
}

bool MeasurementModel_Update(MeasurementModelContext *context,
	const MeasurementModelInput *input, MeasurementModelOutput *output)
{
	const MeasurementModelConfig *config;

	if (context == NULL || input == NULL || output == NULL ||
		!context->is_configured || context->uses_acquisition_pipeline)
	{
		return false;
	}
	config = &context->config;

	Measurement_BeginCycle(context);
	Measurement_ProcessBusVoltage(context, input->bus_voltage_adc,
		input->protection_is_active);
	context->output.current_sequence++;
	context->output.phase_current_status = PHASE_CURRENT_STATUS_OK;

	if (!Measurement_LegacyOffsetsAreValid(config))
	{
		Measurement_ClearPhaseCurrents(context);
		context->output.phase_current_quality = PHASE_CURRENT_QUALITY_INVALID;
		context->output.faults = (MeasurementFaultFlags)
			(context->output.faults | MEASUREMENT_FAULT_CURRENT_OFFSET);
	}
	else
	{
		context->output.phase_a_current_a =
			((float)input->phase_a_adc - config->phase_a_offset_adc) *
			config->current_a_per_count[0];
		context->output.phase_b_current_a =
			((float)input->phase_b_adc - config->phase_b_offset_adc) *
			config->current_a_per_count[1];
		context->output.phase_c_current_a =
			((float)input->phase_c_adc - config->phase_c_offset_adc) *
			config->current_a_per_count[2];
		context->output.phase_current_quality = PHASE_CURRENT_QUALITY_DIRECT;
	}

	Measurement_ProcessOvercurrent(context);
	*output = context->output;
	return true;
}

PhaseCurrentStatus MeasurementModel_UpdateAcquisition(
	MeasurementModelContext *context,
	const MeasurementModelAcquisitionInput *input,
	MeasurementModelOutput *output)
{
	PhaseCurrentAcquisition acquisition;
	PhaseCurrentSample sample;
	PhaseCurrentStatus status;

	if (context == NULL || input == NULL || output == NULL)
		return PHASE_CURRENT_STATUS_NULL_ARGUMENT;
	if (!context->is_configured || !context->uses_acquisition_pipeline)
		return PHASE_CURRENT_STATUS_NOT_CONFIGURED;

	Measurement_BeginCycle(context);
	status = Measurement_CheckAcquisitionSequence(context,
		input->current.sequence);
	if (status != PHASE_CURRENT_STATUS_OK)
	{
		return Measurement_FailAcquisition(context, status,
			input->current.sequence, output);
	}

	status = Measurement_BuildPhaseAcquisition(context, &input->current,
		&acquisition);
	if (status != PHASE_CURRENT_STATUS_OK)
	{
		return Measurement_FailAcquisition(context, status,
			input->current.sequence, output);
	}

	Measurement_ProcessBusVoltage(context, input->bus_voltage_adc,
		input->protection_is_active);
	if (!Measurement_AcquisitionOffsetsAreValid(
			&context->config.current_sense))
	{
		Measurement_ClearPhaseCurrents(context);
		context->output.phase_current_quality = PHASE_CURRENT_QUALITY_INVALID;
		context->output.phase_current_status = PHASE_CURRENT_STATUS_OK;
		context->output.current_sequence = input->current.sequence;
		context->output.faults = (MeasurementFaultFlags)
			(context->output.faults | MEASUREMENT_FAULT_CURRENT_OFFSET);
		Measurement_ProcessOvercurrent(context);
		*output = context->output;
		return PHASE_CURRENT_STATUS_OK;
	}

	status = PhaseCurrentStrategy_Process(&context->phase_current_strategy,
		&acquisition, &sample);
	if (status != PHASE_CURRENT_STATUS_OK)
	{
		return Measurement_FailAcquisition(context, status,
			input->current.sequence, output);
	}

	context->output.phase_a_current_a = sample.ia_a;
	context->output.phase_b_current_a = sample.ib_a;
	context->output.phase_c_current_a = sample.ic_a;
	context->output.phase_current_quality = sample.quality;
	context->output.phase_current_status = PHASE_CURRENT_STATUS_OK;
	context->output.current_sequence = sample.sequence;
	Measurement_ProcessOvercurrent(context);
	*output = context->output;
	return PHASE_CURRENT_STATUS_OK;
}
