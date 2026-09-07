#include "Core/Application/MotorControl/measurement_runtime.h"

#include <string.h>

#include "motor_state_runtime.h"

#if defined(__CC_ARM)
#pragma O3
#pragma Otime
#endif

static uint16_t MeasurementRuntime_ReadOffsetSlot(
	const MotorControlContext *motor, MeasurementCurrentChannelRole role)
{
	switch (role)
	{
		case MEASUREMENT_CURRENT_CHANNEL_ROLE_PHASE_B:
			return motor->configuration.phase_b_current_offset_adc;
		case MEASUREMENT_CURRENT_CHANNEL_ROLE_PHASE_C:
			return motor->configuration.phase_c_current_offset_adc;
		case MEASUREMENT_CURRENT_CHANNEL_ROLE_PHASE_A:
		case MEASUREMENT_CURRENT_CHANNEL_ROLE_DC_LINK:
		default:
			return motor->configuration.phase_a_current_offset_adc;
	}
}

static bool MeasurementRuntime_ApplyStoredOffsets(
	MeasurementModelConfig *config, const MotorControlContext *motor)
{
	uint8_t channel;

	if (config->current_sense.topology == PHASE_CURRENT_TOPOLOGY_INVALID)
	{
		config->phase_a_offset_adc =
			(int16_t)motor->configuration.phase_a_current_offset_adc;
		config->phase_b_offset_adc =
			(int16_t)motor->configuration.phase_b_current_offset_adc;
		config->phase_c_offset_adc =
			(int16_t)motor->configuration.phase_c_current_offset_adc;
		return true;
	}
	if (config->current_sense.physical_channel_count == 0U ||
		config->current_sense.physical_channel_count >
			MEASUREMENT_MODEL_MAX_CURRENT_OBSERVATIONS)
	{
		return false;
	}
	for (channel = 0U;
		channel < config->current_sense.physical_channel_count; channel++)
	{
		MeasurementCurrentChannelRole role =
			config->current_sense.channels[channel].role;

		if (role < MEASUREMENT_CURRENT_CHANNEL_ROLE_PHASE_A ||
			role > MEASUREMENT_CURRENT_CHANNEL_ROLE_DC_LINK)
		{
			return false;
		}
		config->current_sense.channels[channel].offset_adc =
			MeasurementRuntime_ReadOffsetSlot(motor, role);
	}
	return true;
}

static PhaseCurrentDcLinkWindowId MeasurementRuntime_MapWindow(
	BspCurrentSamplingWindow window)
{
	switch (window)
	{
		case BSP_CURRENT_SAMPLING_WINDOW_ACTIVE_VECTOR_FIRST:
			return PHASE_CURRENT_DC_LINK_WINDOW_FIRST_ACTIVE_VECTOR;
		case BSP_CURRENT_SAMPLING_WINDOW_ACTIVE_VECTOR_SECOND:
			return PHASE_CURRENT_DC_LINK_WINDOW_SECOND_ACTIVE_VECTOR;
		default:
			return PHASE_CURRENT_DC_LINK_WINDOW_INVALID;
	}
}

static MotorFaultCode MeasurementRuntime_MapAcquisitionFault(
	PhaseCurrentStatus status)
{
	switch (status)
	{
		case PHASE_CURRENT_STATUS_SATURATED:
			return MOTOR_FAULT_OVER_CURRENT;
		case PHASE_CURRENT_STATUS_NULL_ARGUMENT:
		case PHASE_CURRENT_STATUS_NOT_CONFIGURED:
		case PHASE_CURRENT_STATUS_UNSUPPORTED_TOPOLOGY:
			return MOTOR_FAULT_INVALID_PARAMETER;
		default:
			return MOTOR_FAULT_POWER_STAGE;
	}
}

bool Measurement_Capture(CurrentControlContext *CurrentControl)
{
	if (CurrentControl == 0 || CurrentControl->motor_drive == 0)
		return false;
	return MotorDriveService_ReadSample(CurrentControl->motor_drive,
		&CurrentControl->measurement_acquisition);
}

bool Measurement_Configure(MeasurementModelContext *context,
	const MotorControlContext *motor,
	const MeasurementModelConfig *design_config)
{
	MeasurementModelConfig config;

	if (context == 0 || motor == 0 || design_config == 0)
		return false;
	config = *design_config;
	if (!MeasurementRuntime_ApplyStoredOffsets(&config, motor))
		return false;
	return MeasurementModel_Configure(context, &config);
}

bool Measurement_UpdateCurrentOffsets(MeasurementModelContext *context,
	const MotorControlContext *motor)
{
	if (context == 0 || motor == 0 || !context->is_configured)
		return false;
	return Measurement_Configure(context, motor, &context->config);
}

bool Measurement_Process(MeasurementModelContext *context,
	CurrentControlContext *CurrentControl, bool protection_is_active,
	MotorStateContext *motor_state)
{
	MeasurementModelAcquisitionInput input;
	MeasurementModelOutput output;
	const MotorDriveServiceAcquisition *source;
	PhaseCurrentStatus status;
	uint8_t observation;

	if (context == 0 || CurrentControl == 0 || motor_state == 0 ||
		!context->is_configured)
	{
		return false;
	}
	source = &CurrentControl->measurement_acquisition;
	memset(&input, 0, sizeof(input));
	for (observation = 0U;
		observation < MEASUREMENT_MODEL_MAX_CURRENT_OBSERVATIONS; observation++)
	{
		input.current.raw_observation_adc[observation] =
			source->sample.current_raw[observation];
	}
	input.current.sequence = source->sample.sequence;
	input.current.sample_count = source->sample.current_sample_count;
	input.current.valid_phase_mask = source->sample.valid_phase_currents;
	input.current.sector = source->sampling.modulation_sector;
	for (observation = 0U;
		observation < source->sampling.sampling_point_count &&
		observation < MEASUREMENT_MODEL_MAX_CURRENT_OBSERVATIONS; observation++)
	{
		input.current.windows[observation] = MeasurementRuntime_MapWindow(
			source->sampling.points[observation].window);
	}
	input.bus_voltage_adc = source->sample.bus_voltage_raw;
	input.protection_is_active = protection_is_active;
	output = context->output;
	status = MeasurementModel_UpdateAcquisition(context, &input, &output);
	CurrentControl->phase_current_status = status;
	CurrentControl->phase_a_current_a = output.phase_a_current_a;
	CurrentControl->phase_b_current_a = output.phase_b_current_a;
	CurrentControl->phase_c_current_a = output.phase_c_current_a;
	CurrentControl->bus_voltage_v = output.bus_voltage_v;
	CurrentControl->filtered_bus_voltage_v = output.bus_voltage_filtered_v;
	if (status != PHASE_CURRENT_STATUS_OK)
	{
		MotorState_RaiseFault(motor_state,
			MeasurementRuntime_MapAcquisitionFault(status));
		return false;
	}

	if ((output.faults & MEASUREMENT_FAULT_OVER_VOLTAGE) != 0U)
		MotorState_RaiseFault(motor_state, MOTOR_FAULT_OVER_VOLTAGE);
	if ((output.faults & MEASUREMENT_FAULT_UNDER_VOLTAGE) != 0U)
		MotorState_RaiseFault(motor_state, MOTOR_FAULT_UNDER_VOLTAGE);
	if ((output.faults & MEASUREMENT_FAULT_CURRENT_OFFSET) != 0U)
		MotorState_RaiseFault(motor_state, MOTOR_FAULT_CURRENT_OFFSET);
	if ((output.faults & MEASUREMENT_FAULT_OVER_CURRENT) != 0U)
		MotorState_RaiseFault(motor_state, MOTOR_FAULT_OVER_CURRENT);
	return true;
}
