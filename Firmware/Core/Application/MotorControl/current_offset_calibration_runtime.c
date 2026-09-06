#include "Core/Application/MotorControl/current_offset_calibration_runtime.h"

#include <limits.h>
#include <string.h>

#include "motor_state_runtime.h"

static bool CurrentOffsetCalibration_MapRoleToFlashSlot(
	MeasurementCurrentChannelRole role, uint8_t *flash_slot)
{
	if (flash_slot == 0)
		return false;
	switch (role)
	{
		case MEASUREMENT_CURRENT_CHANNEL_ROLE_PHASE_A:
		case MEASUREMENT_CURRENT_CHANNEL_ROLE_DC_LINK:
			*flash_slot = 0U;
			return true;
		case MEASUREMENT_CURRENT_CHANNEL_ROLE_PHASE_B:
			*flash_slot = 1U;
			return true;
		case MEASUREMENT_CURRENT_CHANNEL_ROLE_PHASE_C:
			*flash_slot = 2U;
			return true;
		default:
			return false;
	}
}

static uint8_t CurrentOffsetCalibration_ConfiguredPhaseMask(
	const MeasurementCurrentSenseConfig *current_sense)
{
	uint8_t phase_mask = PHASE_CURRENT_VALID_NONE;
	uint8_t channel;

	for (channel = 0U; channel < current_sense->physical_channel_count;
		channel++)
	{
		switch (current_sense->channels[channel].role)
		{
			case MEASUREMENT_CURRENT_CHANNEL_ROLE_PHASE_A:
				phase_mask |= PHASE_CURRENT_VALID_A;
				break;
			case MEASUREMENT_CURRENT_CHANNEL_ROLE_PHASE_B:
				phase_mask |= PHASE_CURRENT_VALID_B;
				break;
			case MEASUREMENT_CURRENT_CHANNEL_ROLE_PHASE_C:
				phase_mask |= PHASE_CURRENT_VALID_C;
				break;
			default:
				return PHASE_CURRENT_VALID_NONE;
		}
	}
	return phase_mask;
}

static bool CurrentOffsetCalibration_AddObservation(
	CurrentOffsetCalibrationContext *context, uint8_t flash_slot,
	uint32_t observation)
{
	if (flash_slot >= MEASUREMENT_MODEL_PHASE_COUNT ||
		context->offset_sum[flash_slot] > UINT32_MAX - observation ||
		context->observation_count[flash_slot] == UINT32_MAX)
	{
		return false;
	}
	context->offset_sum[flash_slot] += observation;
	context->observation_count[flash_slot]++;
	return true;
}

static bool CurrentOffsetCalibration_AccumulateDirect(
	CurrentOffsetCalibrationContext *context,
	const CurrentControlContext *current_control,
	const MeasurementCurrentSenseConfig *current_sense)
{
	uint8_t expected_phase_mask =
		CurrentOffsetCalibration_ConfiguredPhaseMask(current_sense);
	uint8_t used_flash_slots = 0U;
	uint8_t channel;

	if ((current_sense->topology != PHASE_CURRENT_TOPOLOGY_INLINE_3_SHUNT &&
		 current_sense->topology != PHASE_CURRENT_TOPOLOGY_LOW_SIDE_3_SHUNT &&
		 current_sense->topology != PHASE_CURRENT_TOPOLOGY_LOW_SIDE_2_SHUNT) ||
		expected_phase_mask == PHASE_CURRENT_VALID_NONE ||
		current_control->measurement_acquisition.sample.current_sample_count !=
			current_sense->physical_channel_count ||
		current_control->measurement_acquisition.sample.valid_phase_currents !=
			expected_phase_mask)
	{
		return false;
	}
	for (channel = 0U; channel < current_sense->physical_channel_count;
		channel++)
	{
		const MeasurementCurrentChannelConfig *channel_config =
			&current_sense->channels[channel];
		uint8_t flash_slot;

		if (channel_config->acquisition_index >=
				MEASUREMENT_MODEL_MAX_CURRENT_OBSERVATIONS ||
			!CurrentOffsetCalibration_MapRoleToFlashSlot(channel_config->role,
				&flash_slot) ||
			(used_flash_slots & (uint8_t)(1U << flash_slot)) != 0U ||
			!CurrentOffsetCalibration_AddObservation(context, flash_slot,
				current_control->measurement_acquisition.sample.current_raw[
					channel_config->acquisition_index]))
		{
			return false;
		}
		used_flash_slots = (uint8_t)(used_flash_slots |
			(uint8_t)(1U << flash_slot));
	}
	return true;
}

static bool CurrentOffsetCalibration_AccumulateDcLink(
	CurrentOffsetCalibrationContext *context,
	const CurrentControlContext *current_control,
	const MeasurementCurrentSenseConfig *current_sense)
{
	const MotorDriveServiceAcquisition *acquisition =
		&current_control->measurement_acquisition;
	uint8_t observation;

	if (current_sense->physical_channel_count != 1U ||
		current_sense->channels[0].role !=
			MEASUREMENT_CURRENT_CHANNEL_ROLE_DC_LINK ||
		acquisition->sample.current_sample_count !=
			PHASE_CURRENT_DC_LINK_WINDOW_COUNT ||
		acquisition->sample.valid_phase_currents != PHASE_CURRENT_VALID_NONE ||
		acquisition->sampling.modulation_sector < 1U ||
		acquisition->sampling.modulation_sector >
			PHASE_CURRENT_DC_LINK_SECTOR_COUNT ||
		acquisition->sampling.sampling_point_count !=
			PHASE_CURRENT_DC_LINK_WINDOW_COUNT ||
		acquisition->sampling.points[0].window !=
			BSP_CURRENT_SAMPLING_WINDOW_ACTIVE_VECTOR_FIRST ||
		acquisition->sampling.points[1].window !=
			BSP_CURRENT_SAMPLING_WINDOW_ACTIVE_VECTOR_SECOND)
	{
		return false;
	}
	for (observation = 0U;
		observation < PHASE_CURRENT_DC_LINK_WINDOW_COUNT; observation++)
	{
		if (!CurrentOffsetCalibration_AddObservation(context, 0U,
			acquisition->sample.current_raw[observation]))
		{
			return false;
		}
	}
	return true;
}

void CurrentOffsetCalibrationRuntime_Reset(
	CurrentOffsetCalibrationContext *context)
{
	if (context != 0)
		memset(context, 0, sizeof(*context));
}

CurrentOffsetCalibrationStatus CurrentOffsetCalibrationRuntime_ExecuteStep(
	CurrentOffsetCalibrationContext *context,
	CurrentControlContext *current_control,
	const MeasurementCurrentSenseConfig *current_sense,
	uint32_t sample_count, MotorStateContext *motor_state)
{
	bool accumulated;
	uint8_t flash_slot;

	if (context == 0 || current_control == 0 || current_sense == 0 ||
		sample_count == 0U)
	{
		if (motor_state != 0)
			MotorState_RaiseFault(motor_state, MOTOR_FAULT_INVALID_PARAMETER);
		return CURRENT_OFFSET_CALIBRATION_RUNNING;
	}
	if (context->result_is_ready)
		return CURRENT_OFFSET_CALIBRATION_COMPLETE;
	if (current_sense->topology == PHASE_CURRENT_TOPOLOGY_DC_LINK_1_SHUNT)
	{
		accumulated = CurrentOffsetCalibration_AccumulateDcLink(context,
			current_control, current_sense);
	}
	else
	{
		accumulated = CurrentOffsetCalibration_AccumulateDirect(context,
			current_control, current_sense);
	}
	if (!accumulated)
	{
		CurrentOffsetCalibrationRuntime_Reset(context);
		if (motor_state != 0)
			MotorState_RaiseFault(motor_state, MOTOR_FAULT_POWER_STAGE);
		return CURRENT_OFFSET_CALIBRATION_RUNNING;
	}

	context->frame_count++;
	if (context->frame_count < sample_count)
		return CURRENT_OFFSET_CALIBRATION_RUNNING;
	for (flash_slot = 0U; flash_slot < MEASUREMENT_MODEL_PHASE_COUNT;
		flash_slot++)
	{
		uint32_t average;

		if (context->observation_count[flash_slot] == 0U)
		{
			/* Deployed Flash ABI slots not used by this topology remain zero. */
			context->offset_adc[flash_slot] = 0U;
			continue;
		}
		average = context->offset_sum[flash_slot] /
			context->observation_count[flash_slot];
		if (average > UINT16_MAX)
		{
			if (motor_state != 0)
				MotorState_RaiseFault(motor_state,
					MOTOR_FAULT_CURRENT_OFFSET);
			return CURRENT_OFFSET_CALIBRATION_RUNNING;
		}
		context->offset_adc[flash_slot] = (uint16_t)average;
	}
	context->result_is_ready = true;
	return CURRENT_OFFSET_CALIBRATION_COMPLETE;
}

bool CurrentOffsetCalibrationRuntime_ReadResult(
	const CurrentOffsetCalibrationContext *context,
	uint16_t *phase_a_offset_adc, uint16_t *phase_b_offset_adc,
	uint16_t *phase_c_offset_adc)
{
	if (context == 0 || phase_a_offset_adc == 0 || phase_b_offset_adc == 0 ||
		phase_c_offset_adc == 0 || !context->result_is_ready)
	{
		return false;
	}
	*phase_a_offset_adc = context->offset_adc[0];
	*phase_b_offset_adc = context->offset_adc[1];
	*phase_c_offset_adc = context->offset_adc[2];
	return true;
}
