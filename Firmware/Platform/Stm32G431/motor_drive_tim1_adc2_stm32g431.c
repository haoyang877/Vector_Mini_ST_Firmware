#include "motor_drive_tim1_adc2_stm32g431.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "adc.h"
#include "tim.h"

#define VECTOR_MINI_PWM_FREQUENCY_HZ 20000UL
#define MOTOR_OUTPUT_ENABLE_MASK \
	(TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE | \
	 TIM_CCER_CC3E | TIM_CCER_CC3NE)
#define ADC_ACQUISITION_OVERRUN_MASK (ADC_ISR_OVR | ADC_ISR_JQOVF)

static bool MotorDrive_AdcSequenceIsSupported(void)
{
	uint32_t sequence = ADC2->JSQR;
	uint32_t trigger =
		(ADC_EXTERNALTRIGINJEC_T1_CC4 & ADC_JSQR_JEXTSEL_Msk) |
		ADC_EXTERNALTRIGINJECCONV_EDGE_FALLING;

	return (sequence & ADC_JSQR_JL_Msk) == ADC_JSQR_JL_Msk &&
		(sequence & (ADC_JSQR_JEXTSEL_Msk | ADC_JSQR_JEXTEN_Msk)) ==
			trigger &&
		((sequence & ADC_JSQR_JSQ1_Msk) >> ADC_JSQR_JSQ1_Pos) == 13U &&
		((sequence & ADC_JSQR_JSQ2_Msk) >> ADC_JSQR_JSQ2_Pos) == 3U &&
		((sequence & ADC_JSQR_JSQ3_Msk) >> ADC_JSQR_JSQ3_Pos) == 5U &&
		((sequence & ADC_JSQR_JSQ4_Msk) >> ADC_JSQR_JSQ4_Pos) == 17U;
}

typedef struct
{
	const BspMotorDriveEndpointCapabilities *capabilities;
	volatile BspMotorDriveFaultSet faults;
	uint32_t queued_sequence;
	uint32_t completed_sequence;
	BspMotorDriveSampleStatusSet completed_status;
	uint8_t fixed_sample_count;
	BspMotorPhaseSet fixed_direct_phase_currents;
	bool initialized;
	volatile bool armed;
	volatile bool cycle_pending;
	volatile bool sample_ready;
	volatile bool primed;
} MotorDriveContext;

static MotorDriveContext MotorDriveState;

static void MotorDrive_DisableImmediate(void *context)
{
	/* This path intentionally bypasses HAL state and is safe to call repeatedly
	 * from any context. CH4 and MOE remain active because CH4 is the injected
	 * ADC trigger; CH1..3 main/complementary enables are the power-stage safety
	 * boundary. Do not add a blocking peripheral operation here. */
	CLEAR_BIT(TIM1->CCER, MOTOR_OUTPUT_ENABLE_MASK);
	if (context == &MotorDriveState)
		MotorDriveState.armed = false;
}

static bool MotorDrive_CapabilitiesAreSupported(
	const BspMotorDriveEndpointCapabilities *capabilities)
{
	const BspCurrentSenseTopologySet topology =
		BSP_CURRENT_SENSE_TOPOLOGY_BIT(
			BSP_CURRENT_SENSE_LOW_SIDE_THREE_SHUNT);
	const BspCurrentSamplingModeSet sampling =
		BSP_CURRENT_SAMPLING_MODE_BIT(BSP_CURRENT_SAMPLING_MODE_FIXED);

	return capabilities != NULL &&
		capabilities->endpoint_id != BSP_ENDPOINT_ID_NONE &&
		capabilities->availability == BSP_ENDPOINT_AVAILABLE &&
		capabilities->supported_current_sense_topologies == topology &&
		capabilities->current_sensor_capacity == BSP_MOTOR_PHASE_COUNT &&
		capabilities->supports_synchronized_sampling &&
		capabilities->supported_sampling_modes == sampling;
}

static BspResult MotorDrive_InitializeSafe(void *context,
	const BspMotorDriveConfiguration *configuration)
{
	MotorDriveContext *state = (MotorDriveContext *)context;

	/* Unsupported and malformed configurations must leave the bridge disabled. */
	MotorDrive_DisableImmediate(context);
	CLEAR_BIT(TIM1->CCER, MOTOR_OUTPUT_ENABLE_MASK);
	if (state != &MotorDriveState || configuration == NULL)
		return BSP_RESULT_INVALID_ARGUMENT;
	state->initialized = false;
	state->faults = 0U;
	if (!MotorDrive_CapabilitiesAreSupported(state->capabilities) ||
		configuration->current_sense_topology !=
			BSP_CURRENT_SENSE_LOW_SIDE_THREE_SHUNT ||
		configuration->sampling_mode != BSP_CURRENT_SAMPLING_MODE_FIXED ||
		configuration->fixed_sample_count != BSP_MOTOR_PHASE_COUNT ||
		configuration->fixed_direct_phase_currents != BSP_MOTOR_PHASE_ALL ||
		configuration->pwm_frequency_hz != VECTOR_MINI_PWM_FREQUENCY_HZ)
	{
		return BSP_RESULT_NOT_SUPPORTED;
	}
	if (htim1.Instance != TIM1 || hadc2.Instance != ADC2 || TIM1->ARR == 0U ||
		!MotorDrive_AdcSequenceIsSupported())
		return BSP_RESULT_NOT_READY;
	/* Guarantee that commit_cycle() targets CH1..3 shadow registers without
	 * disturbing the CH4 ADC trigger channel. */
	SET_BIT(TIM1->CCMR1, TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE);
	SET_BIT(TIM1->CCMR2, TIM_CCMR2_OC3PE);
	state->fixed_sample_count = configuration->fixed_sample_count;
	state->fixed_direct_phase_currents =
		configuration->fixed_direct_phase_currents;
	state->initialized = true;
	return BSP_RESULT_OK;
}

static BspResult MotorDrive_Arm(void *context)
{
	MotorDriveContext *state = (MotorDriveContext *)context;

	MotorDrive_DisableImmediate(context);
	if (state != &MotorDriveState)
		return BSP_RESULT_INVALID_ARGUMENT;
	if (!state->initialized || !state->primed)
		return BSP_RESULT_NOT_READY;
	if (state->faults != 0U)
		return BSP_RESULT_SAFETY_FAULT;

	/* TIM1 CH4, counter/base state, and both ADCs remain untouched. Restore the
	 * global gate first; enabling CH1..3 is the final energizing operation. */
	SET_BIT(TIM1->BDTR, TIM_BDTR_MOE);
	SET_BIT(TIM1->CCER, MOTOR_OUTPUT_ENABLE_MASK);
	if ((TIM1->BDTR & TIM_BDTR_MOE) == 0U ||
		(TIM1->CCER & MOTOR_OUTPUT_ENABLE_MASK) != MOTOR_OUTPUT_ENABLE_MASK)
	{
		MotorDrive_DisableImmediate(context);
		state->faults |= BSP_MOTOR_DRIVE_FAULT_POWER_STAGE;
		return BSP_RESULT_IO_ERROR;
	}
	state->armed = true;
	return BSP_RESULT_OK;
}

static BspResult MotorDrive_Disarm(void *context)
{
	MotorDriveContext *state = (MotorDriveContext *)context;

	MotorDrive_DisableImmediate(context);
	if (state != &MotorDriveState)
		return BSP_RESULT_INVALID_ARGUMENT;
	CLEAR_BIT(TIM1->CCER, MOTOR_OUTPUT_ENABLE_MASK);
	return BSP_RESULT_OK;
}

static BspResult MotorDrive_ReadSample(void *context,
	BspMotorDriveSample *sample)
{
	MotorDriveContext *state = (MotorDriveContext *)context;
	if (state != &MotorDriveState || sample == NULL)
		return BSP_RESULT_INVALID_ARGUMENT;
	if (!state->initialized)
		return BSP_RESULT_NOT_READY;
	if (!state->sample_ready)
		return BSP_RESULT_NOT_READY;
	sample->current_raw[0] = ADC2->JDR1;
	sample->current_raw[1] = ADC2->JDR2;
	sample->current_raw[2] = ADC2->JDR3;
	sample->current_sample_count = state->fixed_sample_count;
	sample->valid_phase_currents = state->fixed_direct_phase_currents;
	sample->bus_voltage_raw = ADC2->JDR4;
	sample->sequence = state->completed_sequence;
	sample->status = BSP_MOTOR_DRIVE_SAMPLE_VALID | state->completed_status;
	state->sample_ready = false;
	return BSP_RESULT_OK;
}

void MotorDriveTim1Adc2Stm32g431_OnInjectedSequenceComplete(void)
{
	uint32_t adc_status;

	if (!MotorDriveState.initialized || !MotorDriveState.cycle_pending)
		return;
	adc_status = ADC2->ISR;
	if ((adc_status & ADC_ISR_JEOS) == 0U)
		return;
	if (MotorDriveState.sample_ready)
	{
		MotorDriveState.faults |= BSP_MOTOR_DRIVE_FAULT_SAMPLING;
		return;
	}
	MotorDriveState.completed_sequence = MotorDriveState.queued_sequence;
	MotorDriveState.completed_status = 0U;
	if ((adc_status & ADC_ACQUISITION_OVERRUN_MASK) != 0U)
	{
		MotorDriveState.completed_status = BSP_MOTOR_DRIVE_SAMPLE_ADC_OVERRUN;
		MotorDriveState.faults |= BSP_MOTOR_DRIVE_FAULT_SAMPLING;
		ADC2->ISR = adc_status & ADC_ACQUISITION_OVERRUN_MASK;
	}
	MotorDriveState.cycle_pending = false;
	MotorDriveState.sample_ready = true;
	MotorDriveState.primed = true;
}

static bool MotorDrive_FixedPlanIsValid(
	const BspCurrentSamplingPlan *plan)
{
	return plan->mode == BSP_CURRENT_SAMPLING_MODE_FIXED &&
		plan->modulation_sector == 0U && plan->sampling_point_count == 0U &&
		plan->cycle_valid_phase_currents == BSP_MOTOR_PHASE_ALL;
}

static BspResult MotorDrive_CommitCycle(void *context,
	const BspMotorDriveCycleCommand *command)
{
	MotorDriveContext *state = (MotorDriveContext *)context;
	uint32_t period;
	uint32_t phase;
	uint32_t compare[BSP_MOTOR_PHASE_COUNT];

	if (state != &MotorDriveState || command == NULL)
		return BSP_RESULT_INVALID_ARGUMENT;
	if (!state->initialized)
		return BSP_RESULT_NOT_READY;
	if (state->cycle_pending || state->sample_ready)
		return BSP_RESULT_BUSY;
	if (!MotorDrive_FixedPlanIsValid(&command->sampling))
		return BSP_RESULT_NOT_SUPPORTED;

	period = TIM1->ARR;
	if (period == 0U)
	{
		state->faults |= BSP_MOTOR_DRIVE_FAULT_TIMING;
		return BSP_RESULT_IO_ERROR;
	}
	for (phase = 0U; phase < BSP_MOTOR_PHASE_COUNT; ++phase)
	{
		float duty = command->pwm.phase_duty[phase];

		if (!isfinite(duty) || duty < 0.0f || duty > 1.0f)
			return BSP_RESULT_INVALID_ARGUMENT;
		compare[phase] = (uint32_t)(duty * (float)period);
	}

	/* CH1..3 preload is configured by MX_TIM1_Init; all three shadow values
	 * become active together at the next timer update event. */
	TIM1->CCR1 = compare[0];
	TIM1->CCR2 = compare[1];
	TIM1->CCR3 = compare[2];
	state->queued_sequence = command->sampling.sequence;
	state->cycle_pending = true;
	return BSP_RESULT_OK;
}

static BspMotorDriveFaultSet MotorDrive_ReadFaults(void *context)
{
	MotorDriveContext *state = (MotorDriveContext *)context;
	BspMotorDriveFaultSet faults;

	if (state != &MotorDriveState)
		return BSP_MOTOR_DRIVE_FAULT_POWER_STAGE;
	faults = state->faults;
	if (state->armed && ((TIM1->BDTR & TIM_BDTR_MOE) == 0U ||
		(TIM1->CCER & MOTOR_OUTPUT_ENABLE_MASK) != MOTOR_OUTPUT_ENABLE_MASK))
		faults |= BSP_MOTOR_DRIVE_FAULT_POWER_STAGE;
	return faults;
}

bool MotorDriveTim1Adc2Stm32g431_CreatePort(
	const BspMotorDriveEndpointCapabilities *capabilities,
	BspMotorDrivePort *port)
{
	if (port == NULL || !MotorDrive_CapabilitiesAreSupported(capabilities))
	{
		MotorDrive_DisableImmediate(NULL);
		return false;
	}
	(void)memset(&MotorDriveState, 0, sizeof(MotorDriveState));
	MotorDriveState.capabilities = capabilities;
	MotorDrive_DisableImmediate(&MotorDriveState);
	port->context = &MotorDriveState;
	port->capabilities = capabilities;
	port->initialize_safe = MotorDrive_InitializeSafe;
	port->arm = MotorDrive_Arm;
	port->disarm = MotorDrive_Disarm;
	port->read_sample = MotorDrive_ReadSample;
	port->commit_cycle = MotorDrive_CommitCycle;
	port->disable_immediate = MotorDrive_DisableImmediate;
	port->read_faults = MotorDrive_ReadFaults;
	return true;
}
