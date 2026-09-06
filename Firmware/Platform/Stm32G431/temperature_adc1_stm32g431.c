#include "temperature_adc1_stm32g431.h"

#include <stddef.h>
#include <string.h>

#include "adc.h"

#define TEMPERATURE_ADC_VDDA_MV 3300UL
#define TEMPERATURE_ADC_RANK_1_CHANNEL 16UL
#define TEMPERATURE_ADC_OVERRUN_MASK (ADC_ISR_OVR | ADC_ISR_JQOVF)
#define TEMPERATURE_ADC_COMPLETION_MASK (ADC_ISR_JEOC | ADC_ISR_JEOS)

typedef struct
{
	const BspTemperatureEndpointCapabilities *capabilities;
	BspMonotonicClockPort clock;
	BspTemperatureSample latest;
	BspTemperatureStatusSet pending_status;
	BspTemperatureStatusSet observed_status;
	bool initialized;
	bool request_pending;
	bool has_latest;
} TemperatureContext;

static TemperatureContext TemperatureState;

static bool Temperature_CapabilitiesAreSupported(
	const BspTemperatureEndpointCapabilities *capabilities)
{
	return capabilities != NULL &&
		capabilities->endpoint_id != BSP_ENDPOINT_ID_NONE &&
		capabilities->availability == BSP_ENDPOINT_AVAILABLE &&
		capabilities->source_kind == BSP_TEMPERATURE_SOURCE_PROCESSOR_DIE &&
		capabilities->location == BSP_TEMPERATURE_LOCATION_PROCESSOR &&
		!capabilities->supports_open_circuit_diagnostic &&
		!capabilities->supports_short_circuit_diagnostic;
}

static BspResult Temperature_Initialize(void *context)
{
	TemperatureContext *state = (TemperatureContext *)context;
	uint32_t injected_channel;

	if (state != &TemperatureState)
		return BSP_RESULT_INVALID_ARGUMENT;
	state->initialized = false;
	state->request_pending = false;
	state->has_latest = false;
	state->pending_status = 0U;
	state->observed_status = BSP_TEMPERATURE_SAMPLE_STALE;
	(void)memset(&state->latest, 0, sizeof(state->latest));
	if (!Temperature_CapabilitiesAreSupported(state->capabilities) ||
		hadc1.Instance != ADC1)
	{
		return BSP_RESULT_NOT_SUPPORTED;
	}
	injected_channel = (ADC1->JSQR & ADC_JSQR_JSQ1_Msk) >>
		ADC_JSQR_JSQ1_Pos;
	if ((ADC1->JSQR & ADC_JSQR_JL_Msk) != 0U ||
		injected_channel != TEMPERATURE_ADC_RANK_1_CHANNEL ||
		(ADC1->JSQR & (ADC_JSQR_JEXTSEL_Msk | ADC_JSQR_JEXTEN_Msk)) !=
			((ADC_EXTERNALTRIGINJEC_T1_CC4 & ADC_JSQR_JEXTSEL_Msk) |
			 ADC_EXTERNALTRIGINJECCONV_EDGE_FALLING))
	{
		return BSP_RESULT_NOT_SUPPORTED;
	}
	state->initialized = true;
	return BSP_RESULT_OK;
}

static BspResult Temperature_RequestSample(void *context)
{
	TemperatureContext *state = (TemperatureContext *)context;
	uint32_t adc_status;

	if (state != &TemperatureState)
		return BSP_RESULT_INVALID_ARGUMENT;
	if (!state->initialized)
		return BSP_RESULT_NOT_READY;
	if (state->request_pending)
		return BSP_RESULT_BUSY;

	adc_status = ADC1->ISR;
	state->pending_status = 0U;
	if ((adc_status & TEMPERATURE_ADC_OVERRUN_MASK) != 0U)
		state->pending_status = BSP_TEMPERATURE_ADC_OVERRUN;
	/* Discard an old completion. The already-running external trigger path will
	 * publish the requested conversion without blocking this call. */
	ADC1->ISR = TEMPERATURE_ADC_COMPLETION_MASK |
		TEMPERATURE_ADC_OVERRUN_MASK;
	state->request_pending = true;
	return BSP_RESULT_OK;
}

static BspResult Temperature_TryReadLatest(void *context,
	BspTemperatureSample *sample)
{
	TemperatureContext *state = (TemperatureContext *)context;
	uint32_t adc_status;
	BspTemperatureStatusSet acquisition_status;

	if (state != &TemperatureState || sample == NULL)
		return BSP_RESULT_INVALID_ARGUMENT;
	if (!state->initialized)
		return BSP_RESULT_NOT_READY;
	if (state->request_pending)
	{
		adc_status = ADC1->ISR;
		if ((adc_status & ADC_ISR_JEOS) == 0U)
			return BSP_RESULT_NOT_READY;
		acquisition_status = state->pending_status;
		if ((adc_status & TEMPERATURE_ADC_OVERRUN_MASK) != 0U)
			acquisition_status |= BSP_TEMPERATURE_ADC_OVERRUN;
		state->latest.temperature_c = (float)__HAL_ADC_CALC_TEMPERATURE(
			TEMPERATURE_ADC_VDDA_MV, ADC1->JDR1, ADC_RESOLUTION_12B);
		state->latest.timestamp_us =
			state->clock.read_ms(state->clock.context) * 1000U;
		state->latest.sequence++;
		state->latest.status = BSP_TEMPERATURE_SAMPLE_VALID |
			acquisition_status;
		state->observed_status = state->latest.status;
		state->request_pending = false;
		state->has_latest = true;
		ADC1->ISR = TEMPERATURE_ADC_COMPLETION_MASK |
			TEMPERATURE_ADC_OVERRUN_MASK;
	}
	if (!state->has_latest)
		return BSP_RESULT_NOT_READY;
	*sample = state->latest;
	return BSP_RESULT_OK;
}

static BspTemperatureStatusSet Temperature_ReadStatus(void *context)
{
	TemperatureContext *state = (TemperatureContext *)context;

	if (state != &TemperatureState)
		return BSP_TEMPERATURE_SENSOR_FAULT;
	if (state->request_pending)
		return state->observed_status | BSP_TEMPERATURE_SAMPLE_STALE;
	return state->observed_status;
}

bool TemperatureAdc1Stm32g431_CreatePort(
	const BspTemperatureEndpointCapabilities *capabilities,
	const BspMonotonicClockPort *clock,
	BspTemperaturePort *port)
{
	if (port == NULL || clock == NULL || clock->read_ms == NULL ||
		!Temperature_CapabilitiesAreSupported(capabilities))
	{
		return false;
	}
	(void)memset(&TemperatureState, 0, sizeof(TemperatureState));
	TemperatureState.capabilities = capabilities;
	TemperatureState.clock = *clock;
	port->context = &TemperatureState;
	port->capabilities = capabilities;
	port->initialize = Temperature_Initialize;
	port->request_sample = Temperature_RequestSample;
	port->try_read_latest = Temperature_TryReadLatest;
	port->read_status = Temperature_ReadStatus;
	return true;
}
