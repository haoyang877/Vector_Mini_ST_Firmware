#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define __ADC_H__

typedef struct
{
	volatile uint32_t ISR;
	volatile uint32_t JSQR;
	volatile uint32_t JDR1;
} ADC_TypeDef;

typedef struct { ADC_TypeDef *Instance; } ADC_HandleTypeDef;

static ADC_TypeDef FakeAdc1;
ADC_HandleTypeDef hadc1;

#define ADC1 (&FakeAdc1)
#define ADC_ISR_OVR     (UINT32_C(1) << 4)
#define ADC_ISR_JEOC    (UINT32_C(1) << 5)
#define ADC_ISR_JEOS    (UINT32_C(1) << 6)
#define ADC_ISR_JQOVF   (UINT32_C(1) << 10)
#define ADC_JSQR_JL_Msk UINT32_C(0x3)
#define ADC_JSQR_JEXTSEL_Msk (UINT32_C(0x1F) << 2)
#define ADC_JSQR_JEXTEN_Msk (UINT32_C(0x3) << 7)
#define ADC_JSQR_JSQ1_Pos 9U
#define ADC_JSQR_JSQ1_Msk (UINT32_C(0x1F) << ADC_JSQR_JSQ1_Pos)
#define ADC_EXTERNALTRIGINJEC_T1_CC4 \
	((UINT32_C(1) << 2) | (UINT32_C(1) << 7))
#define ADC_EXTERNALTRIGINJECCONV_EDGE_FALLING (UINT32_C(2) << 7)
#define ADC_RESOLUTION_12B 12U
#define __HAL_ADC_CALC_TEMPERATURE(vdda_, raw_, resolution_) \
	((void)(vdda_), (void)(resolution_), (raw_) / 10.0f)

#include "../../Firmware/Platform/Stm32G431/temperature_adc1_stm32g431.c"

#define CHECK(condition_) do { \
	if (!(condition_)) { \
		(void)fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #condition_); \
		return 1; \
	} \
} while (0)

typedef struct { uint32_t now_ms; } FakeClock;

static uint32_t FakeClock_ReadMs(void *context)
{
	return ((FakeClock *)context)->now_ms;
}

static BspTemperatureEndpointCapabilities MakeCapabilities(void)
{
	BspTemperatureEndpointCapabilities capabilities;

	(void)memset(&capabilities, 0, sizeof(capabilities));
	capabilities.endpoint_id = 1U;
	capabilities.availability = BSP_ENDPOINT_AVAILABLE;
	capabilities.source_kind = BSP_TEMPERATURE_SOURCE_PROCESSOR_DIE;
	capabilities.location = BSP_TEMPERATURE_LOCATION_PROCESSOR;
	return capabilities;
}

int main(void)
{
	BspTemperatureEndpointCapabilities capabilities = MakeCapabilities();
	BspTemperatureEndpointCapabilities invalid_capabilities = capabilities;
	BspMonotonicClockPort clock_port;
	BspTemperaturePort port;
	BspTemperatureSample sample;
	FakeClock clock = { 123U };

	(void)memset(&FakeAdc1, 0, sizeof(FakeAdc1));
	hadc1.Instance = ADC1;
	clock_port.context = &clock;
	clock_port.read_ms = FakeClock_ReadMs;
	invalid_capabilities.location = BSP_TEMPERATURE_LOCATION_POWER_STAGE;
	CHECK(!TemperatureAdc1Stm32g431_CreatePort(&invalid_capabilities,
		&clock_port, &port));
	CHECK(!TemperatureAdc1Stm32g431_CreatePort(&capabilities, NULL, &port));
	CHECK(TemperatureAdc1Stm32g431_CreatePort(&capabilities, &clock_port,
		&port));

	FakeAdc1.JSQR =
		(ADC_EXTERNALTRIGINJEC_T1_CC4 & ADC_JSQR_JEXTSEL_Msk) |
		ADC_EXTERNALTRIGINJECCONV_EDGE_FALLING |
		(UINT32_C(15) << ADC_JSQR_JSQ1_Pos);
	CHECK(port.initialize(port.context) == BSP_RESULT_NOT_SUPPORTED);
	FakeAdc1.JSQR =
		(ADC_EXTERNALTRIGINJEC_T1_CC4 & ADC_JSQR_JEXTSEL_Msk) |
		ADC_EXTERNALTRIGINJECCONV_EDGE_FALLING |
		(UINT32_C(16) << ADC_JSQR_JSQ1_Pos);
	CHECK(port.initialize(port.context) == BSP_RESULT_OK);
	CHECK(port.try_read_latest(port.context, &sample) == BSP_RESULT_NOT_READY);
	CHECK(port.request_sample(port.context) == BSP_RESULT_OK);
	CHECK(port.request_sample(port.context) == BSP_RESULT_BUSY);
	FakeAdc1.ISR = 0U;
	CHECK(port.try_read_latest(port.context, &sample) == BSP_RESULT_NOT_READY);
	CHECK((port.read_status(port.context) & BSP_TEMPERATURE_SAMPLE_STALE) != 0U);

	FakeAdc1.JDR1 = 555U;
	FakeAdc1.ISR = ADC_ISR_JEOS;
	CHECK(port.try_read_latest(port.context, &sample) == BSP_RESULT_OK);
	CHECK(sample.temperature_c == 55.5f && sample.timestamp_us == 123000U);
	CHECK(sample.sequence == 1U &&
		sample.status == BSP_TEMPERATURE_SAMPLE_VALID);
	CHECK(port.try_read_latest(port.context, &sample) == BSP_RESULT_OK);
	CHECK(sample.sequence == 1U);

	/* An overrun observed when the request is issued is attached to the next
	 * completed sample even if the W1C flag has cleared before completion. */
	FakeAdc1.ISR = ADC_ISR_OVR;
	CHECK(port.request_sample(port.context) == BSP_RESULT_OK);
	clock.now_ms = 124U;
	FakeAdc1.JDR1 = 600U;
	FakeAdc1.ISR = ADC_ISR_JEOS;
	CHECK(port.try_read_latest(port.context, &sample) == BSP_RESULT_OK);
	CHECK(sample.temperature_c == 60.0f && sample.timestamp_us == 124000U);
	CHECK(sample.sequence == 2U);
	CHECK((sample.status & BSP_TEMPERATURE_ADC_OVERRUN) != 0U);
	(void)puts("PASS temperature_adc1_stm32g431_fake");
	return 0;
}
