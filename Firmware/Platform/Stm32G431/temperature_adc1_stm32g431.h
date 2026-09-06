#ifndef PLATFORM_STM32G431_TEMPERATURE_ADC1_STM32G431_H
#define PLATFORM_STM32G431_TEMPERATURE_ADC1_STM32G431_H

#include "Bsp/Api/bsp_temperature.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ADC1 injected rank 1 is configured by MX_ADC1_Init for the processor-die
 * sensor. This adapter observes the next existing external-trigger conversion;
 * it never starts, stops, or reconfigures ADC1. */
bool TemperatureAdc1Stm32g431_CreatePort(
	const BspTemperatureEndpointCapabilities *capabilities,
	const BspMonotonicClockPort *clock,
	BspTemperaturePort *port);

#ifdef __cplusplus
}
#endif

#endif
