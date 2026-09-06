#ifndef CORE_APPLICATION_INDICATORS_LED_H
#define CORE_APPLICATION_INDICATORS_LED_H

#include <stdbool.h>
#include <stdint.h>
#include "Bsp/Api/bsp_indicator.h"

typedef struct
{
	bool mode_or_error;
	bool mode_or_error_last;
	bool on_or_off[2];
	uint8_t blink_num;
	uint8_t blink_num_last;
	uint8_t cnt;
}LedContext;

typedef struct
{
	LedContext state;
	BspIndicatorPort port;
	bool is_initialized;
} LedServiceContext;

void LED_SetState(LedServiceContext *context, bool mode_or_error,
	uint8_t blink_num);
bool LED_Initialize(LedServiceContext *context, const BspIndicatorPort *port);
void LED_Task(LedServiceContext *context);
#endif
