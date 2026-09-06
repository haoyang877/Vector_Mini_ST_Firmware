#ifndef APPLICATION_LED_INDICATOR_H
#define APPLICATION_LED_INDICATOR_H

#include <stdbool.h>
#include <stdint.h>
#include "bsp_indicator.h"

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
