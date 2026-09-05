#ifndef PORTS_INDICATOR_PORT_H
#define PORTS_INDICATOR_PORT_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
	void *context;
	void (*set_status_leds)(void *context, bool red_on, bool green_on);
	bool (*send_rgb_pwm)(void *context, uint32_t *data, uint16_t count);
} IndicatorPort;

#endif
