#ifndef FIRMWARE_BSP_API_BSP_INDICATOR_H
#define FIRMWARE_BSP_API_BSP_INDICATOR_H

#include "bsp_system.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
	void *context;
	BspResult (*set_status_leds)(void *context, bool red_on, bool green_on);
	/* The implementation may retain data for asynchronous transfer. The caller
	 * must keep it valid and unchanged until the next successful call or until
	 * the board is stopped. BSP_RESULT_BUSY means the previous transfer owns it. */
	BspResult (*send_rgb_pwm)(void *context, const uint32_t *data,
		size_t count);
} BspIndicatorPort;

#ifdef __cplusplus
}
#endif

#endif
