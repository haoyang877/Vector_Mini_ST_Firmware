#ifndef PORTS_RESET_REASON_PORT_H
#define PORTS_RESET_REASON_PORT_H

#include <stdint.h>

typedef enum
{
	RESET_REASON_NONE = 0U,
	RESET_REASON_POWER_OR_BROWN_OUT = 1U << 0,
	RESET_REASON_EXTERNAL_PIN = 1U << 1,
	RESET_REASON_SOFTWARE = 1U << 2,
	RESET_REASON_INDEPENDENT_WATCHDOG = 1U << 3,
	RESET_REASON_WINDOW_WATCHDOG = 1U << 4,
	RESET_REASON_LOW_POWER = 1U << 5,
	RESET_REASON_OPTION_BYTES = 1U << 6
} ResetReasonFlag;

typedef struct
{
	void *context;
	uint32_t (*read_and_clear)(void *context);
} ResetReasonPort;

#endif
