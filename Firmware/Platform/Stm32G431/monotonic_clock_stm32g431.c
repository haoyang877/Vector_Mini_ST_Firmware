#include "monotonic_clock_stm32g431.h"

#include "stm32g4xx_hal.h"

static uint32_t MonotonicClockStm32G431_ReadMs(void *context)
{
	(void)context;
	return HAL_GetTick();
}

MonotonicClockPort MonotonicClockStm32G431_CreatePort(void)
{
	MonotonicClockPort port;
	port.context = 0;
	port.read_ms = MonotonicClockStm32G431_ReadMs;
	return port;
}
