#include "execution_timer_stm32g431.h"

#include "stm32g4xx.h"

static uint32_t ExecutionTimerStm32G431_ReadCycles(void *context)
{
	(void)context;
	return DWT->CYCCNT;
}

static uint32_t ExecutionTimerStm32G431_CyclesPerSecond(void *context)
{
	(void)context;
	return SystemCoreClock;
}

ExecutionTimerPort ExecutionTimerStm32G431_CreatePort(void)
{
	ExecutionTimerPort port;

	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0U;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	port.context = 0;
	port.read_cycles = ExecutionTimerStm32G431_ReadCycles;
	port.cycles_per_second = ExecutionTimerStm32G431_CyclesPerSecond;
	return port;
}
