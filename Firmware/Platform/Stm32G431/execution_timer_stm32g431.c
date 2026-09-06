#include "execution_timer_stm32g431.h"

#include "stm32g4xx.h"

static uint32_t ExecutionTimerStm32G431_ReadCycles(void *context)
{
	(void)context;
	return DWT->CYCCNT;
}

BspExecutionTimerPort ExecutionTimerStm32G431_CreatePort(void)
{
	BspExecutionTimerPort port;

	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0U;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	port.context = 0;
	port.frequency_hz = SystemCoreClock;
	port.read_cycles = ExecutionTimerStm32G431_ReadCycles;
	return port;
}
