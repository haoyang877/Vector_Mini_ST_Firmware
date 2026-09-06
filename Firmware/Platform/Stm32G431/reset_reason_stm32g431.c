#include "reset_reason_stm32g431.h"

#include "stm32g4xx_hal.h"

static uint32_t ResetReasonStm32G431_ReadAndClear(void *context)
{
	BspResetReasonFlagSet reasons = BSP_RESET_REASON_NONE;

	(void)context;
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST) != 0U)
		reasons |= BSP_RESET_REASON_POWER_OR_BROWN_OUT;
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST) != 0U)
		reasons |= BSP_RESET_REASON_EXTERNAL_PIN;
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST) != 0U)
		reasons |= BSP_RESET_REASON_SOFTWARE;
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != 0U)
		reasons |= BSP_RESET_REASON_INDEPENDENT_WATCHDOG;
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST) != 0U)
		reasons |= BSP_RESET_REASON_WINDOW_WATCHDOG;
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST) != 0U)
		reasons |= BSP_RESET_REASON_LOW_POWER;
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_OBLRST) != 0U)
		reasons |= BSP_RESET_REASON_OPTION_BYTES;
	__HAL_RCC_CLEAR_RESET_FLAGS();
	return reasons;
}

BspResetReasonPort ResetReasonStm32G431_CreatePort(void)
{
	BspResetReasonPort port;
	port.context = 0;
	port.read_and_clear = ResetReasonStm32G431_ReadAndClear;
	return port;
}
