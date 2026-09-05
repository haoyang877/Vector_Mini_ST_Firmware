#include "board_runtime.h"

#include "adc.h"
#include "tim.h"

static uint32_t BoardRuntimeStm32G431_EnterCritical(void *context)
{
	uint32_t state;

	(void)context;
	state = __get_PRIMASK();
	__disable_irq();
	return state;
}

static void BoardRuntimeStm32G431_ExitCritical(void *context, uint32_t state)
{
	(void)context;
	__set_PRIMASK(state);
}

bool BoardRuntimeStm32G431_Start(void)
{
	if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
		return false;
	if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK)
		return false;
	if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4) != HAL_OK)
		return false;
	if (HAL_ADCEx_InjectedStart(&hadc1) != HAL_OK)
		return false;
	if (HAL_ADCEx_InjectedStart(&hadc2) != HAL_OK)
		return false;

	__HAL_ADC_ENABLE_IT(&hadc2, ADC_IT_JEOC);
	return HAL_TIM_Base_Start_IT(&htim7) == HAL_OK;
}

CriticalSectionPort BoardRuntimeStm32G431_CreateCriticalSectionPort(void)
{
	CriticalSectionPort port;

	port.context = 0;
	port.enter = BoardRuntimeStm32G431_EnterCritical;
	port.exit = BoardRuntimeStm32G431_ExitCritical;
	return port;
}
