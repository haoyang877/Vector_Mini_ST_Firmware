#include "power_stage_tim1.h"

#include <stdint.h>
#include "tim.h"

static bool PowerStageTim1_EnableOutputs(void *context)
{
	HAL_StatusTypeDef status = HAL_OK;
	TIM_HandleTypeDef *timer = (TIM_HandleTypeDef *)context;

	if (timer == NULL)
		return false;

	status = HAL_TIM_PWM_Start(timer, TIM_CHANNEL_1);
	if (status == HAL_OK)
		status = HAL_TIM_PWM_Start(timer, TIM_CHANNEL_2);
	if (status == HAL_OK)
		status = HAL_TIM_PWM_Start(timer, TIM_CHANNEL_3);
	if (status == HAL_OK)
		status = HAL_TIMEx_OCN_Start(timer, TIM_CHANNEL_1);
	if (status == HAL_OK)
		status = HAL_TIMEx_OCN_Start(timer, TIM_CHANNEL_2);
	if (status == HAL_OK)
		status = HAL_TIMEx_OCN_Start(timer, TIM_CHANNEL_3);

	return status == HAL_OK;
}

static void PowerStageTim1_DisableOutputs(void *context)
{
	TIM_HandleTypeDef *timer = (TIM_HandleTypeDef *)context;

	if (timer == NULL)
		return;

	(void)HAL_TIM_PWM_Stop(timer, TIM_CHANNEL_1);
	(void)HAL_TIM_PWM_Stop(timer, TIM_CHANNEL_2);
	(void)HAL_TIM_PWM_Stop(timer, TIM_CHANNEL_3);
	(void)HAL_TIMEx_OCN_Stop(timer, TIM_CHANNEL_1);
	(void)HAL_TIMEx_OCN_Stop(timer, TIM_CHANNEL_2);
	(void)HAL_TIMEx_OCN_Stop(timer, TIM_CHANNEL_3);
}

static void PowerStageTim1_WriteDutyCycles(void *context,
	float phase_a, float phase_b, float phase_c)
{
	TIM_HandleTypeDef *timer = (TIM_HandleTypeDef *)context;
	uint32_t timer_period;

	if (timer == NULL)
		return;
	timer_period = __HAL_TIM_GET_AUTORELOAD(timer);

	__HAL_TIM_SET_COMPARE(timer, TIM_CHANNEL_1,
		(uint16_t)(phase_a * (float)timer_period));
	__HAL_TIM_SET_COMPARE(timer, TIM_CHANNEL_2,
		(uint16_t)(phase_b * (float)timer_period));
	__HAL_TIM_SET_COMPARE(timer, TIM_CHANNEL_3,
		(uint16_t)(phase_c * (float)timer_period));
}

PowerStagePort PowerStageTim1_CreatePort(void)
{
	PowerStagePort port;

	port.context = &htim1;
	port.enable_outputs = PowerStageTim1_EnableOutputs;
	port.disable_outputs = PowerStageTim1_DisableOutputs;
	port.write_duty_cycles = PowerStageTim1_WriteDutyCycles;
	return port;
}
