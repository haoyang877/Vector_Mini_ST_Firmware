#include "indicator_stm32g431.h"

#include "main.h"
#include "tim.h"

static BspResult IndicatorStm32G431_SetStatusLeds(void *context,
	bool red_on, bool green_on)
{
	(void)context;
	LED_R_GPIO_Port->BSRR = red_on ?
		((uint32_t)LED_R_Pin << 16U) : LED_R_Pin;
	LED_G_GPIO_Port->BSRR = green_on ?
		((uint32_t)LED_G_Pin << 16U) : LED_G_Pin;
	return BSP_RESULT_OK;
}

static BspResult IndicatorStm32G431_SendRgbPwm(void *context,
	const uint32_t *data, size_t count)
{
	HAL_StatusTypeDef status;

	(void)context;
	if (data == 0 || count == 0U || count > UINT16_MAX)
		return BSP_RESULT_INVALID_ARGUMENT;
	status = HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_3, (uint32_t *)data,
		(uint16_t)count);
	if (status == HAL_OK)
		return BSP_RESULT_OK;
	if (status == HAL_BUSY)
		return BSP_RESULT_BUSY;
	return BSP_RESULT_IO_ERROR;
}

BspIndicatorPort IndicatorStm32G431_CreatePort(void)
{
	BspIndicatorPort port;
	port.context = 0;
	port.set_status_leds = IndicatorStm32G431_SetStatusLeds;
	port.send_rgb_pwm = IndicatorStm32G431_SendRgbPwm;
	return port;
}
