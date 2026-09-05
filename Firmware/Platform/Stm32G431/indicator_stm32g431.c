#include "indicator_stm32g431.h"

#include "main.h"
#include "tim.h"

static void IndicatorStm32G431_SetStatusLeds(void *context,
	bool red_on, bool green_on)
{
	(void)context;
	LED_R_GPIO_Port->BSRR = red_on ?
		((uint32_t)LED_R_Pin << 16U) : LED_R_Pin;
	LED_G_GPIO_Port->BSRR = green_on ?
		((uint32_t)LED_G_Pin << 16U) : LED_G_Pin;
}

static bool IndicatorStm32G431_SendRgbPwm(void *context,
	uint32_t *data, uint16_t count)
{
	(void)context;
	return HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_3, data, count) == HAL_OK;
}

IndicatorPort IndicatorStm32G431_CreatePort(void)
{
	IndicatorPort port;
	port.context = 0;
	port.set_status_leds = IndicatorStm32G431_SetStatusLeds;
	port.send_rgb_pwm = IndicatorStm32G431_SendRgbPwm;
	return port;
}
