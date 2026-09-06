#include "Core/Application/Indicators/led.h"

#define LED (context->state)
#define LEDIndicatorPort (context->port)

bool LED_Initialize(LedServiceContext *context, const BspIndicatorPort *port)
{
	if (context == 0 || port == 0 || port->set_status_leds == 0)
		return false;
	context->port = *port;
	context->is_initialized = true;
	return true;
}

/**
	* @brief  Set LED state
    * @param  mode_or_error: 0:mode 1:error
    * @param  blink_num: LED blink number in a cycle of 5s
 **/
void LED_SetState(LedServiceContext *context, bool mode_or_error,
	uint8_t blink_num)
{
	if (context == 0 || !context->is_initialized)
		return;
	LED.mode_or_error = mode_or_error;
	LED.blink_num = blink_num;
}
	
/**
	* @brief  Set LED related GPIO 
 **/
static void LED_SetGPIO(LedServiceContext *context)
{
	if (context != 0 && context->is_initialized &&
		LEDIndicatorPort.set_status_leds != 0)
		LEDIndicatorPort.set_status_leds(LEDIndicatorPort.context,
			LED.on_or_off[1], LED.on_or_off[0]);
}

/**
	* @brief  LED Task (0.2s)
    * @param  mode_or_error: 0 mode 1 error
 **/
void LED_Task(LedServiceContext *context)
{
	if (context == 0 || !context->is_initialized)
		return;
	/*type of mode or error changed, reset all LEDs*/
	if(LED.mode_or_error_last != LED.mode_or_error ||
	   LED.blink_num_last != LED.blink_num)
	{
		LED.on_or_off[0] = 0;
		LED.on_or_off[1] = 0;
		LED.cnt = 0;
	}
	
	/*mode instruction:green light*/
	if(LED.mode_or_error == 0)
	{
		/*toggle*/
		if(LED.cnt < 2 * LED.blink_num)
			LED.on_or_off[0] = LED.cnt % 2;
		/*turn off*/
		else if(LED.cnt < 25)
			LED.on_or_off[0] = 0;
		/*clear cnt*/
		else
			LED.cnt = 0;
	}
	/*error instruction:red light*/
	else 
	{
		/*toggle*/
		if(LED.cnt < 2 * LED.blink_num)
			LED.on_or_off[1] = LED.cnt % 2;
		/*turn off*/
		else if(LED.cnt < 25)
			LED.on_or_off[1] = 0;
		/*clear cnt*/
		else
			LED.cnt = 0;
	}
	
	LED.cnt ++;
	LED.mode_or_error_last = LED.mode_or_error;
	LED.blink_num_last = LED.blink_num;
	LED_SetGPIO(context);
}
