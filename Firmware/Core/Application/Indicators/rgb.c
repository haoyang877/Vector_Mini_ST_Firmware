#include "Core/Application/Indicators/rgb.h"

/*2d array of data to be sent by DMA*/
#define Pixel_Buf (context->pixel_buffer)
#define RGB (context->color)
#define rgb_up_down (context->direction)
#define brightness (context->brightness)
#define RGBIndicatorPort (context->port)

bool RGB_Initialize(RgbServiceContext *context, const BspIndicatorPort *port)
{
	if (context == 0 || port == 0 || port->send_rgb_pwm == 0)
		return false;
	context->port = *port;
	context->is_initialized = true;
	return true;
}


/**
	* @brief  Set single color of RGB LED 
	          Encode Color to 24 bits and upload them to sending array
    * @param  LedId: Id of RGB LED
	* @param  *Color: RGB struct pointer
 **/
void RGB_SetColor(RgbServiceContext *context, uint8_t LedId,
	const RGB_Color *Color)
{
    uint8_t i;
	if (context == 0 || !context->is_initialized || Color == 0)
		return;
	/*avoid overflow*/
    if(LedId > LED_NUM){
        return;
    }

	/*G*/
    for(i = 0; i < 8; i++) {
        Pixel_Buf[LedId][i] = ((255 - Color->G & (1 << (7 - i))) ? (CODE_1) : CODE_0);
    }
	/*R*/
    for(i = 8; i < 16; i++)  {
        Pixel_Buf[LedId][i] = ((255 - Color->R & (1 << (15 - i))) ? (CODE_1) : CODE_0);
    }
	/*B*/
    for(i = 16; i < 24; i++) {
        Pixel_Buf[LedId][i] = ((255 - Color->B & (1 << (23 - i))) ? (CODE_1) : CODE_0);
    }
}

/**
	* @brief Reset load value of last row
             Load 24 "0" bits to last row as reset delay 
             24 * 1.2us = 30us > 24us(minimum delay)
 **/
static void Reset_Load(RgbServiceContext *context)
{
    uint8_t i;
	if (context == 0 || !context->is_initialized)
		return;
	for(i=0; i<24; i++)
    {
        Pixel_Buf[LED_NUM][i] = 0;
    }
}

/**
	* @brief Send loaded array with DMA to generate PWM
 **/
static void RGB_SendArray(RgbServiceContext *context)
{
	if (context != 0 && context->is_initialized &&
		RGBIndicatorPort.send_rgb_pwm != 0)
		(void)RGBIndicatorPort.send_rgb_pwm(RGBIndicatorPort.context,
			(uint32_t *)Pixel_Buf, (LED_NUM + 1U) * 24U);
}

/**
	* @brief  write color to RGB LEDs 
    * @param  led_num: total number of RGB LEDs
	* @param  *Color: RGB struct pointer
 **/
static void write_color(RgbServiceContext *context, uint16_t led_num,
	const RGB_Color *color)
{
    uint16_t i;
	
    for(i = 0; i < led_num; i++)
    {
		RGB_SetColor(context, i, color);
    }
	Reset_Load(context);
	RGB_SendArray(context);
}

/**
	* @brief  Set color of RGB LED and add breathing effects 
	* @param  color_type: color type enum 
 **/
void Set_RGB_BreathingColor(RgbServiceContext *context,
	COLOR_Type color_type)
{
	if (context == 0 || !context->is_initialized)
		return;
	if(rgb_up_down == 0)
	{
		brightness += 4;
		if(brightness >= 100)
		{
			brightness = 100;
			rgb_up_down = 1;
		}
	}
	else if(rgb_up_down == 1)
	{
		brightness -= 4;
		if(brightness <= -40)
		{
			brightness = -40;
			rgb_up_down = 0;
		}
	}
	
	switch(color_type)
	{
		case COLOR_NULL:
			RGB.R = 0;
			RGB.G = 0;
			RGB.B = 0;
		break;
		
		case RED:
			RGB.R = brightness >= 0 ? brightness : 0;
			RGB.G = 0;
			RGB.B = 0;	
		break;
		
		case GREEN:
			RGB.R = 0;
			RGB.G = brightness >= 0 ? brightness : 0;
			RGB.B = 0;
		break;
		
		case BLUE:
			RGB.R = 0;
			RGB.G = 0;
			RGB.B = brightness >= 0 ? brightness : 0;
		break;
		
		case YELLOW:
			RGB.R = brightness >= 0 ? brightness : 0;
			RGB.G = brightness >= 0 ? brightness : 0;
			RGB.B = 0;
		break;
		
		case PURPLE:
			RGB.R = brightness >= 0 ? brightness : 0;
			RGB.G = 0;
			RGB.B = brightness >= 0 ? brightness : 0;
		break;
		
		case CYAN:
			RGB.R = 0;
			RGB.G = brightness >= 0 ? brightness : 0;
			RGB.B = brightness >= 0 ? brightness : 0;
		break;
		
		case WHITE:
			RGB.R = brightness >= 0 ? brightness : 0;
			RGB.G = brightness >= 0 ? brightness : 0;
			RGB.B = brightness >= 0 ? brightness : 0;
		break;
		
		default:break;
	}
	
	write_color(context, 1, &RGB);
}
