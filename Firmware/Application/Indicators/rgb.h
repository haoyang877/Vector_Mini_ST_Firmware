#ifndef APPLICATION_RGB_INDICATOR_H
#define APPLICATION_RGB_INDICATOR_H

#include <stdbool.h>
#include <stdint.h>
#include "bsp_indicator.h"

/*code type of WS2812*/
#define CODE_1		(58)    //count of "1" bit
#define CODE_0		(136)    //count of "0" bit

/*total LED number in series*/
#define LED_NUM		1

typedef enum
{
	COLOR_NULL,
	RED,
	GREEN,
	BLUE,
	YELLOW,
	PURPLE,
	CYAN,
	WHITE,
}COLOR_Type;

/*value of R,G,B channel*/
/*larger value leads to brighter light*/
typedef struct
{
	uint8_t R;
	uint8_t G;
	uint8_t B;
}RGB_Color;

typedef struct
{
	uint32_t pixel_buffer[LED_NUM + 1U][24];
	RGB_Color color;
	uint8_t direction;
	int brightness;
	BspIndicatorPort port;
	bool is_initialized;
} RgbServiceContext;

void RGB_SetColor(RgbServiceContext *context, uint8_t LedId,
	const RGB_Color *Color);
bool RGB_Initialize(RgbServiceContext *context, const BspIndicatorPort *port);
void Set_RGB_BreathingColor(RgbServiceContext *context,
	COLOR_Type color_type);

#endif
