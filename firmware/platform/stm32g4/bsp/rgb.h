#ifndef __RGB_H__
#define __RGB_H__

#include "main.h"

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

void RGB_SetColor(uint8_t LedId, RGB_Color *Color);
void Reset_Load(void);
void RGB_SendArray(void);
void write_color(uint16_t led_num,RGB_Color *color);
void Set_RGB_BreathingColor(COLOR_Type color_type);

#endif