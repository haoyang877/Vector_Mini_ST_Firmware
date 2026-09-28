#ifndef __LED_H__
#define __LED_H__

#include "main.h"
#include <stdbool.h>

/*LED switch macro 0:off 1:on*/
#define LED_R(a)	if(a) \
							LED_R_GPIO_Port->BSRR = (uint32_t)LED_R_Pin << 16U; \
					else  \
							LED_R_GPIO_Port->BSRR = LED_R_Pin;
#define LED_G(a)	if(a) \
							LED_G_GPIO_Port->BSRR = (uint32_t)LED_G_Pin << 16U; \
					else  \
							LED_G_GPIO_Port->BSRR = LED_G_Pin;

#define TEST(a)	if(a) \
							TEST_GPIO_Port->BSRR = (uint32_t)TEST_Pin << 16U; \
					else  \
							TEST_GPIO_Port->BSRR = TEST_Pin;
					
/*LED toggle macro*/
#define LED_R_TOGGLE 	{LED_R_GPIO_Port->ODR ^= LED_R_Pin;}	
#define LED_G_TOGGLE 	{LED_G_GPIO_Port->ODR ^= LED_G_Pin;}	

typedef struct
{
	bool mode_or_error;
	bool mode_or_error_last;
	bool on_or_off[2];
	uint8_t blink_num;
	uint8_t blink_num_last;
	uint8_t cnt;
}LED_TypeDef;

void LED_SetState(bool mode_or_error,uint8_t blink_num);
void LED_Task(void);
#endif