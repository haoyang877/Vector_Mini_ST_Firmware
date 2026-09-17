
/**********************************************************/
//Delay function is transplanted from Alientek (Zheng Dian Yuan Zi)
/**********************************************************/

#include "delay.h"

static uint32_t g_fac_us = 0;

/**
	* @brief  Initialize delay time base
	* @param  sysclk: system clock frequency in MHz
 **/
void delay_init(uint16_t sysclk)
{
    g_fac_us = sysclk;
}

/**
	* @brief  Blocking delay in microseconds
	* @param  nus: delay time in microseconds
 **/
void delay_us(uint32_t nus)
{
    uint32_t ticks;
    uint32_t told, tnow, tcnt = 0;
    uint32_t reload = SysTick->LOAD;
    ticks = nus * g_fac_us; 
    
    told = SysTick->VAL;                   
    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            if (tnow < told)
            {
                tcnt += told - tnow;
            }
            else
            {
                tcnt += reload - tnow + told;
            }
            told = tnow;
            if (tcnt >= ticks) 
            {
                break;
            }
        }
    }
}

/**
	* @brief  Blocking delay in milliseconds
	* @param  nms: delay time in milliseconds
 **/
void delay_ms(uint16_t nms)
{
    delay_us((uint32_t)(nms * 1000));
}


