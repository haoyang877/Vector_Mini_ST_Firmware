/* 板级忙等待延时；时基由系统时钟初始化后提供，调用期间占用 CPU。 */

/**********************************************************/
//Delay function is transplanted from Alientek (Zheng Dian Yuan Zi)
/**********************************************************/

#include "delay.h"

static uint32_t s_cycles_per_us = 0;

/**
    * @brief  Initialize delay time base
    * @param  sysclk: system clock frequency in MHz
 **/
void delay_init(uint16_t sysclk)
{
    s_cycles_per_us = sysclk;
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
    ticks = nus * s_cycles_per_us;

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
