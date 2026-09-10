#include "../../../api/motor_hw.h"
#include "stm32g4xx.h"

void motor_hw_outer_init(void)
{
    /* ADC = 0; this worker = 1 (serialized with CAN); USB = 2; TIM7 = 3.
     * This bare-metal target has no RTOS owner of PendSV. */
    NVIC_SetPriority(PendSV_IRQn, 1U);
    SCB->ICSR = SCB_ICSR_PENDSVCLR_Msk;
}

void motor_hw_outer_schedule(void)
{
    __DMB();
    SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
}

void motor_hw_outer_barrier(void)
{
    __DMB();
}
