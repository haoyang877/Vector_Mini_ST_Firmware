#include "critical_hw.h"

#include "main.h"

/* 临界区端口：唯一直接操作中断屏蔽寄存器的实现；芯片级细节留在 bsp 层。 */

uint32_t critical_hw_enter(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    return primask;
}

void critical_hw_exit(uint32_t state)
{
    __set_PRIMASK(state);
}
