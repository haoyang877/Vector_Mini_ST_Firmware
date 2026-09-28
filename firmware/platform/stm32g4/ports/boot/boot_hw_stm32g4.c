/* APP侧窄启动端口：只识别Loader启动、写ENTER邮箱并复位、喂继承看门狗。 */
#include "boot_hw.h"
#include "image_mailbox.h"
#include "vector_mini_st_g431.h"
#include "stm32g4xx_hal.h"
#include <string.h>
extern const uint32_t __Vectors;
static bool available;
static bool outputs_off(void)
{
    const uint32_t mask = TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE |
                          TIM_CCER_CC3E | TIM_CCER_CC3NE;
    return !(TIM1->CCER & mask) && !(GPIOA->IDR & 0x780U) && !(GPIOB->IDR & 3U);
}
bool BootHw_Init(void)
{
    uint32_t key;
    if ((uint32_t)&__Vectors != TARGET_APP_BASE || !outputs_off())
    {
        return false;
    }
    key = __get_PRIMASK();
    __disable_irq();
    SCB->VTOR = TARGET_APP_BASE;
    __DSB();
    __ISB();
    __set_PRIMASK(key);
    available = true;
    return true;
}
bool BootHw_EnterLoader(uint8_t node)
{
    BootMailboxMessage message = {YG_BOOT_ENTER, 0};
    uint8_t raw[YGIM_BYTES];
    volatile uint8_t *mailbox = (volatile uint8_t *)TARGET_MAILBOX_ADDRESS;
    uint32_t key, i;
    if (!available || node == 255)
    {
        return false;
    }
    message.node = node;
    if (!BootMailbox_Encode(&message, raw))
    {
        return false;
    }
    key = __get_PRIMASK();
    __disable_irq();
    if (!outputs_off())
    {
        __set_PRIMASK(key);
        return false;
    }
    *(volatile uint32_t *)mailbox = 0;
    __DMB();
    for (i = 4; i < sizeof(raw); ++i)
    {
        mailbox[i] = raw[i];
    }
    __DMB();
    *(volatile uint32_t *)mailbox = YGIM_MAGIC;
    __DSB();
    for (i = 0; i < sizeof(raw); ++i)
    {
        if (mailbox[i] != raw[i])
        {
            __set_PRIMASK(key);
            return false;
        }
    }
    NVIC_SystemReset();
    __set_PRIMASK(key);
    return false;
}
void BootHw_Watchdog(bool healthy)
{
    if (healthy && available)
    {
        IWDG->KR = 0xAAAAU;
    }
}
