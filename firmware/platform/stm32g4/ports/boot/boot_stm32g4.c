/* APP仅通过此端口访问交接邮箱、UID和复位寄存器。 */
#include "boot_contract.h"
#include "boot_hw.h"
#include "can_deployment.h"
#include "stm32g4xx.h"
#include <string.h>

void boot_hw_early_init(void)
{
    SCB->VTOR = BOOT_APP_BASE;
    __DSB();
    __ISB();
}

void boot_hw_uid(uint8_t out[12])
{
    memcpy(out, (const void *)UID_BASE, 12);
}

void boot_hw_reset_to_loader(uint32_t node, uint32_t token)
{
#if VECTOR_CAN_CLASSIC_ONLY
    (void)node; (void)token;
#else

    uint8_t raw[BOOT_MAILBOX_BYTES];
    volatile uint8_t *mail = (volatile uint8_t *)BOOT_MAILBOX_BASE;
    unsigned i;
    Boot_MailboxEncode(node, token, raw);
    *(volatile uint32_t *)(BOOT_MAILBOX_BASE + 28) = 0;
    for (i = 0; i < 28; ++i)
    {
        mail[i] = raw[i];
    }
    __DSB();
    *(volatile uint32_t *)(BOOT_MAILBOX_BASE + 28) = 0x54494D43UL;
    __DSB();
    NVIC_SystemReset();
#endif
}

uint32_t boot_hw_parameters_crc(void)
{
    return Boot_Crc32((const uint8_t *)BOOT_PARAM_BASE, BOOT_PARAM_CAPACITY);
}
