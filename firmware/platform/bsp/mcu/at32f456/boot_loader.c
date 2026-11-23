/* APP侧窄启动端口：只识别Loader启动、写ENTER邮箱并复位、喂继承看门狗。 */
#include "boot_hw.h"
#include "../../bsp_internal.h"
#include "image_mailbox.h"
#include "image_format.h"
#include "loader_target.h"
#include "at32f45x.h"
#include "mcu_resources.h"
#include "bsp_system.h"
#include "at32f45x_wdt.h"
#include <string.h>
extern const uint32_t __Vectors;
static bool s_available;
static uint32_t s_trial_started_ms;
static uint32_t read_be32(const volatile uint8_t *p)
{
    return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | p[3];
}
static bool marker_valid(uint32_t offset, uint32_t magic)
{
    const volatile uint8_t *p =
        (const volatile uint8_t *)(uintptr_t)(TARGET_APP_BASE + TARGET_DESCRIPTOR_OFFSET + offset);
    return read_be32(p) == magic && read_be32(p + 4) == ~magic;
}
static bool outputs_off(void)
{
    return !(TMR1->cctrl & 0x5550U) && !(TMR1->brk & 0x8000U) && !(GPIOA->idt & 0xE00U) &&
           !(GPIOB->idt & 0x13U);
}
bool BootHw_Init(void)
{
    uint32_t key;
    if ((uint32_t)&__Vectors != TARGET_APP_BASE || !outputs_off())
    {
        return false;
    }
    if (s_available)
    {
        return true;
    }
    key = __get_PRIMASK();
    __disable_irq();
    SCB->VTOR = TARGET_APP_BASE;
    __DSB();
    __ISB();
    __set_PRIMASK(key);
    s_available = true;
    s_trial_started_ms = BspSystem_TimeMs();
    return true;
}
bool BootHw_EnterLoader(uint8_t node, uint32_t started_ms, uint32_t timeout_ms)
{
    BootMailboxMessage message = {YG_BOOT_ENTER, 0};
    uint8_t raw[YGIM_BYTES];
    volatile uint8_t *mailbox = (volatile uint8_t *)TARGET_MAILBOX_ADDRESS;
    uint32_t i;
    if (!s_available || node == 255)
    {
        return false;
    }
    message.node = node;
    if (!BootMailbox_Encode(&message, raw))
    {
        return false;
    }
    if ((uint32_t)(BspSystem_TimeMs() - started_ms) >= timeout_ms || !outputs_off())
    {
        return false;
    }
    *(volatile uint32_t *)mailbox = 0;
    __DMB();
    for (i = 4; i < sizeof(raw); ++i)
    {
        mailbox[i] = raw[i];
    }
    __DMB();
    for (i = 4; i < sizeof(raw); ++i)
    {
        if (mailbox[i] != raw[i])
        {
            return false;
        }
    }
    /* 启动锁保持关相。未到期才发布有效字；失败或过期撤销邮箱。 */
    if ((uint32_t)(BspSystem_TimeMs() - started_ms) >= timeout_ms || !outputs_off())
    {
        return false;
    }
    *(volatile uint32_t *)mailbox = YGIM_MAGIC;
    __DSB();
    if (*(volatile uint32_t *)mailbox == YGIM_MAGIC && outputs_off() &&
        (uint32_t)(BspSystem_TimeMs() - started_ms) < timeout_ms)
    {
        NVIC_SystemReset();
    }
    *(volatile uint32_t *)mailbox = 0U;
    __DSB();
    return false;
}
void BootHw_Watchdog(bool healthy)
{
    if (healthy && s_available &&
        (BootHw_IsConfirmed() || (uint32_t)(BspSystem_TimeMs() - s_trial_started_ms) < 5000U))
    {
        WDT->cmd = 0xAAAAU;
    }
}
bool BootHw_IsConfirmed(void)
{
    return s_available && marker_valid(YGI_ACTIVATION_OFFSET, YGI_ACTIVATION_MAGIC) &&
           marker_valid(YGI_ATTEMPT_OFFSET, YGI_ATTEMPT_MAGIC) &&
           marker_valid(YGI_CONFIRM_OFFSET, YGI_CONFIRM_MAGIC);
}
uint32_t BootHw_ImageCrc32(void)
{
    return s_available
               ? read_be32((const volatile uint8_t *)(TARGET_APP_BASE + TARGET_DESCRIPTOR_OFFSET +
                                                      YGI_DESCRIPTOR_CRC_OFFSET))
               : 0U;
}
bool BootHw_Confirm(uint32_t version, uint32_t crc32)
{
    const volatile uint8_t *descriptor =
        (const volatile uint8_t *)(TARGET_APP_BASE + TARGET_DESCRIPTOR_OFFSET);
    uint32_t address = TARGET_APP_BASE + TARGET_DESCRIPTOR_OFFSET + YGI_CONFIRM_OFFSET;
    uint32_t key;
    uint8_t raw[8];
    bool ok;
    if (!s_available || !outputs_off() || read_be32(descriptor + 28U) != version ||
        read_be32(descriptor + YGI_DESCRIPTOR_CRC_OFFSET) != crc32 ||
        !marker_valid(YGI_ACTIVATION_OFFSET, YGI_ACTIVATION_MAGIC) ||
        !marker_valid(YGI_ATTEMPT_OFFSET, YGI_ATTEMPT_MAGIC))
    {
        return false;
    }
    if (BootHw_IsConfirmed())
    {
        return true;
    }
    if ((uint32_t)(BspSystem_TimeMs() - s_trial_started_ms) >= 5000U ||
        read_be32((const volatile uint8_t *)(uintptr_t)address) != UINT32_MAX ||
        read_be32((const volatile uint8_t *)(uintptr_t)(address + 4U)) != UINT32_MAX)
    {
        return false;
    }
    for (uint32_t i = 0U; i < 4U; ++i)
    {
        raw[i] = (uint8_t)(YGI_CONFIRM_MAGIC >> (24U - i * 8U));
        raw[i + 4U] = (uint8_t)(~YGI_CONFIRM_MAGIC >> (24U - i * 8U));
    }
    key = __get_PRIMASK();
    __disable_irq();
    /* 同一8B标记拆为两次32位编程；任何半写都不再补写。 */
    uint32_t words[2];
    memcpy(words, raw, sizeof(words));
    const uint32_t errors = FLASH_PRGMERR_FLAG | FLASH_EPPERR_FLAG;
    ok = outputs_off() && !(FLASH->sts & (FLASH_OBF_FLAG | errors));
    if (ok)
    {
        flash_unlock();
        ok = !FLASH->ctrl_bit.oplk;
    }
    for (uint32_t i = 0U; ok && i < 2U; ++i)
    {
        uint32_t remaining = 1000000U;
        FLASH->ctrl_bit.fprgm = TRUE;
        *(volatile uint32_t *)(uintptr_t)(address + i * 4U) = words[i];
        while ((FLASH->sts & FLASH_OBF_FLAG) && --remaining)
        {
        }
        ok = remaining && !(FLASH->sts & errors);
        if (!(FLASH->sts & FLASH_OBF_FLAG))
        {
            FLASH->ctrl_bit.fprgm = FALSE;
            FLASH->sts = FLASH_ODF_FLAG | errors;
        }
    }
    if (!(FLASH->sts & FLASH_OBF_FLAG))
    {
        flash_lock();
    }
    ok = ok && BootHw_IsConfirmed();
    __set_PRIMASK(key);
    return ok;
}

bool BootHw_IsAvailable(void)
{
    return s_available;
}
