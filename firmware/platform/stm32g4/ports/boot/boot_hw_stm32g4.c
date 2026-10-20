/* APP侧窄启动端口：只识别Loader启动、写ENTER邮箱并复位、喂继承看门狗。 */
#include "boot_hw.h"
#include "image_mailbox.h"
#include "image_format.h"
#include "loader_target.h"
#include "stm32g4xx_hal.h"
#include <string.h>
extern const uint32_t __Vectors;
static bool available;
static uint32_t trial_started_ms;
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
    if (available)
    {
        return true;
    }
    key = __get_PRIMASK();
    __disable_irq();
    SCB->VTOR = TARGET_APP_BASE;
    __DSB();
    __ISB();
    __set_PRIMASK(key);
    available = true;
    trial_started_ms = HAL_GetTick();
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
    if (healthy && available &&
        (BootHw_IsConfirmed() || (uint32_t)(HAL_GetTick() - trial_started_ms) < 5000U))
    {
        IWDG->KR = 0xAAAAU;
    }
}
bool BootHw_IsConfirmed(void)
{
    return available && marker_valid(YGI_ACTIVATION_OFFSET, YGI_ACTIVATION_MAGIC) &&
           marker_valid(YGI_ATTEMPT_OFFSET, YGI_ATTEMPT_MAGIC) &&
           marker_valid(YGI_CONFIRM_OFFSET, YGI_CONFIRM_MAGIC);
}
uint32_t BootHw_ImageCrc32(void)
{
    return available
               ? read_be32((const volatile uint8_t *)(TARGET_APP_BASE + TARGET_DESCRIPTOR_OFFSET +
                                                      YGI_DESCRIPTOR_CRC_OFFSET))
               : 0U;
}
bool BootHw_Confirm(uint32_t version, uint32_t crc32)
{
    const volatile uint8_t *descriptor =
        (const volatile uint8_t *)(TARGET_APP_BASE + TARGET_DESCRIPTOR_OFFSET);
    uint32_t address = TARGET_APP_BASE + TARGET_DESCRIPTOR_OFFSET + YGI_CONFIRM_OFFSET;
    uint32_t key, acr;
    uint8_t raw[8];
    bool ok;
    if (!available || !outputs_off() || read_be32(descriptor + 28U) != version ||
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
    if ((uint32_t)(HAL_GetTick() - trial_started_ms) >= 5000U ||
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
    acr = FLASH->ACR;
    /* 单bank停顿由继承IWDG兜底；不用HAL的中断毫秒超时，屏蔽IRQ后它不再前进。
     * 只编程当前确认双字，不链接整套HAL Flash控制路径；失败不原位补写。 */
    FLASH->ACR = acr & ~FLASH_ACR_DCEN;
    const uint32_t errors = FLASH_SR_OPERR | FLASH_SR_PROGERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR |
                            FLASH_SR_PGSERR | FLASH_SR_SIZERR | FLASH_SR_MISERR | FLASH_SR_FASTERR |
                            FLASH_SR_RDERR | FLASH_SR_OPTVERR;
    ok = outputs_off() && !(FLASH->SR & (FLASH_SR_BSY | errors));
    if (ok && (FLASH->CR & FLASH_CR_LOCK) != 0U)
    {
        FLASH->KEYR = 0x45670123U;
        FLASH->KEYR = 0xCDEF89ABU;
    }
    ok = ok && !(FLASH->CR & FLASH_CR_LOCK);
    if (ok)
    {
        uint32_t words[2], remaining = 1000000U;
        memcpy(words, raw, sizeof(words));
        FLASH->CR |= FLASH_CR_PG;
        *(volatile uint32_t *)(uintptr_t)address = words[0];
        __ISB();
        *(volatile uint32_t *)(uintptr_t)(address + 4U) = words[1];
        while ((FLASH->SR & FLASH_SR_BSY) != 0U && remaining != 0U)
        {
            --remaining;
        }
        ok = remaining != 0U && !(FLASH->SR & errors);
        FLASH->CR &= ~FLASH_CR_PG;
        FLASH->SR = FLASH_SR_EOP | errors;
    }
    FLASH->CR |= FLASH_CR_LOCK;
    FLASH->ACR = (acr & ~FLASH_ACR_DCEN) | FLASH_ACR_DCRST;
    __DSB();
    FLASH->ACR = acr;
    __DSB();
    ok = ok && BootHw_IsConfirmed();
    __set_PRIMASK(key);
    return ok;
}
