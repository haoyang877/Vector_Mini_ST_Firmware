/* AT32F456独立启动/Flash端口；写入仅限96KiB APP，参数与身份只有读取能力。 */
#include "platform.h"
#include "can_port.h"
#include "board.h"
#include "loader_target.h"
#include "at32f45x.h"
#include "at32f45x_debug.h"
#include "at32f45x_wdt.h"
#include <string.h>

static const LoaderFlashRegion s_app = {TARGET_APP_BASE, TARGET_APP_BYTES};
static bool s_flash_failed;
static uint16_t s_last_ticks;
static uint32_t s_milliseconds, s_remainder;

static bool inside(uint32_t address, uint32_t bytes, const LoaderFlashRegion *region)
{
    return bytes != 0U && bytes <= region->bytes && address >= region->base &&
           address - region->base <= region->bytes - bytes;
}
static bool flash_ready(bool completed)
{
    uint32_t start = LoaderPlatform_NowMs();
    while (FLASH->sts & FLASH_OBF_FLAG)
    {
        if ((uint32_t)(LoaderPlatform_NowMs() - start) >= 1000U)
        {
            s_flash_failed = true;
            return false;
        }
    }
    bool ok = !s_flash_failed && !(FLASH->sts & (FLASH_PRGMERR_FLAG | FLASH_EPPERR_FLAG)) &&
              (!completed || (FLASH->sts & FLASH_ODF_FLAG));
    FLASH->sts = FLASH_ODF_FLAG | FLASH_PRGMERR_FLAG | FLASH_EPPERR_FLAG;
    if (!ok)
    {
        s_flash_failed = true;
    }
    return ok;
}
static bool read_flash(void *context, uint32_t address, void *data, uint32_t bytes)
{
    if (data == NULL || s_flash_failed || !inside(address, bytes, context) ||
        (FLASH->sts & FLASH_OBF_FLAG))
    {
        return false;
    }
    memcpy(data, (const void *)(uintptr_t)address, bytes);
    return true;
}
static void lock_flash(void)
{
    if (!(FLASH->sts & FLASH_OBF_FLAG))
    {
        flash_lock();
    }
}
static bool erase_flash(void *context, uint32_t address, uint32_t bytes)
{
    if (bytes != TARGET_ERASE_BYTES || address % bytes || !inside(address, bytes, context) ||
        !LoaderPlatform_OutputsOff() || !flash_ready(false))
    {
        return false;
    }
    flash_unlock();
    if (FLASH->ctrl_bit.oplk)
    {
        s_flash_failed = true;
        return false;
    }
    FLASH->ctrl_bit.secers = TRUE;
    FLASH->addr = address;
    FLASH->ctrl_bit.erstr = TRUE;
    bool ok = flash_ready(true);
    if (!(FLASH->sts & FLASH_OBF_FLAG))
    {
        FLASH->ctrl_bit.secers = FALSE;
    }
    lock_flash();
    const volatile uint32_t *words = (const volatile uint32_t *)(uintptr_t)address;
    for (uint32_t i = 0; ok && i < bytes / 4U; ++i)
    {
        ok = words[i] == UINT32_MAX;
    }
    s_flash_failed = s_flash_failed || !ok;
    return ok;
}
static bool program_flash(void *context, uint32_t address, const void *data, uint32_t bytes)
{
    uint32_t words[2];
    if (data == NULL || bytes != 8U || address % 8U || !inside(address, bytes, context) ||
        !LoaderPlatform_OutputsOff() || !flash_ready(false))
    {
        return false;
    }
    volatile uint32_t *destination = (volatile uint32_t *)(uintptr_t)address;
    if (destination[0] != UINT32_MAX || destination[1] != UINT32_MAX)
    {
        return false;
    }
    memcpy(words, data, sizeof(words));
    flash_unlock();
    bool ok = !FLASH->ctrl_bit.oplk;
    for (uint32_t i = 0; ok && i < 2U; ++i)
    {
        if (words[i] != UINT32_MAX)
        {
            FLASH->ctrl_bit.fprgm = TRUE;
            destination[i] = words[i];
            ok = flash_ready(true);
            if (!(FLASH->sts & FLASH_OBF_FLAG))
            {
                FLASH->ctrl_bit.fprgm = FALSE;
            }
        }
        ok = ok && destination[i] == words[i];
    }
    lock_flash();
    s_flash_failed = s_flash_failed || !ok;
    return ok;
}
void LoaderPlatform_ImageIo(LoaderFlashIo *io)
{
    *io = (LoaderFlashIo){(void *)&s_app,
                          TARGET_APP_BASE,
                          TARGET_APP_BYTES,
                          TARGET_ERASE_BYTES,
                          8U,
                          read_flash,
                          erase_flash,
                          program_flash};
}
void LoaderPlatform_ReadIdentity(uint8_t uid[12], DeviceIdentity *identity)
{
    memcpy(uid, (const void *)0x1FFFF7E8U, 12U);
    memset(identity, 0, sizeof(*identity));
    (void)DeviceIdentity_Decode((const uint8_t *)TARGET_IDENTITY_BASE, identity);
}
bool LoaderPlatform_ReadParameters(uint32_t offset, void *data, uint32_t bytes)
{
    const LoaderFlashRegion parameters = {TARGET_PARAM_BASE,
                                          TARGET_IDENTITY_BASE - TARGET_PARAM_BASE};
    return offset <= parameters.bytes &&
           read_flash((void *)&parameters, parameters.base + offset, data, bytes);
}
bool LoaderPlatform_OutputsOff(void)
{
    return LoaderBoard_OutputsOff();
}
void LoaderPlatform_Watchdog(void)
{
    WDT->cmd = 0xAAAAU;
}
void LoaderPlatform_Fatal(void)
{
    LoaderBoard_Safe();
    __disable_irq();
    for (;;)
    {
        /* 不喂独立看门狗；复位后保持救援，不恢复运动。 */
    }
}
bool LoaderPlatform_Init(bool *software_reset, bool *watchdog_reset)
{
    uint32_t cause = CRM->ctrlsts;
    if (software_reset == NULL || watchdog_reset == NULL)
    {
        return false;
    }
    *software_reset = (cause & (1U << 28U)) && !(cause & 0xE8000000U);
    *watchdog_reset = (cause & 0x60000000U) && !(cause & 0x88000000U);
    if (*(const uint16_t *)0x1FFFF7E0U != 256U || !LoaderBoard_Init())
    {
        return false;
    }
    crm_periph_clock_enable(CRM_TMR2_PERIPH_CLOCK, TRUE);
    tmr_reset(TMR2);
    tmr_base_init(TMR2, UINT16_MAX, LOADER_AT32_CLOCK_HZ / 10000U - 1U);
    tmr_counter_enable(TMR2, TRUE);
    crm_clock_source_enable(CRM_CLOCK_SOURCE_LICK, TRUE);
    uint32_t remaining = 2000000U;
    while (crm_flag_get(CRM_LICK_STABLE_FLAG) == RESET && --remaining)
    {
    }
    if (!remaining)
    {
        return false;
    }
    DEBUGMCU->apb1_frz &= ~(1U << 12U);
    WDT->cmd = 0x5555U;
    WDT->div = 6U;
    WDT->rld = 4095U;
    WDT->win = 4095U;
    remaining = 2000000U;
    while (WDT->sts && --remaining)
    {
    }
    if (!remaining)
    {
        return false;
    }
    LoaderPlatform_Watchdog();
    WDT->cmd = 0xCCCCU;
    crm_flag_clear(CRM_ALL_RESET_FLAG);
    return LoaderBoard_OutputsOff();
}
uint32_t LoaderPlatform_NowMs(void)
{
    /* TMR2 的 10 kHz 计数器每 65536 拍回绕，前台轮询间隔短于一圈。 */
    uint16_t now = (uint16_t)TMR2->cval;
    uint16_t delta = (uint16_t)(now - s_last_ticks);
    s_last_ticks = now;
    s_remainder += delta % 10U;
    s_milliseconds += delta / 10U + s_remainder / 10U;
    s_remainder %= 10U;
    return s_milliseconds;
}
uint32_t LoaderPlatform_Lock(void)
{
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    return key;
}
void LoaderPlatform_Unlock(uint32_t key)
{
    __set_PRIMASK(key);
}
void LoaderPlatform_Reset(void)
{
    NVIC_SystemReset();
}
bool LoaderPlatform_TakeMailbox(BootMailboxMessage *message)
{
    uint8_t raw[YGIM_BYTES];
    volatile uint8_t *mailbox = (volatile uint8_t *)TARGET_MAILBOX_ADDRESS;
    for (uint32_t i = 0; i < YGIM_BYTES; ++i)
    {
        raw[i] = mailbox[i];
        mailbox[i] = 0;
    }
    __DMB();
    return BootMailbox_Decode(raw, message);
}
// clang-format off
__asm static void tail_jump(uint32_t sp, uint32_t entry)
{
    MOVS r2,#0
    MSR CONTROL,r2
    MSR BASEPRI,r2
    MSR FAULTMASK,r2
    MSR MSP,r0
    DSB
    ISB
    CPSIE i
    BX r1
}
// clang-format on
void LoaderPlatform_Jump(void)
{
    const uint32_t *vectors = (const uint32_t *)TARGET_APP_BASE;
    if (!LoaderPlatform_OutputsOff() || vectors[0] <= TARGET_RAM_BASE ||
        vectors[0] > TARGET_RAM_END || vectors[0] % 8U || !(vectors[1] & 1U) ||
        (vectors[1] & ~1U) < TARGET_APP_BASE + TARGET_DESCRIPTOR_OFFSET + YGI_DESCRIPTOR_BYTES ||
        (vectors[1] & ~1U) >= TARGET_APP_BASE + TARGET_APP_BYTES)
    {
        LoaderPlatform_Fatal();
    }
    LoaderCan_Shutdown();
    __disable_irq();
    tmr_counter_enable(TMR2, FALSE);
    SysTick->CTRL = 0U;
    for (uint32_t i = 0; i < 8U; ++i)
    {
        NVIC->ICER[i] = UINT32_MAX;
        NVIC->ICPR[i] = UINT32_MAX;
    }
    SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk | SCB_ICSR_PENDSVCLR_Msk;
    SCB->VTOR = TARGET_APP_BASE;
    __DSB();
    __ISB();
    tail_jump(vectors[0], vectors[1]);
    LoaderPlatform_Fatal();
}
void NMI_Handler(void)
{
    LoaderPlatform_Fatal();
}
void HardFault_Handler(void)
{
    LoaderPlatform_Fatal();
}
