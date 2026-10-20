/* G431 硬件边界：固定写域、ECC读取、安全GPIO及真实复位；不连接APP运行态。 */
#include "platform.h"
#include "board.h"
#include "flash_guard.h"
#include "can_port.h"
#include "loader_target.h"
#include "stm32g4xx_hal.h"
#include <string.h>

static uint32_t last_ticks, milliseconds, remainder;
static const LoaderFlashRegion image_domain = {TARGET_APP_BASE, TARGET_APP_BYTES};

static bool inside(const LoaderFlashRegion *region, uint32_t address, uint32_t bytes)
{
    return bytes != 0 && bytes <= region->bytes && address >= region->base &&
           address - region->base <= region->bytes - bytes;
}
static bool read_flash(void *context, uint32_t address, void *out, uint32_t bytes)
{
    return inside(context, address, bytes) && LoaderFlash_Read(address, out, bytes);
}
/* 寄存器级 Flash 编程：G431 单 bank、2 KiB 页、双字写入；不依赖 HAL Flash，省其链接开销。 */
#define FLASH_ERR_MASK                                                                             \
    (FLASH_SR_OPERR | FLASH_SR_PROGERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_PGSERR |     \
     FLASH_SR_SIZERR | FLASH_SR_MISERR | FLASH_SR_FASTERR | FLASH_SR_RDERR | FLASH_SR_OPTVERR)
#define FLASH_TIMEOUT_MS 1000U
#define FLASH_KEY1_VALUE 0x45670123U
#define FLASH_KEY2_VALUE 0xCDEF89ABU

static bool flash_unlock(void)
{
    if ((FLASH->CR & FLASH_CR_LOCK) != 0U)
    {
        FLASH->KEYR = FLASH_KEY1_VALUE;
        FLASH->KEYR = FLASH_KEY2_VALUE;
    }
    return (FLASH->CR & FLASH_CR_LOCK) == 0U;
}
static void flash_lock(void)
{
    FLASH->CR |= FLASH_CR_LOCK;
}
/* 等待操作完成并判错误位；完成后清 EOP 与全部错误标志。 */
static bool flash_ready(void)
{
    uint32_t started = LoaderPlatform_NowMs();
    while ((FLASH->SR & FLASH_SR_BSY) != 0U)
    {
        if ((uint32_t)(LoaderPlatform_NowMs() - started) >= FLASH_TIMEOUT_MS)
        {
            return false;
        }
    }
    bool ok = (FLASH->SR & FLASH_ERR_MASK) == 0U;
    FLASH->SR = FLASH_SR_EOP | FLASH_ERR_MASK | FLASH_SR_OPTVERR;
    return ok;
}
static bool flash_erase_page(uint32_t page)
{
    bool ok;
    if (!flash_unlock())
    {
        return false;
    }
    FLASH->SR = FLASH_SR_OPTVERR;
    if (!flash_ready())
    {
        flash_lock();
        return false;
    }
    FLASH->CR = (FLASH->CR & ~FLASH_CR_PNB_Msk) | FLASH_CR_PER | (page << FLASH_CR_PNB_Pos);
    FLASH->CR |= FLASH_CR_STRT;
    ok = flash_ready();
    FLASH->CR &= ~FLASH_CR_PER;
    flash_lock();
    return ok;
}
static bool flash_program_doubleword(uint32_t address, uint64_t value)
{
    uint32_t acr = FLASH->ACR;
    uint32_t key;
    bool ok;
    if (!flash_unlock())
    {
        return false;
    }
    FLASH->SR = FLASH_SR_OPTVERR;
    if (!flash_ready())
    {
        flash_lock();
        return false;
    }
    /* 两次32位写不可被普通中断分开；写前关缓存，写后复位并恢复。 */
    FLASH->ACR = acr & ~FLASH_ACR_DCEN;
    __DSB();
    key = __get_PRIMASK();
    __disable_irq();
    FLASH->CR |= FLASH_CR_PG;
    *(volatile uint32_t *)address = (uint32_t)value;
    __ISB();
    *(volatile uint32_t *)(address + 4U) = (uint32_t)(value >> 32U);
    __set_PRIMASK(key);
    ok = flash_ready();
    FLASH->CR &= ~FLASH_CR_PG;
    FLASH->ACR = (acr & ~FLASH_ACR_DCEN) | FLASH_ACR_DCRST;
    __DSB();
    FLASH->ACR = acr;
    __DSB();
    flash_lock();
    return ok;
}
static bool erase_flash(void *context, uint32_t address, uint32_t bytes)
{
    if (bytes != TARGET_ERASE_BYTES || address % TARGET_ERASE_BYTES != 0 ||
        !inside(context, address, bytes) || !LoaderPlatform_OutputsOff())
    {
        return false;
    }
    return flash_erase_page((address - TARGET_FLASH_BASE) / TARGET_ERASE_BYTES) &&
           LoaderFlash_EraseCompleted(address);
}
static bool program_flash(void *context, uint32_t address, const void *data, uint32_t bytes)
{
    uint64_t value, check;
    if (data == NULL || bytes != 8 || address % 8 != 0 || !inside(context, address, bytes) ||
        !LoaderPlatform_OutputsOff())
    {
        return false;
    }
    memcpy(&value, data, 8);
    return flash_program_doubleword(address, value) && LoaderFlash_Read(address, &check, 8) &&
           check == value;
}
static void bind_io(LoaderFlashIo *io, const LoaderFlashRegion *domain)
{
    io->context = (void *)domain;
    io->base = domain->base;
    io->bytes = domain->bytes;
    io->erase_bytes = TARGET_ERASE_BYTES;
    io->program_bytes = 8;
    io->read = read_flash;
    io->erase = erase_flash;
    io->program = program_flash;
}
void LoaderPlatform_ImageIo(LoaderFlashIo *io)
{
    if (io != NULL)
    {
        bind_io(io, &image_domain);
    }
}
void LoaderPlatform_ReadIdentity(uint8_t uid[12], DeviceIdentity *identity)
{
    uint8_t record[64];
    const uint32_t words[3] = {HAL_GetUIDw0(), HAL_GetUIDw1(), HAL_GetUIDw2()};
    for (uint32_t i = 0; i < 3U; ++i)
    {
        memcpy(uid + i * 4U, words + i, 4U);
    }
    memset(identity, 0, sizeof(*identity));
    if (LoaderFlash_Read(TARGET_IDENTITY_BASE, record, sizeof(record)))
    {
        (void)DeviceIdentity_Decode(record, identity);
    }
}
void SysTick_Handler(void)
{
    HAL_IncTick();
}
void LoaderPlatform_Watchdog(void)
{
    IWDG->KR = 0xAAAAU;
}
bool LoaderPlatform_OutputsOff(void)
{
    return LoaderBoard_OutputsOff();
}
void LoaderPlatform_Fatal(void)
{
    LoaderBoard_Safe();
    __disable_irq();
    for (;;)
    { /* 未确认安全恢复条件，不自动恢复升级或功率输出。 */
    }
}
bool LoaderPlatform_Init(bool *software_reset, bool *watchdog_reset)
{
    uint32_t cause = RCC->CSR, remaining = 4000000U;
    if (software_reset == NULL || watchdog_reset == NULL)
    {
        return false;
    }
    *software_reset = (cause & RCC_CSR_SFTRSTF) != 0 &&
                      !(cause & (RCC_CSR_BORRSTF | RCC_CSR_IWDGRSTF | RCC_CSR_WWDGRSTF |
                                 RCC_CSR_OBLRSTF | RCC_CSR_LPWRRSTF));
    *watchdog_reset = (cause & (RCC_CSR_IWDGRSTF | RCC_CSR_WWDGRSTF)) != 0 &&
                      !(cause & (RCC_CSR_BORRSTF | RCC_CSR_OBLRSTF | RCC_CSR_LPWRRSTF));
    LoaderBoard_Safe();
    if ((DBGMCU->IDCODE & 0xFFFU) != 0x468U || *(const uint16_t *)FLASHSIZE_BASE != 128U)
    {
        return false;
    }
    HAL_Init();
    SystemClock_Config();
    IWDG->KR = 0xCCCCU;
    IWDG->KR = 0x5555U;
    IWDG->PR = 6;
    IWDG->RLR = 4095;
    while (IWDG->SR != 0)
    {
        if (--remaining == 0)
        {
            LoaderPlatform_Fatal();
        }
    }
    LoaderPlatform_Watchdog();
    __HAL_RCC_CLEAR_RESET_FLAGS();
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
    (void)RCC->APB1ENR1;
    TIM2->CR1 = 0;
    TIM2->DIER = 0;
    TIM2->PSC = 16999;
    TIM2->ARR = UINT32_MAX;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->SR = 0;
    TIM2->CNT = 0;
    last_ticks = milliseconds = remainder = 0;
    TIM2->CR1 = TIM_CR1_CEN;
    return LoaderPlatform_OutputsOff();
}
uint32_t LoaderPlatform_NowMs(void)
{
    uint32_t ticks = TIM2->CNT, delta = ticks - last_ticks;
    last_ticks = ticks;
    remainder += delta % 10U;
    milliseconds += delta / 10U + remainder / 10U;
    remainder %= 10U;
    return milliseconds;
}
uint32_t LoaderPlatform_Lock(void)
{
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    __DMB();
    return key;
}
void LoaderPlatform_Unlock(uint32_t key)
{
    __DMB();
    __set_PRIMASK(key);
}
void LoaderPlatform_Reset(void)
{
    NVIC_SystemReset();
}
bool LoaderPlatform_ReadParameters(uint32_t offset, void *destination, uint32_t bytes)
{
    const uint32_t capacity = TARGET_IDENTITY_BASE - TARGET_PARAM_BASE;
    return destination != NULL && bytes != 0U && bytes <= capacity && offset <= capacity - bytes &&
           LoaderFlash_Read(TARGET_PARAM_BASE + offset, destination, bytes);
}
bool LoaderPlatform_TakeMailbox(BootMailboxMessage *message)
{
    uint8_t raw[YGIM_BYTES];
    volatile uint8_t *mailbox = (volatile uint8_t *)TARGET_MAILBOX_ADDRESS;
    unsigned i;
    for (i = 0; i < sizeof(raw); ++i)
    {
        raw[i] = mailbox[i];
        mailbox[i] = 0;
    }
    __DMB();
    return BootMailbox_Decode(raw, message);
}
void LoaderPlatform_NmiDispatch(uint32_t *stack, uint32_t exc_return)
{
    if (!LoaderFlash_HandleNmi(stack, exc_return))
    {
        LoaderPlatform_Fatal();
    }
}
/* ARMCC汇编的行边界是语法，不交给C格式化器合并。 */
// clang-format off
__asm void NMI_Handler(void)
{
    IMPORT LoaderPlatform_NmiDispatch
    TST lr,#4
    ITE EQ
    MRSEQ r0,MSP
    MRSNE r0,PSP
    MOV r1,lr
    B LoaderPlatform_NmiDispatch
}
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
    uint32_t vectors[2];
    unsigned i;
    if (!LoaderPlatform_OutputsOff() || !LoaderFlash_Read(TARGET_APP_BASE, vectors, 8) ||
        vectors[0] <= TARGET_RAM_BASE || vectors[0] > TARGET_RAM_END || (vectors[0] & 7U) ||
        !(vectors[1] & 1U) ||
        (vectors[1] & ~1U) < TARGET_APP_BASE + TARGET_DESCRIPTOR_OFFSET + YGI_DESCRIPTOR_BYTES ||
        (vectors[1] & ~1U) >= TARGET_APP_BASE + TARGET_APP_BYTES)
    {
        LoaderPlatform_Fatal();
    }
    LoaderCan_Shutdown();
    /* APP CubeMX启动按复位时钟配置PLL；切回HSI且关闭PLL后再交接。 */
    if (HAL_RCC_DeInit() != HAL_OK)
    {
        LoaderPlatform_Fatal();
    }
    __disable_irq();
    TIM2->CR1 = 0;
    RCC->APB1RSTR1 |= RCC_APB1RSTR1_TIM2RST;
    RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_TIM2RST;
    RCC->APB1ENR1 &= ~RCC_APB1ENR1_TIM2EN;
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;
    for (i = 0; i < 8; ++i)
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
