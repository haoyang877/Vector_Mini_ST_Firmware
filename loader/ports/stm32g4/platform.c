/* G431 硬件边界：固定写域、ECC读取、安全GPIO及真实复位；不连接APP运行态。 */
#include "platform.h"
#include "board.h"
#include "flash_guard.h"
#include "can_port.h"
#include "vector_mini_st_g431.h"
#include "stm32g4xx_hal.h"
#include <string.h>

static uint32_t last_ticks, milliseconds, remainder;
static const LoaderFlashRegion image_domain = {TARGET_RECORD_BASE,
                                               TARGET_ERASE_BYTES + TARGET_APP_BYTES};
static const LoaderFlashRegion boot_domain = {TARGET_BOOT_BASE, TARGET_BOOT_BYTES};

static bool inside(const LoaderFlashRegion *region, uint32_t address, uint32_t bytes)
{
    return bytes != 0 && bytes <= region->bytes && address >= region->base &&
           address - region->base <= region->bytes - bytes;
}
static bool read_flash(void *context, uint32_t address, void *out, uint32_t bytes)
{
    return inside(context, address, bytes) && LoaderFlash_Read(address, out, bytes);
}
static bool erase_flash(void *context, uint32_t address, uint32_t bytes)
{
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t error;
    bool result;
    if (bytes != TARGET_ERASE_BYTES || address % TARGET_ERASE_BYTES != 0 ||
        !inside(context, address, bytes) || !LoaderPlatform_OutputsOff() ||
        HAL_FLASH_Unlock() != HAL_OK)
    {
        return false;
    }
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);
    erase.Banks = FLASH_BANK_1;
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Page = (address - TARGET_FLASH_BASE) / TARGET_ERASE_BYTES;
    erase.NbPages = 1;
    result = HAL_FLASHEx_Erase(&erase, &error) == HAL_OK;
    (void)HAL_FLASH_Lock();
    return result && LoaderFlash_EraseCompleted(address);
}
static bool program_flash(void *context, uint32_t address, const void *data, uint32_t bytes)
{
    uint64_t value, check;
    bool result;
    if (data == NULL || bytes != 8 || address % 8 != 0 || !inside(context, address, bytes) ||
        !LoaderPlatform_OutputsOff() || HAL_FLASH_Unlock() != HAL_OK)
    {
        return false;
    }
    memcpy(&value, data, 8);
    result = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, value) == HAL_OK;
    (void)HAL_FLASH_Lock();
    return result && LoaderFlash_Read(address, &check, 8) && check == value;
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
void LoaderPlatform_BootIo(LoaderFlashIo *io)
{
    if (io != NULL)
    {
        bind_io(io, &boot_domain);
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
    return !(GPIOA->ODR & 0x780U) && !(GPIOB->ODR & 3U) &&
           (GPIOA->MODER & 0x3FC000U) == 0x154000U && (GPIOB->MODER & 15U) == 5U &&
           !(GPIOA->OTYPER & 0x780U) && !(GPIOB->OTYPER & 3U) && !(TIM1->BDTR & TIM_BDTR_MOE);
}
void LoaderPlatform_Fatal(void)
{
    LoaderBoard_Safe();
    __disable_irq();
    for (;;)
    { /* 未确认安全恢复条件，不自动恢复升级或功率输出。 */
    }
}
bool LoaderPlatform_Init(bool *software_reset)
{
    uint32_t cause = RCC->CSR, remaining = 4000000U;
    if (software_reset == NULL)
    {
        return false;
    }
    *software_reset = (cause & RCC_CSR_SFTRSTF) != 0 &&
                      !(cause & (RCC_CSR_BORRSTF | RCC_CSR_IWDGRSTF | RCC_CSR_WWDGRSTF |
                                 RCC_CSR_OBLRSTF | RCC_CSR_LPWRRSTF));
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
void LoaderPlatform_Uid(uint8_t uid[12])
{
    if (uid != NULL)
    {
        memcpy(uid, (const void *)UID_BASE, 12);
    }
}
LoaderParameterState LoaderPlatform_Parameters(uint8_t *node)
{
    uint8_t block[64];
    uint32_t offset, i, words[2], node_bits, candidate, shift;
    bool blank = true;
    /* raw11 无整记录CRC；完整扫描只确认可读/空白，不能代替APP参数合法性检查。 */
    for (offset = 0; offset < TARGET_PARAM_BYTES; offset += sizeof(block))
    {
        LoaderPlatform_Watchdog();
        if (!LoaderFlash_Read(TARGET_PARAM_BASE + offset, block, sizeof(block)))
        {
            return YI_PARAMS_UNSAFE;
        }
        for (i = 0; i < sizeof(block); ++i)
        {
            if (block[i] != 255)
            {
                blank = false;
            }
        }
    }
    if (blank)
    {
        return YI_PARAMS_BLANK;
    }
    if (node == NULL || !LoaderFlash_Read(TARGET_PARAM_BASE + TARGET_RAW_SCHEMA_OFFSET, words, 8) ||
        words[0] != TARGET_SCHEMA || words[1] != TARGET_RAW_MAGIC ||
        !LoaderFlash_Read(TARGET_PARAM_BASE, &node_bits, 4))
    {
        return YI_PARAMS_UNSAFE;
    }
    /* node_id 的现有ABI是 IEEE754 float，逐位接纳精确整数，拒绝NaN/负值/小数。 */
    if (node_bits == 0)
    {
        *node = 0;
        return YI_PARAMS_RAW11;
    }
    if ((node_bits >> 23) < 127U || (node_bits >> 23) > 134U)
    {
        return YI_PARAMS_UNSAFE;
    }
    shift = 150U - (node_bits >> 23);
    candidate = (node_bits & 0x7FFFFFU) | 0x800000U;
    if ((candidate & ((1U << shift) - 1U)) != 0 || (candidate >> shift) >= 255U)
    {
        return YI_PARAMS_UNSAFE;
    }
    *node = (uint8_t)(candidate >> shift);
    return YI_PARAMS_RAW11;
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
bool LoaderPlatform_PutMailbox(const BootMailboxMessage *message)
{
    uint8_t raw[YGIM_BYTES];
    volatile uint8_t *mailbox = (volatile uint8_t *)TARGET_MAILBOX_ADDRESS;
    unsigned i;
    if (!BootMailbox_Encode(message, raw))
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
    *(volatile uint32_t *)mailbox = YGIM_MAGIC;
    __DSB();
    for (i = 0; i < sizeof(raw); ++i)
    {
        if (mailbox[i] != raw[i])
        {
            return false;
        }
    }
    return true;
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
        !(vectors[1] & 1U) || (vectors[1] & ~1U) < TARGET_APP_BASE + 0x240U ||
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
