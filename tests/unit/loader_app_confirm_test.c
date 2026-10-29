/* 注入生产APP确认端口原文；仅固定地址内存映射，寄存器全部为桩，不加载驱动。 */
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#if defined(_WIN32)
#include <windows.h>
#else
#include <sys/mman.h>
#endif
#include "boot_hw.h"
#include "image_format.h"
#include "vector_mini_st_g431.h"

static struct
{
    uint32_t ACR, SR, CR, KEYR;
} flash_registers;
static struct
{
    uint32_t KR;
} watchdog_registers;
static struct
{
    uint32_t CCER;
} timer_registers;
static struct
{
    uint32_t IDR;
} gpio_a, gpio_b;
#define FLASH (&flash_registers)
#define IWDG (&watchdog_registers)
#define TIM1 (&timer_registers)
#define GPIOA (&gpio_a)
#define GPIOB (&gpio_b)
static bool available;
bool BootHw_IsAvailable(void);
static uint32_t trial_started_ms, ticks, irq_mask;
static uint32_t HAL_GetTick(void)
{
    return ticks;
}
static uint32_t __get_PRIMASK(void)
{
    return irq_mask;
}
static void __disable_irq(void)
{
    irq_mask = 1U;
}
static void __set_PRIMASK(uint32_t value)
{
    irq_mask = value;
}
static void __ISB(void)
{
}
static void __DSB(void)
{
}

static uint8_t *descriptor;
static void put_marker(uint32_t offset, uint32_t value)
{
    for (uint32_t i = 0U; i < 4U; ++i)
    {
        descriptor[offset + i] = (uint8_t)(value >> (24U - i * 8U));
        descriptor[offset + 4U + i] = (uint8_t)(~value >> (24U - i * 8U));
    }
}
static void reset_trial(void)
{
    memset(descriptor, 255, YGI_DESCRIPTOR_BYTES);
    descriptor[28U] = 0U;
    descriptor[29U] = 1U;
    descriptor[30U] = descriptor[31U] = 0U;
    descriptor[48U] = descriptor[49U] = descriptor[50U] = 0U;
    descriptor[51U] = 7U;
    put_marker(YGI_ACTIVATION_OFFSET, YGI_ACTIVATION_MAGIC);
    put_marker(YGI_ATTEMPT_OFFSET, YGI_ATTEMPT_MAGIC);
    memset(&flash_registers, 0, sizeof(flash_registers));
    flash_registers.ACR = FLASH_ACR_DCEN;
    ticks = 4999U;
    trial_started_ms = 0U;
    available = true;
    irq_mask = 0U;
    timer_registers.CCER = gpio_a.IDR = gpio_b.IDR = watchdog_registers.KR = 0U;
}
int main(void)
{
    available = false;
    trial_started_ms = 1234U;
    ticks = 7890U;
    irq_mask = 1U;
    assert(!BootHw_IsAvailable());
    available = true;
    assert(BootHw_IsAvailable());
    assert(trial_started_ms == 1234U && ticks == 7890U && irq_mask == 1U);
#if defined(_WIN32)
    void *memory = VirtualAlloc((void *)(uintptr_t)TARGET_FLASH_BASE,
                                TARGET_FLASH_BYTES,
                                MEM_RESERVE | MEM_COMMIT,
                                PAGE_READWRITE);
#else
    void *memory = mmap((void *)(uintptr_t)TARGET_FLASH_BASE,
                        TARGET_FLASH_BYTES,
                        PROT_READ | PROT_WRITE,
                        MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED,
                        -1,
                        0);
#endif
    assert(memory == (void *)(uintptr_t)TARGET_FLASH_BASE);
    descriptor = (uint8_t *)(uintptr_t)(TARGET_APP_BASE + TARGET_DESCRIPTOR_OFFSET);
    reset_trial();
    assert(!BootHw_IsConfirmed() && BootHw_ImageCrc32() == 7U);
    BootHw_Watchdog(true);
    assert(watchdog_registers.KR == 0xAAAAU);
    watchdog_registers.KR = 0U;
    ticks = 5000U;
    BootHw_Watchdog(true);
    assert(watchdog_registers.KR == 0U);
    assert(!BootHw_Confirm(0x10000U, 7U));
    reset_trial();
    assert(!BootHw_Confirm(0x10001U, 7U) && !BootHw_Confirm(0x10000U, 8U));
    timer_registers.CCER = TIM_CCER_CC1NE;
    assert(!BootHw_Confirm(0x10000U, 7U));
    timer_registers.CCER = 0U;
    gpio_b.IDR = 1U;
    assert(!BootHw_Confirm(0x10000U, 7U));
    gpio_b.IDR = 0U;
    flash_registers.SR = FLASH_SR_MISERR;
    assert(!BootHw_Confirm(0x10000U, 7U) && !BootHw_IsConfirmed());
    reset_trial();
    assert(BootHw_Confirm(0x10000U, 7U) && BootHw_IsConfirmed());
    assert(irq_mask == 0U && flash_registers.ACR == FLASH_ACR_DCEN &&
           (flash_registers.CR & FLASH_CR_LOCK) != 0U);
    ticks = 10000U;
    assert(BootHw_Confirm(0x10000U, 7U)); /* 已确认重试不再编程。 */
    BootHw_Watchdog(true);
    assert(watchdog_registers.KR == 0xAAAAU);
    watchdog_registers.KR = 0U;
    BootHw_Watchdog(false);
    assert(watchdog_registers.KR == 0U);
    for (uint32_t cut = 1U; cut < 8U; ++cut)
    {
        reset_trial();
        uint8_t marker[8];
        put_marker(YGI_CONFIRM_OFFSET, YGI_CONFIRM_MAGIC);
        memcpy(marker, descriptor + YGI_CONFIRM_OFFSET, 8U);
        memset(descriptor + YGI_CONFIRM_OFFSET, 255, 8U);
        memcpy(descriptor + YGI_CONFIRM_OFFSET, marker, cut);
        assert(!BootHw_Confirm(0x10000U, 7U) && !BootHw_IsConfirmed());
    }
    reset_trial();
    trial_started_ms = UINT32_MAX - 100U;
    ticks = 4898U;
    assert(BootHw_Confirm(0x10000U, 7U)); /* 4999ms，计时回绕不能提前终止。 */
    puts("PASS real APP confirmation identity/output/time/partial-write guards and watchdog "
         "deadline");
    return 0;
}
