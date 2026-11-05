#include "../../bsp_internal.h"
#include "mcu_resources.h"
#include <stddef.h>
#include <string.h>
#include "bsp_system.h"
#include "at32f45x.h"
#if defined(BSP_APP_LIFECYCLE) && BSP_APP_LIFECYCLE
#include "at32f45x_debug.h"
#include "at32f45x_wdt.h"
#endif
#include "bsp.h"
#include "boot_hw.h"
#include "bsp_pwm.h"
#include "bsp_can.h"

/* 外设初始化按默认成功处理，切换时钟前等待硬件就绪。 */
static void wait_clock_flag(uint32_t flag)
{
    while (crm_flag_get(flag) != SET)
    {
    }
}

static void clock_init(void)
{
    /* 复用SDK在192MHz使用的5等待周期和1.3V内核档位，不降低访问裕量。 */
    flash_psr_set(FLASH_WAIT_CYCLE_5);
    crm_periph_clock_enable(CRM_PWC_PERIPH_CLOCK, TRUE);
    pwc_ldo_output_voltage_set(PWC_LDO_OUTPUT_1V3);
    crm_clock_source_enable(CRM_CLOCK_SOURCE_HEXT, TRUE);
    wait_clock_flag(CRM_HEXT_STABLE_FLAG);
    crm_pll_config(CRM_PLL_SOURCE_HEXT, AT32_PLL_MULTIPLIER, AT32_PLL_PRE_DIVIDER, CRM_PLL_FP_4);
    crm_clock_source_enable(CRM_CLOCK_SOURCE_PLL, TRUE);
    wait_clock_flag(CRM_PLL_STABLE_FLAG);
    crm_ahb_div_set(CRM_AHB_DIV_1);
    crm_apb1_div_set(CRM_APB1_DIV_1);
    crm_apb2_div_set(CRM_APB2_DIV_1);
    crm_apb3_div_set(CRM_APB3_DIV_4);
    crm_auto_step_mode_enable(TRUE);
    crm_sysclk_switch(CRM_SCLK_PLL);
    while (crm_sysclk_switch_status_get() != CRM_SCLK_PLL)
    {
    }
    crm_auto_step_mode_enable(FALSE);
    system_core_clock_update();
}

/* 中央对齐完整周期为2*PR，单向监督计数周期才是PR+1（手册15.4.3.2）。 */
#define AT32_SUPERVISOR_TOP (1000000U / SUPERVISOR_FREQ - 1U)

#if (AT32_CORE_CLOCK_DERIVED_HZ % (2U * FOC_FREQ)) != 0U || (1000000U % SUPERVISOR_FREQ) != 0U
#error "AT32F456 timer clock must divide the frozen control frequencies exactly"
#endif

static void supervisor_timer_init(void)
{
    crm_periph_clock_enable(MCU_CONTROL_TIMER_CLOCK, TRUE);
    tmr_reset(MCU_CONTROL_TIMER);
    tmr_base_init(MCU_CONTROL_TIMER, AT32_SUPERVISOR_TOP, AT32_TIMER_HZ / 1000000U - 1U);
    tmr_flag_clear(MCU_CONTROL_TIMER, TMR_OVF_FLAG);
}

/* 链接脚本将初始化函数单独留在Flash，避免内联进SRAM中的调用者。 */
__attribute__((noinline)) void McuSystem_Init(void)
{
    clock_init();
    McuPwm_Init();
    supervisor_timer_init();
}

__attribute__((noinline)) void McuSystem_Start(void)
{
    (void)SysTick_Config(AT32_CORE_HZ / 1000U);
    NVIC_SetPriority(MCU_SYSTICK_IRQ, MCU_SYSTICK_IRQ_PRIORITY);
    tmr_flag_clear(MCU_CONTROL_TIMER, TMR_OVF_FLAG);
    tmr_interrupt_enable(MCU_CONTROL_TIMER, TMR_OVF_INT, TRUE);
    NVIC_SetPriority(MCU_CONTROL_IRQ, MCU_CONTROL_IRQ_PRIORITY);
#if defined(BSP_APP_LIFECYCLE) && BSP_APP_LIFECYCLE && OUTER_LOOP_DIVIDER != SUPERVISOR_DIVIDER
    NVIC_SetPriority(PendSV_IRQn, MCU_CONTROL_IRQ_PRIORITY);
#endif
    NVIC_EnableIRQ(MCU_CONTROL_IRQ);
    tmr_counter_enable(MCU_PWM_TIMER, TRUE);
    tmr_counter_enable(MCU_CONTROL_TIMER, TRUE);
}

/* 1ms时基归本端口所有；沿用uint32_t自然回绕语义。 */
static volatile uint32_t s_milliseconds;

void BspSystem_HandleTickInterrupt(void)
{
    ++s_milliseconds;
}

uint32_t BspSystem_TimeMs(void)
{
    return s_milliseconds;
}

uint32_t BspSystem_EnterCritical(void)
{
    uint32_t state = __get_PRIMASK();
    __disable_irq();
    return state;
}

void BspSystem_ExitCritical(uint32_t state)
{
    __set_PRIMASK(state);
}

BspResult BspSystem_ReadUid(uint8_t uid[BSP_SYSTEM_UID_BYTES])
{
    if (uid == NULL)
    {
        return BSP_INVALID_ARGUMENT;
    }
    /* AT32F45x手册6.1：96位UID起始0x1FFFF7E8，按三个小端字输出。 */
    const volatile uint32_t *words = (const volatile uint32_t *)0x1FFFF7E8U;
    for (uint32_t word = 0U; word < 3U; ++word)
    {
        uint32_t value = words[word];
        for (uint32_t byte = 0U; byte < 4U; ++byte)
        {
            uid[word * 4U + byte] = (uint8_t)(value >> (byte * 8U));
        }
    }
    return BSP_OK;
}

BspResult BspSystem_ReadIdentity(uint8_t record[64])
{
    if (record == NULL)
    {
        return BSP_INVALID_ARGUMENT;
    }
    /* 新板生产身份布局尚未分配；明确无有效记录，不借用旧板或参数区。 */
    return BSP_NOT_SUPPORTED;
}

#if defined(BSP_APP_LIFECYCLE) && BSP_APP_LIFECYCLE
static void halt_disabled(void)
{
    __disable_irq();
    BspPwm_Disable();
    __DSB();
    /* 不再喂狗，已启用的硬件看门狗负责复位。 */
    for (;;)
    {
        __WFI();
    }
}

#ifndef BSP_WATCHDOG_RELOAD
#define BSP_WATCHDOG_RELOAD 511U
#endif
static bool s_watchdog_started;
static bool s_flash_watchdog_active;
static uint32_t s_flash_watchdog_reload;

static bool set_watchdog_reload(uint32_t reload)
{
    WDT->cmd = 0x5555U;
    WDT->rld = reload;
    uint32_t remaining = 200000U;
    while (WDT->sts != 0U && remaining > 0U)
    {
        --remaining;
    }
    if (remaining == 0U || WDT->rld != reload)
    {
        return false;
    }
    WDT->cmd = 0xAAAAU;
    return true;
}

bool McuSystem_SetFlashWatchdog(bool saving)
{
    if (!s_watchdog_started)
    {
        return true;
    }
    if (!BspPwm_ArePhasesDisabled() || __get_PRIMASK() == 0U || __get_IPSR() != 0U)
    {
        return false;
    }
    if (saving)
    {
        if (s_flash_watchdog_active || WDT->sts != 0U)
        {
            return false;
        }
        s_flash_watchdog_reload = WDT->rld;
        s_flash_watchdog_active = true;
        /* 分频保持不变；仅停机擦写期间使用12位重载上限，等待不循环喂狗。 */
        return set_watchdog_reload(4095U);
    }
    if (!s_flash_watchdog_active)
    {
        return true;
    }
    bool restored = set_watchdog_reload(s_flash_watchdog_reload);
    if (restored)
    {
        s_flash_watchdog_active = false;
    }
    return restored;
}

static bool watchdog_init(void)
{
    DEBUGMCU->apb1_frz &= ~(1U << 12U);
    crm_clock_source_enable(CRM_CLOCK_SOURCE_LICK, TRUE);
    uint32_t remaining = 200000U;
    while (crm_flag_get(CRM_LICK_STABLE_FLAG) == RESET && remaining > 0U)
    {
        --remaining;
    }
    if (remaining == 0U)
    {
        return false;
    }
    WDT->cmd = 0x5555U;
    WDT->div = 0U;
    WDT->rld = BSP_WATCHDOG_RELOAD;
    WDT->win = 4095U;
    remaining = 200000U;
    while (WDT->sts != 0U && remaining > 0U)
    {
        --remaining;
    }
    if (remaining == 0U || WDT->div != 0U || WDT->rld != BSP_WATCHDOG_RELOAD)
    {
        return false;
    }
    WDT->cmd = 0xAAAAU;
    WDT->cmd = 0xCCCCU;
    return true;
}

BspResult Bsp_Init(void)
{
    (void)BootHw_Init();
    McuSystem_Init();
    McuCan_Init();
    McuAdc_Init();
    (void)Flash_Init();
    McuEncoder_Init();
    return BSP_OK;
}

BspResult Bsp_Start(void)
{
    Adc_Start();
    McuSystem_Start();

    return BSP_OK;
}

BspResult BspWatchdog_Start(void)
{
    if (s_watchdog_started)
    {
        return BSP_NOT_READY;
    }
    if (!watchdog_init())
    {
        return BSP_IO_ERROR;
    }
    s_watchdog_started = true;
    return BSP_OK;
}

void BspWatchdog_Feed(void)
{
    if (s_watchdog_started)
    {
        WDT->cmd = 0xAAAAU;
    }
}

void BspSystem_PendOuterTick(void)
{
    SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
}

void BspSystem_HandleControlInterrupt(void)
{
    if (tmr_flag_get(MCU_CONTROL_TIMER, TMR_OVF_FLAG))
    {
        tmr_flag_clear(MCU_CONTROL_TIMER, TMR_OVF_FLAG);
        BspApp_OnControlTick();
    }
}

void NMI_Handler(void)
{
    halt_disabled();
}

void HardFault_Handler(void)
{
    halt_disabled();
}

void MemManage_Handler(void)
{
    halt_disabled();
}

void BusFault_Handler(void)
{
    halt_disabled();
}

void UsageFault_Handler(void)
{
    halt_disabled();
}

#endif

/* ADC为0，载波/采样DMA为1；前台只与低频监督互斥。 */
uint32_t BspSystem_EnterControlCritical(void)
{
    uint32_t state = __get_BASEPRI();
    __set_BASEPRI_MAX(MCU_CONTROL_CRITICAL_PRIORITY << (8U - __NVIC_PRIO_BITS));
    __DSB();
    __ISB();
    return state;
}

void BspSystem_ExitControlCritical(uint32_t state)
{
    __DMB();
    __set_BASEPRI(state);
    __ISB();
}

void BspSystem_MemoryBarrier(void)
{
    __DMB();
}

bool BspSystem_CompareExchange32(volatile uint32_t *address, uint32_t expected, uint32_t desired)
{
    do
    {
        if (__LDREXW(address) != expected)
        {
            __CLREX();
            return false;
        }
        __DMB();
    } while (__STREXW(desired, address) != 0U);
    __DMB();
    return true;
}

#if !defined(BSP_APP_LIFECYCLE) || !BSP_APP_LIFECYCLE
bool McuSystem_SetFlashWatchdog(bool saving)
{
    (void)saving;
    return true;
}
#endif
