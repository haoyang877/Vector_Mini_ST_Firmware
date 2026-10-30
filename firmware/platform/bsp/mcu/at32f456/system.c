#include "../../bsp_internal.h"
#include "board_config.h"
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

/* 等待上限是启动阶段的轮询次数，不冒充准确的毫秒超时。 */
#define AT32_CLOCK_WAIT_ITERATIONS 2000000U

static bool wait_clock_flag(uint32_t flag)
{
    for (uint32_t remaining = AT32_CLOCK_WAIT_ITERATIONS; remaining > 0U; --remaining)
    {
        if (crm_flag_get(flag) == SET)
        {
            return true;
        }
    }
    return false;
}

static bool clock_init(void)
{
    /* 复用SDK在192MHz使用的5等待周期和1.3V内核档位，不降低访问裕量。 */
    flash_psr_set(FLASH_WAIT_CYCLE_5);
    crm_periph_clock_enable(CRM_PWC_PERIPH_CLOCK, TRUE);
    pwc_ldo_output_voltage_set(PWC_LDO_OUTPUT_1V3);
    crm_clock_source_enable(CRM_CLOCK_SOURCE_HEXT, TRUE);
    if (!wait_clock_flag(CRM_HEXT_STABLE_FLAG))
    {
        return false;
    }
    crm_pll_config(CRM_PLL_SOURCE_HEXT, AT32_PLL_MULTIPLIER, AT32_PLL_PRE_DIVIDER, CRM_PLL_FP_4);
    crm_clock_source_enable(CRM_CLOCK_SOURCE_PLL, TRUE);
    if (!wait_clock_flag(CRM_PLL_STABLE_FLAG))
    {
        return false;
    }
    crm_ahb_div_set(CRM_AHB_DIV_1);
    crm_apb1_div_set(CRM_APB1_DIV_1);
    crm_apb2_div_set(CRM_APB2_DIV_1);
    crm_apb3_div_set(CRM_APB3_DIV_4);
    crm_auto_step_mode_enable(TRUE);
    crm_sysclk_switch(CRM_SCLK_PLL);
    bool switched = false;
    for (uint32_t remaining = AT32_CLOCK_WAIT_ITERATIONS; remaining > 0U; --remaining)
    {
        if (crm_sysclk_switch_status_get() == CRM_SCLK_PLL)
        {
            switched = true;
            break;
        }
    }
    crm_auto_step_mode_enable(FALSE);
    if (!switched)
    {
        return false;
    }
    system_core_clock_update();
    crm_clocks_freq_type clocks;
    crm_clocks_freq_get(&clocks);
    return system_core_clock == AT32_CORE_HZ && clocks.ahb_freq == AT32_CORE_HZ &&
           clocks.apb1_freq == AT32_CORE_HZ && clocks.apb2_freq == AT32_CORE_HZ &&
           CRM->misc2_bit.apb3div == CRM_APB3_DIV_4;
}

/* 中央对齐完整周期为2*PR，单向监督计数周期才是PR+1（手册15.4.3.2）。 */
#define AT32_SUPERVISOR_TOP (1000000U / SUPERVISOR_FREQ - 1U)

#if (AT32_CORE_CLOCK_DERIVED_HZ % (2U * FOC_FREQ)) != 0U || (1000000U % SUPERVISOR_FREQ) != 0U
#error "R40 timer clock must divide the frozen control frequencies exactly"
#endif

static bool s_hardware_initialized;
static bool s_timing_started;

static void supervisor_timer_init(void)
{
    crm_periph_clock_enable(CRM_TMR6_PERIPH_CLOCK, TRUE);
    tmr_reset(TMR6);
    tmr_base_init(TMR6, AT32_SUPERVISOR_TOP, AT32_TIMER_HZ / 1000000U - 1U);
    tmr_flag_clear(TMR6, TMR_OVF_FLAG);
}

bool McuSystem_Init(void)
{
    if (s_hardware_initialized)
    {
        return false;
    }
    if (!clock_init())
    {
        return false;
    }
    McuPwm_Init();
    supervisor_timer_init();
    s_hardware_initialized = true;
    return true;
}

bool McuSystem_Start(void)
{
    if (!s_hardware_initialized || s_timing_started)
    {
        return false;
    }
    if (SysTick_Config(AT32_CORE_HZ / 1000U) != 0U)
    {
        return false;
    }
    NVIC_SetPriority(SysTick_IRQn, 3U);
    tmr_flag_clear(TMR6, TMR_OVF_FLAG);
    tmr_interrupt_enable(TMR6, TMR_OVF_INT, TRUE);
    NVIC_SetPriority(TMR6_DAC_GLOBAL_IRQn, 2U);
    NVIC_EnableIRQ(TMR6_DAC_GLOBAL_IRQn);
    tmr_counter_enable(TMR1, TRUE);
    tmr_counter_enable(TMR6, TRUE);
    s_timing_started = true;
    return true;
}

/* 1ms时基归本端口所有；沿用uint32_t自然回绕语义。 */
static volatile uint32_t s_milliseconds;

void SysTick_Handler(void)
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
/* 启动状态只由本MCU实现持有；失败不允许同次启动重试。 */
static unsigned s_startup_state;

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
    if (s_startup_state != 0U)
    {
        return BSP_NOT_READY;
    }
    s_startup_state = 1U;
    (void)BootHw_Init();
    if (!McuSystem_Init())
    {
        goto failed;
    }
#if BSP_ENABLED_CAN
    McuCan_Init();
#endif
    McuAdc_Init();

    if (!Flash_Init())
    {
        goto failed;
    }
    McuEncoder_Init();
    s_startup_state = 2U;
    return BSP_OK;
failed:
    BspPwm_Disable();
    s_startup_state = 4U;
    return BSP_IO_ERROR;
}

BspResult Bsp_Start(void)
{
    if (s_startup_state != 2U)
    {
        return BSP_NOT_READY;
    }
    s_startup_state = 3U;
    Adc_Start();
    if (!McuSystem_Start())
    {
        goto failed;
    }

    return BSP_OK;
failed:
    BspPwm_Disable();
    s_startup_state = 4U;
    return BSP_IO_ERROR;
}

BspResult BspWatchdog_Start(void)
{
    if ((s_startup_state != 2U && s_startup_state != 3U) || s_watchdog_started)
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

void TMR6_DAC_GLOBAL_IRQHandler(void)
{
    if (tmr_flag_get(TMR6, TMR_OVF_FLAG))
    {
        tmr_flag_clear(TMR6, TMR_OVF_FLAG);
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
    __set_BASEPRI_MAX(2U << (8U - __NVIC_PRIO_BITS));
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
