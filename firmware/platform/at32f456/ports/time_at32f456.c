#include "time_hw.h"

#include "at32f45x.h"
#include "critical_hw.h"
#if defined(R40_APP_BINDINGS) && R40_APP_BINDINGS
#include "r40_indicator.h"
volatile uint32_t g_r40_max_critical_cycles, g_r40_max_critical_caller;
static uint32_t s_critical_started_cycles;
#endif

/* 1ms时基归本端口所有；沿用uint32_t自然回绕语义。 */
static volatile uint32_t s_milliseconds;

void SysTick_Handler(void)
{
    ++s_milliseconds;
#if defined(R40_APP_BINDINGS) && R40_APP_BINDINGS
    R40_IndicatorTick();
#endif
}

uint32_t time_hw_now_ms(void)
{
    return s_milliseconds;
}

uint32_t critical_hw_enter(void)
{
    uint32_t state = __get_PRIMASK();
    __disable_irq();
#if defined(R40_APP_BINDINGS) && R40_APP_BINDINGS
    if (state == 0U)
    {
        s_critical_started_cycles = DWT->CYCCNT;
    }
#endif
    return state;
}

void critical_hw_exit(uint32_t state)
{
#if defined(R40_APP_BINDINGS) && R40_APP_BINDINGS
    if (state == 0U)
    {
        uint32_t elapsed = DWT->CYCCNT - s_critical_started_cycles;
        if (elapsed > g_r40_max_critical_cycles)
        {
            g_r40_max_critical_cycles = elapsed;
            g_r40_max_critical_caller = (uint32_t)(uintptr_t)__builtin_return_address(0);
        }
    }
#endif
    __set_PRIMASK(state);
}
