#include "power_stage_hw.h"

#include "at32f45x.h"
#include "critical_hw.h"
#if defined(R40_APP_BOUNDED_TEST) && R40_APP_BOUNDED_TEST
#include "r40_power_window.h"
#include "time_hw.h"
#endif

/* TMR1 CH2/3/4及互补输出；CH1预留给采样触发，不能随六路功率输出一起清除。 */
#define R40_MAIN_OUTPUT_MASK ((1U << 4U) | (1U << 8U) | (1U << 12U))
#define R40_COMPLEMENT_OUTPUT_MASK ((1U << 6U) | (1U << 10U) | (1U << 14U))

void power_stage_hw_stop(void)
{
    /* 先禁止总输出，再按主/互补顺序关闭通道；保留CCR和采样通道状态。 */
    uint32_t state = critical_hw_enter();
    tmr_output_enable(TMR1, FALSE);
    TMR1->cctrl &= ~R40_MAIN_OUTPUT_MASK;
    TMR1->cctrl &= ~R40_COMPLEMENT_OUTPUT_MASK;
#if defined(R40_APP_BOUNDED_TEST) && R40_APP_BOUNDED_TEST
    R40_PowerWindowStop();
#endif
    critical_hw_exit(state);
}

#if (defined(R40_APP_PHASE_LOCK) && R40_APP_PHASE_LOCK) ||                                         \
    (defined(R40_APP_BOUNDED_TEST) && R40_APP_BOUNDED_TEST)
/* APP诊断计数；永久关相锁与限时许可分别由目标配置选择。 */
volatile uint32_t g_r40_rejected_power_starts;
#endif

void power_stage_hw_start(void)
{
#if defined(R40_APP_PHASE_LOCK) && R40_APP_PHASE_LOCK
    ++g_r40_rejected_power_starts;
    power_stage_hw_stop();
    return;
#else
    /* 有界临界区防止STOP打断中间步骤后，旧的启动调用继续使能残余通道。 */
    uint32_t state = critical_hw_enter();
#if defined(R40_APP_BOUNDED_TEST) && R40_APP_BOUNDED_TEST
    if (!R40_PowerWindowStart(time_hw_now_ms()))
    {
        ++g_r40_rejected_power_starts;
        power_stage_hw_stop();
        critical_hw_exit(state);
        return;
    }
#endif
    TMR1->cctrl |= R40_COMPLEMENT_OUTPUT_MASK;
    TMR1->cctrl |= R40_MAIN_OUTPUT_MASK;
    tmr_output_enable(TMR1, TRUE);
    critical_hw_exit(state);
#endif
}

bool power_stage_hw_phases_disabled(void)
{
    return (TMR1->cctrl & (R40_MAIN_OUTPUT_MASK | R40_COMPLEMENT_OUTPUT_MASK)) == 0U;
}
