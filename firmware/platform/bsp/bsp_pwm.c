/* 快环不可抢占；其它所有者自行保证互斥。本层不屏蔽中断，提交期限与故障关相仍保留。 */
#include "bsp_pwm.h"
#include "bsp_config.h"
#include "bsp_internal.h"
#include <string.h>

static bool s_initialized;
static float s_period_counts;
static bool s_output_fault;
static volatile uint32_t s_disable_sequence;
/* 保留给Keil Watch观测启相拒绝次数；由唯一功率所有者递增。 */
volatile uint32_t g_bsp_pwm_rejected_enables;

void Pwm_Init(void)
{
    if (!s_initialized)
    {
        McuPwm_Init();
        /* PWM周期在初始化后固定，快环只使用该换算尺度。 */
        s_period_counts = McuPwm_PeriodCounts();
        s_initialized = true;
    }
}

/* binary32正数按位有序；另收负零，拒绝负数、无穷及NaN，避免多次浮点比较。 */
static inline bool duty_valid(float duty)
{
    uint32_t bits;
    memcpy(&bits, &duty, sizeof(bits));
    return bits <= UINT32_C(0x3F800000) || bits == UINT32_C(0x80000000);
}

/* 一次读取，不在BSP内处理抢占或重试；一致快照由调用上下文保证。 */
BspPwmStatus BspPwm_GetStatus(void)
{
    bool any_channel, all_channels, master;
    McuPwm_ReadOutputs(&any_channel, &all_channels, &master);
    BspPwmState state = master ? (all_channels ? BSP_PWM_ON : BSP_PWM_INCOMPLETE)
                               : (any_channel ? BSP_PWM_INCOMPLETE : BSP_PWM_OFF);
    return (BspPwmStatus){state, s_disable_sequence};
}

void BspPwm_Disable(void)
{
    McuPwm_DisableOutputs();
    ++s_disable_sequence;
}

/* 错过提交窗口时不写比较值，先关相并锁存故障。 */
static BspResult pwm_commit_missed(void)
{
    s_output_fault = true;
    BspPwm_Disable();
    return BSP_TIMEOUT;
}

BspResult BspPwm_SetDuty(float phase_a, float phase_b, float phase_c)
{
    if (!duty_valid(phase_a) || !duty_valid(phase_b) || !duty_valid(phase_c))
    {
        return BSP_INVALID_ARGUMENT;
    }
    if (!s_initialized)
    {
        return BSP_NOT_READY;
    }
    const uint16_t a = (uint16_t)(phase_a * s_period_counts),
                   b = (uint16_t)(phase_b * s_period_counts),
                   c = (uint16_t)(phase_c * s_period_counts);
    return McuPwm_CommitThree(a, b, c) ? BSP_OK : pwm_commit_missed();
}

BspResult BspPwm_SetPhaseDuty(BspPwmPhase phase, float duty)
{
    if ((unsigned)phase >= 3U || !duty_valid(duty))
    {
        return BSP_INVALID_ARGUMENT;
    }
    if (!s_initialized)
    {
        return BSP_NOT_READY;
    }
    uint16_t counts = (uint16_t)(duty * s_period_counts);
    return McuPwm_CommitPhase(phase, counts) ? BSP_OK : pwm_commit_missed();
}

BspResult BspPwm_Enable(void)
{
    BspResult result;
    if (!s_initialized)
    {
        result = BSP_NOT_READY;
    }
    else if (!BSP_ENABLED_PWM || s_output_fault || BSP_PWM_HARD_LOCK)
    {
        result = BSP_REJECTED;
    }
    else
    {
        McuPwm_EnableOutputs();
        if (BspPwm_GetStatus().state == BSP_PWM_ON)
        {
            return BSP_OK;
        }
        s_output_fault = true;
        result = BSP_IO_ERROR;
    }
    ++g_bsp_pwm_rejected_enables;
    BspPwm_Disable();
    if (BspPwm_GetStatus().state != BSP_PWM_OFF)
    {
        s_output_fault = true;
        McuPwm_Fatal();
    }
    return result;
}
