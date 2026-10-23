#include "openloop_keil_control.h"

/* 只替代Keil会话的主机心跳来源，保留原保护模型的一次性准入和故障锁存。 */

volatile uint32_t g_openloop_enable;
volatile uint32_t g_openloop_rejected_starts;

void R40KeilControl_Init(R40KeilControl *control)
{
    control->active = false;
    g_openloop_enable = 0U;
    g_openloop_rejected_starts = 0U;
}

bool R40KeilControl_Poll(R40KeilControl *control, R40OlModel *model, uint32_t now_ms)
{
    uint32_t requested = g_openloop_enable;
    if (!control->active)
    {
        if (requested == 0U)
        {
            return false;
        }
        if (requested != 1U || !R40Ol_Begin(model, now_ms))
        {
            ++g_openloop_rejected_starts;
            g_openloop_enable = 0U;
            return false;
        }
        control->active = true;
    }
    if (model->state == R40_OL_DONE || model->state == R40_OL_FAULT)
    {
        g_openloop_enable = 0U;
        control->active = false;
        return true;
    }
    if (requested != 1U)
    {
        R40Ol_Stop(model, R40_OL_USER_STOP);
        g_openloop_enable = 0U;
        control->active = false;
        return true;
    }
    R40Ol_Heartbeat(model, now_ms);
    return false;
}
