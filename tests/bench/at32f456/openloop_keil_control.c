#include "openloop_keil_control.h"

/* 只替代Keil会话的主机心跳来源，保留原保护模型的一次性准入和故障锁存。 */

volatile uint32_t g_openloop_enable;
volatile uint32_t g_openloop_rejected_starts;

void AT32KeilControl_Init(AT32KeilControl *control)
{
    control->active = false;
    g_openloop_enable = 0U;
    g_openloop_rejected_starts = 0U;
}

bool AT32KeilControl_Poll(AT32KeilControl *control,
                          AT32OlModel *model,
                          uint32_t requested,
                          uint32_t now_ms)
{
    if (!control->active)
    {
        if (requested == 0U)
        {
            return false;
        }
        if (requested != 1U || !AT32Ol_Begin(model, now_ms))
        {
            ++g_openloop_rejected_starts;
            g_openloop_enable = 0U;
            return false;
        }
        control->active = true;
    }
    if (model->state == AT32_OL_DONE || model->state == AT32_OL_FAULT)
    {
        g_openloop_enable = 0U;
        control->active = false;
        return true;
    }
    if (requested != 1U)
    {
        AT32Ol_Stop(model, AT32_OL_USER_STOP);
        g_openloop_enable = 0U;
        control->active = false;
        return true;
    }
    AT32Ol_Heartbeat(model, now_ms);
    return false;
}
