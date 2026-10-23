#include "r40_power_window.h"

/* 一次上电只消费一个窗口；毫秒与快拍独立截止，停止后不能重新装载。 */
volatile R40PowerWindow g_r40_power_window;

void R40_PowerWindowInit(void)
{
    g_r40_power_window = (R40PowerWindow){0};
}

bool R40_PowerWindowArm(uint32_t duration_ms, uint32_t now_ms)
{
    if (g_r40_power_window.state != R40_POWER_DISARMED || duration_ms < 200U ||
        duration_ms > 45000U)
    {
        return false;
    }
    g_r40_power_window.duration_ms = duration_ms;
    g_r40_power_window.armed_ms = now_ms;
    g_r40_power_window.state = R40_POWER_ARMED;
    return true;
}

bool R40_PowerWindowStart(uint32_t now_ms)
{
    if (g_r40_power_window.state == R40_POWER_ARMED)
    {
        if ((uint32_t)(now_ms - g_r40_power_window.armed_ms) >= 5000U)
        {
            g_r40_power_window.state = R40_POWER_EXPIRED;
            return false;
        }
        g_r40_power_window.started_ms = now_ms;
        g_r40_power_window.fast_ticks = 0U;
        g_r40_power_window.state = R40_POWER_ACTIVE;
    }
    if (g_r40_power_window.state != R40_POWER_ACTIVE)
    {
        return false;
    }
    if ((uint32_t)(now_ms - g_r40_power_window.started_ms) >= g_r40_power_window.duration_ms)
    {
        g_r40_power_window.state = R40_POWER_EXPIRED;
        return false;
    }
    return true;
}

void R40_PowerWindowStop(void)
{
    if (g_r40_power_window.state == R40_POWER_ACTIVE)
    {
        g_r40_power_window.state = R40_POWER_DONE;
    }
}

void R40_PowerWindowAbort(void)
{
    g_r40_power_window.state = R40_POWER_EXPIRED;
}

bool R40_PowerWindowTick(uint32_t now_ms)
{
    if (g_r40_power_window.state == R40_POWER_ARMED &&
        (uint32_t)(now_ms - g_r40_power_window.armed_ms) >= 5000U)
    {
        g_r40_power_window.state = R40_POWER_EXPIRED;
    }
    if (g_r40_power_window.state == R40_POWER_ACTIVE)
    {
        ++g_r40_power_window.fast_ticks;
        if (g_r40_power_window.fast_ticks >= g_r40_power_window.duration_ms * 20U ||
            (uint32_t)(now_ms - g_r40_power_window.started_ms) >= g_r40_power_window.duration_ms)
        {
            g_r40_power_window.state = R40_POWER_EXPIRED;
        }
    }
    return g_r40_power_window.state != R40_POWER_EXPIRED;
}
