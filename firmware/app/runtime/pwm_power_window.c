#include "pwm_power_window.h"
#include "bsp_pwm.h"

/* 一次上电只消费一个窗口；毫秒与快拍独立截止，停止后不能重新装载。 */
volatile PwmPowerWindow g_pwm_power_window;

void PwmPowerWindow_Init(void)
{
    g_pwm_power_window = (PwmPowerWindow){.state = PWM_POWER_DISARMED};
}

bool PwmPowerWindow_Arm(uint32_t duration_ms, uint32_t now_ms)
{
    if (g_pwm_power_window.state != PWM_POWER_DISARMED || duration_ms < 200U ||
        duration_ms > 45000U)
    {
        return false;
    }
    g_pwm_power_window.duration_ms = duration_ms;
    g_pwm_power_window.armed_ms = now_ms;
    g_pwm_power_window.state = PWM_POWER_ARMED;
    return true;
}

bool PwmPowerWindow_Start(uint32_t now_ms)
{
    BspPwmStatus status = BspPwm_GetStatus();
    if (g_pwm_power_window.state == PWM_POWER_ACTIVE &&
        status.disable_sequence != g_pwm_power_window.disable_sequence)
    {
        PwmPowerWindow_Stop();
        return false;
    }
    if (g_pwm_power_window.state == PWM_POWER_ARMED)
    {
        if ((uint32_t)(now_ms - g_pwm_power_window.armed_ms) >= 5000U)
        {
            g_pwm_power_window.state = PWM_POWER_EXPIRED;
            return false;
        }
        g_pwm_power_window.disable_sequence = status.disable_sequence;
        g_pwm_power_window.started_ms = now_ms;
        g_pwm_power_window.fast_ticks = 0U;
        g_pwm_power_window.state = PWM_POWER_ACTIVE;
    }
    if (g_pwm_power_window.state != PWM_POWER_ACTIVE)
    {
        return false;
    }
    if ((uint32_t)(now_ms - g_pwm_power_window.started_ms) >= g_pwm_power_window.duration_ms)
    {
        g_pwm_power_window.state = PWM_POWER_EXPIRED;
        return false;
    }
    return true;
}

void PwmPowerWindow_Stop(void)
{
    if (g_pwm_power_window.state == PWM_POWER_ACTIVE)
    {
        g_pwm_power_window.state = PWM_POWER_DONE;
    }
}

void PwmPowerWindow_Abort(void)
{
    g_pwm_power_window.state = PWM_POWER_EXPIRED;
}

bool PwmPowerWindow_Tick(uint32_t now_ms)
{
    if (g_pwm_power_window.state == PWM_POWER_ACTIVE &&
        BspPwm_GetStatus().disable_sequence != g_pwm_power_window.disable_sequence)
    {
        PwmPowerWindow_Stop();
    }
    if (g_pwm_power_window.state == PWM_POWER_ARMED &&
        (uint32_t)(now_ms - g_pwm_power_window.armed_ms) >= 5000U)
    {
        g_pwm_power_window.state = PWM_POWER_EXPIRED;
    }
    if (g_pwm_power_window.state == PWM_POWER_ACTIVE)
    {
        ++g_pwm_power_window.fast_ticks;
        if (g_pwm_power_window.fast_ticks >= g_pwm_power_window.duration_ms * 20U ||
            (uint32_t)(now_ms - g_pwm_power_window.started_ms) >= g_pwm_power_window.duration_ms)
        {
            g_pwm_power_window.state = PWM_POWER_EXPIRED;
        }
    }
    return g_pwm_power_window.state != PWM_POWER_EXPIRED;
}
