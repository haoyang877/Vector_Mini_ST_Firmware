/* 实际功率端口与限时许可联测；不访问硬件。 */
#include <assert.h>
#include <stdio.h>
#include "at32f45x.h"
#include "bsp_pwm.h"
#include "pwm_power_window.h"
#include "bsp_internal.h"

tmr_type fake_tmr1;
static uint32_t s_key, s_now;
uint32_t BspSystem_TimeMs(void)
{
    return s_now;
}
uint32_t BspSystem_EnterCritical(void)
{
    uint32_t key = s_key;
    s_key = 1U;
    return key;
}
void BspSystem_ExitCritical(uint32_t key)
{
    s_key = key;
}
void tmr_output_enable(tmr_type *timer, confirm_state enabled)
{
    if (enabled)
    {
        timer->brk |= 0x8000U;
    }
    else
    {
        timer->brk &= ~0x8000U;
    }
}
static BspResult owned_enable(void)
{
    uint32_t key = BspSystem_EnterCritical();
    BspResult result = PwmPowerWindow_Start(s_now) ? BspPwm_Enable() : BSP_REJECTED;
    if (result != BSP_OK)
    {
        PwmPowerWindow_Stop();
        BspPwm_Disable();
    }
    BspSystem_ExitCritical(key);
    return result;
}
int main(void)
{
    PwmPowerWindow_Init();
    fake_tmr1.cctrl = 1U;
    assert(owned_enable() == BSP_REJECTED);
    assert(BspPwm_ArePhasesDisabled() && !(fake_tmr1.brk & 0x8000U));
    assert(!PwmPowerWindow_Arm(199U, 0U));
    assert(!PwmPowerWindow_Arm(45001U, 0U));
    assert(PwmPowerWindow_Arm(200U, 0U));
    assert(!PwmPowerWindow_Arm(40000U, 0U));
    assert(owned_enable() == BSP_OK);
    assert((fake_tmr1.cctrl & 0x5550U) == 0x5550U && (fake_tmr1.brk & 0x8000U));
    for (unsigned n = 0U; n < 3999U; ++n)
    {
        assert(PwmPowerWindow_Tick(0U));
    }
    assert(!PwmPowerWindow_Tick(0U)); /* 即使毫秒卡住也按快拍到期。 */
    BspPwm_Disable();
    owned_enable();
    assert(BspPwm_ArePhasesDisabled() && fake_tmr1.cctrl == 1U);
    assert(!PwmPowerWindow_Arm(200U, 0U));

    PwmPowerWindow_Init();
    s_now = UINT32_MAX - 10U;
    assert(PwmPowerWindow_Arm(200U, s_now));
    owned_enable();
    s_now += 199U;
    owned_enable(); /* 重复START不能更新截止时间。 */
    assert(!BspPwm_ArePhasesDisabled());
    assert(PwmPowerWindow_Tick(s_now));
    assert(!PwmPowerWindow_Tick(s_now + 1U));
    BspPwm_Disable();
    assert(g_pwm_power_window.state == PWM_POWER_EXPIRED);

    PwmPowerWindow_Init();
    assert(PwmPowerWindow_Arm(200U, 0U));
    s_now = 5000U;
    owned_enable();
    assert(BspPwm_ArePhasesDisabled());
    assert(g_pwm_power_window.state == PWM_POWER_EXPIRED);

    PwmPowerWindow_Init();
    assert(PwmPowerWindow_Arm(200U, 0U));
    s_now = 0U;
    owned_enable();
    BspPwm_Disable();
    (void)PwmPowerWindow_Tick(s_now);
    assert(g_pwm_power_window.state == PWM_POWER_DONE);
    owned_enable();
    assert(BspPwm_ArePhasesDisabled() && !(fake_tmr1.brk & 0x8000U) && s_key == 0U);
    PwmPowerWindow_Init();
    assert(PwmPowerWindow_Arm(200U, 0U));
    PwmPowerWindow_Abort();
    assert(!PwmPowerWindow_Tick(0U));
    owned_enable();
    assert(BspPwm_ArePhasesDisabled() && !PwmPowerWindow_Arm(200U, 0U));
    puts("PASS bounded APP: disarmed denial, time/tick cutoff, wrap, no renewal/restart, real "
         "gates");
    return 0;
}
