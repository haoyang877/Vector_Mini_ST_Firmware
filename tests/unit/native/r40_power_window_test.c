/* 实际功率端口与限时许可联测；不访问硬件。 */
#include <assert.h>
#include <stdio.h>
#include "at32f45x.h"
#include "power_stage_hw.h"
#include "r40_power_window.h"

tmr_type fake_tmr1;
static uint32_t s_key, s_now;
uint32_t time_hw_now_ms(void)
{
    return s_now;
}
uint32_t critical_hw_enter(void)
{
    uint32_t key = s_key;
    s_key = 1U;
    return key;
}
void critical_hw_exit(uint32_t key)
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
int main(void)
{
    R40_PowerWindowInit();
    fake_tmr1.cctrl = 1U;
    power_stage_hw_start();
    assert(power_stage_hw_phases_disabled() && !(fake_tmr1.brk & 0x8000U));
    assert(!R40_PowerWindowArm(199U, 0U));
    assert(!R40_PowerWindowArm(45001U, 0U));
    assert(R40_PowerWindowArm(200U, 0U));
    assert(!R40_PowerWindowArm(40000U, 0U));
    power_stage_hw_start();
    assert((fake_tmr1.cctrl & 0x5550U) == 0x5550U && (fake_tmr1.brk & 0x8000U));
    for (unsigned n = 0U; n < 3999U; ++n)
    {
        assert(R40_PowerWindowTick(0U));
    }
    assert(!R40_PowerWindowTick(0U)); /* 即使毫秒卡住也按快拍到期。 */
    power_stage_hw_stop();
    power_stage_hw_start();
    assert(power_stage_hw_phases_disabled() && fake_tmr1.cctrl == 1U);
    assert(!R40_PowerWindowArm(200U, 0U));

    R40_PowerWindowInit();
    s_now = UINT32_MAX - 10U;
    assert(R40_PowerWindowArm(200U, s_now));
    power_stage_hw_start();
    s_now += 199U;
    power_stage_hw_start(); /* 重复START不能更新截止时间。 */
    assert(!power_stage_hw_phases_disabled());
    assert(R40_PowerWindowTick(s_now));
    assert(!R40_PowerWindowTick(s_now + 1U));
    power_stage_hw_stop();
    assert(g_r40_power_window.state == R40_POWER_EXPIRED);

    R40_PowerWindowInit();
    assert(R40_PowerWindowArm(200U, 0U));
    s_now = 5000U;
    power_stage_hw_start();
    assert(power_stage_hw_phases_disabled());
    assert(g_r40_power_window.state == R40_POWER_EXPIRED);

    R40_PowerWindowInit();
    assert(R40_PowerWindowArm(200U, 0U));
    s_now = 0U;
    power_stage_hw_start();
    power_stage_hw_stop();
    assert(g_r40_power_window.state == R40_POWER_DONE);
    power_stage_hw_start();
    assert(power_stage_hw_phases_disabled() && !(fake_tmr1.brk & 0x8000U) && s_key == 0U);
    R40_PowerWindowInit();
    assert(R40_PowerWindowArm(200U, 0U));
    R40_PowerWindowAbort();
    assert(!R40_PowerWindowTick(0U));
    power_stage_hw_start();
    assert(power_stage_hw_phases_disabled() && !R40_PowerWindowArm(200U, 0U));
    puts("PASS bounded APP: disarmed denial, time/tick cutoff, wrap, no renewal/restart, real "
         "gates");
    return 0;
}
