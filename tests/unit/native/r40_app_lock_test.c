/* 使用真实功率端口验证关相APP编译锁，无运行时解锁入口。 */
#include <assert.h>
#include <stdio.h>
#include "at32f45x.h"
#include "bsp_pwm.h"
tmr_type fake_tmr1;
static uint32_t s_key;
uint32_t BspSystem_EnterCritical(void)
{
    uint32_t old = s_key;
    s_key = 1;
    return old;
}
void BspSystem_ExitCritical(uint32_t key)
{
    s_key = key;
}
void tmr_output_enable(tmr_type *timer, confirm_state enabled)
{
    if (enabled)
        timer->brk |= 0x8000U;
    else
        timer->brk &= ~0x8000U;
}
extern volatile uint32_t g_bsp_pwm_rejected_enables;
int main(void)
{
    fake_tmr1.cctrl = 0x5551U;
    fake_tmr1.brk = 0x8000U;
    BspPwm_Disable();
    for (unsigned n = 0; n < 100; ++n)
    {
        assert(BspPwm_Enable() == BSP_REJECTED);
        assert(BspPwm_ArePhasesDisabled());
        assert(!(fake_tmr1.brk & 0x8000U));
        assert(fake_tmr1.cctrl == 1U);
    }
    assert(g_bsp_pwm_rejected_enables == 100U && s_key == 0);
    puts(
        "PASS R40 APP power lock: repeated START cannot enable any phase; sample channel retained");
    return 0;
}
