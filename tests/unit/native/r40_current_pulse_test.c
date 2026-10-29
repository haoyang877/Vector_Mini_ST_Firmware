/* 沿用原台架夹具文件，仅验证仍保留的功率许可契约；不再注入电流。 */
#include <assert.h>
#include <setjmp.h>
#include <stdint.h>
#include <stdio.h>
#include "pwm_power_window.h"
#include "pwm_output_config.h"
#include "bsp_pwm.h"
#define PWM_ALL_OUTPUT_MASK 0x5550U
volatile PwmPowerWindow g_pwm_power_window;
static uint32_t probe_channels, probe_break, mask_depth;
static volatile uint32_t s_disable_sequence;
static bool close_on_unlock;
static jmp_buf halt_exit;
static unsigned halt_stage;
static void __disable_irq(void)
{
    assert(halt_stage == 0U);
    halt_stage = 1U;
}
void BspPwm_Disable(void)
{
    assert(halt_stage == 1U);
    probe_channels = probe_break = 0U;
    halt_stage = 2U;
}
static void __DSB(void)
{
    assert(halt_stage == 2U);
    halt_stage = 3U;
}
static void __WFI(void)
{
    assert(halt_stage == 3U);
    longjmp(halt_exit, 1);
}
/* HALT_DISABLED_PRODUCTION */
static void test_fault_shutdown(void)
{
    probe_channels = 0x5550U;
    probe_break = 0x8000U;
    halt_stage = 0U;
    if (setjmp(halt_exit) == 0)
    {
        halt_disabled();
    }
    assert(halt_stage == 3U);
    assert(probe_channels == 0U && probe_break == 0U);
}
static uint32_t probe_cctrl(void)
{
    return probe_channels;
}
static uint32_t probe_brk(void)
{
    uint32_t sampled = probe_break;
    if (close_on_unlock)
    {
        close_on_unlock = false;
        probe_channels = probe_break = 0U;
        ++s_disable_sequence;
        g_pwm_power_window.state = PWM_POWER_DONE;
    }
    return sampled;
}
/* PWM_SNAPSHOT_PRODUCTION */
/* POWER_CONTRACT_PRODUCTION */
static void test_power_snapshot(void)
{
    probe_channels = 0x5550U;
    probe_break = 0x8000U;
    g_pwm_power_window.state = PWM_POWER_ACTIVE;
    close_on_unlock = true;
    /* 首次读取中快环关闭输出并撤销窗口；复读识别实际关相，不使用全局锁。 */
    assert(power_contract_valid());
    close_on_unlock = false;
    assert(power_contract_valid());
    probe_channels = 0x5550U;
    probe_break = 0x8000U;
    assert(!power_contract_valid()); /* 已关闭许可不能授权开启的功率级。 */
    g_pwm_power_window.state = PWM_POWER_ACTIVE;
    probe_channels = 0x0050U;
    assert(!power_contract_valid());
    assert(mask_depth == 0U);
}
int main(void)
{
    test_fault_shutdown();
    test_power_snapshot();
    close_on_unlock = false;
    for (unsigned state = PWM_POWER_DISARMED; state <= PWM_POWER_EXPIRED; ++state)
    {
        for (uint32_t channels = 0U; channels <= 0xffffU; ++channels)
        {
            for (unsigned enabled = 0U; enabled < 2U; ++enabled)
            {
                probe_channels = channels;
                probe_break = enabled ? 0x8000U : 0U;
                g_pwm_power_window.state = (PwmPowerWindowState)state;
                bool expected = (channels & 0x5550U) == 0U && !enabled;
#if R40_APP_BOUNDED_TEST
                expected |= (channels & 0x5550U) == 0x5550U && enabled && state == PWM_POWER_ACTIVE;
#endif
                assert(power_contract_valid() == expected);
                assert(mask_depth == 0U);
            }
        }
    }
    puts("PASS fault shutdown and power contract: phase lock, bounded permit, atomic snapshot");
}
