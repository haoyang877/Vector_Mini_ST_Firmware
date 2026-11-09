/* 同一前台和 ADC 交接入口，只有物理 PWM 和时钟用替身。 */
#include <assert.h>
#include <stdio.h>
#include "openloop_keil_control.h"

static AT32OlModel s_model;
static AT32KeilControl s_keil_control;
static uint32_t s_nonce, s_heartbeat, clock_ms;
static bool disabled, s_output_on, s_hold_snapshot, s_wdt_test;
static uint32_t BspSystem_TimeMs(void)
{
    return clock_ms;
}
static bool BspPwm_ArePhasesDisabled(void)
{
    return disabled;
}
static void BspPwm_Disable(void)
{
    disabled = true;
}
static void BspSystem_MemoryBarrier(void)
{
}
static void stop_outputs(void)
{
    disabled = true;
    s_output_on = false;
}

/* REAL_HANDOFF */

int main(void)
{
    disabled = true;
    s_model.state = AT32_OL_READY;
    g_openloop_command.command = 1U;
    g_openloop_command.inverse = ~1U;
    g_openloop_command.nonce = 1U;
    post_controls();
    assert(s_model.state == AT32_OL_READY && g_openloop_command.completed_nonce == 0U);
    clock_ms = AT32_OL_LEASE_MS;
    consume_controls();
    assert(s_model.state == AT32_OL_READY && g_openloop_command.result == 4U);

    g_openloop_command.nonce = 2U;
    post_controls();
    /* 新 STOP 在旧启动仍占槽时越过它。前台关相，不等 ADC。 */
    g_openloop_command.command = 2U;
    g_openloop_command.inverse = ~2U;
    g_openloop_command.nonce = 3U;
    disabled = false;
    post_controls();
    assert(disabled && s_stop_requested);
    consume_controls();
    assert(s_model.state == AT32_OL_FAULT && g_openloop_command.result == 4U);
    post_controls();
    consume_controls();
    assert(g_openloop_command.completed_nonce == 3U && g_openloop_command.result == 2U);

    s_model = (AT32OlModel){.state = AT32_OL_READY};
    g_openloop_enable = 1U;
    post_controls();
    consume_controls();
    assert(s_keil_control.active && s_model.heartbeat_ms == clock_ms);
    uint32_t previous = clock_ms++;
    consume_controls();
    assert(s_model.heartbeat_ms == previous);
    puts("PASS ADC handoff: stale start, STOP before ADC, ordered ACK, foreground-only lease");
    return 0;
}
