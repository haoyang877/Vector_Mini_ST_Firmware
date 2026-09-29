/* 注入生产Board_ServiceBoot原文，验证进展、健康、间隔与回绕；硬件全为桩。 */
#include <assert.h>
#include <stdio.h>
#include "board_config.h"
#include "boot_hw.h"
#include "motor_state_machine.h"
#include "comm_hw.h"
#include "yg_protocol_link.h"

static uint32_t s_health_checked_ms, s_observed_supervisor_epoch;
static bool s_communication_ready;
static void poll_entry(uint32_t now_ms)
{
    (void)now_ms;
}

static uint32_t model_progress, feeds, calls;
static bool model_healthy, model_link, last_healthy;

bool MotorStateMachine_ReadBootHealth(uint32_t *progress)
{
    *progress = model_progress;
    return model_healthy;
}
bool MotorStateMachine_BootHandoffReady(uint64_t ticket)
{
    (void)ticket;
    return false;
}
bool MotorAxis_SaveIdle(void)
{
    return true;
}
bool YgProtocolLink_IsReady(void)
{
    return model_link;
}
bool comm_hw_can_is_healthy(void)
{
    return model_link;
}
void BootHw_Watchdog(bool healthy)
{
    ++calls;
    last_healthy = healthy;
    feeds += healthy ? 1U : 0U;
}
int main(void)
{
    s_communication_ready = model_healthy = model_link = true;
    model_progress = 1U;
    Board_ServiceBoot(99U);
    assert(calls == 0U);
    Board_ServiceBoot(100U);
    assert(feeds == 1U && last_healthy);
    Board_ServiceBoot(200U);
    assert(feeds == 1U && !last_healthy);
    ++model_progress;
    model_link = false;
    Board_ServiceBoot(300U);
    assert(feeds == 1U && !last_healthy);
    model_link = true;
    model_healthy = false;
    ++model_progress;
    Board_ServiceBoot(400U);
    assert(feeds == 1U && !last_healthy);
    model_healthy = true;
    s_health_checked_ms = UINT32_MAX - 50U;
    s_observed_supervisor_epoch = UINT32_MAX;
    model_progress = 0U;
    Board_ServiceBoot(49U);
    assert(feeds == 2U && last_healthy);
    puts("PASS bounded APP watchdog requires new supervisor progress and local health");
    return 0;
}
