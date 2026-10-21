/* 注入真实APP入口及私有健康实现，验证后台顺序、时钟、监督分频与健康轨迹；无硬件。 */
#include <assert.h>
#include <stdio.h>
#include "app.h"
#include "motor_axis.h"
#include "control_config.h"
#include "indicator_hw.h"
#include "time_hw.h"
#include "yg_protocol_runtime.h"
#include "yg_protocol_status_adapter.h"
#include "boot_hw.h"
#include "motor_state_machine.h"
#include "comm_hw.h"
#include "yg_protocol_link.h"
#include "power_stage_hw.h"

static uint32_t s_health_checked_ms, s_observed_supervisor_epoch;
static bool s_communication_ready;
static uint8_t s_healthy_intervals;
static bool s_boot_supported, s_entry_requested;
static bool confirm_image(void *context, uint32_t version, uint32_t crc32);
static void publish_identity(void)
{
}
static uint32_t model_now_ms, clock_reads, foreground_stage;
static uint32_t fast_calls, supervisor_calls, led_calls, rgb_calls;

void MotorAxis_ServiceBackground(void)
{
    assert(foreground_stage == 0U);
    foreground_stage = 1U;
}
void YgProtocolStatusAdapter_Refresh(void)
{
    assert(foreground_stage == 1U);
    foreground_stage = 2U;
}
uint32_t time_hw_now_ms(void)
{
    ++clock_reads;
    if (clock_reads == 1U)
    {
        assert(foreground_stage == 2U);
        foreground_stage = 3U;
        return model_now_ms - 1U;
    }
    assert(clock_reads == 2U && foreground_stage == 4U);
    foreground_stage = 5U;
    return model_now_ms;
}
void YgProtocolRuntime_Service(uint32_t now_ms)
{
    assert(foreground_stage == 3U && now_ms == model_now_ms - 1U);
    foreground_stage = 4U;
}
static void poll_entry(uint32_t now_ms)
{
    assert(foreground_stage == 5U && now_ms == model_now_ms);
    foreground_stage = 6U;
}
void MotorAxis_FastTick(void)
{
    ++fast_calls;
}
MotorControlMode MotorAxis_SupervisorTick(void)
{
    ++supervisor_calls;
    return MOTOR_CONTROL_SPEED;
}
MotorMaintenanceKind MotorStateMachine_GetMaintenanceKind(void)
{
    return MOTOR_MAINTENANCE_NONE;
}
void indicator_hw_led_task(void)
{
    assert(supervisor_calls != 0U && supervisor_calls % (SUPERVISOR_FREQ / 5U) == 0U);
    ++led_calls;
}
void indicator_hw_set_color(IndicatorHwColor color)
{
    assert(supervisor_calls != 0U && supervisor_calls % (SUPERVISOR_FREQ / 20U) == 0U);
    assert(color == INDICATOR_HW_COLOR_CYAN);
    ++rgb_calls;
}
static void process_at(uint32_t now_ms)
{
    model_now_ms = now_ms;
    foreground_stage = clock_reads = 0U;
    App_Process();
    assert(foreground_stage == 6U && clock_reads == 2U);
}
static void test_tick_entries(void)
{
    App_FastTick();
    assert(fast_calls == 1U && supervisor_calls == 0U);
    for (unsigned tick = 0U; tick < SUPERVISOR_FREQ / 5U; ++tick)
    {
        App_SupervisorTick();
    }
    assert(supervisor_calls == SUPERVISOR_FREQ / 5U && fast_calls == 1U);
    assert(led_calls == 1U && rgb_calls == 4U);
}

static uint32_t model_progress, feeds, calls;
static bool model_healthy, model_link, last_healthy;
static bool model_idle = true, model_off = true;
static uint32_t confirm_calls;

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
    return model_idle;
}
bool power_stage_hw_phases_disabled(void)
{
    return model_off;
}
bool BootHw_Confirm(uint32_t version, uint32_t crc32)
{
    ++confirm_calls;
    return version == 0x10000U && crc32 == 7U;
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
    test_tick_entries();
    s_communication_ready = model_healthy = model_link = true;
    model_progress = 1U;
    process_at(99U);
    assert(calls == 0U);
    process_at(100U);
    assert(feeds == 1U && last_healthy);
    process_at(200U);
    assert(feeds == 1U && !last_healthy);
    ++model_progress;
    model_link = false;
    process_at(300U);
    assert(feeds == 1U && !last_healthy);
    model_link = true;
    model_healthy = false;
    ++model_progress;
    process_at(400U);
    assert(feeds == 1U && !last_healthy);
    model_healthy = true;
    s_health_checked_ms = UINT32_MAX - 50U;
    s_observed_supervisor_epoch = UINT32_MAX;
    model_progress = 0U;
    process_at(49U);
    assert(feeds == 2U && last_healthy);
    s_boot_supported = true;
    s_healthy_intervals = 0U;
    s_health_checked_ms = 0U;
    for (uint32_t i = 1U; i <= 10U; ++i)
    {
        ++model_progress;
        process_at(i * 100U);
        assert(!confirm_image(NULL, 0x10000U, 7U) && confirm_calls == 0U);
    }
    ++model_progress;
    process_at(1100U);
    assert(s_healthy_intervals == 11U && confirm_image(NULL, 0x10000U, 7U));
    model_idle = false;
    assert(!confirm_image(NULL, 0x10000U, 7U));
    model_idle = true;
    model_off = false;
    assert(!confirm_image(NULL, 0x10000U, 7U));
    model_off = true;
    s_entry_requested = true;
    assert(!confirm_image(NULL, 0x10000U, 7U));
    s_entry_requested = false;
    model_healthy = false;
    assert(!confirm_image(NULL, 0x10000U, 7U));
    process_at(1200U);
    model_healthy = true;
    assert(s_healthy_intervals == 0U && !confirm_image(NULL, 0x10000U, 7U));
    assert(confirm_calls == 1U);
    puts("PASS actual APP foreground/tick order and bounded watchdog health/progress");
    return 0;
}
