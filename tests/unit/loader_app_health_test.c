/* 注入真实APP入口及私有健康实现，验证后台顺序、时钟、监督分频与健康轨迹；无硬件。 */
#include <assert.h>
#include <stdio.h>
#include "app.h"
#include "motor_axis.h"
#include "application_config.h"
#include "bsp_system.h"
#include "yg_protocol.h"
#include "app_protocol.h"
#include "boot_hw.h"
#include "motor_lifecycle.h"
#include "bsp_can.h"
#include "bsp_pwm.h"

static AppContext g_app;
static bool confirm_image(void *context, uint32_t version, uint32_t crc32);
static void publish_identity(void *context)
{
    (void)context;
}
static uint32_t model_now_ms, clock_reads, foreground_stage;
static uint32_t fast_calls, supervisor_calls;
static void watchdog_progress(void *context)
{
    (void)context;
}
static void halt_disabled(AppContext *app)
{
    (void)app;
    assert(false);
}
void BspWatchdog_Feed(void)
{
}

void MotorAxis_ProcessPositionPlan(MotorAxis *instance)
{
    (void)instance;
}
void MotorCalibration_ProcessPublication(MotorCalibration *cal,
                                         OutputEncoderCalibrationProgress progress,
                                         void *context)
{
    assert(cal == &g_app.axis.calibration && context == &g_app);
    (void)progress;
    assert(foreground_stage == 0U);
    foreground_stage = 1U;
}
void AppProtocol_Refresh(AppProtocol *protocol)
{
    assert(protocol == &g_app.protocol);
    assert(foreground_stage == 1U);
    foreground_stage = 2U;
}
uint32_t BspSystem_TimeMs(void)
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
void Protocol_Process(ProtocolEndpoint *instance, uint32_t now_ms)
{
    (void)instance;
    assert(foreground_stage == 3U && now_ms == model_now_ms - 1U);
    foreground_stage = 4U;
}
static void poll_entry(AppLoader *loader, uint32_t now_ms)
{
    assert(loader == &g_app.loader);
    assert(foreground_stage == 5U && now_ms == model_now_ms);
    foreground_stage = 6U;
}
void MotorAxis_SampleFast(MotorAxis *instance)
{
    (void)instance;
}

void MotorAxis_ControlFast(MotorAxis *instance)
{
    (void)instance;
    ++fast_calls;
}
void MotorAxis_SupervisorTick(MotorAxis *instance)
{
    (void)instance;
    ++supervisor_calls;
}

static void process_at(uint32_t now_ms)
{
    model_now_ms = now_ms;
    foreground_stage = clock_reads = 0U;
    App_Process(&g_app);
    assert(foreground_stage == 6U && clock_reads == 2U);
}
static void test_tick_entries(void)
{
    BspApp_OnFastSample();
    assert(fast_calls == 1U && supervisor_calls == 0U);
    for (unsigned tick = 0U; tick < SUPERVISOR_FREQ / 5U; ++tick)
    {
        App_SupervisorTick(&g_app.axis, &g_app.supervisor_ticks);
    }
    assert(supervisor_calls == SUPERVISOR_FREQ / 5U && fast_calls == 1U);
}

static uint32_t model_progress, feeds, calls;
static bool model_healthy, model_link, last_healthy;
static bool model_idle = true, model_off = true;
static uint32_t confirm_calls;

bool MotorLifecycle_ReadBootHealth(MotorLifecycle *instance, uint32_t *progress)
{
    (void)instance;
    *progress = model_progress;
    return model_healthy;
}
bool MotorLifecycle_IsStopped(const MotorLifecycle *instance)
{
    (void)instance;
    return false;
}
bool MotorAxis_SaveIdle(void)
{
    return model_idle;
}
bool MotorLifecycle_ArePhasesDisabled(const MotorLifecycle *life)
{
    (void)life;
    return model_off;
}
bool BootHw_Confirm(uint32_t version, uint32_t crc32)
{
    ++confirm_calls;
    return version == 0x10000U && crc32 == 7U;
}
bool Protocol_IsReady(ProtocolEndpoint *instance)
{
    (void)instance;
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
    g_app.loader = (AppLoader){.lifecycle = &g_app.axis.lifecycle,
                               .communication_ready = &g_app.communication_ready,
                               .endpoint = &g_app.endpoint,
                               .publish_identity = publish_identity};
    test_tick_entries();
    g_app.communication_ready = model_healthy = model_link = true;
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
    g_app.loader.health_checked_ms = UINT32_MAX - 50U;
    g_app.loader.observed_supervisor_ticks = UINT32_MAX;
    model_progress = 0U;
    process_at(49U);
    assert(feeds == 2U && last_healthy);
    g_app.loader.supported = true;
    g_app.loader.healthy_intervals = 0U;
    g_app.loader.health_checked_ms = 0U;
    for (uint32_t i = 1U; i <= 10U; ++i)
    {
        ++model_progress;
        process_at(i * 100U);
        assert(!confirm_image(&g_app.loader, 0x10000U, 7U) && confirm_calls == 0U);
    }
    ++model_progress;
    process_at(1100U);
    assert(g_app.loader.healthy_intervals == 11U && confirm_image(&g_app.loader, 0x10000U, 7U));
    model_off = false;
    assert(!confirm_image(&g_app.loader, 0x10000U, 7U));
    model_off = true;
    g_app.loader.requested = true;
    assert(!confirm_image(&g_app.loader, 0x10000U, 7U));
    g_app.loader.requested = false;
    model_healthy = false;
    assert(!confirm_image(&g_app.loader, 0x10000U, 7U));
    process_at(1200U);
    model_healthy = true;
    assert(g_app.loader.healthy_intervals == 0U && !confirm_image(&g_app.loader, 0x10000U, 7U));
    assert(confirm_calls == 1U);
    puts("PASS actual APP foreground/tick order and bounded watchdog health/progress");
    return 0;
}
