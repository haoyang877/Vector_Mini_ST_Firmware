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
#include "motor_state_machine.h"
#include "bsp_can.h"
#include "bsp_pwm.h"

static AppContext g_app;
static bool try_confirm_trial_image(void *context, uint32_t version, uint32_t crc32);
static void publish_identity(void *context)
{
    (void)context;
}
static uint32_t model_now_ms, clock_reads, foreground_stage;
static uint32_t fast_calls, supervisor_calls;
void MotorAxis_ProcessPositionPlan(MotorAxis *instance)
{
    (void)instance;
}
void MotorCalibration_ProcessPublication(MotorCalibration *cal)
{
    assert(cal == &g_app.axis.calibration);
    assert(foreground_stage == 0U);
    foreground_stage = 1U;
}
uint32_t BspSystem_TimeMs(void)
{
    ++clock_reads;
    if (clock_reads == 1U)
    {
        assert(foreground_stage == 1U);
        foreground_stage = 2U;
        return model_now_ms - 1U;
    }
    assert(clock_reads == 2U && foreground_stage == 3U);
    foreground_stage = 4U;
    return model_now_ms;
}
void AppMotorConfiguration_Process(AppMotorConfiguration *service, uint32_t now_ms)
{
    (void)now_ms;
    assert(!service->restart_requested);
}
void Protocol_Process(ProtocolEndpoint *instance, uint32_t now_ms)
{
    (void)instance;
    assert(foreground_stage == 2U && now_ms == model_now_ms - 1U);
    foreground_stage = 3U;
}
static void update_loader_entry_readiness(AppLoader *loader, uint32_t now_ms)
{
    assert(loader == &g_app.loader);
    assert(foreground_stage == 4U && now_ms == model_now_ms);
    foreground_stage = 5U;
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
    assert(foreground_stage == 5U && clock_reads == 2U);
}
static void test_tick_entries(void)
{
    BspApp_OnFastSample();
    assert(fast_calls == 1U && supervisor_calls == 0U);
    for (unsigned tick = 0U; tick < SUPERVISOR_FREQ / 5U; ++tick)
    {
        MotorAxis_SupervisorTick(&g_app.axis);
    }
    assert(supervisor_calls == SUPERVISOR_FREQ / 5U && fast_calls == 1U);
}

static MotorRuntimeProgress model_progress;
static uint32_t feeds, calls;
static bool model_boot_healthy, model_runtime_healthy, model_link, model_confirmed, last_healthy;
static bool model_idle = true, model_off = true;
static uint32_t confirm_calls;

bool MotorStateMachine_ReadBootHealth(MotorStateMachine *instance, uint32_t *progress)
{
    (void)instance;
    *progress = model_progress.supervisor_ticks;
    return model_boot_healthy;
}
bool MotorStateMachine_ReadRuntimeHealth(const MotorStateMachine *instance,
                                         MotorRuntimeProgress *progress)
{
    (void)instance;
    *progress = model_progress;
    return model_runtime_healthy;
}
bool BootHw_IsConfirmed(void)
{
    return model_confirmed;
}
bool MotorStateMachine_IsStopped(const MotorStateMachine *instance)
{
    (void)instance;
    return false;
}
bool MotorAxis_SaveIdle(void)
{
    return model_idle;
}
bool MotorStateMachine_ArePhasesDisabled(const MotorStateMachine *machine)
{
    (void)machine;
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
static void advance_progress(void)
{
    ++model_progress.fast_ticks;
    ++model_progress.supervisor_ticks;
    ++model_progress.outer_ticks;
}

static void reset_health(void)
{
    g_app.loader = (AppLoader){.state_machine = &g_app.axis.state_machine,
                               .communication_ready = &g_app.communication_ready,
                               .endpoint = &g_app.endpoint,
                               .publish_identity = publish_identity};
    g_app.communication_ready = model_boot_healthy = model_runtime_healthy = model_link = true;
    model_confirmed = true;
    model_off = true;
    model_progress = (MotorRuntimeProgress){0};
    feeds = calls = confirm_calls = 0U;
}

static void test_watchdog_requires_task_progress(void)
{
    reset_health();
    process_at(99U);
    assert(calls == 0U);
    advance_progress();
    process_at(100U);
    assert(feeds == 1U && last_healthy);

    /* 其余两个任务继续运行，仍不能掩盖单个关键任务停止。 */
    for (unsigned task = 0U; task < 3U; ++task)
    {
        advance_progress();
        if (task == 0U)
        {
            --model_progress.fast_ticks;
        }
        else if (task == 1U)
        {
            --model_progress.supervisor_ticks;
        }
        else
        {
            --model_progress.outer_ticks;
        }
        process_at(200U + task * 200U);
        assert(feeds == 1U + task && !last_healthy);
        advance_progress();
        process_at(300U + task * 200U);
        assert(feeds == 2U + task && last_healthy);
    }

    advance_progress();
    model_link = false;
    process_at(800U);
    assert(feeds == 4U && !last_healthy);
    model_link = true;
    g_app.communication_ready = false;
    advance_progress();
    process_at(900U);
    assert(feeds == 4U && !last_healthy);
    g_app.communication_ready = true;
    model_runtime_healthy = false;
    advance_progress();
    process_at(1000U);
    assert(feeds == 4U && !last_healthy);
    model_runtime_healthy = true;

    /* 毫秒、快环、监督和外环计数自然回绕后仍可证明新进展。 */
    g_app.loader.health_checked_ms = UINT32_MAX - 50U;
    g_app.loader.observed_progress = (MotorRuntimeProgress){UINT32_MAX, UINT32_MAX, UINT32_MAX};
    model_progress = (MotorRuntimeProgress){0};
    process_at(49U);
    assert(feeds == 5U && last_healthy);
}

static void test_confirmed_fault_preserves_watchdog(void)
{
    reset_health();
    model_boot_healthy = false;
    /* 对应零偏未完成的安全故障：超过原设备复位时长仍保持运行期喂狗。 */
    for (uint32_t now_ms = 100U; now_ms <= 60000U; now_ms += 100U)
    {
        advance_progress();
        process_at(now_ms);
        assert(last_healthy && g_app.loader.healthy_intervals == 0U);
    }
    assert(feeds == 600U);

    /* 相同运行期条件不放宽未确认镜像的采样与故障门禁。 */
    model_confirmed = false;
    advance_progress();
    process_at(60100U);
    assert(feeds == 600U && !last_healthy);
    model_boot_healthy = true;
    advance_progress();
    process_at(60200U);
    assert(feeds == 601U && last_healthy);
}

static void test_strict_confirmation_window(void)
{
    reset_health();
    model_confirmed = false;
    g_app.loader.supported = true;
    for (uint32_t i = 1U; i <= 10U; ++i)
    {
        advance_progress();
        process_at(i * 100U);
        assert(!try_confirm_trial_image(&g_app.loader, 0x10000U, 7U) && confirm_calls == 0U);
    }
    advance_progress();
    process_at(1100U);
    assert(g_app.loader.healthy_intervals == 11U &&
           try_confirm_trial_image(&g_app.loader, 0x10000U, 7U));
    model_off = false;
    assert(!try_confirm_trial_image(&g_app.loader, 0x10000U, 7U));
    model_off = true;
    g_app.loader.request.boot_epoch = 1U;
    assert(!try_confirm_trial_image(&g_app.loader, 0x10000U, 7U));
    g_app.loader.request.boot_epoch = 0U;
    model_boot_healthy = false;
    assert(!try_confirm_trial_image(&g_app.loader, 0x10000U, 7U));
    process_at(1200U);
    model_boot_healthy = true;
    assert(g_app.loader.healthy_intervals == 0U &&
           !try_confirm_trial_image(&g_app.loader, 0x10000U, 7U));
    assert(confirm_calls == 1U);
    advance_progress();
    model_runtime_healthy = false;
    process_at(1300U);
    assert(!last_healthy && g_app.loader.healthy_intervals == 0U);
}

int main(void)
{
    test_tick_entries();
    test_watchdog_requires_task_progress();
    test_confirmed_fault_preserves_watchdog();
    test_strict_confirmation_window();
    puts("PASS actual APP foreground/tick order and bounded watchdog health/progress");
    return 0;
}
