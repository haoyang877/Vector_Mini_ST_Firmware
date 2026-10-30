#include "app.h"
#include "motor_axis.h"
#include "application_config.h"
#include "motor_state_machine.h"

#include "app_tasks.h"
#include "bsp.h"
#include "bsp_system.h"
#include "bsp_pwm.h"
#include "bsp_adc.h"
#include "application_config.h"
#include "bsp_can.h"
#include "application_config.h"
#include "yg_protocol_link.h"
#include <math.h>

#ifndef APP_ENABLED_DIAGNOSTIC_TELEMETRY
#define APP_ENABLED_DIAGNOSTIC_TELEMETRY 0
#endif
#ifndef APP_ENFORCE_POWER_CONTRACT
#define APP_ENFORCE_POWER_CONTRACT 0
#endif
#if APP_ENABLED_DIAGNOSTIC_TELEMETRY
#include "app_telemetry.h"
#endif
#if BSP_ENABLED_BUS_CURRENT
#include "bus_current_feedback.h"
#endif
#if APP_PWM_WINDOWED
#include "pwm_power_window.h"
#endif

static volatile uint32_t s_fast_samples, s_supervisor_ticks;
static uint32_t s_watchdog_last_ms, s_watchdog_last_fast, s_watchdog_last_slow;
static bool s_watchdog_enabled;

#if APP_PWM_WINDOWED
volatile uint32_t g_at32_app_test_window_ms, g_at32_app_test_arm_result;
#endif

static void halt_disabled(void)
{
    (void)BspSystem_EnterCritical();
    BspPwm_Disable();
    for (;;)
    {
        /* 应用失败停止健康喂狗；硬件故障IRQ由BSP独立关断。 */
    }
}

static bool power_contract_valid(void)
{
    if (BspPwm_ArePhasesDisabled())
    {
        return true;
    }
#if APP_PWM_WINDOWED
    return g_pwm_power_window.state == PWM_POWER_ACTIVE;
#else
    return false;
#endif
}

void AppTasks_Init(void)
{
    s_fast_samples = s_supervisor_ticks = 0U;
    s_watchdog_enabled = false;
#if APP_PWM_WINDOWED
    PwmPowerWindow_Init();
#endif
#if BSP_ENABLED_BUS_CURRENT
    BusCurrentFeedback_Init();
#endif
#if APP_ENABLED_DIAGNOSTIC_TELEMETRY
    AT32AppTelemetry_Init();
#endif
}

void AppTasks_Start(void)
{
    BspResult result = BspWatchdog_Start();
    if (result != BSP_OK && result != BSP_NOT_SUPPORTED)
    {
        halt_disabled();
    }
    s_watchdog_enabled = result == BSP_OK;
    s_watchdog_last_ms = BspSystem_TimeMs();
    s_watchdog_last_fast = s_fast_samples;
    s_watchdog_last_slow = s_supervisor_ticks;
}

void BspApp_OnFastSample(void)
{
#if APP_PWM_WINDOWED
    if (!PwmPowerWindow_Tick(BspSystem_TimeMs()))
    {
        BspPwm_Disable();
    }
#endif
    App_FastTick();
    ++s_fast_samples;
#if APP_ENABLED_DIAGNOSTIC_TELEMETRY || APP_PWM_WINDOWED
    MotorAxis_ServiceFastDiagnostics(s_fast_samples);
#endif
#if APP_ENFORCE_POWER_CONTRACT
    if (!power_contract_valid())
    {
        halt_disabled();
    }
#endif
}

void BspApp_OnControlTick(void)
{
#if BSP_ENABLED_BUS_CURRENT
    BusCurrentFeedback_Sample();
#endif
    App_SupervisorTick();
    ++s_supervisor_ticks;
}

void BspApp_OnCanReceive(void)
{
    YgProtocolCanfd_RxIrqHandler();
}

void AppTasks_Process(void)
{
#if APP_PWM_WINDOWED
    if (g_pwm_power_window.state == PWM_POWER_EXPIRED)
    {
        (void)MotorStateMachine_RequestRemoteStop();
    }
    if (g_at32_app_test_window_ms != 0U)
    {
        uint32_t key = BspSystem_EnterControlCritical();
        uint32_t duration = g_at32_app_test_window_ms;
        g_at32_app_test_window_ms = 0U;
        bool allowed =
            MotorStateMachine_GetState() == MOTOR_STATE_IDLE && BspPwm_ArePhasesDisabled();
        g_at32_app_test_arm_result =
            allowed && PwmPowerWindow_Arm(duration, BspSystem_TimeMs()) ? 1U : 2U;
        BspSystem_ExitControlCritical(key);
    }
#endif
#if APP_ENABLED_DIAGNOSTIC_TELEMETRY
    AT32AppTelemetry_Service();
#endif
    if (!s_watchdog_enabled)
    {
        return;
    }
    if (!power_contract_valid())
    {
        halt_disabled();
    }
    uint32_t now = BspSystem_TimeMs();
    if ((uint32_t)(now - s_watchdog_last_ms) >= 10U)
    {
        if (s_fast_samples == s_watchdog_last_fast || s_supervisor_ticks == s_watchdog_last_slow)
        {
            halt_disabled();
        }
        s_watchdog_last_fast = s_fast_samples;
        s_watchdog_last_slow = s_supervisor_ticks;
        s_watchdog_last_ms = now;
        BspWatchdog_Feed();
    }
}
/* 中断入口不持有控制对象，只调用 APP 执行所有者。 */
void App_FastTick(void)
{
    MotorAxis_FastTick();
}

/**
 * @brief 执行2 kHz电机监督。
 * @note 仅由监督上下文调用；不得阻塞、不得动态分配。
 */
void App_SupervisorTick(void)
{
    (void)MotorAxis_SupervisorTick();
}
