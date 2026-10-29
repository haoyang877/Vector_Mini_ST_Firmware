#include "app.h"
#include "motor_axis.h"
#include "control_config.h"
#include "motor_state_machine.h"
#include "bsp_led.h"

#include "app_tasks.h"
#include "bsp.h"
#include "bsp_system.h"
#include "bsp_pwm.h"
#include "bsp_adc.h"
#include "app_feature_requirements.h"
#include "bsp_can.h"
#include "pwm_output_config.h"
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
#if BSP_PWM_PERMIT_MODE == BSP_PWM_PERMIT_WINDOW
#include "pwm_power_window.h"
#endif

static volatile uint32_t s_fast_samples, s_supervisor_ticks;
static uint32_t s_watchdog_last_ms, s_watchdog_last_fast, s_watchdog_last_slow;
static bool s_watchdog_enabled;

#if BSP_PWM_PERMIT_MODE == BSP_PWM_PERMIT_WINDOW
volatile uint32_t g_r40_app_test_window_ms, g_r40_app_test_arm_result;
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
    /* 快环可在前台读数之间启停输出；只在不一致时复读一次，不屏蔽ADC。 */
    for (unsigned attempt = 0U; attempt < 2U; ++attempt)
    {
        BspPwmState outputs = BspPwm_GetStatus().state;
        if (outputs == BSP_PWM_OFF)
        {
            return true;
        }
#if BSP_PWM_PERMIT_MODE == BSP_PWM_PERMIT_WINDOW
        if (outputs == BSP_PWM_ON && g_pwm_power_window.state == PWM_POWER_ACTIVE)
        {
            return true;
        }
#endif
    }
    return false;
}

void AppTasks_Init(void)
{
    s_fast_samples = s_supervisor_ticks = 0U;
    s_watchdog_enabled = false;
#if BSP_PWM_PERMIT_MODE == BSP_PWM_PERMIT_WINDOW
    PwmPowerWindow_Init();
#endif
#if BSP_ENABLED_BUS_CURRENT
    BusCurrentFeedback_Init();
#endif
#if APP_ENABLED_DIAGNOSTIC_TELEMETRY
    R40AppTelemetry_Init();
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
#if BSP_PWM_PERMIT_MODE == BSP_PWM_PERMIT_WINDOW
    if (!PwmPowerWindow_Tick(BspSystem_TimeMs()))
    {
        BspPwm_Disable();
    }
#endif
    App_FastTick();
    ++s_fast_samples;
#if APP_ENABLED_DIAGNOSTIC_TELEMETRY || BSP_PWM_PERMIT_MODE == BSP_PWM_PERMIT_WINDOW
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
#if BSP_PWM_PERMIT_MODE == BSP_PWM_PERMIT_WINDOW
    if (g_pwm_power_window.state == PWM_POWER_EXPIRED)
    {
        (void)MotorStateMachine_RequestRemoteStop();
    }
    if (g_r40_app_test_window_ms != 0U)
    {
        uint32_t key = BspSystem_EnterControlCritical();
        uint32_t duration = g_r40_app_test_window_ms;
        g_r40_app_test_window_ms = 0U;
        bool allowed = MotorStateMachine_GetState() == MOTOR_STATE_IDLE &&
                       BspPwm_ArePhasesDisabled() && BspPwm_GetStatus().state == BSP_PWM_OFF;
        g_r40_app_test_arm_result =
            allowed && PwmPowerWindow_Arm(duration, BspSystem_TimeMs()) ? 1U : 2U;
        BspSystem_ExitControlCritical(key);
    }
#endif
#if APP_ENABLED_DIAGNOSTIC_TELEMETRY
    R40AppTelemetry_Service();
#endif
    if (!s_watchdog_enabled)
    {
        return;
    }
    BspAdcFastSample sample;
    if (BspAdc_ReadFast(&sample) != BSP_OK || !power_contract_valid())
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

/* 2 kHz 板级监督任务：电机监督与指示器组合。CAN FD 协议在主循环后台服务，
 * 不在监督中混入旧标准 CAN 的队列、波特率或心跳维护。 */

#define RGB_DIVIDER (SUPERVISOR_FREQ / 20U)

static uint16_t s_rgb_cnt;

/* 模式→呼吸色映射：应用策略，保持既有颜色语义不变。 */
static BspLedColor mode_color(MotorControlMode mode)
{
    MotorMaintenanceKind kind = MotorStateMachine_GetMaintenanceKind();
    if (kind == MOTOR_MAINTENANCE_ENCODER_LUT || kind == MOTOR_MAINTENANCE_COGGING ||
        kind == MOTOR_MAINTENANCE_FRICTION)
    {
        return BSP_LED_PURPLE;
    }
    switch (mode)
    {
    case MOTOR_CONTROL_CURRENT:
        return BSP_LED_GREEN;
    case MOTOR_CONTROL_SPEED:
        return BSP_LED_CYAN;
    case MOTOR_CONTROL_POSITION:
        return BSP_LED_BLUE;
    default:
        return BSP_LED_OFF;
    }
}

/**
 * @brief 执行2 kHz电机监督与原指示器分频任务。
 * @note 仅由 TIM7 中断上下文调用；不得阻塞、不得动态分配。
 */
void App_SupervisorTick(void)
{
    /* CAN FD 收发由 RX ISR + 主循环 YgProtocolLink_Service 完成。 */
    /* 电机监督：编码器慢估计、外环控制、主状态机与温度。 */
    MotorControlMode mode = MotorAxis_SupervisorTick();

    if (++s_rgb_cnt >= RGB_DIVIDER)
    {
        BspLed_SetColor(mode_color(mode));
        s_rgb_cnt = 0;
    }
}
