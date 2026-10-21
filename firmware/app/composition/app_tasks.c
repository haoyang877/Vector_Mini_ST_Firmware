#include "app_tasks.h"
#include "motor_axis.h"
#include "control_config.h"
#include "motor_state_machine.h"
#include "indicator_hw.h"

/* 中断入口不持有控制对象，只调用 APP 执行所有者。 */
void FOC20kHzIRQHandler(void)
{
    MotorAxis_FastTick();
}
MotorControlMode FOC2kHzSupervisor(void)
{
    return MotorAxis_SupervisorTick();
}

/* 2 kHz 板级监督任务：电机监督与指示器组合。CAN FD 协议在主循环后台服务，
 * 不在监督中混入旧标准 CAN 的队列、波特率或心跳维护。 */

#define LED_DIVIDER (SUPERVISOR_FREQ / 5U)
#define RGB_DIVIDER (SUPERVISOR_FREQ / 20U)

static uint16_t s_led_cnt;
static uint16_t s_rgb_cnt;

/* 模式→呼吸色映射：应用策略，保持既有颜色语义不变。 */
static IndicatorHwColor mode_color(MotorControlMode mode)
{
    MotorMaintenanceKind kind = MotorStateMachine_GetMaintenanceKind();
    if (kind == MOTOR_MAINTENANCE_ENCODER_LUT || kind == MOTOR_MAINTENANCE_COGGING ||
        kind == MOTOR_MAINTENANCE_FRICTION)
    {
        return INDICATOR_HW_COLOR_PURPLE;
    }
    switch (mode)
    {
    case MOTOR_CONTROL_CURRENT:
        return INDICATOR_HW_COLOR_GREEN;
    case MOTOR_CONTROL_SPEED:
        return INDICATOR_HW_COLOR_CYAN;
    case MOTOR_CONTROL_POSITION:
        return INDICATOR_HW_COLOR_BLUE;
    default:
        return INDICATOR_HW_COLOR_NULL;
    }
}

/**
 * @brief 2 kHz 板级监督任务：推进通信服务、指示器、CAN 维护与监督组合。
 * @note 仅由 TIM7 中断上下文调用；不得阻塞、不得动态分配。
 */
void BSP2kHzIRQHandler(void)
{
    /* CAN FD 收发由 RX ISR + 主循环 YgProtocolLink_Service 完成。 */
    /* 电机监督：编码器慢估计、外环控制、主状态机与温度。 */
    MotorControlMode mode = FOC2kHzSupervisor();

    if (++s_led_cnt >= LED_DIVIDER)
    {
        indicator_hw_led_task();
        s_led_cnt = 0;
    }

    if (++s_rgb_cnt >= RGB_DIVIDER)
    {
        indicator_hw_set_color(mode_color(mode));
        s_rgb_cnt = 0;
    }
}
