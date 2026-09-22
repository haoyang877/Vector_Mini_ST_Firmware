#include "bsp_task.h"

#include "control_config.h"
#include "data_type.h"
#include "foc_task.h"
#include "indicator_hw.h"
#include "motor_state.h"
#include "yg_protocol_app_adapter.h"

/* 2 kHz 板级监督任务：电机监督与指示器组合。CAN FD 协议在主循环后台服务，
 * 不在监督中混入旧标准 CAN 的队列、波特率或心跳维护。 */

#define LED_DIVIDER (SUPERVISOR_FREQ / 5U)
#define RGB_DIVIDER (SUPERVISOR_FREQ / 20U)

uint16_t Led_Cnt;
uint16_t RGB_Cnt;

/* 模式→呼吸色映射：应用策略，保持既有颜色语义不变。 */
static IndicatorHwColor Mode_Color(ModeNow_TypeDef mode)
{
    switch (mode)
    {
    case Current_Mode:
        return INDICATOR_HW_COLOR_GREEN;
    case Speed_Mode:
        return INDICATOR_HW_COLOR_CYAN;
    case Sensorless_Speed_Mode:
        return INDICATOR_HW_COLOR_YELLOW;
    case Position_Mode:
        return INDICATOR_HW_COLOR_BLUE;
    case Position_Impedance_Mode:
        return INDICATOR_HW_COLOR_WHITE;
    case Calib_Motor_R_L_Flux:
    case Calib_EncoderOffset:
    case Calib_EncoderObserver:
    case Calib_Anticogging:
    case Calib_Friction:
        return INDICATOR_HW_COLOR_PURPLE;
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
    YgProtocolAppAdapter_RefreshStatus();
    /* 电机监督：编码器慢估计、外环控制、主状态机与温度。 */
    FOC2kHzSupervisor();

    if (++Led_Cnt >= LED_DIVIDER)
    {
        indicator_hw_led_task();
        Led_Cnt = 0;
    }

    if (++RGB_Cnt >= RGB_DIVIDER)
    {
        indicator_hw_set_color(Mode_Color(MotorControl.ModeNow));
        RGB_Cnt = 0;
    }

}
