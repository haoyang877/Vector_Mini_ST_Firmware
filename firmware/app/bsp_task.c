#include "bsp_task.h"

#include "control_config.h"
#include "data_type.h"
#include "foc_task.h"
#include "indicator_hw.h"
#include "interface_can.h"
#include "motor_state.h"

/* 2 kHz 板级监督任务：通信服务优先执行，随后是指示器、CAN 维护与监督组合；
 * LED/RGB 驱动与 CAN 波特率维护按 SUPERVISOR_FREQ 整数分频，保持迁移前
 * 5 Hz / 20 Hz / 10 Hz 的节拍。 */

#define LED_DIVIDER (SUPERVISOR_FREQ / 5U)
#define RGB_DIVIDER (SUPERVISOR_FREQ / 20U)
#define CAN_BR_DIVIDER (SUPERVISOR_FREQ / 10U)

uint16_t Led_Cnt;
uint16_t RGB_Cnt;
uint16_t CANBRSwitching_Cnt;

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
    /* 1. 通信服务：排空接收队列并派发（写路径短临界区），发送应答与状态流。 */
    CAN_Service();

    /* 2. 监督组合：编码器慢估计、外环控制、主状态机与温度。 */
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

    if (++CANBRSwitching_Cnt >= CAN_BR_DIVIDER)
    {
        CAN_BaudRateSwitching();
        CANBRSwitching_Cnt = 0;
    }

    CAN_DisConnect_Handle();
}
