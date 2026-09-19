#include "bsp_task.h"

#include "data_type.h"
#include "foc_task.h"
#include "indicator_hw.h"
#include "interface_can.h"
#include "motor_state.h"

/* 1 kHz 板级任务：只做调度与应用指示策略；LED/RGB 驱动经平台指示器契约。
 * 三个分频计数（式样与迁移前一致）：LED 200、RGB 50、CAN 波特率维护 100。 */

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
 * @brief 1 kHz 板级任务：推进指示器、CAN 维护与 1 kHz 监督。
 * @note 仅由 TIM7 中断上下文调用；不得阻塞、不得动态分配。
 */
void BSP1kHzIRQHandler(void)
{
    FOC1kHzSupervisor();

    if (++Led_Cnt >= 200)
    {
        indicator_hw_led_task();
        Led_Cnt = 0;
    }

    if (++RGB_Cnt >= 50)
    {
        indicator_hw_set_color(Mode_Color(MotorControl.ModeNow));
        RGB_Cnt = 0;
    }

    if (++CANBRSwitching_Cnt >= 100)
    {
        CAN_BaudRateSwitching();
        CANBRSwitching_Cnt = 0;
    }

    CAN_DisConnect_Handle();
}
