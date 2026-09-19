#include "board_config.h"

#include "board_hw.h"
#include "interface_can.h"
#include "motor_hw.h"
#include "motor_state.h"
#include "param_store.h"

/* 启动编排：顺序与迁移前一致——参数装载 → 应用状态初始化 →
 * 板级启动序列（条件启用三相输出）→ CAN 滤波器初始化。
 * 硬件细节全部位于 platform 契约实现内；本文件不再包含任何硬件头。 */

void Board_Init(void)
{
    /* 从非易失参数区读取参数与标定；无效记录回退编译期默认值。 */
    param_store_load();

    /* 电机控制相关运行态初始化。 */
    MotorControl_Init();

    /* 板级启动：仅配置有效时启动三相互补 PWM。 */
    board_hw_start(MotorControl_IsConfigurationValid());

    /* CAN1 滤波器初始化。 */
    FDCAN1_Param_Init();
}
