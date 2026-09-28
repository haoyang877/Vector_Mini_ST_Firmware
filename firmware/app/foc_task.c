#include "foc_task.h"
#include "motor_state.h"

/* 中断入口不持有控制对象，只调用 APP 执行所有者。 */
void FOC20kHzIRQHandler(void)
{
    MotorControl_FastTick();
}
ModeNow_TypeDef FOC2kHzSupervisor(void)
{
    return MotorControl_SlowTick();
}
