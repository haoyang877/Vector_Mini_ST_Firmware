#include "foc_task.h"
#include "motor_axis.h"

/* 中断入口不持有控制对象，只调用 APP 执行所有者。 */
void FOC20kHzIRQHandler(void)
{
    MotorAxis_FastTick();
}
MotorControlMode FOC2kHzSupervisor(void)
{
    return MotorAxis_SupervisorTick();
}
