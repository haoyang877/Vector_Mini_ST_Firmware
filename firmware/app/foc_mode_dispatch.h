#ifndef FOC_MODE_DISPATCH_H
#define FOC_MODE_DISPATCH_H

#include "motor_work.h"

/**
 * @brief 按当前运行模式分发到对应控制任务，并处理模式退出清理。
 * @return 本周期 worker 的结果；未请求转换或尚未迁移的 worker 返回 MOTOR_WORK_RUNNING。
 * @note 仅由 20 kHz 快速中断调用；覆盖禁用预载、全部控制/校准模式与参数/清障命令。
 *       返回值只由运行状态机消费，本函数不写 ModeNow 或功率级。
 */
MotorWorkOutcome_TypeDef FocMode_Dispatch(void);

#endif
