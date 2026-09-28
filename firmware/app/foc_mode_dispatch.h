#ifndef FOC_MODE_DISPATCH_H
#define FOC_MODE_DISPATCH_H

#include "motor_work.h"
#include "foc_algorithm.h"
#include "foc_pid.h"
#include "angle_feedback.h"
#include "foc_sensorless.h"

/**
 * @brief 按当前运行模式分发到对应控制任务，并处理模式退出清理。
 * @param foc 所属 FOC 状态。
 * @param motor 所属电机状态。
 * @param pi 所属速度控制器。
 * @param encoder 所属编码器。
 * @param observer 所属观测器。
 * @param startup 所属无感启动状态。
 * @return 本周期 worker 的结果；未请求转换或尚未迁移的 worker 返回 MOTOR_WORK_RUNNING。
 * @note 仅由 20 kHz 快速中断调用；覆盖禁用预载、全部控制/校准模式与参数/清障命令。
 *       返回值只由运行状态机消费，本函数不写 ModeNow 或功率级。
 */
MotorWorkOutcome_TypeDef FocMode_Dispatch(FOC_TypeDef *foc,
                                          MotorControl_TypeDef *motor,
                                          PI_Controller_TypeDef *pi,
                                          Encoder_TypeDef *encoder,
                                          Fluxobserver_TypeDef *observer,
                                          SensorlessStartup_TypeDef *startup);

/**
 * @brief 校验 READY 下的明确模式请求并建立安全初始目标。
 * @param motor owner 独占的电机状态。
 * @param foc 本拍采样值。
 * @param encoder 本拍反馈和校准标志。
 * @param mode 既有控制或维护模式。
 * @return 准入并登记成功返回 true；不启动功率输出。
 * @note APP owner 在短临界区调用，只允许禁用态，不开放远程使能。
 */
bool FocMode_Admit(MotorControl_TypeDef *motor,
                   const FOC_TypeDef *foc,
                   const Encoder_TypeDef *encoder,
                   ModeNow_TypeDef mode);
#endif
