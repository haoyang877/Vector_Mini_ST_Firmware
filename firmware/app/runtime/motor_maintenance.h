#ifndef MOTOR_MAINTENANCE_H
#define MOTOR_MAINTENANCE_H

#include "foc_algorithm.h"
#include "foc_pid.h"
#include "angle_feedback.h"
#include "foc_sensorless.h"
#include "motor_task_result.h"
#include "motor_lifecycle_request.h"

/**
 * @brief 检查指定维护任务的功能前提，不提交状态或启相。
 * @param configuration 活动配置只读视图，含轴策略、限值与设备校准。
 * @param kind 实际维护任务类型。
 * @param foc 母线和控制测量。
 * @param encoder 编码器运行健康。
 * @return 输入和功能前提全部满足返回 MOTOR_FAULT_NONE，否则返回拒绝原因；不锁存故障。
 */
MotorFaultCode MotorMaintenance_Admit(const MotorConfiguration *configuration,
                                      MotorMaintenanceKind kind,
                                      const FocState *foc,
                                      const EncoderState *encoder);
/**
 * @brief 执行已受理任务的一个有界快速周期。
 * @param configuration 维护所有者提供的独占配置，仅默认恢复或校准提交修改。
 * @param kind 所有者选择的实际任务。
 * @param foc 电流环对象。
 * @param motor 易失运行状态与控制目标。
 * @param pi 速度控制器。
 * @param encoder 编码器状态。
 * @param observer 磁链观测器。
 * @param startup 无感启动过程。
 * @return 功能事实；不包含下一任务或功率动作。
 * @note 20 kHz 调用，所有者须已校验 epoch、故障及取消；保存只在前台执行。
 */
MotorTaskResult MotorMaintenance_Step(MotorConfiguration *configuration,
                                      MotorMaintenanceKind kind,
                                      FocState *foc,
                                      MotorControlState *motor,
                                      PiController *pi,
                                      EncoderState *encoder,
                                      FluxObserverState *observer,
                                      SensorlessStartupState *startup);
/**
 * @brief 仅取消当前功能，不遍历无关任务。
 * @param kind 当前实际任务。
 * @note 所有者先关相再调用；归还仍须由 IsReleased 取得事实。
 */
void MotorMaintenance_Cancel(MotorMaintenanceKind kind);
/**
 * @brief 核对所有功能工作区均已释放。
 * @return 全部工作区静默返回 true，不代表后台借用归还或硬件关闭。
 */
bool MotorMaintenance_IsReleased(void);

#endif
