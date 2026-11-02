#ifndef FOC_INTERNAL_H
#define FOC_INTERNAL_H

#include "foc.h"

/**
 * @brief 计算一拍电流或电压控制与占空比，不提交硬件。
 * @param configuration 本拍稳定的活动配置。
 * @param foc 控制器状态，成功时包含待提交占空比。
 * @param motor 动态限流及故障对象。
 * @param law 控制律。
 * @param phase 电角度，rad。
 * @param phase_vel 电角速度，rad/s。
 * @param d_reference d轴电流A或电压V。
 * @param q_reference q轴电流A或电压V。
 * @return 占空比已准备好返回true，故障或非法输入返回false。
 * @note 仅供FOC执行模块使用；成功后须提交PWM并调用Foc_Complete，失败已结束计时。
 */
bool Foc_Prepare(const MotorConfiguration *configuration,
                 FocState *foc,
                 MotorControlState *motor,
                 FocControlLaw law,
                 float phase,
                 float phase_vel,
                 float d_reference,
                 float q_reference);

/**
 * @brief 提交PWM后更新滤波、功率和PI跟踪并结束本拍计时。
 * @param foc 与Foc_Prepare相同的控制状态。
 * @param law 与Foc_Prepare相同的控制律。
 * @note 无硬件访问；PWM提交失败也必须调用，以保持原故障路径顺序。
 */
void Foc_Complete(FocState *foc, FocControlLaw law);

#endif
