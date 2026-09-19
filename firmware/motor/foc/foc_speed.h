#ifndef FOC_SPEED_H
#define FOC_SPEED_H

#include "data_type.h"
#include "foc_algorithm.h"
#include "foc_pid.h"

/* 速度环模块：参考斜坡、速度 PI 更新与顺序任务入口。 */

/**
 * @brief  速度参考斜坡：把 speedShadow 按加减速限制推进到 speedRef。
 * @param  MotorControl 电机控制状态指针。
 * @note 速度模式与无感启动共用；在快速环上下文调用。
 */
void MotorControl_UpdateSpeedRamp(MotorControl_TypeDef *MotorControl);

/**
 * @brief  速度模式控制更新：斜坡推进速度参考并生成速度环输出。
 * @param  MotorControl 电机控制状态指针，读写 speedShadow、idRef 与 iqRef。
 * @param  controller 速度 PI 控制器指针。
 * @param  vel_mech 机械角速度反馈，单位 rad/s。
 * @note 在快速环上下文调用；归一化输出按 current_limit 缩放为电流参考。
 */
void SpeedMode_UpdateControl(MotorControl_TypeDef *MotorControl,
                             PI_Controller_TypeDef *controller,
                             float vel_mech);

/**
 * @brief  顺序速度模式任务核心：分频执行速度环并保持电流闭环。
 * @param  FOC FOC 状态指针。
 * @param  MotorControl 电机控制状态指针。
 * @param  controller 速度 PI 控制器指针。
 * @param  theta_elec 电角度，单位 rad。
 * @param  vel_elec 电角速度，单位 rad/s。
 * @param  vel_mech 机械角速度反馈，单位 rad/s。
 * @note 在 20kHz 快速环上下文调用；速度环按 SPEED_LOOP_DIVIDER 分频。
 */
void SpeedMode_Run(FOC_TypeDef *FOC,
                   MotorControl_TypeDef *MotorControl,
                   PI_Controller_TypeDef *controller,
                   float theta_elec,
                   float vel_elec,
                   float vel_mech);

#endif
