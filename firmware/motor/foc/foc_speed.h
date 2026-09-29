#ifndef FOC_SPEED_H
#define FOC_SPEED_H

#include "motor_types.h"
#include "foc_algorithm.h"
#include "foc_pid.h"

/* 速度环模块：参考斜坡、速度 PI 更新与顺序任务入口。 */

/**
 * @brief  速度参考斜坡：把 speed_ramped_rad_s 按加减速限制推进到 speed_target_rad_s。
 * @param  motor_state 电机控制状态指针。
 * @note 速度模式与无感启动共用；在快速环上下文调用。
 */
void MotorAxis_UpdateSpeedRamp(MotorControlState *motor_state);

/**
 * @brief  速度模式控制更新：斜坡推进速度参考并生成速度环输出。
 * @param  motor_state 电机控制状态指针，读写 speed_ramped_rad_s、id_target_a 与 iq_target_a。
 * @param  controller 速度 PI 控制器指针。
 * @param  mechanical_velocity_rad_s 机械角速度反馈，单位 rad/s。
 * @note 在快速环上下文调用；归一化输出按 current_limit_a 缩放为电流参考。
 */
void SpeedMode_UpdateControl(MotorControlState *motor_state,
                             PiController *controller,
                             float mechanical_velocity_rad_s);

/**
 * @brief  顺序速度模式任务核心：分频执行速度环并保持电流闭环。
 * @param  foc foc 状态指针。
 * @param  motor_state 电机控制状态指针。
 * @param  controller 速度 PI 控制器指针。
 * @param  electrical_angle_rad 电角度，单位 rad。
 * @param  electrical_velocity_rad_s 电角速度，单位 rad/s。
 * @param  mechanical_velocity_rad_s 机械角速度反馈，单位 rad/s。
 * @note 在 20kHz 快速环上下文调用；速度环按 SPEED_LOOP_DIVIDER 分频。
 */
void SpeedMode_Run(FocState *foc,
                   MotorControlState *motor_state,
                   PiController *controller,
                   float electrical_angle_rad,
                   float electrical_velocity_rad_s,
                   float mechanical_velocity_rad_s);

#endif
