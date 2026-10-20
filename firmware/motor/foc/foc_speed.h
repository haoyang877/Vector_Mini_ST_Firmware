#ifndef FOC_SPEED_H
#define FOC_SPEED_H

#include "motor_config.h"
#include "motor_types.h"
#include "foc_algorithm.h"
#include "foc_pid.h"
#include "speed_feedforward.h"

/* 速度环模块：参考斜坡、速度 PI 更新与顺序任务入口。 */

/**
 * @brief  速度参考斜坡：把 speed_ramped_rad_s 按加减速限制推进到 speed_target_rad_s。
 * @param configuration 活动配置只读视图；本次调用期间须保持稳定。
 * @param  motor_state 电机控制状态指针。
 * @note 速度模式与无感启动共用；在快速环上下文调用。
 */
void MotorAxis_UpdateSpeedRamp(const MotorConfiguration *configuration,
                               MotorControlState *motor_state);

/**
 * @brief  速度模式控制更新：斜坡推进速度参考并生成速度环输出。
 * @param configuration 活动配置只读视图；本次调用期间须保持稳定。
 * @param  motor_state 电机控制状态指针，读写 speed_ramped_rad_s、id_target_a 与 iq_target_a。
 * @param  controller 速度 PI 控制器指针。
 * @param  mechanical_velocity_rad_s 机械角速度反馈，单位 rad/s。
 * @note 供顺序标定任务使用；保持原斜坡与归一化 PI，不启用惯量前馈。
 */
void SpeedMode_UpdateControl(const MotorConfiguration *configuration,
                             MotorControlState *motor_state,
                             PiController *controller,
                             float mechanical_velocity_rad_s);

/**
 * @brief 运行速度外环，按显式动态限流装配可选目标整形和惯量前馈。
 * @param configuration 活动配置只读视图；本次调用期间须保持稳定。
 * @param motor_state 非空运行状态，读取目标，发布斜坡和电流参考。
 * @param controller 调用方独占的非空归一化速度 PI 状态。
 * @param state 2 kHz 外环独占的非空前馈状态；停机、故障或切模式后须复位。
 * @param mechanical_velocity_rad_s 机械速度反馈，rad/s。
 * @param current_limit_a 当前有效总限流，A；只允许收窄配置上限，零值关断电流。
 * @return 输入合法且输出有限时返回 true；非法输入或计算失败返回 false。
 * @note 只在 2 kHz 运行速度模式调用；前馈关闭且未降额时逐拍保留原数学。
 *       PI 仍以原配置电流缩放，动态限流和前馈只改变抗饱和输出区间。
 */
bool SpeedMode_UpdateOuterControl(const MotorConfiguration *configuration,
                                  MotorControlState *motor_state,
                                  PiController *controller,
                                  SpeedFeedforwardState *state,
                                  float mechanical_velocity_rad_s,
                                  float current_limit_a);

/**
 * @brief  顺序速度模式任务核心：分频执行速度环并保持电流闭环。
 * @param configuration 活动配置只读视图；本次调用期间须保持稳定。
 * @param  foc foc 状态指针。
 * @param  motor_state 电机控制状态指针。
 * @param  controller 速度 PI 控制器指针。
 * @param  electrical_angle_rad 电角度，单位 rad。
 * @param  electrical_velocity_rad_s 电角速度，单位 rad/s。
 * @param  mechanical_velocity_rad_s 机械角速度反馈，单位 rad/s。
 * @note 在 20kHz 快速环上下文调用；速度环按 SPEED_LOOP_DIVIDER 分频。
 */
void SpeedMode_Run(const MotorConfiguration *configuration,
                   FocState *foc,
                   MotorControlState *motor_state,
                   PiController *controller,
                   float electrical_angle_rad,
                   float electrical_velocity_rad_s,
                   float mechanical_velocity_rad_s);

#endif
