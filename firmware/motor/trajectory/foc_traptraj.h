#ifndef __FOC_TRAPTRAJ_H__
#define __FOC_TRAPTRAJ_H__

#include <stdbool.h>
#include <stdint.h>

/* 梯形轨迹规划：纯计算模块，无硬件依赖。 */

typedef struct
{
    // Step
    float Y;
    float Yd;
    float Ydd;

    float start_position;
    float start_velocity;
    float end_position;

    float acc;
    float vel;
    float dec;

    float acc_distance;

    float t_acc;
    float t_vel;
    float t_dec;
    float t_total;

    uint32_t tick;

    bool profile_done;
} Traj_TypeDef;

/**
 * @brief  规划一段梯形速度轨迹并复位内部状态。
 * @param  position 目标位置，单位 rad。
 * @param  start_position 起始位置，单位 rad。
 * @param  start_velocity 起始速度，单位 rad/s。
 * @param  Vmax 最大速度，单位 rad/s。
 * @param  Amax 加速度上限，单位 rad/s²。
 * @param  Dmax 减速度上限，单位 rad/s²。
 * @note 纯计算；由位置控制器在目标变化时调用。
 */
void TRAJ_plan(
    float position, float start_position, float start_velocity, float Vmax, float Amax, float Dmax);

/**
 * @brief  按采样周期推进轨迹一步。
 * @param  sample_time 采样周期，单位 s。
 * @note 纯计算；由控制环按更新周期调用。
 */
void TRAJ_eval(float sample_time);

/**
 * @brief  读取轨迹当前位置。
 * @return 位置，单位 rad。
 */
float TRAJ_Get_Y(void);

/**
 * @brief  读取轨迹当前速度。
 * @return 速度，单位 rad/s。
 */
float TRAJ_Get_Yd(void);

#endif
