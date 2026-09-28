#ifndef POSITION_SMOOTH_TRAJECTORY_H
#define POSITION_SMOOTH_TRAJECTORY_H

#include <stdbool.h>
#include <stdint.h>

/* Rest-to-rest, quintic-velocity profile. Units: rad and seconds.
 * All state belongs to the caller; no allocation or hardware dependencies. */
typedef struct
{
    float start, target, distance, direction;
    float acceleration, deceleration, speed_limit, jerk_limit;
    /* Precomputed per-move search coefficients: ta = accel_time_per_speed*speed,
     * td = decel_time_per_speed*speed, tj = sqrtf(jerk_time_sq_per_speed*speed). */
    float accel_time_per_speed, decel_time_per_speed, jerk_time_sq_per_speed;
    float lower_speed, upper_speed, speed;
    float accel_time, cruise_time, decel_time;
    float inv_accel_time, inv_decel_time;
    float elapsed, duration;
    uint8_t iterations;
    bool ready, failed;
} PositionSmoothTrajectory;

typedef struct
{
    float position, speed, acceleration, jerk;
} PositionSmoothSample;

/**
 * @brief 初始化静止到静止的平滑轨迹规划。
 * @param state 调用方拥有的非空轨迹状态；同一实例串行调用。
 * @param start 有限起点，单位 rad。
 * @param target 有限终点，单位 rad。
 * @param speed_limit 有限正速度上限，单位 rad/s。
 * @param acceleration 有限正加速度上限，单位 rad/s²。
 * @param deceleration 有限正减速度幅值，单位 rad/s²。
 * @param jerk_limit 有限正加加速度上限，单位 rad/s³。
 * @return 输入有效返回 true；非法输入返回 false，不开始规划。
 * @note 只初始化规划；必须分批 Prepare 至 ready 后推进。
 */
bool PositionSmooth_Begin(PositionSmoothTrajectory *state,
                          float start,
                          float target,
                          float speed_limit,
                          float acceleration,
                          float deceleration,
                          float jerk_limit);
/**
 * @brief 执行最多四轮规划搜索，不推进轨迹时间。
 * @param state 调用方拥有的非空轨迹状态；同一实例串行调用。
 * @return ready 返回 true；仍在搜索或 failed 返回 false。
 * @note 只在 Begin 成功后调用；调用方须另查 failed 区分失败与未完成。
 */
bool PositionSmooth_Prepare(PositionSmoothTrajectory *state);
/**
 * @brief 按绝对时间计算轨迹值，端点精确钳位。
 * @param state 调用方拥有的非空轨迹状态；同一实例串行调用。
 * @param time_s 有限绝对时间，单位 s。
 * @param sample 非空调用方输出，位置 rad、速度 rad/s、加速度 rad/s²、jerk rad/s³。
 * @note 未 ready 时返回起点零速度；无 I/O。
 */
void PositionSmooth_Sample(const PositionSmoothTrajectory *state,
                           float time_s,
                           PositionSmoothSample *sample);
/**
 * @brief 推进轨迹时间并输出本次轨迹值。
 * @param state 调用方拥有的非空轨迹状态；同一实例串行调用。
 * @param dt 有限非负步长，单位 s；调用方保证有效。
 * @param sample 非空调用方输出，单位同 Sample。
 * @return ready 且到达终点返回 true，否则返回 false。
 * @note 已完成轨迹保持精确终点与零速度；无 I/O。
 */
bool PositionSmooth_Advance(PositionSmoothTrajectory *state,
                            float dt,
                            PositionSmoothSample *sample);

#endif
