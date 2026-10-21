#ifndef SPEED_FEEDFORWARD_H
#define SPEED_FEEDFORWARD_H

#include <stdbool.h>

/** @brief 速度目标整形及惯量前馈配置；调用方拥有，仅停机时修改。 */
typedef struct {
    bool enabled;
    float gain_a_per_rad_s2;
    float acceleration_limit_rad_s2;
    float jerk_limit_rad_s3;
    float current_limit_a;
} SpeedFeedforwardConfig;

/** @brief 调用方拥有的规划状态；速度为rad/s，加速度为rad/s²，电流为A。 */
typedef struct {
    float speed_rad_s;
    float acceleration_rad_s2;
    float current_a;
    bool initialized;
} SpeedFeedforwardState;

/** @brief 设置默认关闭的保守配置。
 * @param config 接收默认配置的对象，非空。
 */
void SpeedFeedforward_Default(SpeedFeedforwardConfig *config);
/** @brief 校验所有配置字段的有限性与保守调参边界。
 * @param config 待校验配置。
 * @return 非空且全部字段合法时返回true。
 */
bool SpeedFeedforward_IsValid(const SpeedFeedforwardConfig *config);
/** @brief 清除规划及前馈状态；不改变调用方控制增益或输出使能。
 * @param state 需要清除的规划状态，非空。
 */
void SpeedFeedforward_Reset(SpeedFeedforwardState *state);
/** @brief 推进一步限加速度/jerk的速度目标，计算有界惯量前馈电流。
 * @param state 调用方独占的状态。
 * @param config 已开启且合法的配置。
 * @param target_rad_s 目标机械速度，rad/s。
 * @param initial_speed_rad_s 首次初始化使用的机械速度，rad/s。
 * @param dt_s 固定调用周期，s。
 * @param total_current_limit_a 当前总限流，A。
 * @return 计算成功返回true；非法输入不修改状态，返回false。
 * @note 纯计算，无分配或硬件访问；前馈不超过总限流50%，加减速共用上限。
 */
bool SpeedFeedforward_Update(SpeedFeedforwardState *state,
    const SpeedFeedforwardConfig *config, float target_rad_s,
    float initial_speed_rad_s, float dt_s, float total_current_limit_a);

#endif
