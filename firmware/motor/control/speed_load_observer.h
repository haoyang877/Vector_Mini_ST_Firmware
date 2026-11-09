#ifndef SPEED_LOAD_OBSERVER_H
#define SPEED_LOAD_OBSERVER_H

#include <stdbool.h>

/** @brief 由目标配置预计算的只读系数；纯算法不包含APP或板级配置头。 */
typedef struct
{
    float alpha;           /**< T×w/(1+T×w)，范围(0,1)。 */
    float derivative_gain; /**< (J/Kt)×w，A/(rad/s)，非负有限值。 */
    float gain;            /**< 负载估计到补偿的增益，[0,1]。 */
    float maximum_step_a;  /**< slew×T，每个外环周期允许变化的A，非负有限值。 */
    float limit_fraction;  /**< 补偿占可用电流的比例，[0,1]。 */
} SpeedLoadObserverCoefficients;

/** @brief 单一外环拥有的动态状态；不保存旧CAN诊断和运行时模式。 */
typedef struct
{
    float speed_filtered_rad_s; /**< 速度低通状态。 */
    float current_filtered_a;   /**< 实际剩余Iq低通状态。 */
    float compensation_a;       /**< 经过幅值及斜率约束的输出。 */
    bool initialized;           /**< 首拍速度已经对齐，避免初始化微分冲击。 */
} SpeedLoadObserver;

/**
 * @brief 复位观测状态，补偿输出归零。
 * @param[out] state 非空、由外环独占的状态。
 * @note 停机、故障、模式或运行代次改变时在外环执行；不修改PI。
 */
void SpeedLoadObserver_Reset(SpeedLoadObserver *state);

/**
 * @brief 按调用方配置的周期计算连续负载补偿。
 * @param[in,out] state 非空且已复位的外环状态，调用方串行访问。
 * @param[in] coefficients 非空、满足字段范围且与实际周期一致的只读编译配置。
 * @param[in] speed_rad_s 与速度PI同源的连续机械速度rad/s，来源不限于KF。
 * @param[in] residual_iq_a 本窗口实际Iq均值A；若存在已知摩擦前馈，由调用方先扣除。
 * @param[in] current_limit_a 当前可用电流A，非负有限值。
 * @return 成功返回true，输出在state->compensation_a；非法测量或数值溢出返回false并复位。
 * @note 本核不设运行时开关。关闭构建由调用方旁路，不能把失败当作有效零补偿。
 */
bool SpeedLoadObserver_Update(SpeedLoadObserver *state,
                              const SpeedLoadObserverCoefficients *coefficients,
                              float speed_rad_s,
                              float residual_iq_a,
                              float current_limit_a);

#endif
