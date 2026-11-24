#include "speed_load_observer.h"

#include <math.h>

/**
 * @brief 清除外环持有的观测动态状态，保留调用方配置。
 * @param[out] state 非空且由外环独占的RAM对象。
 * @note 无硬件访问、等待、动态分配或PI状态修改。
 */
void SpeedLoadObserver_Reset(SpeedLoadObserver *state)
{
    *state = (SpeedLoadObserver){0};
}

/**
 * @brief 低通实际电流及速度，扣除带限惯性项后产生有界补偿。
 * @param[in,out] state 非空、已复位、由外环串行更新的状态。
 * @param[in] coefficients 非空、由调用方预计算及验证的只读系数。
 * @param[in] speed_rad_s 同周期速度反馈rad/s。
 * @param[in] residual_iq_a 同窗口的实际剩余Iq均值A，不能传Iq目标值。
 * @param[in] current_limit_a 当前可用电流A，非负有限值。
 * @return true表示输出有效；false清零状态，调用方按控制失败处理。
 * @note 减小电流上限立即生效，优先于斜率约束；不依赖KF、PI、CAN或BSP。
 */
bool SpeedLoadObserver_Update(SpeedLoadObserver *state,
                              const SpeedLoadObserverCoefficients *coefficients,
                              float speed_rad_s,
                              float residual_iq_a,
                              float current_limit_a)
{
    if (!isfinite(speed_rad_s) || !isfinite(residual_iq_a) || !isfinite(current_limit_a) ||
        current_limit_a < 0.0f)
    {
        SpeedLoadObserver_Reset(state);
        return false;
    }
    if (!state->initialized)
    {
        state->speed_filtered_rad_s = speed_rad_s;
        state->initialized = true;
    }
    state->speed_filtered_rad_s +=
        coefficients->alpha * (speed_rad_s - state->speed_filtered_rad_s);
    state->current_filtered_a += coefficients->alpha * (residual_iq_a - state->current_filtered_a);
    float load_a = state->current_filtered_a -
                   coefficients->derivative_gain * (speed_rad_s - state->speed_filtered_rad_s);
    if (!isfinite(load_a))
    {
        SpeedLoadObserver_Reset(state);
        return false;
    }
    float bound_a = coefficients->limit_fraction * current_limit_a;
    float desired_a = fminf(fmaxf(coefficients->gain * load_a, -bound_a), bound_a);
    float delta_a = desired_a - state->compensation_a;
    delta_a = fminf(fmaxf(delta_a, -coefficients->maximum_step_a), coefficients->maximum_step_a);
    state->compensation_a += delta_a;
    state->compensation_a = fminf(fmaxf(state->compensation_a, -bound_a), bound_a);
    return true;
}
