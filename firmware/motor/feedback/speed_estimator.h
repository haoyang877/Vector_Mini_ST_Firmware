#ifndef SPEED_ESTIMATOR_H
#define SPEED_ESTIMATOR_H

#include <stdbool.h>
#include <stdint.h>

#define SPEED_ESTIMATOR_SAMPLE_HZ 20000U
#define SPEED_ESTIMATOR_TAPS 15U

/** @brief 单路KF600及15点均值状态，编码器快环独占；不是Flash记录。 */
typedef struct
{
    float angle_rad;                           /**< KF单圈角度，[0,2π)。 */
    float speed_rad_s;                         /**< KF连续速度，均值之前，±2000 rad/s。 */
    int32_t samples_q15[SPEED_ESTIMATOR_TAPS]; /**< 速度样本，1/32768 rad/s。 */
    int32_t sum_q15;                           /**< 环形窗口整数和；average_ready时有效。 */
    unsigned cursor;                           /**< 下一覆盖位置，[0,15)。 */
    bool initialized;                          /**< KF已对齐首个有效角度；缺帧期间仍可预测。 */
    bool average_ready;                        /**< 均值窗口有效；缺帧后首个有效速度重新填窗。 */
} SpeedEstimator;

/**
 * @brief 复位测速；不清无效数组，下次有效输入重建窗口。
 * @param[out] state 非空、调用方拥有的状态。
 * @note 仅快环或已排除快环并发时调用；目标工程应在消费复位请求时调用。
 */
void SpeedEstimator_Reset(SpeedEstimator *state);

/**
 * @brief 每50 μs执行一次KF600和15点整数均值，输出连续机械速度。
 * @param[in,out] state 非空、已复位的状态，由同一快环独占。
 * @param[in] angle_rad 方向及LUT校正后的单圈角度，[0,2π)，单位rad。
 * @param[in] valid 本拍角度是否有效；非有限或越界角度同样按无效处理。
 * @return 有效时返回均值速度rad/s；缺帧时返回KF预测值并使均值失效。
 * @note 不含零速死区，不处理硬件离线故障；无I/O、等待、分配或OBS依赖。
 */
float SpeedEstimator_Update(SpeedEstimator *state, float angle_rad, bool valid);

#endif
