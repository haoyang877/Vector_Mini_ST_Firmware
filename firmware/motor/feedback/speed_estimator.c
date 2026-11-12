#include "speed_estimator.h"

#include <math.h>

#define SPEED_TWO_PI 6.2831853071795864769f
#define SPEED_SAMPLE_PERIOD_S (1.0f / (float)SPEED_ESTIMATOR_SAMPLE_HZ)
#define SPEED_KF_ALPHA 0.23399980789699215f
#define SPEED_KF_BETA 0.03109687052554217f
#define SPEED_MAX_RAD_S 2000.0f
#define SPEED_Q15_SCALE 32768

/* 先减后加，窗口中间和不超过15×2000×32768=983040000，int32_t可容纳。 */
#if SPEED_ESTIMATOR_TAPS > INT32_MAX / SPEED_Q15_SCALE / 2000
#error "Speed estimator sum exceeds int32_t range"
#endif

/**
 * @brief 使KF和均值状态同时失效，避免重启时混入上次运行样本。
 * @param[out] state 非空、由快环独占或已排除并发更新的RAM对象。
 * @note 只清元数据，不访问硬件或Flash。
 */
void SpeedEstimator_Reset(SpeedEstimator *state)
{
    state->angle_rad = 0.0f;
    state->speed_rad_s = 0.0f;
    state->sum_q15 = 0;
    state->cursor = 0U;
    state->initialized = false;
    state->average_ready = false;
}

/**
 * @brief 推进固定20 kHz的KF600，再执行定长整数递推均值。
 * @param[in,out] state 非空、已复位且由快环独占的状态。
 * @param[in] angle_rad 线性化机械单圈角度rad；有效范围[0,2π)。
 * @param[in] valid 外部角度有效位；无效时KF只预测、均值失效。
 * @return 连续机械速度rad/s；无效帧返回预测值，未初始化时为0。
 * @note 限幅保持单次回绕有界；调用方负责传感器离线停机，不在此增加硬件策略。
 */
float SpeedEstimator_Update(SpeedEstimator *state, float angle_rad, bool valid)
{
    if (!isfinite(state->angle_rad) || !isfinite(state->speed_rad_s) || state->angle_rad < 0.0f ||
        state->angle_rad >= SPEED_TWO_PI || fabsf(state->speed_rad_s) > SPEED_MAX_RAD_S)
    {
        SpeedEstimator_Reset(state);
    }
    valid = valid && isfinite(angle_rad) && angle_rad >= 0.0f && angle_rad < SPEED_TWO_PI;
    if (!state->initialized)
    {
        if (valid)
        {
            state->angle_rad = angle_rad;
            state->initialized = true;
        }
    }
    else
    {
        float predicted = state->angle_rad + SPEED_SAMPLE_PERIOD_S * state->speed_rad_s;
        if (predicted >= SPEED_TWO_PI)
        {
            predicted -= SPEED_TWO_PI;
        }
        else if (predicted < 0.0f)
        {
            predicted += SPEED_TWO_PI;
        }
        if (valid)
        {
            float residual = angle_rad - predicted;
            if (residual > 0.5f * SPEED_TWO_PI)
            {
                residual -= SPEED_TWO_PI;
            }
            else if (residual < -0.5f * SPEED_TWO_PI)
            {
                residual += SPEED_TWO_PI;
            }
            predicted += SPEED_KF_ALPHA * residual;
            state->speed_rad_s += (SPEED_KF_BETA / SPEED_SAMPLE_PERIOD_S) * residual;
            state->speed_rad_s =
                fminf(fmaxf(state->speed_rad_s, -SPEED_MAX_RAD_S), SPEED_MAX_RAD_S);
        }
        if (predicted >= SPEED_TWO_PI)
        {
            predicted -= SPEED_TWO_PI;
        }
        else if (predicted < 0.0f)
        {
            predicted += SPEED_TWO_PI;
        }
        /* 极小负值加2π可能舍入为2π，避免下拍误判状态损坏。 */
        state->angle_rad = predicted >= SPEED_TWO_PI ? 0.0f : predicted;
    }
    if (!valid)
    {
        state->average_ready = false;
        return state->speed_rad_s;
    }

    int32_t sample_q15 = (int32_t)(state->speed_rad_s * (float)SPEED_Q15_SCALE);
    if (!state->average_ready)
    {
        for (unsigned i = 0U; i < SPEED_ESTIMATOR_TAPS; ++i)
        {
            state->samples_q15[i] = sample_q15;
        }
        state->sum_q15 = sample_q15 * (int32_t)SPEED_ESTIMATOR_TAPS;
        state->cursor = 0U;
        state->average_ready = true;
    }
    else
    {
        state->sum_q15 -= state->samples_q15[state->cursor];
        state->sum_q15 += sample_q15;
        state->samples_q15[state->cursor] = sample_q15;
    }
    if (++state->cursor == SPEED_ESTIMATOR_TAPS)
    {
        state->cursor = 0U;
    }
    return (float)state->sum_q15 * (1.0f / ((float)SPEED_Q15_SCALE * (float)SPEED_ESTIMATOR_TAPS));
}
