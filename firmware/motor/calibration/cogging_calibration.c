#include "cogging_calibration.h"
#include <math.h>
#include <stddef.h>

#define FIRST_POINT_TIMEOUT_TICKS 20000U /* 首点无保持电流积分，允许 2 倍定位时间 */
#define POINT_TIMEOUT_TICKS 10000U       /* 2 kHz 下单点 5 s 截止 */
#define TOTAL_TIMEOUT_TICKS 3600000U     /* 2 kHz 下全程 30 分钟 */
#define SATURATION_TICKS 400U            /* 2 kHz 下连续 200 ms 饱和判定限流退出 */

/* 表点数必须与索引位宽一致：整圈 16 位 / 1024 点 = 64 个子步。 */
typedef char
    CoggingMapPointsMustMatchLog2[(COGGING_MAP_POINTS == (1U << COGGING_MAP_POINTS_LOG2)) ? 1 : -1];

void CoggingCalibration_Abort(CoggingCalibration *c, CoggingReason reason)
{
    /* 仅中止进行中的标定，不覆盖已完成或已失败的状态。 */
    if (c != NULL && (c->state == COGGING_SETTLING || c->state == COGGING_SAMPLING ||
                      c->state == COGGING_TURNAROUND))
    {
        c->state = COGGING_FAILED;
        c->reason = reason;
    }
}

bool CoggingCalibration_Start(CoggingCalibration *c, float position_rad, float full_scale_a)
{
    int32_t grid;
    if (c == NULL || !isfinite(position_rad) || position_rad < 0.0f || position_rad >= 6.283186f ||
        !isfinite(full_scale_a) || full_scale_a <= 0.0f)
    {
        return false;
    }
    grid = (int32_t)floorf(position_rad / COGGING_STEP_RAD) + 1;
    c->full_scale_a = full_scale_a;
    c->state = COGGING_SETTLING;
    c->reason = COGGING_OK;
    c->index = (uint32_t)grid & (COGGING_MAP_POINTS - 1U);
    c->target_grid = grid;
    c->target_rad = (float)grid * COGGING_STEP_RAD;
    c->direction_pass = c->points_done = c->point_ticks = c->stable_ticks = 0U;
    c->samples = c->saturation_ticks = c->total_ticks = 0U;
    c->mean_iq_a = 0.0f;
    c->filtered_velocity_rad_s = 0.0f;
    c->window_rejected_ticks = c->consecutive_rejected_ticks = 0U;
    /* 首次遍历会在任何结果有效之前覆盖每个表项。 */
    return true;
}

void CoggingCalibration_Update(
    CoggingCalibration *c, float position_rad, float velocity_rad_s, float iq_a, bool saturated)
{
    bool stable;
    if (c == NULL || (c->state != COGGING_SETTLING && c->state != COGGING_SAMPLING &&
                      c->state != COGGING_TURNAROUND))
    {
        return;
    }
    if (!isfinite(position_rad) || !isfinite(velocity_rad_s) || !isfinite(iq_a) ||
        fabsf(iq_a) > c->full_scale_a)
    {
        CoggingCalibration_Abort(c, COGGING_INVALID_INPUT);
        return;
    }
    /* 超时：点超时发生时不再推进全程计数。 */
    if (++c->point_ticks >
            (c->points_done == 0U ? FIRST_POINT_TIMEOUT_TICKS : POINT_TIMEOUT_TICKS) ||
        ++c->total_ticks > TOTAL_TIMEOUT_TICKS)
    {
        CoggingCalibration_Abort(c, COGGING_POINT_TIMEOUT);
        return;
    }
    /* 连续饱和 200 ms 判定限流退出。 */
    c->saturation_ticks = saturated ? c->saturation_ticks + 1U : 0U;
    if (c->saturation_ticks >= SATURATION_TICKS)
    {
        CoggingCalibration_Abort(c, COGGING_CURRENT_LIMIT);
        return;
    }
    /* 慢漂移滤波（约 5 Hz）与合格门限：位置、滤波速度、原始速度同时满足。 */
    c->filtered_velocity_rad_s += 0.015f * (velocity_rad_s - c->filtered_velocity_rad_s);
    stable = !saturated && fabsf(c->target_rad - position_rad) <= COGGING_POSITION_GATE_RAD &&
             fabsf(c->filtered_velocity_rad_s) <= 0.01f && fabsf(velocity_rad_s) <= 0.15f;
    if (!stable)
    {
        ++c->window_rejected_ticks;
        ++c->consecutive_rejected_ticks;
        /* 窗口覆盖率下限或突发无效时重开资格判定。 */
        if (c->window_rejected_ticks > 10U || c->consecutive_rejected_ticks >= 4U || saturated ||
            fabsf(c->target_rad - position_rad) > 2.0f * COGGING_POSITION_TOL_RAD ||
            fabsf(velocity_rad_s) > 0.15f)
        {
            c->stable_ticks = c->samples = 0U;
            c->mean_iq_a = 0.0f;
            c->window_rejected_ticks = c->consecutive_rejected_ticks = 0U;
            if (c->state == COGGING_SAMPLING)
            {
                c->state = COGGING_SETTLING;
            }
        }
        return;
    }
    c->consecutive_rejected_ticks = 0U;
    if (c->state != COGGING_SAMPLING)
    {
        if (++c->stable_ticks < COGGING_SETTLE_TICKS)
        {
            return;
        }
        c->window_rejected_ticks = 0U;
        if (c->state == COGGING_TURNAROUND)
        {
            /* 反向逼近：从相反方向进入最后一个正向点。 */
            c->target_rad = (float)(--c->target_grid) * COGGING_STEP_RAD;
            c->point_ticks = c->stable_ticks = 0U;
            c->state = COGGING_SETTLING;
        }
        else
        {
            c->state = COGGING_SAMPLING;
        }
        return;
    }
    c->mean_iq_a += (iq_a - c->mean_iq_a) / (float)(++c->samples);
    if (c->samples < COGGING_SAMPLE_TICKS)
    {
        return;
    }
    /* 单点完成：双向平均后按 Q15 满量程钳位写入，再推进索引/方向/换向或结束。 */
    {
        float counts = c->mean_iq_a * (32768.0f / c->full_scale_a);
        if (c->direction_pass != 0U)
        {
            counts = (counts + (float)c->iq_q15[c->index]) * 0.5f;
        }
        c->iq_q15[c->index] = (int16_t)fmaxf(-32768.0f, fminf(32767.0f, roundf(counts)));
    }
    ++c->points_done;
    c->point_ticks = c->stable_ticks = c->samples = 0U;
    c->mean_iq_a = 0.0f;
    c->window_rejected_ticks = c->consecutive_rejected_ticks = 0U;
    if (c->points_done == 2U * COGGING_MAP_POINTS)
    {
        c->state = COGGING_COMPLETE;
        return;
    }
    if (c->points_done == COGGING_MAP_POINTS)
    {
        c->direction_pass = 1U;
        c->target_rad = (float)(++c->target_grid) * COGGING_STEP_RAD;
        c->state = COGGING_TURNAROUND;
        return;
    }
    if (c->direction_pass == 0U)
    {
        c->index = (c->index + 1U) & (COGGING_MAP_POINTS - 1U);
        c->target_rad = (float)(++c->target_grid) * COGGING_STEP_RAD;
    }
    else
    {
        c->index = (c->index - 1U) & (COGGING_MAP_POINTS - 1U);
        c->target_rad = (float)(--c->target_grid) * COGGING_STEP_RAD;
    }
    c->state = COGGING_SETTLING;
}
