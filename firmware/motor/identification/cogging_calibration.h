#ifndef COGGING_CALIBRATION_H
#define COGGING_CALIBRATION_H
#include <stdbool.h>
#include <stdint.h>

#include "cogging_map.h"

/* 齿槽标定过程状态机：网格定位、稳定/采样窗口与双向遍历。
 * 只处理采样过程，表格式与成表见 cogging_map；硬件交互见 FOC 适配层。 */

#define COGGING_STEP_RAD (6.283185307179586f / (float)COGGING_MAP_POINTS)
#define COGGING_POSITION_TOL_RAD (6.283185307179586f / 16384.0f) /* 0.0219727 度 */
/* 跨圈浮点相减可能把恰好 4 个 Q15 码位的误差舍入到门限之上；
 * 该 <0.011 码位余量是数值容差，不是新增一格。 */
#define COGGING_POSITION_GATE_RAD (COGGING_POSITION_TOL_RAD + 0.000001f)
#define COGGING_SETTLE_TICKS 200U /* 2 kHz 下 100 ms 的合格保持时间 */
#define COGGING_SAMPLE_TICKS 200U /* 2 kHz 下 100 ms 的合格块平均采样 */

/** 标定状态：定位、采样、换向与完成/失败。 */
typedef enum
{
    COGGING_IDLE = 0,
    COGGING_SETTLING,
    COGGING_SAMPLING,
    COGGING_TURNAROUND,
    COGGING_COMPLETE,
    COGGING_FAILED,
    COGGING_SAVING
} CoggingState;

/** 结束原因：0 正常，其余对应取消、输入无效、超时、限流、保护退出与保存失败。 */
typedef enum
{
    COGGING_OK = 0,
    COGGING_CANCELLED,
    COGGING_INVALID_INPUT,
    COGGING_POINT_TIMEOUT,
    COGGING_CURRENT_LIMIT,
    COGGING_SAFETY_FAULT,
    COGGING_SAVE_FAILED
} CoggingReason;

/** 标定状态机与暂存表：由固定 2 kHz 所有者更新，前台只读快照。 */
typedef struct
{
    CoggingState state;
    CoggingReason reason;
    uint32_t index, direction_pass, points_done;
    uint32_t point_ticks, stable_ticks, samples, saturation_ticks, total_ticks;
    int32_t target_grid;
    float target_rad, mean_iq_a, full_scale_a, filtered_velocity_rad_s;
    /* 窗口覆盖率与突发无效计数：仅算法判据，不对外观测。 */
    uint32_t window_rejected_ticks, consecutive_rejected_ticks;
    int16_t iq_q15[COGGING_MAP_POINTS];
} CoggingCalibration;

/**
 * @brief 从当前位置之后最近的网格点开始一整圈双向标定。
 * @param c 标定状态机，所有权归调用方。
 * @param position_rad 当前有方向线性化机械角，单位 rad，范围 [0, 2π)。
 * @param full_scale_a 采样满量程，单位 A，用于 Q15 编码。
 * @return 参数合法并已复位状态机返回 true；否则返回 false 且不修改状态。
 * @note 前台调用；后续 Update 由固定 2 kHz 所有者驱动。
 */
bool CoggingCalibration_Start(CoggingCalibration *c, float position_rad, float full_scale_a);

/**
 * @brief 标定单拍：超时、饱和、稳定性与合格覆盖率判定，合格时累加 Iq 均值。
 * @param c 标定状态机。
 * @param position_rad 当前机械角，单位 rad。
 * @param velocity_rad_s 连续机械速度，单位 rad/s。
 * @param iq_a 本拍块平均 Iq 反馈，单位 A。
 * @param saturated 本拍电流是否饱和。
 * @note 固定 2 kHz 所有者调用；不做浮点以外的阻塞操作。
 */
void CoggingCalibration_Update(
    CoggingCalibration *c, float position_rad, float velocity_rad_s, float iq_a, bool saturated);

/**
 * @brief 中止进行中的标定并记录原因；非活动状态调用无效果。
 * @param c 标定状态机。
 * @param reason 中止原因，写入 reason 字段。
 * @note 可从保护路径调用；不操作 PWM、不切换模式。
 */
void CoggingCalibration_Abort(CoggingCalibration *c, CoggingReason reason);
#endif
