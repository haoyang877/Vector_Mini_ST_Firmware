#ifndef POSITION_OTG_H
#define POSITION_OTG_H

#include <stdbool.h>

typedef struct
{
    float position_rad, speed_rad_s, acceleration_rad_s2;
} PositionOtgReference;

typedef struct
{
    float speed_rad_s, acceleration_rad_s2, jerk_rad_s3;
} PositionOtgLimits;

/* 用两个单精度分量记录段边界和累计时间，seconds 是最近可表示值。 */
typedef struct
{
    float seconds, remainder_s;
} PositionOtgTime;

/* 实例内的系数表，供静态分配；调用者通过下面的接口更新。 */
typedef struct
{
    PositionOtgReference start;
    float jerk_rad_s3;
    PositionOtgTime duration;
} PositionOtgSegment;

typedef struct
{
    PositionOtgReference reference;
    PositionOtgLimits limits;
    PositionOtgSegment segments[7];
    PositionOtgTime segment_elapsed;
    unsigned segment_count, segment_index;
    float target_rad;
    bool initialized, has_plan, finished;
} PositionOtg;

/**
 * @brief 从给定运动状态初始化或重置一个规划器。
 * @param otg 调用者持有的独立实例。
 * @param initial 当前机械位置 rad、速度 rad/s、加速度 rad/s²。
 * @return 输入有限时初始化成功，否则保留原实例。
 * @note 实例间可重入；同一实例的写操作由调用者串行调度。
 */
bool PositionOtg_Reset(PositionOtg *otg, PositionOtgReference initial);

/**
 * @brief 从当前参考接续到新位置，终点速度和加速度均为零。
 * @param otg 已初始化的实例。
 * @param target_rad 有限目标机械位置，rad。
 * @param limits 速度 rad/s、加速度 rad/s²、jerk rad/s³ 的正绝对上限。
 * @return 成功或已有相同计划返回 true；不可行时返回 false 并保留旧计划。
 * @note 仅在目标或限制变化时建表，本调用不推进时间。
 */
bool PositionOtg_SetTarget(PositionOtg *otg, float target_rad, PositionOtgLimits limits);

/**
 * @brief 采样当前计划并把新参考发布到实例的 reference 字段。
 * @param otg 已接受目标的实例；finished 表示已到达静止终点。
 * @param dt_s 正且有限的推进时间，s。
 * @return 成功返回 true，输入非法时保留原参考。
 * @note 普通拍仅求段内多项式，跨段拍最多经过七段。
 */
bool PositionOtg_Sample(PositionOtg *otg, float dt_s);

#endif
